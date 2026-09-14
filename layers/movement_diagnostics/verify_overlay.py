#!/usr/bin/env python3
"""Live verification that BOTH the target-chain shim and the existing
corrected-daemon overlay are active. Run with the DAEMON's Python interpreter
and the daemon's effective environment (``manage.sh`` does this).

    verify_overlay.py <host> <port> [<expected_reachy_mini_dir>]

Checks:
  1. ``/api/state/ws/full`` returns the instrumented target and applied-state
     fields, plus a successful cached body-motor control-table snapshot — the
     ``target_*`` keys (may be ``null`` when no target is set) and
     ``automatic_body_yaw_enabled`` as an **actual boolean** (a ``null`` there
     means the shim could not read the applied kinematics state → fail);
  2. ``reachy_mini`` imports, and — when ``<expected_reachy_mini_dir>`` is given
     (the corrected-overlay source dir) — resolves under it (the overlay is not
     shadowed);
  3. best-effort: the corrected overlay's added module still imports.

Prints one JSON line. Exit 0 only when 1 and 2 pass.
"""

from __future__ import annotations

import json
import sys
import time


# target_* may legitimately be null (no target commanded yet) — key presence is
# enough. automatic_body_yaw_enabled must be an ACTUAL boolean: a null there
# means the shim could not read the applied kinematics state.
_KEY_PRESENT_FIELDS = ("target_body_yaw", "target_head_joints")
_BOOL_FIELDS = ("automatic_body_yaw_enabled",)
_MOTOR_FIELDS = ("body_motor_diagnostics",)
_REQUIRED_FIELDS = _KEY_PRESENT_FIELDS + _BOOL_FIELDS + _MOTOR_FIELDS
_MIN_DISTINCT_MOTOR_POLLS = 5
_MIN_MOTOR_POLL_SUCCESS_RATIO = 0.8


def _field_ok(doc: dict, field: str) -> bool:
    if field in _BOOL_FIELDS:
        return isinstance(doc.get(field), bool)
    if field in _MOTOR_FIELDS:
        value = doc.get(field)
        return isinstance(value, dict) and value.get("ok") is True
    return field in doc


def _check_instrumented_fields(host: str, port: str, seconds: float = 3.0) -> dict:
    try:
        from websockets.sync.client import connect
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"websockets unavailable: {e}"}
    qs = ("frequency=20&with_body_yaw=true&with_target_body_yaw=true"
          "&with_target_head_joints=true&use_pose_matrix=false")
    url = f"ws://{host}:{port}/api/state/ws/full?{qs}"
    frames = 0
    last_present: dict[str, bool] = {}
    poll_results: dict[int, bool] = {}
    last_doc: dict = {}
    try:
        with connect(url, open_timeout=5) as ws:
            t0 = time.monotonic()
            while time.monotonic() - t0 < seconds:
                try:
                    raw = ws.recv(timeout=2.0)
                except Exception:  # noqa: BLE001 - TimeoutError / clean close: stop, evaluate
                    break
                try:
                    doc = json.loads(raw)
                except ValueError:
                    continue
                frames += 1
                last_doc = doc
                last_present = {f: _field_ok(doc, f) for f in _REQUIRED_FIELDS}
                motor = doc.get("body_motor_diagnostics")
                health = motor.get("health") if isinstance(motor, dict) else None
                sequence = health.get("poll_sequence") if isinstance(health, dict) else None
                last_ok = health.get("last_poll_ok") if isinstance(health, dict) else None
                if isinstance(sequence, int) and isinstance(last_ok, bool) and sequence > 0:
                    poll_results[sequence] = last_ok
    except Exception as e:  # noqa: BLE001 - could not open the stream
        return {"ok": False, "error": f"state stream unreadable: {e}", "url": url}
    if frames == 0:
        return {"ok": False, "error": "no state frames received"}
    successes = sum(poll_results.values())
    ratio = successes / len(poll_results) if poll_results else 0.0
    motor = last_doc.get("body_motor_diagnostics")
    poll_period = motor.get("poll_period_s") if isinstance(motor, dict) else None
    fresh = (
        isinstance(motor, dict)
        and motor.get("ok") is True
        and isinstance(poll_period, (int, float))
        and 0 < poll_period <= 1.0
        and isinstance(motor.get("sample_age_s"), (int, float))
        and motor["sample_age_s"] <= 2 * poll_period
    )
    polls_ok = (
        len(poll_results) >= _MIN_DISTINCT_MOTOR_POLLS
        and ratio >= _MIN_MOTOR_POLL_SUCCESS_RATIO
        and fresh
    )
    if all(last_present.values()) and polls_ok:
        return {
            "ok": True,
            "frames": frames,
            "distinct_motor_polls": len(poll_results),
            "motor_poll_success_ratio": ratio,
            "target_body_yaw": last_doc.get("target_body_yaw"),
            "target_head_joints_len": len(last_doc.get("target_head_joints") or []),
            "automatic_body_yaw_enabled": last_doc.get("automatic_body_yaw_enabled"),
            "body_motor_diagnostics": motor,
        }
    bad = [f for f, ok in last_present.items() if not ok]
    return {"ok": False, "frames": frames,
            "distinct_motor_polls": len(poll_results),
            "motor_poll_success_ratio": ratio,
            "error": (f"instrumented fields or motor poll health not valid: "
                      f"{bad or list(_REQUIRED_FIELDS)} "
                      f"(automatic_body_yaw_enabled must be a real boolean and "
                      f"body_motor_diagnostics must finish at least "
                      f"{_MIN_DISTINCT_MOTOR_POLLS} distinct polls with >= "
                      f"{_MIN_MOTOR_POLL_SUCCESS_RATIO:.0%} success and a fresh "
                      f"successful final sample relative to its declared "
                      f"poll_period_s)"),
            "present": last_present}


def _check_reachy_mini(expected_dir: str | None) -> dict:
    try:
        import reachy_mini
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"import reachy_mini failed: {e}"}
    path = getattr(reachy_mini, "__file__", "") or ""
    out: dict = {"reachy_mini_file": path}
    if expected_dir:
        ok = path.startswith(expected_dir.rstrip("/") + "/") or expected_dir in path
        out["ok"] = ok
        if not ok:
            out["error"] = f"reachy_mini resolves outside the corrected overlay {expected_dir}"
    else:
        out["ok"] = True
    # best-effort: the frame-pose-sync patch adds this module
    try:
        import reachy_mini.media.camera_timestamps  # noqa: F401
        out["camera_timestamps_import"] = "ok"
    except Exception as e:  # noqa: BLE001
        out["camera_timestamps_import"] = f"absent ({e})"
    return out


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = sys.argv[2] if len(sys.argv) > 2 else "8000"
    expected_dir = sys.argv[3] if len(sys.argv) > 3 else None

    fields = _check_instrumented_fields(host, port)
    overlay = _check_reachy_mini(expected_dir)
    ok = bool(fields.get("ok") and overlay.get("ok"))
    print(json.dumps({"ok": ok, "instrumented_fields": fields, "corrected_overlay": overlay}))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
