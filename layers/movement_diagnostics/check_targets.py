#!/usr/bin/env python3
"""Report whether the daemon's full-state WebSocket exposes the instrumented fields.

Connects to ``/api/state/ws/full`` for a couple of seconds and prints a JSON
line: ``{"instrumented_fields_present": bool, "frames": N, "sample": {...}}``.
Requires the ``target_*`` keys (may be ``null``),
``automatic_body_yaw_enabled`` as an **actual boolean**, and a successful
``body_motor_diagnostics`` snapshot. Exit 0
if present, 2 if absent/invalid, 1 on connection error. Used by ``manage.sh
status`` and by hand after install / uninstall.
"""

from __future__ import annotations

import json
import sys
import time

QS = (
    "frequency=20&with_body_yaw=true&with_target_body_yaw=true"
    "&with_target_head_joints=true&use_pose_matrix=false"
)
MIN_DISTINCT_MOTOR_POLLS = 5
MIN_MOTOR_POLL_SUCCESS_RATIO = 0.8


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = sys.argv[2] if len(sys.argv) > 2 else "8000"
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5
    try:
        from websockets.sync.client import connect
    except Exception as e:  # noqa: BLE001
        print(json.dumps({"error": f"websockets unavailable: {e}"}))
        return 1

    url = f"ws://{host}:{port}/api/state/ws/full?{QS}"
    frames = 0
    sample = None
    last_doc = None
    poll_results = {}
    try:
        with connect(url, open_timeout=5) as ws:
            t0 = time.monotonic()
            while time.monotonic() - t0 < seconds:
                try:
                    raw = ws.recv(timeout=2.0)
                except Exception:  # TimeoutError or a clean/abrupt close: evaluate evidence
                    break
                frames += 1
                try:
                    doc = json.loads(raw)
                except ValueError:
                    continue
                motor = doc.get("body_motor_diagnostics")
                health = motor.get("health") if isinstance(motor, dict) else None
                sequence = health.get("poll_sequence") if isinstance(health, dict) else None
                last_ok = health.get("last_poll_ok") if isinstance(health, dict) else None
                if isinstance(sequence, int) and isinstance(last_ok, bool) and sequence > 0:
                    poll_results[sequence] = last_ok
                last_doc = doc
    except Exception as e:  # noqa: BLE001
        print(json.dumps({"error": str(e), "url": url}))
        return 1

    motor = last_doc.get("body_motor_diagnostics") if isinstance(last_doc, dict) else None
    ratio = sum(poll_results.values()) / len(poll_results) if poll_results else 0.0
    poll_period = motor.get("poll_period_s") if isinstance(motor, dict) else None
    fresh = (
        isinstance(motor, dict)
        and motor.get("ok") is True
        and isinstance(poll_period, (int, float))
        and 0 < poll_period <= 1.0
        and isinstance(motor.get("sample_age_s"), (int, float))
        and motor["sample_age_s"] <= 2 * poll_period
    )
    present = bool(
        isinstance(last_doc, dict)
        and "target_body_yaw" in last_doc
        and "target_head_joints" in last_doc
        and isinstance(last_doc.get("automatic_body_yaw_enabled"), bool)
        and len(poll_results) >= MIN_DISTINCT_MOTOR_POLLS
        and ratio >= MIN_MOTOR_POLL_SUCCESS_RATIO
        and fresh
    )
    if present:
        sample = {
            k: last_doc.get(k)
            for k in ("body_yaw", "target_body_yaw", "target_head_joints",
                      "automatic_body_yaw_enabled", "body_motor_diagnostics")
        }
    print(json.dumps({"instrumented_fields_present": present, "frames": frames,
                      "distinct_motor_polls": len(poll_results),
                      "motor_poll_success_ratio": ratio, "sample": sample}))
    return 0 if present else 2


if __name__ == "__main__":
    raise SystemExit(main())
