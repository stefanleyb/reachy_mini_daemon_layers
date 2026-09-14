"""Verifier for the body-motor I-gain + clamp hook.

The point of this file is that it must NOT report success unless the behavioural
correction is actually in force. An earlier version returned 0 when motor
telemetry was unavailable, which meant the layer manager could print "enabled
and verified" while the register write had failed and the motor sat at I=0.
Failing open is right for daemon availability; it is wrong for a verification
claim.

Four checks, and every one must pass:

1. the hook is applied in this interpreter;
2. all three patched methods carry this version's marker;
3. the clamp bounds an out-of-range angle;
4. the gain is CONFIRMED in force -- from live motor telemetry if the
   movement-diagnostics layer is enabled, otherwise from the status record the
   hook writes at apply time. If neither can confirm it, this exits non-zero.
"""

from __future__ import annotations

import json
import math

import body_motor_igain as igain
from reachy_mini.daemon.backend.robot.backend import RobotBackend

_MARKER = "__body_motor_igain__"
_PATCHED = ("__init__", "update_target_head_joints_from_ik",
            "set_target_head_joint_positions")


def _live_gain(host: str = "127.0.0.1", port: str = "8000",
               seconds: float = 5.0) -> int | None:
    """The gain the motor reports on the state stream, or None if unavailable."""
    try:
        import time

        from websockets.sync.client import connect
    except Exception:  # noqa: BLE001
        return None
    url = (f"ws://{host}:{port}/api/state/ws/full"
           f"?frequency=10&with_body_yaw=true")
    try:
        with connect(url, open_timeout=5) as ws:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                try:
                    raw = ws.recv(timeout=2.0)
                except Exception:  # noqa: BLE001
                    break
                try:
                    doc = json.loads(raw)
                except ValueError:
                    continue
                motor = doc.get("body_motor_diagnostics")
                if isinstance(motor, dict) and motor.get("ok"):
                    cfg = motor.get("configuration") or {}
                    if "position_i_gain" in cfg:
                        return int(cfg["position_i_gain"])
    except Exception:  # noqa: BLE001
        return None
    return None


def main() -> None:
    if not igain.is_applied():
        raise SystemExit("body-motor I-gain hook is not applied")

    for name in _PATCHED:
        method = getattr(RobotBackend, name)
        if getattr(method, _MARKER, None) != igain.VERSION:
            raise SystemExit(f"hook marker missing from RobotBackend.{name}")

    # 3. the clamp must actually bound an out-of-range angle
    beyond = math.radians(igain.BODY_YAW_LIMIT_DEG + 40.0)
    for probe in (beyond, -beyond):
        bounded = math.degrees(igain.clamp_body_yaw_rad(probe))
        if abs(abs(bounded) - igain.BODY_YAW_LIMIT_DEG) > 1e-6:
            raise SystemExit(
                f"clamp failed: {math.degrees(probe):+.1f} deg -> {bounded:+.1f}")

    # 4. the gain must be CONFIRMED, not assumed
    live = _live_gain()
    if live is not None:
        if live != igain.POSITION_I_GAIN:
            raise SystemExit(
                f"motor reports position_i_gain {live}, "
                f"expected {igain.POSITION_I_GAIN}")
        source, confirmed = "live telemetry", live
    else:
        status = igain.read_status()
        if status is None:
            raise SystemExit(
                "cannot confirm the gain reached the motor: no live telemetry "
                "(enable movement-diagnostics) and no fresh status record from "
                "the hook. Refusing to report this layer as verified.")
        if not status.get("applied"):
            raise SystemExit(
                f"the hook did NOT apply the gain: {status.get('error')}")
        if status.get("readback") != igain.POSITION_I_GAIN:
            raise SystemExit(
                f"the hook read back {status.get('readback')}, "
                f"expected {igain.POSITION_I_GAIN}")
        source, confirmed = "hook status record", status["readback"]

    print(json.dumps({
        "applied": True, "version": igain.VERSION,
        "position_i_gain": confirmed, "confirmed_by": source,
        "body_yaw_clamp_deg": igain.BODY_YAW_LIMIT_DEG,
    }))


if __name__ == "__main__":
    main()
