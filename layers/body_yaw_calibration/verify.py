"""Offline/startup verifier for the body-yaw calibration hook."""

from __future__ import annotations

import json
import math

import body_yaw_calibration as calibration
from reachy_mini.daemon.backend.robot.backend import RobotBackend


def main() -> None:
    if not calibration.is_applied():
        raise SystemExit("body-yaw calibration hook is not applied")
    for name in (
        "update_target_head_joints_from_ik",
        "set_target_head_joint_positions",
        "enable_motors",
        "goto_target",
    ):
        method = getattr(RobotBackend, name)
        if getattr(method, "__body_yaw_calibration__", None) != calibration.VERSION:
            raise SystemExit(f"body-yaw calibration wrapper missing from {name}")

    expected = {
        -100.0: -110.1363669,
        0.0: 0.9617463,
        100.0: 108.1277955,
    }
    for logical, wanted in expected.items():
        actual = calibration.calibrated_body_yaw_deg(logical)
        if not math.isclose(actual, wanted, abs_tol=1e-6):
            raise SystemExit(
                f"calibration mismatch at {logical:+g}: {actual:+.9f} != {wanted:+.9f}"
            )

    print(json.dumps({"applied": True, "version": calibration.VERSION}))


if __name__ == "__main__":
    main()
