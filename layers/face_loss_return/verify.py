"""Offline/startup verifier for the face-loss return-rate hook."""

from __future__ import annotations

import json

import face_loss_return as limiter
from reachy_mini.daemon.backend.robot.backend import (  # type: ignore[import-untyped]
    RobotBackend,
)


def main() -> None:
    if not limiter.is_applied():
        raise SystemExit("face-loss return hook is not applied")
    marker = getattr(RobotBackend.step_head_tracking, "__face_loss_return__", None)
    if marker != limiter.VERSION:
        raise SystemExit("face-loss return wrapper is missing or has the wrong version")
    if limiter.MAX_ANGULAR_SPEED_DEG_S != 40.0:
        raise SystemExit("face-loss return angular-speed limit is not 40 deg/s")
    print(
        json.dumps(
            {
                "applied": True,
                "version": limiter.VERSION,
                "max_angular_speed_deg_s": limiter.MAX_ANGULAR_SPEED_DEG_S,
            }
        )
    )


if __name__ == "__main__":
    main()
