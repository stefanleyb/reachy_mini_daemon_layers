"""Rate-limit only the daemon's sustained face-loss return.

The stock daemon holds the last tracking aim for its existing loss timeout and
then targets fixed world-neutral.  This temporary layer preserves both choices
and only caps the orientation change of that return.  Ordinary face tracking,
target acquisition and reacquisition remain owned by the original daemon
method.
"""

from __future__ import annotations

import functools
import math
import time
from typing import Any

import numpy as np

from reachy_mini.utils.interpolation import (  # type: ignore[import-untyped]
    linear_pose_interpolation,
)


VERSION = "2026-09-13.1"
MAX_ANGULAR_SPEED_DEG_S = 40.0
# Do not turn a delayed control-loop tick into one large actuator setpoint.
MAX_STEP_ELAPSED_S = 0.1

_ORIGINAL: Any = None
_OWNED_BEFORE = False
_PATCHED_CLASS: Any = None
_LAST_STEP_ATTRIBUTE = "_face_loss_return_last_step_at"


def _copy_pose(value: Any) -> Any:
    return value.copy() if value is not None and hasattr(value, "copy") else value


def _rotation_distance(start: np.ndarray, target: np.ndarray) -> float:
    """Shortest orientation distance between two homogeneous poses."""
    relative = start[:3, :3].T @ target[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def _limited_orientation(
    start: np.ndarray,
    target: np.ndarray,
    stock_result: np.ndarray,
    elapsed_s: float,
) -> np.ndarray:
    """Keep stock translation but cap the shortest-path orientation step."""
    distance = _rotation_distance(start, target)
    max_step = math.radians(MAX_ANGULAR_SPEED_DEG_S) * max(
        0.0, min(float(elapsed_s), MAX_STEP_ELAPSED_S)
    )
    fraction = 1.0 if distance <= max_step or distance <= 1e-12 else max_step / distance
    limited = linear_pose_interpolation(start, target, fraction)
    limited[:3, 3] = stock_result[:3, 3]
    return limited


def _apply_to_class(robot_backend: Any) -> None:
    global _ORIGINAL, _OWNED_BEFORE, _PATCHED_CLASS
    if _PATCHED_CLASS is not None:
        return

    original_step = robot_backend.step_head_tracking

    @functools.wraps(original_step)
    def limited_step(self: Any, *args: Any, **kwargs: Any) -> Any:
        previous_time = getattr(self, _LAST_STEP_ATTRIBUTE, None)
        previous_aim = _copy_pose(getattr(self, "_tracking_aim", None))
        if previous_aim is None:
            previous_aim = _copy_pose(self.get_current_head_pose())

        result = original_step(self, *args, **kwargs)
        # Sample after the daemon step so crossing the timeout inside the
        # original call cannot leave one uncapped stock interpolation tick.
        now = time.monotonic()

        lock = getattr(self, "_tracking_lock")
        with lock:
            setattr(self, _LAST_STEP_ATTRIBUTE, now)
            last_seen = getattr(self, "_last_face_seen", None)
            timeout = float(getattr(self, "_tracking_lost_timeout"))
            active = bool(
                getattr(self, "_tracking_enabled", False)
                and float(getattr(self, "_tracking_requested_weight", 0.0)) > 0.0
                and last_seen is not None
                and now - float(last_seen) >= timeout
            )
            stock_aim = getattr(self, "_tracking_aim", None)
            target = getattr(self, "_tracking_target_pose", None)
            init = getattr(self, "INIT_HEAD_POSE")
            if (
                active
                and previous_aim is not None
                and stock_aim is not None
                and target is not None
                and np.allclose(target, init, atol=1e-12)
            ):
                elapsed = 0.0 if previous_time is None else now - float(previous_time)
                self._tracking_aim = _limited_orientation(
                    previous_aim, target, stock_aim, elapsed
                )
                self.ik_required = True
                self.face_loss_return_active = True
            else:
                self.face_loss_return_active = False
            self.face_loss_return_max_angular_speed_deg_s = MAX_ANGULAR_SPEED_DEG_S

        return result

    limited_step.__face_loss_return__ = VERSION  # type: ignore[attr-defined]
    _OWNED_BEFORE = "step_head_tracking" in robot_backend.__dict__
    _ORIGINAL = original_step
    robot_backend.step_head_tracking = limited_step
    _PATCHED_CLASS = robot_backend


def apply() -> None:
    """Install the loss-return limiter on the physical robot backend, once."""
    from reachy_mini.daemon.backend.robot.backend import (  # type: ignore[import-untyped]
        RobotBackend,
    )

    _apply_to_class(RobotBackend)


def revert() -> None:
    """Restore the backend method (primarily useful to isolated tests)."""
    global _ORIGINAL, _OWNED_BEFORE, _PATCHED_CLASS
    if _PATCHED_CLASS is None:
        return
    if _OWNED_BEFORE:
        _PATCHED_CLASS.step_head_tracking = _ORIGINAL
    else:
        delattr(_PATCHED_CLASS, "step_head_tracking")
    _ORIGINAL = None
    _OWNED_BEFORE = False
    _PATCHED_CLASS = None


def is_applied() -> bool:
    return _PATCHED_CLASS is not None
