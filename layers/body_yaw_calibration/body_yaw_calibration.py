"""Stateless body-yaw calibration for this Reachy Mini.

The daemon and kinematics continue to use logical physical angles.  On the
real robot backend only, joint 0 is converted to the raw motor target measured
for this unit.  Mock and simulation backends are untouched.
"""

from __future__ import annotations

import asyncio
import functools
import math
import threading
from typing import Any


VERSION = "2026-09-12.3"
COEFFICIENTS = (-0.000196603199, 1.09132081, 0.961746311)
RAW_LIMIT_DEG = 160.0

_ORIGINAL: dict[str, Any] = {}
_OWNED_BEFORE: set[str] = set()
_PATCHED_CLASS: type | None = None
_PINNING_ATTRIBUTE = "_body_yaw_calibration_pinning"
_TRAJECTORY_ATTRIBUTE = "_body_yaw_calibration_trajectory"
_STATE_LOCK_ATTRIBUTE = "_body_yaw_calibration_state_lock"


def _state_lock(backend: Any) -> threading.RLock:
    lock = getattr(backend, _STATE_LOCK_ATTRIBUTE, None)
    if lock is None:
        lock = threading.RLock()
        setattr(backend, _STATE_LOCK_ATTRIBUTE, lock)
    return lock


def calibrated_body_yaw_deg(logical_deg: float) -> float:
    """Convert a logical physical body angle to this unit's raw motor angle."""
    logical_deg = max(-RAW_LIMIT_DEG, min(RAW_LIMIT_DEG, float(logical_deg)))
    quadratic, linear, offset = COEFFICIENTS
    raw = quadratic * logical_deg**2 + linear * logical_deg + offset
    return max(-RAW_LIMIT_DEG, min(RAW_LIMIT_DEG, raw))


def calibrated_body_yaw_rad(logical_rad: float) -> float:
    return math.radians(calibrated_body_yaw_deg(math.degrees(float(logical_rad))))


def logical_body_yaw_for_raw_deg(raw_deg: float) -> float:
    """Invert the monotonic calibration over the daemon's existing range."""
    raw_deg = max(-RAW_LIMIT_DEG, min(RAW_LIMIT_DEG, float(raw_deg)))
    low = -RAW_LIMIT_DEG
    high = RAW_LIMIT_DEG
    for _ in range(60):
        middle = (low + high) / 2.0
        if calibrated_body_yaw_deg(middle) < raw_deg:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def logical_body_yaw_for_raw_rad(raw_rad: float) -> float:
    return math.radians(logical_body_yaw_for_raw_deg(math.degrees(float(raw_rad))))


def _correct_positions(positions: Any) -> tuple[Any, float, float]:
    """Return a copy with calibrated joint 0 plus logical/raw values."""
    copied = positions.copy() if hasattr(positions, "copy") else list(positions)
    logical = float(copied[0])
    raw = calibrated_body_yaw_rad(logical)
    copied[0] = raw
    return copied, logical, raw


def _trajectory_raw_target(backend: Any, logical: float, calibrated: float) -> float:
    """Interpolate monotonically between the applied and final raw targets.

    Static inverse calibration is correct at a requested endpoint, but applying
    it directly to a daemon goto's measured starting angle changes the motor
    target at t=0. During a body-yaw goto, use the daemon's logical trajectory
    only for its progress parameter. Apply that progress directly between the
    raw target in force at entry and the calibrated final endpoint. This is
    continuous and cannot reverse between its endpoints. Direct set_target
    commands keep the ordinary static mapping.
    """
    trajectory = getattr(backend, _TRAJECTORY_ATTRIBUTE, None)
    if trajectory is None:
        return calibrated

    if not trajectory["started"]:
        prior = trajectory["prior_logical"]
        if prior is not None and abs(logical - prior) <= 1e-9:
            trajectory["raw_last"] = trajectory["raw_start"]
            return trajectory["raw_start"]
        # This is the first waypoint actually emitted by GotoMove. Capturing
        # here preserves its exact interpolation origin without allowing a
        # preceding tracking-IK tick to start the blend.
        trajectory["logical_start"] = logical
        trajectory["started"] = True
    start = trajectory["logical_start"]
    target = trajectory["logical_target"]
    span = target - start
    if abs(span) <= 1e-12:
        progress = 0.0
    else:
        progress = max(0.0, min(1.0, (logical - start) / span))
    if trajectory["raw_target"] is None:
        candidate = calibrated_body_yaw_rad(target)
        # At a large angle, a small inward logical move can have a calibrated
        # endpoint beyond the raw target already in force. That would make the
        # entire motor path run opposite to the requested motion. Preserve the
        # requested direction in this narrow case and forego endpoint
        # compensation for this move; the measured controller can reassess.
        if abs(span) > 1e-12 and (candidate - trajectory["raw_start"]) * span < 0.0:
            candidate = trajectory["raw_start"] + span
        limit = math.radians(RAW_LIMIT_DEG)
        trajectory["raw_target"] = max(-limit, min(limit, candidate))
    raw = trajectory["raw_start"] + progress * (
        trajectory["raw_target"] - trajectory["raw_start"]
    )
    limit = math.radians(RAW_LIMIT_DEG)
    raw = max(-limit, min(limit, raw))
    trajectory["progress"] = progress
    trajectory["raw_last"] = raw
    return raw


def _record(backend: Any, logical: float, raw: float) -> None:
    # These attributes are deliberately simple: the optional diagnostic layer
    # can expose them without calibration owning any telemetry route.
    backend.body_yaw_calibration_logical_rad = logical
    backend.body_yaw_calibration_raw_rad = raw


def _apply_to_class(robot_backend: type) -> None:
    global _PATCHED_CLASS
    if _PATCHED_CLASS is not None:
        return

    original_ik = robot_backend.update_target_head_joints_from_ik
    original_joints = robot_backend.set_target_head_joint_positions
    original_enable = robot_backend.enable_motors
    original_goto = robot_backend.goto_target

    @functools.wraps(original_ik)
    def calibrated_ik(self: Any, *args: Any, **kwargs: Any) -> Any:
        with _state_lock(self):
            result = original_ik(self, *args, **kwargs)
            positions = getattr(self, "target_head_joint_positions", None)
            if positions is not None:
                corrected, logical, raw = _correct_positions(positions)
                raw = _trajectory_raw_target(self, logical, raw)
                corrected[0] = raw
                self.target_head_joint_positions = corrected
                _record(self, logical, raw)
            return result

    @functools.wraps(original_joints)
    def calibrated_joint_target(self: Any, positions: Any) -> Any:
        # enable_motors() intentionally pins the measured joints before it does
        # anything else.  Skip exactly that operation, regardless of whether
        # torque happened to be on already when enable_motors() was called.
        with _state_lock(self):
            if positions is not None and not bool(
                getattr(self, _PINNING_ATTRIBUTE, False)
            ):
                positions, logical, raw = _correct_positions(positions)
                _record(self, logical, raw)
            return original_joints(self, positions)

    @functools.wraps(original_enable)
    def calibrated_enable_motors(self: Any, *args: Any, **kwargs: Any) -> Any:
        missing = object()
        previous = getattr(self, _PINNING_ATTRIBUTE, missing)
        with _state_lock(self):
            setattr(self, _PINNING_ATTRIBUTE, True)
            try:
                result = original_enable(self, *args, **kwargs)
                return result
            finally:
                positions = getattr(self, "target_head_joint_positions", None)
                if positions is not None:
                # The daemon pins the raw measured joint and also copies that
                # raw value into its logical body-yaw slot. Reconcile the slot
                # now so the next tracking IK tick reproduces the same raw
                # target instead of calibrating the measurement a second time.
                    self.target_body_yaw = logical_body_yaw_for_raw_rad(
                        float(positions[0])
                    )
                if previous is missing:
                    delattr(self, _PINNING_ATTRIBUTE)
                else:
                    setattr(self, _PINNING_ATTRIBUTE, previous)

    @functools.wraps(original_goto)
    async def calibrated_goto_target(
        self: Any, *args: Any, **kwargs: Any
    ) -> Any:
        # ``body_yaw`` is the fifth positional parameter on the daemon method;
        # normal callers, including the task endpoint, pass it by keyword.
        body_yaw = kwargs.get("body_yaw", args[4] if len(args) > 4 else 0.0)
        if body_yaw is None:
            return await original_goto(self, *args, **kwargs)

        missing = object()
        owner_task = asyncio.current_task()
        with _state_lock(self):
            positions = getattr(self, "target_head_joint_positions", None)
            raw_start = (float(positions[0]) if positions is not None
                         else float(self.get_present_body_yaw()))
            previous = getattr(self, _TRAJECTORY_ATTRIBUTE, missing)
            if previous is not missing and previous.get("owner_task") is not owner_task:
                raise RuntimeError("overlapping calibrated body-yaw gotos are unsupported")
            trajectory = {
                "logical_start": None,
                "logical_target": float(body_yaw),
                "prior_logical": getattr(self, "target_body_yaw", None),
                "started": False,
                "raw_start": raw_start,
                "raw_target": None,
                "raw_last": raw_start,
                "progress": 0.0,
                "owner_task": owner_task,
            }
            setattr(self, _TRAJECTORY_ATTRIBUTE, trajectory)
        try:
            return await original_goto(self, *args, **kwargs)
        finally:
            with _state_lock(self):
                positions = getattr(self, "target_head_joint_positions", None)
                raw_exit = (float(positions[0]) if positions is not None
                            else trajectory["raw_last"])
                # Reconcile every exit, including normal fallback/zero-span
                # completion, before making the context invisible to IK.
                self.target_body_yaw = logical_body_yaw_for_raw_rad(
                    raw_exit
                )
                if previous is missing:
                    delattr(self, _TRAJECTORY_ATTRIBUTE)
                else:
                    setattr(self, _TRAJECTORY_ATTRIBUTE, previous)

    calibrated_ik.__body_yaw_calibration__ = VERSION  # type: ignore[attr-defined]
    calibrated_joint_target.__body_yaw_calibration__ = VERSION  # type: ignore[attr-defined]
    calibrated_enable_motors.__body_yaw_calibration__ = VERSION  # type: ignore[attr-defined]
    calibrated_goto_target.__body_yaw_calibration__ = VERSION  # type: ignore[attr-defined]
    names_and_methods = {
        "update_target_head_joints_from_ik": original_ik,
        "set_target_head_joint_positions": original_joints,
        "enable_motors": original_enable,
        "goto_target": original_goto,
    }
    for name, original in names_and_methods.items():
        if name in robot_backend.__dict__:
            _OWNED_BEFORE.add(name)
        _ORIGINAL[name] = original
    robot_backend.update_target_head_joints_from_ik = calibrated_ik
    robot_backend.set_target_head_joint_positions = calibrated_joint_target
    robot_backend.enable_motors = calibrated_enable_motors
    robot_backend.goto_target = calibrated_goto_target
    _PATCHED_CLASS = robot_backend


def apply() -> None:
    """Install the correction on the physical robot backend, once."""
    from reachy_mini.daemon.backend.robot.backend import RobotBackend

    _apply_to_class(RobotBackend)


def revert() -> None:
    """Restore the backend methods (primarily useful to isolated tests)."""
    global _PATCHED_CLASS
    if _PATCHED_CLASS is None:
        return
    for name, original in _ORIGINAL.items():
        if name in _OWNED_BEFORE:
            setattr(_PATCHED_CLASS, name, original)
        else:
            delattr(_PATCHED_CLASS, name)
    _ORIGINAL.clear()
    _OWNED_BEFORE.clear()
    _PATCHED_CLASS = None


def is_applied() -> bool:
    return _PATCHED_CLASS is not None
