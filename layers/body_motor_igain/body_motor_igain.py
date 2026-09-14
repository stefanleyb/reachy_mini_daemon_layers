"""Body motor position I gain, plus the commanded-range clamp it requires.

Why the gain
------------
The body motor ships with ``position_i_gain = 0``. Proportional-only control
stops wherever ``Kp * error`` balances the cable spring and friction, so the body
never quite arrives: measured 1.7 deg short at +-20 deg and 6-8.5 deg at
+-120 deg. A small integrator removes that standing error.

Measured on this robot (2026-09-13/14), a ladder bracketing the useful value:

    I=12   0.97 deg short          -- too weak to arrive
    I=25   0.39/0.53/0.70 deg at +-20/45/90 deg, one crossing   <- chosen
    I=50   1.23 deg overshoot, 2 crossings      -- starting to oscillate
    I=200  1.92 deg overshoot, 8 reversals at ~2 Hz, and it jammed the robot

Live face tracking with I=25: endpoint error 1.04 deg median against 3.5-5.5 deg
before, and the best-followed session on record (median head-body separation
6.7 deg, roughly half the previous best).

The gain lives in the motor's RAM, so every power cycle clears it and every
daemon start reapplies it. Disabling this layer returns the robot to stock.

Why the clamp is part of the SAME layer
---------------------------------------
An integrator never gives up. Handed a target the mechanism cannot reach it
winds up and pushes until something gives -- that jammed this robot twice, at
+-160 deg (I=200, 567 PWM, motor overload) and at -150 deg (I=25, stalled at
-143.6 deg holding 324 PWM). The reachable envelope is asymmetric, roughly
+150 / -143 deg.

The guard therefore belongs with the thing that needs guarding. Clamping in one
client (the body-following app) leaves every other client -- a demo script,
another app, the dashboard -- driving an integrator with no bound at all. So the
bound lives here, in the daemon, and is enabled and disabled together with the
gain.

Where exactly it is applied, and where it is deliberately NOT:

* ``update_target_head_joints_from_ik`` -- the SEMANTIC body yaw is bounded
  BEFORE IK runs. Clamping the solution afterwards would leave the six Stewart
  joints solving for the original angle, so the head would point somewhere IK
  never intended (160 deg clamped to 120 deg leaves the head 40 deg out).
* ``set_target_head_joint_positions`` -- joint 0 is bounded BEFORE the target is
  published, so the 50 Hz motor thread can never read an unbounded value.
* ``set_head_operation_mode`` assigns ``target_head_joint_positions`` directly
  and is deliberately left alone. It pins the target to the MEASURED position to
  avoid a jump on a mode change; clamping a measured position would command
  exactly the sudden movement that pin exists to prevent, and a measured
  position cannot exceed the mechanism anyway. This does mean the two wrappers
  are not literally "every writer" -- that claim was too strong.

+-120 deg is inside the region measured safe at I=25: endpoint error
0.41-0.47 deg, overshoot <= 1.49 deg, standing PWM 51-129 of 885.

What it does NOT touch: P gain, D gain, PWM limit, current limit, temperature
limit, shutdown mask. Only address 82, two bytes.
"""

from __future__ import annotations

import json
import logging
import math
import os
import struct
import tempfile
import time
from typing import Any

VERSION = "2026-09-14.4"

POSITION_I_GAIN_ADDRESS = 82
POSITION_I_GAIN = 25
BODY_YAW_LIMIT_DEG = 120.0
BODY_MOTOR_NAME = "body_rotation"

STATUS_PATH = os.path.join(tempfile.gettempdir(), "reachy_body_motor_igain.json")

_ORIGINAL: dict[str, Any] = {}
_PATCHED_CLASS: type | None = None
_MARKER = "__body_motor_igain__"

logger = logging.getLogger(__name__)


class MotorIdentityError(RuntimeError):
    """The body motor could not be identified authoritatively."""


def resolve_motor_id(backend: Any) -> int:
    """The body motor's id from the runtime mapping. No fallback.

    A register-writing layer must not guess. If ``name2id`` is missing or does
    not name ``body_rotation``, something is wrong with the backend and writing
    to a hardcoded id could reach a different motor on a different robot. Refuse
    instead: the gain is an improvement, not a necessity, and the daemon runs
    perfectly well without it.
    """
    mapping = getattr(backend, "name2id", None)
    if not isinstance(mapping, dict) or BODY_MOTOR_NAME not in mapping:
        raise MotorIdentityError(
            f"cannot resolve {BODY_MOTOR_NAME!r} from backend.name2id "
            f"({type(mapping).__name__}); refusing to write a motor register")
    try:
        motor_id = int(mapping[BODY_MOTOR_NAME])
    except (TypeError, ValueError) as exc:
        raise MotorIdentityError(
            f"{BODY_MOTOR_NAME} id is not an integer: {mapping[BODY_MOTOR_NAME]!r}"
        ) from exc
    if not 1 <= motor_id <= 252:
        raise MotorIdentityError(f"{BODY_MOTOR_NAME} id {motor_id} out of range")
    return motor_id


def read_position_i_gain(backend: Any) -> int:
    controller = getattr(backend, "c", None)
    if controller is None:
        raise RuntimeError("motor controller unavailable")
    raw = bytes(controller.async_read_raw_bytes(
        resolve_motor_id(backend), POSITION_I_GAIN_ADDRESS, 2))
    return int(struct.unpack("<H", raw)[0])


def write_position_i_gain(backend: Any, value: int) -> int:
    """Write ONLY address 82 and return what the motor reports afterwards.

    Deliberately a raw two-byte write rather than ``async_write_pid_gains``,
    which would rewrite P and D as well.
    """
    if not 0 <= value <= 16383:
        raise ValueError(f"position I gain {value} out of range 0..16383")
    controller = getattr(backend, "c", None)
    if controller is None:
        raise RuntimeError("motor controller unavailable")
    motor_id = resolve_motor_id(backend)
    controller.async_write_raw_bytes(
        motor_id, POSITION_I_GAIN_ADDRESS, list(struct.pack("<H", int(value))))
    return read_position_i_gain(backend)


def clamp_body_yaw_rad(value: float) -> float:
    limit = math.radians(BODY_YAW_LIMIT_DEG)
    return max(-limit, min(limit, float(value)))


def _write_status(**fields: Any) -> None:
    """Record what actually happened, for the out-of-process verifier.

    The hook deliberately does not raise when the write fails -- an unavailable
    improvement must never take the daemon down. But then the manager must not
    be able to report "verified" either, and the verifier runs in a different
    process, so it needs a channel. This is it. The pid and timestamp let the
    verifier reject a stale file from an earlier daemon.
    """
    payload = {"version": VERSION, "pid": os.getpid(), "written_at": time.time(),
               **fields}
    try:
        tmp = f"{STATUS_PATH}.{os.getpid()}.tmp"
        with open(tmp, "w") as fh:
            json.dump(payload, fh)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, STATUS_PATH)
        os.chmod(STATUS_PATH, 0o644)
    except Exception as exc:  # noqa: BLE001
        logger.warning("could not record I-gain status: %s", exc)


def _apply_to_class(robot_backend: type) -> None:
    global _PATCHED_CLASS
    original_init = robot_backend.__init__
    original_ik = robot_backend.update_target_head_joints_from_ik
    original_set = robot_backend.set_target_head_joint_positions

    def igain_init(self: Any, *args: Any, **kwargs: Any) -> None:
        original_init(self, *args, **kwargs)
        # After __init__ the controller exists and name2id is populated: the
        # first moment the motor can be reached at all.
        try:
            motor_id = resolve_motor_id(self)
            seen = write_position_i_gain(self, POSITION_I_GAIN)
            ok = seen == POSITION_I_GAIN
            _write_status(applied=ok, motor_id=motor_id,
                          requested=POSITION_I_GAIN, readback=seen,
                          clamp_deg=BODY_YAW_LIMIT_DEG,
                          error=None if ok else f"readback {seen}")
            if ok:
                logger.info("body motor position_i_gain = %d (verified), "
                            "joint-0 clamp +-%.0f deg", seen, BODY_YAW_LIMIT_DEG)
            else:
                logger.error("body motor position_i_gain wrote %d but reads %d",
                             POSITION_I_GAIN, seen)
        except Exception as exc:  # noqa: BLE001 - never take the daemon down
            _write_status(applied=False, motor_id=None,
                          requested=POSITION_I_GAIN, readback=None,
                          clamp_deg=BODY_YAW_LIMIT_DEG, error=str(exc))
            logger.error("body motor position_i_gain not applied: %s", exc)

    def clamped_ik(self: Any, pose: Any = None, body_yaw: Any = None,
                   *args: Any, **kwargs: Any) -> Any:
        """Bound the SEMANTIC body yaw before IK, never the solution after it.

        IK solves all seven joints as a function of body_yaw: the six Stewart
        joints position the head *relative to the body*. Clamping joint 0 after
        the fact would leave those six solving for the original angle, so the
        head would end up pointing somewhere IK never intended -- a requested
        160 deg clamped to 120 deg would leave the head 40 deg out. Clamping the
        input instead keeps the whole solution self-consistent, and also keeps
        ``_last_target_body_yaw`` truthful.
        """
        if body_yaw is not None:
            body_yaw = clamp_body_yaw_rad(body_yaw)
        else:
            # The original falls back to self.target_body_yaw; bound that source
            # too, or an out-of-range stored target slips past.
            stored = getattr(self, "target_body_yaw", None)
            if stored is not None:
                bounded = clamp_body_yaw_rad(stored)
                if bounded != stored:
                    self.target_body_yaw = bounded
        return original_ik(self, pose, body_yaw, *args, **kwargs)

    def clamped_set(self: Any, positions: Any, *args: Any, **kwargs: Any) -> Any:
        """Bound joint 0 BEFORE the target is published.

        Correcting the slot afterwards left a window in which the 50 Hz motor
        thread could read an unbounded target. A safety boundary must never
        publish an unsafe value, even briefly.
        """
        return original_set(self, _bounded_positions(positions), *args, **kwargs)

    for fn in (igain_init, clamped_ik, clamped_set):
        setattr(fn, _MARKER, VERSION)
    _ORIGINAL["__init__"] = original_init
    _ORIGINAL["update_target_head_joints_from_ik"] = original_ik
    _ORIGINAL["set_target_head_joint_positions"] = original_set
    # Remember WHICH class was patched: assuming RobotBackend in revert() would
    # write this class's methods into a different one.
    _PATCHED_CLASS = robot_backend
    robot_backend.__init__ = igain_init
    robot_backend.update_target_head_joints_from_ik = clamped_ik
    robot_backend.set_target_head_joint_positions = clamped_set


def _bounded_positions(positions: Any) -> Any:
    """A copy of a joint-space target with joint 0 bounded. Other joints kept.

    Direct joint-space commands carry no IK solution to invalidate -- the caller
    supplied the joint values -- so bounding joint 0 alone is correct here, and
    is the last guard before a client-chosen body angle reaches the motor.

    Returns a COPY: mutating the caller's array would be a surprising
    side-effect, and the original may be reused.
    """
    if positions is None:
        return positions
    try:
        if len(positions) == 0:
            return positions
        current = float(positions[0])
    except (TypeError, ValueError, IndexError):
        return positions
    bounded = clamp_body_yaw_rad(current)
    if bounded == current:
        return positions
    try:
        copied = positions.copy()
    except AttributeError:
        copied = list(positions)
    copied[0] = bounded
    logger.warning("body yaw target %.1f deg clamped to %.1f deg "
                   "(integrator active)", math.degrees(current),
                   math.degrees(bounded))
    return copied


def apply() -> None:
    if _ORIGINAL:
        return  # idempotent
    from reachy_mini.daemon.backend.robot.backend import RobotBackend

    _apply_to_class(RobotBackend)


def revert() -> None:
    """Restore the original methods. Does NOT clear the motor's gain.

    Clearing it here would be wrong: ``revert`` runs in whatever process imported
    the hook, which may not own the motor. Power-cycle instead -- the gain is in
    RAM -- or disable the layer and restart the daemon.
    """
    global _PATCHED_CLASS
    if not _ORIGINAL or _PATCHED_CLASS is None:
        return
    for name, original in _ORIGINAL.items():
        setattr(_PATCHED_CLASS, name, original)
    _ORIGINAL.clear()
    _PATCHED_CLASS = None


def is_applied() -> bool:
    return bool(_ORIGINAL)


def _process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True          # exists, owned by someone else
    except Exception:  # noqa: BLE001
        return False
    return True


def read_status(require_live_process: bool = True) -> dict[str, Any] | None:
    """The recorded outcome of the last apply, or None if it cannot be trusted.

    Freshness is decided by whether the process that wrote the record is still
    running -- NOT by wall-clock age. The record is written once, when the
    daemon constructs its backend, and stays true for as long as that daemon
    lives; a healthy robot up for a week has a week-old record describing the
    current state perfectly. An age limit would have started failing
    verification after an hour of uptime, which matters because this is the only
    confirmation channel when the movement-diagnostics layer is disabled.

    A daemon restart rewrites the record, so a dead pid means the record belongs
    to a daemon that is gone and must not be believed.
    """
    try:
        with open(STATUS_PATH) as fh:
            status = json.load(fh)
    except Exception:  # noqa: BLE001
        return None
    if status.get("version") != VERSION:
        return None
    try:
        pid = int(status.get("pid", -1))
    except (TypeError, ValueError):
        return None
    if require_live_process and not _process_alive(pid):
        return None
    return status
