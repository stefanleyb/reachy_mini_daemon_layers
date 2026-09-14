"""Optional, reversible re-exposure of diagnostic state on the daemon stream.

Three read-only additions to ``/api/state/ws/full``, for the movement-diagnostic
session only:

1. **The target chain.** On daemon ``1.10.0`` the ``/api/state/(ws/)full``
   handler *computes* ``target_body_yaw`` / ``target_head_joints`` when asked,
   but its ``FullState`` response model does not declare them, so pydantic drops
   them. The subclass declares them and coerces the daemon's real runtime types
   (``target_head_joint_positions`` is a NumPy array) to JSON-safe values in a
   ``mode="before"`` validator, so ``model_dump_json()`` succeeds.

2. **``automatic_body_yaw_enabled``.** On daemon ``1.10.0``, ``/ws/sdk``
   commands (including ``SetAutomaticBodyYawCmd``) are fire-and-forget — the
   handler discards the response callback — so the diagnostic cannot confirm the
   disable took effect. ``apply()`` wraps ``get_full_state`` so every frame also
   carries the value **actually applied to the kinematics engine**
   (``backend.head_kinematics.automatic_body_yaw``), not the value the
   diagnostic requested. If the backend cannot report it, the declared field
   keeps its ``None`` default and is **serialised as ``null``** (never a
   fabricated boolean); ``verify_overlay.py`` / ``check_targets.py`` / the
   diagnostic precondition all require an actual ``true`` / ``false`` and treat
   ``null`` (or a missing key) as a failure.

3. **Body-motor control-table evidence.** The ID is resolved from the backend's
   ``body_rotation`` mapping (with the repository's ID 10 only as a recorded
   fallback). One controller-bound background thread reads through the daemon's
   low-level queue at 5 Hz; consumers only copy its cache. Failures are therefore
   rate-limited too. Configuration refreshes every 5 s with a fingerprint and
   change revision; live Goal/trajectory/present/effort/status values refresh at
   5 Hz. Poll sequence and success/failure health make verification operate on
   distinct reads rather than duplicate WebSocket frames. No register is written.

``apply()``:

* changes **no** robot behaviour — it only adds read-only fields to a response;
* edits **nothing** on disk (no ``site-packages`` write);
* affects the ``/api/state/ws/full`` **WebSocket** stream (what the diagnostic
  and ``verify_overlay.py`` consume). The REST ``/api/state/full`` route has a
  ``-> FullState`` return annotation, so FastAPI's response-model layer still
  strips the extra fields there; that path is not used;
* is reverted by ``revert()`` or by removing the systemd drop-in (see
  ``manage.sh``).
"""

from __future__ import annotations

import copy
import functools
import hashlib
import json
import struct
import threading
import time
from typing import Any, Optional

_ORIGINAL: dict[str, Any] = {}

AUTO_BODY_YAW_FIELD = "automatic_body_yaw_enabled"
BODY_MOTOR_FIELD = "body_motor_diagnostics"
BODY_MOTOR_FALLBACK_ID = 10
BODY_MOTOR_POLL_PERIOD_S = 0.2
BODY_MOTOR_CONFIG_POLL_PERIOD_S = 5.0

_POLLER_LOCK = threading.Lock()
_POLLER: Optional["_MotorPoller"] = None

_MODEL_NAMES = {
    1190: "XL330-M077",
    1200: "XL330-M288",
    1210: "XC330-T181",
    1220: "XC330-T288",
    1230: "XC330-M181",
    1240: "XC330-M288",
}


def _to_json_safe(v: Any) -> Any:
    """NumPy array / scalar (or anything with ``.tolist()``) -> plain Python."""
    if v is None:
        return None
    tolist = getattr(v, "tolist", None)
    if callable(tolist):
        return tolist()
    return v


def read_applied_auto_body_yaw(backend: Any) -> bool:
    """The automatic-body-yaw state ACTUALLY APPLIED by the kinematics engine.

    ``Backend.set_automatic_body_yaw`` forwards to
    ``head_kinematics.set_automatic_body_yaw`` which sets
    ``head_kinematics.automatic_body_yaw``. That attribute — not any value the
    caller sent — is the applied state. Raises if it cannot be read.
    """
    kin = getattr(backend, "head_kinematics", None)
    if kin is None or not hasattr(kin, "automatic_body_yaw"):
        raise AttributeError("backend.head_kinematics.automatic_body_yaw unavailable")
    return bool(kin.automatic_body_yaw)


def _raw(controller: Any, motor_id: int, address: int, length: int) -> bytes:
    value = bytes(controller.async_read_raw_bytes(motor_id, address, length))
    if len(value) != length:
        raise RuntimeError(
            f"motor {motor_id} register {address} returned "
            f"{len(value)} bytes, expected {length}"
        )
    return value


def _u16(data: bytes, address: int, base: int) -> int:
    return struct.unpack_from("<H", data, address - base)[0]


def _i16(data: bytes, address: int, base: int) -> int:
    return struct.unpack_from("<h", data, address - base)[0]


def _u32(data: bytes, address: int, base: int) -> int:
    return struct.unpack_from("<I", data, address - base)[0]


def _i32(data: bytes, address: int, base: int) -> int:
    return struct.unpack_from("<i", data, address - base)[0]


def _read_motor_configuration(controller: Any, motor_id: int) -> dict[str, Any]:
    """Read defined configuration registers without spanning reserved gaps."""
    identity = _raw(controller, motor_id, 0, 14)
    calibration = _raw(controller, motor_id, 20, 8)
    electrical_limits = _raw(controller, motor_id, 31, 9)
    motion_limits = _raw(controller, motor_id, 44, 12)
    startup = _raw(controller, motor_id, 60, 1)[0]
    pwm_shutdown = _raw(controller, motor_id, 62, 2)
    pid = _raw(controller, motor_id, 76, 10)
    feedforward = _raw(controller, motor_id, 88, 4)
    model_number = _u16(identity, 0, 0)
    return {
        "motor_id": motor_id,
        "model_number": model_number,
        "model_name": _MODEL_NAMES.get(model_number, "unknown"),
        "firmware_version": identity[6],
        "configured_id": identity[7],
        "baud_rate_code": identity[8],
        "return_delay_time": identity[9],
        "drive_mode": identity[10],
        "operating_mode": identity[11],
        "homing_offset_pulses": _i32(calibration, 20, 20),
        "moving_threshold_raw": _u32(calibration, 24, 20),
        "drive_mode_reverse": bool(identity[10] & 0x01),
        "secondary_id": identity[12],
        "protocol_type": identity[13],
        "temperature_limit_c": electrical_limits[0],
        "max_voltage_limit_raw": _u16(electrical_limits, 32, 31),
        "min_voltage_limit_raw": _u16(electrical_limits, 34, 31),
        "pwm_limit_raw": _u16(electrical_limits, 36, 31),
        "current_limit_raw": _u16(electrical_limits, 38, 31),
        "velocity_limit_raw": _u32(motion_limits, 44, 44),
        "max_position_limit_pulses": _u32(motion_limits, 48, 44),
        "min_position_limit_pulses": _u32(motion_limits, 52, 44),
        "startup_configuration": startup,
        "pwm_slope_raw": pwm_shutdown[0],
        "shutdown_mask": pwm_shutdown[1],
        "velocity_i_gain": _u16(pid, 76, 76),
        "velocity_p_gain": _u16(pid, 78, 76),
        "position_d_gain": _u16(pid, 80, 76),
        "position_i_gain": _u16(pid, 82, 76),
        "position_p_gain": _u16(pid, 84, 76),
        "feedforward_2nd_gain": _u16(feedforward, 88, 88),
        "feedforward_1st_gain": _u16(feedforward, 90, 88),
    }


def _read_dynamic_motor_state(controller: Any, motor_id: int) -> dict[str, Any]:
    # Read live singleton registers separately; this avoids relying on firmware
    # behaviour for blocks that span reserved addresses.
    torque_enabled = bool(_raw(controller, motor_id, 64, 1)[0])
    hardware_error = _raw(controller, motor_id, 70, 1)[0]
    bus_watchdog = struct.unpack("<b", _raw(controller, motor_id, 98, 1))[0]
    data = _raw(controller, motor_id, 100, 47)
    moving_status = data[123 - 100]
    profile_code = (moving_status >> 4) & 0x03
    profile_names = {
        0: "step",
        1: "rectangular",
        2: "triangular",
        3: "trapezoidal",
    }
    goal = _i32(data, 116, 100)
    present = _i32(data, 132, 100)
    raw_error_pulses = goal - present
    # Position mode is one turn. Report the shortest encoder difference so a
    # wrap at 0/4095 cannot masquerade as a full-turn following error.
    error_pulses = (raw_error_pulses + 2048) % 4096 - 2048
    return {
        "torque_enabled": torque_enabled,
        "hardware_error_status": hardware_error,
        "bus_watchdog_raw": bus_watchdog,
        "goal_pwm_raw": _i16(data, 100, 100),
        "goal_current_raw": _i16(data, 102, 100),
        "goal_velocity_raw": _i32(data, 104, 100),
        "profile_acceleration_raw": _u32(data, 108, 100),
        "profile_velocity_raw": _u32(data, 112, 100),
        "goal_position_pulses": goal,
        "realtime_tick_ms": _u16(data, 120, 100),
        "moving": bool(data[122 - 100]),
        "moving_status_raw": moving_status,
        "velocity_profile_code": profile_code,
        "velocity_profile": profile_names[profile_code],
        "following_error": bool(moving_status & 0x08),
        "profile_ongoing": bool(moving_status & 0x02),
        "in_position": bool(moving_status & 0x01),
        "present_pwm_raw": _i16(data, 124, 100),
        "present_current_raw": _i16(data, 126, 100),
        "present_velocity_raw": _i32(data, 128, 100),
        "present_position_pulses": present,
        "velocity_trajectory_raw": _i32(data, 136, 100),
        "position_trajectory_pulses": _i32(data, 140, 100),
        "present_voltage_raw": _u16(data, 144, 100),
        "present_temperature_c": data[146 - 100],
        "goal_present_error_raw_pulses": raw_error_pulses,
        "goal_present_error_shortest_pulses": error_pulses,
        "goal_present_error_shortest_deg": error_pulses * (360.0 / 4096.0),
    }


def _configuration_fingerprint(configuration: dict[str, Any]) -> str:
    raw = json.dumps(configuration, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def _resolve_motor_id(backend: Any) -> tuple[int, str]:
    mapping = getattr(backend, "name2id", None)
    if isinstance(mapping, dict) and "body_rotation" in mapping:
        return int(mapping["body_rotation"]), "backend.name2id"
    controller = getattr(backend, "c", None)
    get_mapping = getattr(controller, "get_motor_name_id", None)
    if callable(get_mapping):
        mapping = get_mapping()
        if isinstance(mapping, dict) and "body_rotation" in mapping:
            return int(mapping["body_rotation"]), "controller.get_motor_name_id"
    return BODY_MOTOR_FALLBACK_ID, "fallback"


class _MotorPoller:
    """One controller-bound poller; state-stream consumers only copy its cache."""

    def __init__(self, backend: Any) -> None:
        self.controller = getattr(backend, "c", None)
        self.motor_id, self.motor_id_source = _resolve_motor_id(backend)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="movement-diagnostic-motor-poll", daemon=True
        )
        self._configuration: Optional[dict[str, Any]] = None
        self._configuration_fingerprint: Optional[str] = None
        self._configuration_revision = 0
        self._configuration_changed_at_poll: Optional[int] = None
        self._next_configuration_poll = 0.0
        self._dynamic: Optional[dict[str, Any]] = None
        self._last_poll_at = 0.0
        self._last_success_at = 0.0
        self._health: dict[str, Any] = {
            "poll_sequence": 0,
            "poll_attempts": 0,
            "poll_successes": 0,
            "poll_failures": 0,
            "consecutive_failures": 0,
            "last_poll_ok": False,
            "last_error": "first motor poll pending",
        }

    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        """Signal shutdown without waiting; safe from the daemon event loop."""
        self._stop.set()

    def stop(self) -> None:
        """Signal and join for synchronous teardown paths such as revert/tests."""
        self.request_stop()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _poll_once(self) -> None:
        now = time.monotonic()
        with self._lock:
            sequence = int(self._health["poll_sequence"]) + 1
            configuration_due = (
                self._configuration is None or now >= self._next_configuration_poll
            )
        try:
            if self.controller is None or not hasattr(
                self.controller, "async_read_raw_bytes"
            ):
                raise RuntimeError("backend motor controller raw-read unavailable")
            new_configuration = None
            new_fingerprint = None
            if configuration_due:
                new_configuration = _read_motor_configuration(
                    self.controller, self.motor_id
                )
                new_fingerprint = _configuration_fingerprint(new_configuration)
            new_dynamic = _read_dynamic_motor_state(self.controller, self.motor_id)
            completed_at = time.monotonic()
            with self._lock:
                if new_configuration is not None:
                    if (
                        self._configuration_fingerprint is not None
                        and new_fingerprint != self._configuration_fingerprint
                    ):
                        self._configuration_revision += 1
                        self._configuration_changed_at_poll = sequence
                    self._configuration = new_configuration
                    self._configuration_fingerprint = new_fingerprint
                    self._next_configuration_poll = (
                        completed_at + BODY_MOTOR_CONFIG_POLL_PERIOD_S
                    )
                self._dynamic = new_dynamic
                self._last_poll_at = completed_at
                self._last_success_at = completed_at
                self._health.update(
                    poll_sequence=sequence,
                    poll_attempts=self._health["poll_attempts"] + 1,
                    poll_successes=self._health["poll_successes"] + 1,
                    consecutive_failures=0,
                    last_poll_ok=True,
                    last_error=None,
                )
        except Exception as exc:  # noqa: BLE001 - publish failure; never guess
            completed_at = time.monotonic()
            with self._lock:
                self._last_poll_at = completed_at
                self._health.update(
                    poll_sequence=sequence,
                    poll_attempts=self._health["poll_attempts"] + 1,
                    poll_failures=self._health["poll_failures"] + 1,
                    consecutive_failures=self._health["consecutive_failures"] + 1,
                    last_poll_ok=False,
                    last_error=str(exc),
                )

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            self._poll_once()
            remaining = BODY_MOTOR_POLL_PERIOD_S - (time.monotonic() - started)
            # Even an over-period serial timeout leaves a small idle window so
            # a failing motor cannot cause back-to-back queue submissions.
            self._stop.wait(max(0.02, remaining))

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            health = copy.deepcopy(self._health)
            configuration = copy.deepcopy(self._configuration)
            dynamic = copy.deepcopy(self._dynamic)
            last_poll_at = self._last_poll_at
            last_success_at = self._last_success_at
            fingerprint = self._configuration_fingerprint
            revision = self._configuration_revision
            changed_at = self._configuration_changed_at_poll
        ok = bool(
            health["last_poll_ok"] and configuration is not None and dynamic is not None
        )
        return {
            "ok": ok,
            "motor_id": self.motor_id,
            "motor_id_source": self.motor_id_source,
            "poll_period_s": BODY_MOTOR_POLL_PERIOD_S,
            "configuration": configuration,
            "configuration_fingerprint": fingerprint,
            "configuration_revision": revision,
            "configuration_changed_at_poll": changed_at,
            "dynamic": dynamic,
            "health": health,
            "sample_age_s": None if not last_poll_at else now - last_poll_at,
            "last_success_age_s": None if not last_success_at else now - last_success_at,
        }


def _get_or_start_poller(backend: Any) -> _MotorPoller:
    global _POLLER
    controller = getattr(backend, "c", None)
    with _POLLER_LOCK:
        if _POLLER is not None and _POLLER.controller is controller:
            return _POLLER
        if _POLLER is not None:
            # Never join here: this function runs inline on the daemon API event
            # loop. The old poller owns a different controller and cache, so it
            # can finish an in-flight read independently after being signalled.
            _POLLER.request_stop()
        _POLLER = _MotorPoller(backend)
        _POLLER.start()
    return _POLLER


def build_model(base: type) -> type:
    from pydantic import field_validator

    class FullStateWithDiagState(base):  # type: ignore[misc, valid-type]
        # Declared explicitly (not extra="allow") so serialisation is defined.
        target_body_yaw: Optional[float] = None
        target_head_joints: Optional[list[float]] = None
        automatic_body_yaw_enabled: Optional[bool] = None
        body_motor_diagnostics: Optional[dict[str, Any]] = None

        @field_validator("target_body_yaw", "target_head_joints", mode="before")
        @classmethod
        def _coerce_target_types(cls, value: Any) -> Any:
            return _to_json_safe(value)

    FullStateWithDiagState.__name__ = "FullStateWithDiagState"
    return FullStateWithDiagState


def _wrap_get_full_state(orig: Any) -> Any:
    @functools.wraps(orig)
    async def _patched(*args: Any, **kwargs: Any) -> Any:
        fs = await orig(*args, **kwargs)
        backend = kwargs.get("backend")
        if backend is None and args:
            backend = args[-1]
        if backend is not None:
            try:
                fs.automatic_body_yaw_enabled = read_applied_auto_body_yaw(backend)
            except Exception:  # noqa: BLE001 - leave the field null rather than guess
                pass
            # One controller-bound background poller owns all raw reads. This
            # per-frame path performs no I/O and no executor submission.
            fs.body_motor_diagnostics = _get_or_start_poller(backend).snapshot()
        return fs

    return _patched


def apply() -> None:
    from reachy_mini.daemon.app.routers import state as state_router

    if "FullState" in _ORIGINAL:
        return  # already applied — idempotent
    _ORIGINAL["FullState"] = state_router.FullState
    _ORIGINAL["get_full_state"] = state_router.get_full_state
    state_router.FullState = build_model(_ORIGINAL["FullState"])  # type: ignore[attr-defined]
    state_router.get_full_state = _wrap_get_full_state(  # type: ignore[attr-defined]
        _ORIGINAL["get_full_state"]
    )


def revert() -> None:
    global _POLLER
    if "FullState" not in _ORIGINAL:
        return
    from reachy_mini.daemon.app.routers import state as state_router

    state_router.FullState = _ORIGINAL.pop("FullState")
    state_router.get_full_state = _ORIGINAL.pop("get_full_state")
    with _POLLER_LOCK:
        poller = _POLLER
        _POLLER = None
    if poller is not None:
        poller.stop()


def is_applied() -> bool:
    return "FullState" in _ORIGINAL
