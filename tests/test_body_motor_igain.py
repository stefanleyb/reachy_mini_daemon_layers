"""The body-motor I-gain hook: what it writes, and what it refuses to touch."""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MANAGER_ROOT = os.environ.get(
    "REACHY_LAYER_MANAGER_ROOT",
    os.path.join(os.path.dirname(ROOT), "reachy_mini_layer_manager"),
)
sys.path.insert(0, os.path.join(ROOT, "layers", "body_motor_igain"))

import body_motor_igain as igain  # noqa: E402


class _FakeController:
    """A motor whose control table we can inspect."""

    def __init__(self, i_gain=0, p_gain=200, d_gain=0):
        self.mem = bytearray(200)
        struct.pack_into("<H", self.mem, 80, d_gain)
        struct.pack_into("<H", self.mem, 82, i_gain)
        struct.pack_into("<H", self.mem, 84, p_gain)
        self.writes = []

    def async_read_raw_bytes(self, motor_id, address, length):
        return list(self.mem[address:address + length])

    def async_write_raw_bytes(self, motor_id, address, data):
        self.writes.append((motor_id, address, list(data)))
        self.mem[address:address + len(data)] = bytes(data)


class _FakeBackend:
    def __init__(self, controller=None):
        self.c = controller or _FakeController()
        self.name2id = {"body_rotation": 10, "stewart_1": 11}


class GainValueTest(unittest.TestCase):
    def test_the_chosen_gain_is_the_one_the_ladder_bracketed(self) -> None:
        # 12 too weak, 25 good, 50 oscillates, 200 jammed the robot
        self.assertEqual(igain.POSITION_I_GAIN, 25)

    def test_writes_only_the_i_gain_register(self) -> None:
        b = _FakeBackend()
        igain.write_position_i_gain(b, 25)
        self.assertEqual([w[1] for w in b.c.writes], [82],
                         "only address 82 may ever be written")
        self.assertEqual([len(w[2]) for w in b.c.writes], [2])

    def test_p_and_d_are_left_alone(self) -> None:
        b = _FakeBackend(_FakeController(i_gain=0, p_gain=200, d_gain=0))
        igain.write_position_i_gain(b, 25)
        self.assertEqual(struct.unpack_from("<H", b.c.mem, 84)[0], 200, "P changed")
        self.assertEqual(struct.unpack_from("<H", b.c.mem, 80)[0], 0, "D changed")
        self.assertEqual(igain.read_position_i_gain(b), 25)

    def test_out_of_range_is_refused_before_any_write(self) -> None:
        b = _FakeBackend()
        for bad in (-1, 16384, 70000):
            with self.assertRaises(ValueError):
                igain.write_position_i_gain(b, bad)
        self.assertEqual(b.c.writes, [])

    def test_motor_id_comes_from_the_runtime_mapping(self) -> None:
        b = _FakeBackend()
        b.name2id = {"body_rotation": 7}
        igain.write_position_i_gain(b, 25)
        self.assertEqual(b.c.writes[0][0], 7)

    def test_refuses_to_write_when_the_motor_cannot_be_identified(self) -> None:
        """No fallback id: guessing could reach a different motor entirely."""
        for mapping in ({}, None, {"stewart_1": 11}, "not-a-dict",
                        {"body_rotation": "ten"}, {"body_rotation": 0},
                        {"body_rotation": 300}):
            b = _FakeBackend()
            b.name2id = mapping
            with self.assertRaises((igain.MotorIdentityError, Exception)):
                igain.write_position_i_gain(b, 25)
            self.assertEqual(b.c.writes, [],
                             f"wrote despite unresolvable mapping {mapping!r}")


def _fake_backend_class(calls=None, controller_cls=None, name2id=None):
    """A stand-in with the three members the hook patches."""
    import numpy as np

    class FakeRobotBackend:
        def __init__(self, tag="x"):
            self.tag = tag
            self.c = (controller_cls or _FakeController)()
            self.name2id = ({"body_rotation": 10} if name2id is None else name2id)
            self.target_head_joint_positions = None
            if calls is not None:
                calls.append(tag)

        def update_target_head_joints_from_ik(self, pose=None, body_yaw=None):
            # Mimics the real IK: the six Stewart joints are a FUNCTION of
            # body_yaw, so a post-hoc clamp of joint 0 would desynchronise them.
            if body_yaw is None:
                body_yaw = getattr(self, "target_body_yaw", 0.0) or 0.0
            self.ik_body_yaw = body_yaw
            self.target_head_joint_positions = np.array(
                [body_yaw] + [body_yaw * 0.1] * 6)

        def set_target_head_joint_positions(self, positions):
            self.target_head_joint_positions = np.array(positions, dtype=float)

    return FakeRobotBackend


class ClampTest(unittest.TestCase):
    """The clamp must bound joint 0 on EVERY path, not just in one client."""

    def setUp(self) -> None:
        igain.revert()
        igain._ORIGINAL.clear()
        self.cls = _fake_backend_class()
        igain._apply_to_class(self.cls)

    def tearDown(self) -> None:
        igain.revert()
        igain._ORIGINAL.clear()

    def test_limit_is_inside_the_measured_safe_envelope(self) -> None:
        # -150 deg stalled at -143.6 deg; +-120 deg measured safe at I=25
        self.assertLessEqual(igain.BODY_YAW_LIMIT_DEG, 120.0)

    def test_bounds_the_ik_path_BEFORE_the_solve(self) -> None:
        """The clamp must reach IK's input, not its output.

        Clamping joint 0 after the solve leaves the six Stewart joints computed
        for the original angle, so the head points somewhere IK never intended.
        """
        b = self.cls()
        b.update_target_head_joints_from_ik(body_yaw=math.radians(165.0))
        self.assertAlmostEqual(math.degrees(b.ik_body_yaw),
                               igain.BODY_YAW_LIMIT_DEG, places=6)
        self.assertAlmostEqual(math.degrees(b.target_head_joint_positions[0]),
                               igain.BODY_YAW_LIMIT_DEG, places=6)

    def test_the_whole_ik_solution_is_consistent_after_clamping(self) -> None:
        """Every joint must correspond to the SAME body angle."""
        b = self.cls()
        b.update_target_head_joints_from_ik(body_yaw=math.radians(165.0))
        solved = b.target_head_joint_positions
        expected_others = solved[0] * 0.1
        for joint in solved[1:]:
            self.assertAlmostEqual(joint, expected_others, places=9,
                                   msg="Stewart joints solve a different angle "
                                       "than joint 0 — inconsistent head pose")

    def test_an_out_of_range_stored_target_is_bounded_too(self) -> None:
        # the original falls back to self.target_body_yaw when body_yaw is None
        b = self.cls()
        b.target_body_yaw = math.radians(170.0)
        b.update_target_head_joints_from_ik()
        self.assertAlmostEqual(math.degrees(b.ik_body_yaw),
                               igain.BODY_YAW_LIMIT_DEG, places=6)

    def test_bounds_the_set_target_path(self) -> None:
        b = self.cls()
        b.set_target_head_joint_positions([math.radians(-165.0)] + [0.0] * 6)
        self.assertAlmostEqual(math.degrees(b.target_head_joint_positions[0]),
                               -igain.BODY_YAW_LIMIT_DEG, places=6)

    def test_reachable_targets_pass_through_untouched(self) -> None:
        b = self.cls()
        for deg in (0.0, 45.0, -90.0, 119.0, -120.0):
            b.set_target_head_joint_positions([math.radians(deg)] + [0.0] * 6)
            self.assertAlmostEqual(
                math.degrees(b.target_head_joint_positions[0]), deg, places=6)

    def test_direct_joint_commands_keep_their_other_joints(self) -> None:
        """Correct for JOINT-SPACE commands only.

        Here the caller supplied the joint values directly, so there is no IK
        solution to invalidate and bounding joint 0 alone is right. (On the IK
        path the opposite holds, which is why that one clamps the input.)
        """
        b = self.cls()
        others = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        b.set_target_head_joint_positions([math.radians(200.0)] + others)
        for got, want in zip(b.target_head_joint_positions[1:], others):
            self.assertAlmostEqual(got, want, places=9)

    def test_an_unbounded_target_is_never_published(self) -> None:
        """No window in which the 50 Hz motor thread could read an unsafe value."""
        seen = []
        cls = _fake_backend_class()
        original = cls.set_target_head_joint_positions

        def recording(self, positions):
            seen.append(float(positions[0]))    # what the ORIGINAL setter receives
            return original(self, positions)

        cls.set_target_head_joint_positions = recording
        igain.revert()
        igain._ORIGINAL.clear()
        igain._apply_to_class(cls)
        cls().set_target_head_joint_positions([math.radians(200.0)] + [0.0] * 6)
        self.assertAlmostEqual(math.degrees(seen[0]), igain.BODY_YAW_LIMIT_DEG,
                               places=6,
                               msg="the setter received an unbounded value")

    def test_the_callers_array_is_not_mutated(self) -> None:
        b = self.cls()
        caller = [math.radians(200.0)] + [0.0] * 6
        b.set_target_head_joint_positions(caller)
        self.assertAlmostEqual(math.degrees(caller[0]), 200.0, places=6)

    def test_clamp_disappears_on_revert(self) -> None:
        igain.revert()
        b = self.cls()
        b.set_target_head_joint_positions([math.radians(165.0)] + [0.0] * 6)
        self.assertAlmostEqual(math.degrees(b.target_head_joint_positions[0]),
                               165.0, places=6)


class StatusRecordTest(unittest.TestCase):
    """The verifier runs out-of-process, so the hook must record what happened."""

    def setUp(self) -> None:
        igain.revert()
        igain._ORIGINAL.clear()
        if os.path.exists(igain.STATUS_PATH):
            os.unlink(igain.STATUS_PATH)

    def tearDown(self) -> None:
        igain.revert()
        igain._ORIGINAL.clear()

    def test_records_success(self) -> None:
        cls = _fake_backend_class()
        igain._apply_to_class(cls)
        cls()
        status = igain.read_status()
        self.assertIsNotNone(status)
        self.assertTrue(status["applied"])
        self.assertEqual(status["readback"], igain.POSITION_I_GAIN)

    def test_records_failure_so_verification_cannot_pass(self) -> None:
        class _Broken(_FakeController):
            def async_write_raw_bytes(self, *a, **k):
                raise RuntimeError("bus error")

        cls = _fake_backend_class(controller_cls=_Broken)
        igain._apply_to_class(cls)
        cls()                                   # must not raise
        status = igain.read_status()
        self.assertIsNotNone(status, "a failed write must still be recorded")
        self.assertFalse(status["applied"])
        self.assertIn("bus error", str(status["error"]))

    def test_a_record_from_a_live_process_is_trusted_regardless_of_age(self) -> None:
        """Freshness is process liveness, not wall-clock age.

        The record is written once when the daemon builds its backend and stays
        true for that daemon's whole life. An age limit would have failed
        verification after an hour of uptime — which matters now that this is
        the only confirmation channel with movement-diagnostics disabled.
        """
        cls = _fake_backend_class()
        igain._apply_to_class(cls)
        cls()
        status = igain.read_status()
        self.assertIsNotNone(status)
        # backdate it by a week: still this process, so still trustworthy
        status["written_at"] = time.time() - 7 * 24 * 3600
        with open(igain.STATUS_PATH, "w") as fh:
            json.dump(status, fh)
        self.assertIsNotNone(igain.read_status(),
                             "an old record from a LIVE daemon is still valid")

    def test_a_record_from_a_dead_process_is_rejected(self) -> None:
        cls = _fake_backend_class()
        igain._apply_to_class(cls)
        cls()
        status = igain.read_status()
        self.assertIsNotNone(status)
        status["pid"] = 2 ** 22          # a pid that cannot be running
        with open(igain.STATUS_PATH, "w") as fh:
            json.dump(status, fh)
        self.assertIsNone(igain.read_status(),
                          "a record from a daemon that is gone must not be believed")

    def test_a_foreign_version_is_rejected(self) -> None:
        cls = _fake_backend_class()
        igain._apply_to_class(cls)
        cls()
        status = igain.read_status()
        status["version"] = "some-older-version"
        with open(igain.STATUS_PATH, "w") as fh:
            json.dump(status, fh)
        self.assertIsNone(igain.read_status())


class HookLifecycleTest(unittest.TestCase):
    """apply/revert must be idempotent and must not break the constructor."""

    def setUp(self) -> None:
        self.calls = []
        self.cls = _fake_backend_class(calls=self.calls)
        igain.revert()
        igain._ORIGINAL.clear()

    def tearDown(self) -> None:
        igain._ORIGINAL.clear()

    def test_applies_the_gain_when_a_backend_is_constructed(self) -> None:
        igain._apply_to_class(self.cls)
        b = self.cls("first")
        self.assertEqual(igain.read_position_i_gain(b), igain.POSITION_I_GAIN)
        self.assertEqual(self.calls, ["first"], "original __init__ must still run")

    def test_a_failing_write_never_breaks_the_daemon(self) -> None:
        class _Broken(_FakeController):
            def async_write_raw_bytes(self, *a, **k):
                raise RuntimeError("bus error")

        cls = _fake_backend_class(controller_cls=_Broken)
        igain._apply_to_class(cls)
        cls()                       # must not raise

    def test_an_unidentifiable_motor_never_breaks_the_daemon(self) -> None:
        # name2id cannot resolve body_rotation: the hook must refuse to write,
        # and the daemon must still come up.
        cls = _fake_backend_class(name2id={})
        igain._apply_to_class(cls)
        b = cls()                   # must not raise
        self.assertEqual(b.c.writes, [], "must refuse to write")
        status = igain.read_status()
        self.assertIsNotNone(status)
        self.assertFalse(status["applied"], "a refusal must be recorded")

    def test_revert_restores_the_original_constructor(self) -> None:
        original = self.cls.__init__
        igain._apply_to_class(self.cls)
        self.assertIsNot(self.cls.__init__, original)
        igain.revert()
        self.assertIs(self.cls.__init__, original)

    def test_apply_is_idempotent(self) -> None:
        igain._apply_to_class(self.cls)
        patched = self.cls.__init__
        igain.apply()               # no-op: _ORIGINAL already populated
        self.assertIs(self.cls.__init__, patched)


class ManifestTest(unittest.TestCase):
    """The layer must be registered so the manager owns it."""

    def setUp(self) -> None:
        with open(os.path.join(ROOT, "layers.json")) as fh:
            self.manifest = json.load(fh)
        self.layer = next(entry for entry in self.manifest["layers"]
                          if entry["name"] == "body-motor-igain")

    def test_is_registered_with_a_verifier_the_manager_knows(self) -> None:
        self.assertEqual(self.layer["verifier"]["type"], "body_motor_igain")
        with open(os.path.join(MANAGER_ROOT, "layerctl.py")) as fh:
            self.assertIn('verifier_type == "body_motor_igain"', fh.read())

    def test_hashes_match_the_files_on_disk(self) -> None:
        import hashlib
        for name, want in self.layer["sha256"].items():
            path = os.path.join(ROOT, "layers", "body_motor_igain", name)
            with open(path, "rb") as fh:
                got = hashlib.sha256(fh.read()).hexdigest()
            self.assertEqual(got, want, f"{name} hash does not match the manifest")

    def test_applies_after_the_calibration_layer(self) -> None:
        # calibration rewrites joint targets; this only touches a motor register,
        # but a deterministic order keeps the manager's drop-in stable.
        cal = next(entry for entry in self.manifest["layers"]
                   if entry["name"] == "body-yaw-calibration")
        self.assertGreater(self.layer["order"], cal["order"])

    def test_required_files_are_listed_and_present(self) -> None:
        for name in self.layer["required_files"]:
            self.assertTrue(
                os.path.isfile(
                    os.path.join(ROOT, "layers", "body_motor_igain", name)
                ),
                name,
            )


if __name__ == "__main__":
    unittest.main()
