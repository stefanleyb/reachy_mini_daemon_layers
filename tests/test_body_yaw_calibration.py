from __future__ import annotations

import importlib.util
import asyncio
import math
from pathlib import Path
import sys
import types
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
MODULE_PATH = ROOT / "layers" / "body_yaw_calibration" / "body_yaw_calibration.py"


def load_module():
    spec = importlib.util.spec_from_file_location("test_body_yaw_calibration", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeRobotBackend:
    def __init__(self, torque_enabled: bool = True) -> None:
        self._torque_enabled = torque_enabled
        self.raise_during_enable = False
        self.raise_during_goto = False
        self._stop_move_requested = False
        self.target_head_joint_positions = None
        self.target_body_yaw = None
        self.present_head_joint_positions = np.array([math.radians(37), 3.0, 4.0])
        self.goto_raw_targets = []
        self.goto_progresses = (0.0, 0.5, 1.0)
        self.pre_goto_ik_logical = None

    def update_target_head_joints_from_ik(self, logical: float) -> None:
        self.target_head_joint_positions = np.array([logical, 1.0, 2.0])

    def set_target_head_joint_positions(self, positions):
        self.target_head_joint_positions = positions

    def enable_motors(self) -> None:
        self.set_target_head_joint_positions(self.present_head_joint_positions.copy())
        self.target_body_yaw = float(self.present_head_joint_positions[0])
        if self.raise_during_enable:
            raise RuntimeError("fake enable failure")
        self._torque_enabled = True

    def get_present_body_yaw(self) -> float:
        return float(self.present_head_joint_positions[0])

    async def goto_target(
        self,
        head=None,
        antennas=None,
        duration=0.5,
        method=None,
        body_yaw=0.0,
    ) -> None:
        if self.raise_during_goto:
            raise RuntimeError("fake goto failure")
        start = self.get_present_body_yaw()
        target = start if body_yaw is None else float(body_yaw)
        self.goto_raw_targets = []
        if self.pre_goto_ik_logical is not None:
            self.update_target_head_joints_from_ik(self.pre_goto_ik_logical)
            self.goto_raw_targets.append(float(self.target_head_joint_positions[0]))
        for progress in self.goto_progresses:
            logical = start + (target - start) * progress
            self.target_body_yaw = logical
            self.update_target_head_joints_from_ik(logical)
            self.goto_raw_targets.append(float(self.target_head_joint_positions[0]))


class FakeSimulationBackend:
    def update_target_head_joints_from_ik(self) -> None:
        pass

    def set_target_head_joint_positions(self, positions) -> None:
        pass

    def enable_motors(self) -> None:
        pass

    async def goto_target(self) -> None:
        pass


class BodyYawCalibrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cal = load_module()
        self.cal._apply_to_class(FakeRobotBackend)

    def tearDown(self) -> None:
        self.cal.revert()

    def test_one_continuous_formula_including_zero(self) -> None:
        self.assertAlmostEqual(self.cal.calibrated_body_yaw_deg(-100), -110.1363669, places=6)
        self.assertAlmostEqual(self.cal.calibrated_body_yaw_deg(0), 0.9617463, places=6)
        self.assertAlmostEqual(self.cal.calibrated_body_yaw_deg(100), 108.1277955, places=6)
        self.assertLess(
            abs(self.cal.calibrated_body_yaw_deg(-0.001) - self.cal.calibrated_body_yaw_deg(0.001)),
            0.01,
        )

    def test_calibration_inverse_round_trips(self) -> None:
        for logical in (-130.0, -100.0, -20.0, 0.0, 20.0, 100.0, 145.0):
            raw = self.cal.calibrated_body_yaw_deg(logical)
            self.assertAlmostEqual(
                self.cal.logical_body_yaw_for_raw_deg(raw), logical, places=9
            )

    def test_ik_keeps_other_joints_and_calibrates_joint_zero(self) -> None:
        backend = FakeRobotBackend()
        logical = math.radians(80)
        backend.update_target_head_joints_from_ik(logical)
        self.assertAlmostEqual(
            math.degrees(backend.target_head_joint_positions[0]),
            self.cal.calibrated_body_yaw_deg(80),
        )
        np.testing.assert_array_equal(backend.target_head_joint_positions[1:], [1.0, 2.0])
        self.assertEqual(backend.body_yaw_calibration_logical_rad, logical)

    def test_direct_joint_target_is_calibrated_independent_of_torque_state(self) -> None:
        for torque_enabled in (False, True):
            with self.subTest(torque_enabled=torque_enabled):
                backend = FakeRobotBackend(torque_enabled=torque_enabled)
                source = np.array([math.radians(-80), 3.0, 4.0])
                backend.set_target_head_joint_positions(source)
                self.assertAlmostEqual(
                    math.degrees(backend.target_head_joint_positions[0]),
                    self.cal.calibrated_body_yaw_deg(-80),
                )
                self.assertAlmostEqual(math.degrees(source[0]), -80.0)

    def test_enable_motors_pin_is_never_calibrated(self) -> None:
        for torque_enabled in (False, True):
            with self.subTest(torque_enabled=torque_enabled):
                backend = FakeRobotBackend(torque_enabled=torque_enabled)
                backend.enable_motors()
                np.testing.assert_array_equal(
                    backend.target_head_joint_positions,
                    backend.present_head_joint_positions,
                )
                self.assertFalse(hasattr(backend, "body_yaw_calibration_raw_rad"))
                self.assertFalse(hasattr(backend, self.cal._PINNING_ATTRIBUTE))

                pinned_raw = float(backend.target_head_joint_positions[0])
                backend.update_target_head_joints_from_ik(backend.target_body_yaw)
                self.assertAlmostEqual(
                    float(backend.target_head_joint_positions[0]), pinned_raw
                )

    def test_repeated_ik_updates_do_not_compound(self) -> None:
        backend = FakeRobotBackend()
        logical = math.radians(80)
        backend.update_target_head_joints_from_ik(logical)
        first = backend.target_head_joint_positions.copy()
        backend.update_target_head_joints_from_ik(logical)
        np.testing.assert_array_equal(backend.target_head_joint_positions, first)

    def test_goto_starts_at_existing_raw_target_and_ends_calibrated(self) -> None:
        backend = FakeRobotBackend()
        existing_raw = math.radians(41.0)
        backend.target_head_joint_positions = np.array([existing_raw, 1.0, 2.0])
        target = math.radians(-20.0)

        asyncio.run(backend.goto_target(body_yaw=target))

        start = backend.get_present_body_yaw()
        middle = (start + target) / 2.0
        expected_middle = existing_raw + 0.5 * (
            self.cal.calibrated_body_yaw_rad(target) - existing_raw
        )
        self.assertAlmostEqual(backend.goto_raw_targets[0], existing_raw)
        self.assertAlmostEqual(backend.goto_raw_targets[1], expected_middle)
        self.assertAlmostEqual(
            backend.goto_raw_targets[2],
            self.cal.calibrated_body_yaw_rad(target),
        )
        self.assertFalse(hasattr(backend, self.cal._TRAJECTORY_ATTRIBUTE))

    def test_run_10_inward_takeover_has_no_outward_target_step(self) -> None:
        backend = FakeRobotBackend()
        backend.present_head_joint_positions[0] = math.radians(96.50)
        retained_b_raw = self.cal.calibrated_body_yaw_rad(math.radians(89.94))
        backend.target_head_joint_positions = np.array([retained_b_raw, 1.0, 2.0])

        asyncio.run(backend.goto_target(body_yaw=math.radians(52.14)))

        first, middle, final = backend.goto_raw_targets
        self.assertAlmostEqual(first, retained_b_raw)
        self.assertLess(middle, first)
        self.assertLess(final, middle)

    def test_tracking_ik_before_first_waypoint_keeps_existing_raw_target(self) -> None:
        for start, prior, target in (
            (96.50, 89.94, 52.14),
            (96.50, 89.94, 130.0),
            (-96.50, -89.94, -52.14),
            (-96.50, -89.94, -130.0),
        ):
            with self.subTest(start=start, target=target):
                backend = FakeRobotBackend()
                backend.present_head_joint_positions[0] = math.radians(start)
                backend.target_body_yaw = math.radians(prior)
                retained_raw = self.cal.calibrated_body_yaw_rad(
                    backend.target_body_yaw
                )
                backend.target_head_joint_positions = np.array(
                    [retained_raw, 1.0, 2.0]
                )
                backend.pre_goto_ik_logical = backend.target_body_yaw

                asyncio.run(backend.goto_target(body_yaw=math.radians(target)))

                self.assertAlmostEqual(backend.goto_raw_targets[0], retained_raw)
                self.assertAlmostEqual(backend.goto_raw_targets[1], retained_raw)
                sign = 1.0 if target > start else -1.0
                self.assertTrue(all(
                    (later - earlier) * sign >= 0.0
                    for earlier, later in zip(
                        backend.goto_raw_targets[1:], backend.goto_raw_targets[2:]
                    )
                ))

    def test_completed_direction_fallback_has_continuous_exit(self) -> None:
        backend = FakeRobotBackend()
        backend.present_head_joint_positions[0] = math.radians(130.0)
        backend.target_body_yaw = math.radians(130.0)
        backend.target_head_joint_positions = np.array(
            [math.radians(130.029), 1.0, 2.0]
        )

        asyncio.run(backend.goto_target(body_yaw=math.radians(123.0)))

        raw_at_exit = backend.goto_raw_targets[-1]
        self.assertLess(raw_at_exit, backend.goto_raw_targets[0])
        backend.update_target_head_joints_from_ik(backend.target_body_yaw)
        self.assertAlmostEqual(
            float(backend.target_head_joint_positions[0]), raw_at_exit
        )

    def test_interrupted_goto_is_continuous_after_context_is_cleared(self) -> None:
        for start_deg, retained_deg, target_deg in (
            (96.50, 89.94, 52.14),
            (-96.50, -89.94, -52.14),
        ):
            for progress in (0.1, 0.4, 0.8):
                with self.subTest(start=start_deg, progress=progress):
                    backend = FakeRobotBackend()
                    backend.present_head_joint_positions[0] = math.radians(start_deg)
                    retained_raw = self.cal.calibrated_body_yaw_rad(
                        math.radians(retained_deg)
                    )
                    backend.target_head_joint_positions = np.array(
                        [retained_raw, 1.0, 2.0]
                    )
                    backend.goto_progresses = (0.0, progress)
                    backend._stop_move_requested = True

                    asyncio.run(
                        backend.goto_target(body_yaw=math.radians(target_deg))
                    )

                    raw_at_stop = backend.goto_raw_targets[-1]
                    self.assertFalse(
                        hasattr(backend, self.cal._TRAJECTORY_ATTRIBUTE)
                    )
                    backend.update_target_head_joints_from_ik(
                        backend.target_body_yaw
                    )
                    self.assertAlmostEqual(
                        float(backend.target_head_joint_positions[0]), raw_at_stop
                    )

    def test_raw_goto_path_is_monotonic_for_small_extreme_moves(self) -> None:
        for start_deg, target_deg in (
            (120.0, 111.0),
            (-120.0, -111.0),
            (111.0, 120.0),
            (-111.0, -120.0),
        ):
            with self.subTest(start_deg=start_deg, target_deg=target_deg):
                backend = FakeRobotBackend()
                backend.present_head_joint_positions[0] = math.radians(start_deg)
                backend.target_head_joint_positions = np.array(
                    [math.radians(start_deg), 1.0, 2.0]
                )
                backend.goto_progresses = tuple(i / 20.0 for i in range(21))
                asyncio.run(
                    backend.goto_target(body_yaw=math.radians(target_deg))
                )
                deltas = [
                    b - a
                    for a, b in zip(
                        backend.goto_raw_targets, backend.goto_raw_targets[1:]
                    )
                ]
                expected_sign = 1.0 if target_deg > start_deg else -1.0
                self.assertTrue(all(delta * expected_sign >= 0.0 for delta in deltas))

    def test_goto_failure_clears_trajectory_scope(self) -> None:
        backend = FakeRobotBackend()
        backend.target_head_joint_positions = np.array([math.radians(41), 1.0, 2.0])
        raw_before = float(backend.target_head_joint_positions[0])
        backend.raise_during_goto = True
        with self.assertRaisesRegex(RuntimeError, "fake goto failure"):
            asyncio.run(backend.goto_target(body_yaw=math.radians(-20)))
        self.assertFalse(hasattr(backend, self.cal._TRAJECTORY_ATTRIBUTE))
        backend.update_target_head_joints_from_ik(backend.target_body_yaw)
        self.assertAlmostEqual(
            float(backend.target_head_joint_positions[0]), raw_before
        )

    def test_enable_failure_clears_pin_scope_and_next_command_is_calibrated(self) -> None:
        backend = FakeRobotBackend(torque_enabled=True)
        backend.raise_during_enable = True
        with self.assertRaisesRegex(RuntimeError, "fake enable failure"):
            backend.enable_motors()
        self.assertFalse(hasattr(backend, self.cal._PINNING_ATTRIBUTE))

        pinned_raw = float(backend.target_head_joint_positions[0])
        backend.update_target_head_joints_from_ik(backend.target_body_yaw)
        self.assertAlmostEqual(
            float(backend.target_head_joint_positions[0]), pinned_raw
        )

        logical = np.array([math.radians(80), 3.0, 4.0])
        backend.set_target_head_joint_positions(logical)
        self.assertAlmostEqual(
            math.degrees(backend.target_head_joint_positions[0]),
            self.cal.calibrated_body_yaw_deg(80),
        )

    def test_raw_command_is_clamped_to_existing_joint_limit(self) -> None:
        self.assertEqual(self.cal.calibrated_body_yaw_deg(1000), 160.0)
        self.assertEqual(self.cal.calibrated_body_yaw_deg(-1000), -160.0)

    def test_apply_is_idempotent_and_revert_restores_methods(self) -> None:
        patched = FakeRobotBackend.update_target_head_joints_from_ik
        self.cal._apply_to_class(FakeRobotBackend)
        self.assertIs(FakeRobotBackend.update_target_head_joints_from_ik, patched)
        self.cal.revert()
        self.assertFalse(
            hasattr(FakeRobotBackend.update_target_head_joints_from_ik, "__body_yaw_calibration__")
        )


class ApplyScopeTest(unittest.TestCase):
    def test_apply_patches_physical_class_only(self) -> None:
        cal = load_module()
        full_name = "reachy_mini.daemon.backend.robot.backend"
        module = types.ModuleType(full_name)
        module.RobotBackend = FakeRobotBackend
        previous = sys.modules.get(full_name)
        sys.modules[full_name] = module
        try:
            cal.apply()
            self.assertEqual(
                getattr(
                    FakeRobotBackend.update_target_head_joints_from_ik,
                    "__body_yaw_calibration__",
                ),
                cal.VERSION,
            )
            self.assertFalse(
                hasattr(
                    FakeSimulationBackend.update_target_head_joints_from_ik,
                    "__body_yaw_calibration__",
                )
            )
        finally:
            cal.revert()
            if previous is None:
                del sys.modules[full_name]
            else:
                sys.modules[full_name] = previous


if __name__ == "__main__":
    unittest.main()
