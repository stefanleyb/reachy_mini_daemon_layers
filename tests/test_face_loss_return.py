from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation as R


ROOT = Path(__file__).resolve().parent.parent
MODULE_PATH = ROOT / "layers" / "face_loss_return" / "face_loss_return.py"
UPSTREAM_SRC = Path(
    os.environ.get(
        "REACHY_MINI_UPSTREAM_SRC",
        ROOT.parents[1] / "upstream" / "reachy_mini" / "src",
    )
)
sys.path.insert(0, str(UPSTREAM_SRC))


def load_module():
    spec = importlib.util.spec_from_file_location("test_face_loss_return", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(yaw_deg: float, x: float = 0.0) -> np.ndarray:
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = R.from_euler("z", yaw_deg, degrees=True).as_matrix()
    value[0, 3] = x
    return value


def yaw_deg(value: np.ndarray) -> float:
    return float(R.from_matrix(value[:3, :3]).as_euler("xyz", degrees=True)[2])


def rotation_distance_deg(start: np.ndarray, end: np.ndarray) -> float:
    relative = R.from_matrix(start[:3, :3]).inv() * R.from_matrix(end[:3, :3])
    return math.degrees(float(np.linalg.norm(relative.as_rotvec())))


class FakeRobotBackend:
    INIT_HEAD_POSE = np.eye(4, dtype=np.float64)

    def __init__(self) -> None:
        self._tracking_lock = threading.Lock()
        self._tracking_enabled = True
        self._tracking_requested_weight = 1.0
        self._tracking_lost_timeout = 2.0
        self._last_face_seen = 99.0
        self._tracking_aim = pose(60.0)
        self._tracking_target_pose = pose(80.0, x=0.2)
        self.current_head_pose = self._tracking_aim.copy()
        self.ik_required = False
        self.stock_calls = 0
        self.transition_to_loss_during_step = False

    def get_current_head_pose(self) -> np.ndarray:
        return self.current_head_pose.copy()

    def step_head_tracking(self) -> None:
        self.stock_calls += 1
        if self.transition_to_loss_during_step:
            self._last_face_seen = 0.0
            self._tracking_target_pose = self.INIT_HEAD_POSE.copy()
        self._tracking_aim = self._stock_interpolate(
            self._tracking_aim, self._tracking_target_pose, 0.15
        )
        self.ik_required = True

    @staticmethod
    def _stock_interpolate(start: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
        from reachy_mini.utils.interpolation import linear_pose_interpolation

        return linear_pose_interpolation(start, target, alpha)


class FakeSimulationBackend:
    def step_head_tracking(self) -> None:
        pass


class FakeBaseBackend:
    def step_head_tracking(self) -> None:
        pass


class FakeInheritedRobotBackend(FakeBaseBackend):
    pass


class FaceLossReturnTest(unittest.TestCase):
    def setUp(self) -> None:
        self.layer = load_module()
        self.layer._apply_to_class(FakeRobotBackend)

    def tearDown(self) -> None:
        self.layer.revert()

    def _prime(self, backend: FakeRobotBackend) -> None:
        with mock.patch.object(self.layer.time, "monotonic", return_value=100.0):
            backend.step_head_tracking()

    def test_normal_tracking_is_delegated_unchanged(self) -> None:
        backend = FakeRobotBackend()
        expected = backend._stock_interpolate(
            backend._tracking_aim, backend._tracking_target_pose, 0.15
        )
        self._prime(backend)
        np.testing.assert_allclose(backend._tracking_aim, expected)
        self.assertFalse(backend.face_loss_return_active)
        self.assertEqual(backend.stock_calls, 1)

    def test_sustained_loss_is_capped_at_forty_degrees_per_second(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = pose(60.0)
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 97.0

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.05):
            backend.step_head_tracking()

        self.assertAlmostEqual(yaw_deg(backend._tracking_aim), 58.0, places=6)
        self.assertTrue(backend.face_loss_return_active)
        self.assertEqual(backend.face_loss_return_max_angular_speed_deg_s, 40.0)

    def test_first_loss_tick_has_no_jump_without_elapsed_history(self) -> None:
        backend = FakeRobotBackend()
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 0.0

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.0):
            backend.step_head_tracking()

        self.assertAlmostEqual(yaw_deg(backend._tracking_aim), 60.0, places=6)

    def test_timeout_crossed_inside_stock_step_is_capped_on_that_tick(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = pose(60.0)
        backend.transition_to_loss_during_step = True

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.02):
            backend.step_head_tracking()

        self.assertAlmostEqual(yaw_deg(backend._tracking_aim), 59.2, places=6)
        self.assertTrue(backend.face_loss_return_active)

    def test_transient_loss_before_timeout_keeps_stock_behavior(self) -> None:
        backend = FakeRobotBackend()
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 98.5
        expected = backend._stock_interpolate(
            backend._tracking_aim, backend._tracking_target_pose, 0.15
        )

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.0):
            backend.step_head_tracking()

        np.testing.assert_allclose(backend._tracking_aim, expected)
        self.assertFalse(backend.face_loss_return_active)

    def test_long_scheduler_gap_is_limited_to_four_degree_setpoint_step(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = pose(60.0)
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 0.0

        with mock.patch.object(self.layer.time, "monotonic", return_value=101.0):
            backend.step_head_tracking()

        self.assertAlmostEqual(yaw_deg(backend._tracking_aim), 56.0, places=6)

    def test_combined_rotation_uses_the_same_geodesic_rate_limit(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = np.eye(4, dtype=np.float64)
        backend._tracking_aim[:3, :3] = R.from_euler(
            "xyz", [25.0, -30.0, 45.0], degrees=True
        ).as_matrix()
        start = backend._tracking_aim.copy()
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 0.0

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.05):
            backend.step_head_tracking()

        self.assertAlmostEqual(
            rotation_distance_deg(start, backend._tracking_aim), 2.0, places=6
        )

    def test_stock_translation_behavior_is_preserved_during_loss(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = pose(60.0, x=0.2)
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 0.0
        expected = backend._stock_interpolate(
            backend._tracking_aim, backend._tracking_target_pose, 0.15
        )

        with mock.patch.object(self.layer.time, "monotonic", return_value=100.02):
            backend.step_head_tracking()

        self.assertAlmostEqual(backend._tracking_aim[0, 3], expected[0, 3])

    def test_reacquisition_immediately_returns_to_stock_tracking(self) -> None:
        backend = FakeRobotBackend()
        self._prime(backend)
        backend._tracking_aim = pose(60.0)
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        backend._last_face_seen = 0.0
        with mock.patch.object(self.layer.time, "monotonic", return_value=100.02):
            backend.step_head_tracking()

        start = backend._tracking_aim.copy()
        backend._tracking_target_pose = pose(80.0)
        backend._last_face_seen = 100.03
        expected = backend._stock_interpolate(start, backend._tracking_target_pose, 0.15)
        with mock.patch.object(self.layer.time, "monotonic", return_value=100.04):
            backend.step_head_tracking()

        np.testing.assert_allclose(backend._tracking_aim, expected)
        self.assertFalse(backend.face_loss_return_active)

    def test_world_neutral_target_and_timeout_are_not_changed(self) -> None:
        backend = FakeRobotBackend()
        timeout = backend._tracking_lost_timeout
        backend._tracking_target_pose = backend.INIT_HEAD_POSE.copy()
        self._prime(backend)
        np.testing.assert_array_equal(
            backend._tracking_target_pose, backend.INIT_HEAD_POSE
        )
        self.assertEqual(backend._tracking_lost_timeout, timeout)

    def test_apply_is_idempotent_and_revert_restores_original_method(self) -> None:
        patched = FakeRobotBackend.step_head_tracking
        original = self.layer._ORIGINAL
        self.layer._apply_to_class(FakeRobotBackend)
        self.assertIs(FakeRobotBackend.step_head_tracking, patched)
        self.layer.revert()
        self.assertIs(FakeRobotBackend.step_head_tracking, original)
        self.assertFalse(
            hasattr(FakeRobotBackend.step_head_tracking, "__face_loss_return__")
        )


class ApplyScopeTest(unittest.TestCase):
    def test_apply_patches_physical_class_only(self) -> None:
        layer = load_module()
        full_name = "reachy_mini.daemon.backend.robot.backend"
        module = types.ModuleType(full_name)
        module.RobotBackend = FakeRobotBackend
        previous = sys.modules.get(full_name)
        sys.modules[full_name] = module
        try:
            layer.apply()
            self.assertEqual(
                getattr(FakeRobotBackend.step_head_tracking, "__face_loss_return__"),
                layer.VERSION,
            )
            self.assertFalse(
                hasattr(FakeSimulationBackend.step_head_tracking, "__face_loss_return__")
            )
        finally:
            layer.revert()
            if previous is None:
                del sys.modules[full_name]
            else:
                sys.modules[full_name] = previous

    def test_revert_removes_production_style_inherited_override(self) -> None:
        layer = load_module()
        original = FakeBaseBackend.step_head_tracking

        layer._apply_to_class(FakeInheritedRobotBackend)
        self.assertIn("step_head_tracking", FakeInheritedRobotBackend.__dict__)
        self.assertEqual(
            getattr(
                FakeInheritedRobotBackend.step_head_tracking,
                "__face_loss_return__",
            ),
            layer.VERSION,
        )

        layer.revert()
        self.assertNotIn("step_head_tracking", FakeInheritedRobotBackend.__dict__)
        self.assertIs(FakeInheritedRobotBackend.step_head_tracking, original)


if __name__ == "__main__":
    unittest.main()
