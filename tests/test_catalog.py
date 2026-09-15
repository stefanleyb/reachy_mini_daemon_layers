"""The production catalog itself: structure, hashes and manager compatibility.

The layer-manager repository owns ``layer_plan``/``layerctl`` and tests them
against a small synthetic fixture. Nothing there loads *this* repository's real
``layers.json``, so a catalog edit that duplicates a hook, reuses an install
path, records a stale hash or names a verifier the manager does not implement
would otherwise reach the robot before any test noticed.

The manager is located through ``REACHY_LAYER_MANAGER_ROOT`` (default: the
sibling checkout), matching ``test_body_motor_igain``. The upstream-backed
``face-frame-sync`` layer ships inside the daemon source tree rather than this
repository, so its files are resolved through ``REACHY_MINI_UPSTREAM_SRC``
(default: the sibling upstream checkout) and skipped when it is absent.
"""

from __future__ import annotations

import hashlib
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MANAGER_ROOT = Path(
    os.environ.get(
        "REACHY_LAYER_MANAGER_ROOT", ROOT.parent / "reachy_mini_layer_manager"
    )
)
UPSTREAM_SRC = Path(
    os.environ.get(
        "REACHY_MINI_UPSTREAM_SRC", ROOT.parent.parent / "upstream" / "reachy_mini" / "src"
    )
)
CATALOG = ROOT / "layers.json"

sys.path.insert(0, str(MANAGER_ROOT))

from layer_plan import Manifest  # noqa: E402

# Layers whose files live in this repository, keyed by catalog name.
LOCAL_SOURCES = {
    "movement-diagnostics": ROOT / "layers" / "movement_diagnostics",
    "body-yaw-calibration": ROOT / "layers" / "body_yaw_calibration",
    "body-motor-igain": ROOT / "layers" / "body_motor_igain",
    "face-loss-return": ROOT / "layers" / "face_loss_return",
}
# Layers whose files live in the upstream daemon checkout.
UPSTREAM_LAYERS = {"face-frame-sync"}


def source_dir(name: str) -> Path | None:
    """Where this layer's recorded files are kept, or None when unavailable."""
    if name in LOCAL_SOURCES:
        return LOCAL_SOURCES[name]
    if name in UPSTREAM_LAYERS:
        return UPSTREAM_SRC if UPSTREAM_SRC.is_dir() else None
    return None


class CatalogLoadsTest(unittest.TestCase):
    """``Manifest.load`` is the manager's own gate on catalog validity."""

    def test_the_real_catalog_loads_through_the_manager(self) -> None:
        # Enforces unique names, unique canonical paths and unique hook modules.
        manifest = Manifest.load(CATALOG)
        self.assertTrue(manifest.layers, "the catalog declares no layers")
        self.assertTrue(manifest.bootstrap_path.startswith("/"),
                        "bootstrap_path must be absolute on the robot")

    def test_every_layer_has_a_known_source_location(self) -> None:
        # Guards the test itself: a new layer must be mapped here deliberately.
        manifest = Manifest.load(CATALOG)
        for layer in manifest.layers:
            self.assertTrue(
                layer.name in LOCAL_SOURCES or layer.name in UPSTREAM_LAYERS,
                f"{layer.name} is not mapped to a source location in this test",
            )


class CatalogContentTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = Manifest.load(CATALOG)

    def test_install_order_is_unique_and_ascending(self) -> None:
        orders = [layer.order for layer in self.manifest.layers]
        self.assertEqual(len(orders), len(set(orders)), "layer orders must be unique")
        self.assertEqual(orders, sorted(orders),
                         "layers must be listed in install order")

    def test_every_verifier_type_is_implemented_by_the_manager(self) -> None:
        source = (MANAGER_ROOT / "layerctl.py").read_text()
        for layer in self.manifest.layers:
            verifier_type = layer.verifier.get("type")
            self.assertIsNotNone(verifier_type, f"{layer.name} declares no verifier")
            self.assertIn(
                f'verifier_type == "{verifier_type}"', source,
                f"{layer.name} names verifier {verifier_type!r}, "
                "which layerctl.py does not implement",
            )

    def test_recorded_hashes_match_the_files_on_disk(self) -> None:
        checked = 0
        for layer in self.manifest.layers:
            directory = source_dir(layer.name)
            if directory is None:
                self.skipTest(f"source tree for {layer.name} is not available")
            self.assertTrue(layer.sha256, f"{layer.name} records no hashes")
            for name, want in layer.sha256.items():
                path = directory / name
                self.assertTrue(path.is_file(), f"{layer.name}: missing {name}")
                got = hashlib.sha256(path.read_bytes()).hexdigest()
                self.assertEqual(
                    got, want, f"{layer.name}: {name} does not match the catalog",
                )
                checked += 1
        self.assertGreater(checked, 0, "no layer files were hashed")

    def test_required_files_are_present_and_hashed(self) -> None:
        for layer in self.manifest.layers:
            directory = source_dir(layer.name)
            if directory is None:
                continue
            for name in layer.required_files:
                self.assertTrue((directory / name).is_file(),
                                f"{layer.name}: required file {name} is missing")
                self.assertIn(name, layer.sha256,
                              f"{layer.name}: required file {name} has no hash")


if __name__ == "__main__":
    unittest.main()
