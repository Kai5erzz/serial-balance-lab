"""Portable map installation: integrity, containment and interrupted-install cleanup."""
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from terrain_rmuc2027 import runtime_assets as assets


class RuntimeAssetsTests(unittest.TestCase):
    def archive(self, directory, extra=None):
        archive = directory / "runtime.zip"
        with zipfile.ZipFile(archive, "w") as output:
            for name in ("field.xml", "field_collision.xml", "field_perimeter.xml", "manifest.json"):
                output.writestr("terrain_rmuc2027/pack/" + name, name)
            output.writestr("terrain_rmuc2027/README.md", "old docs must not be installed")
            if extra:
                output.writestr(*extra)
        return archive, hashlib.sha256(archive.read_bytes()).hexdigest()

    def test_install_is_idempotent_and_extracts_only_runtime(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            archive, digest = self.archive(directory)
            with patch.object(assets, "ARCHIVE", archive), patch.object(assets, "SHA256", digest), \
                 patch.dict(os.environ, RL_LAB_RMUC2027_STORAGE=str(directory / "cache")):
                pack = assets.ensure_pack()
                self.assertEqual((pack / "field.xml").read_text(), "field.xml")
                self.assertFalse((pack / "README.md").exists())
                stamp = (pack / "field.xml").stat().st_mtime_ns
                self.assertEqual(assets.ensure_pack(), pack)
                self.assertEqual((pack / "field.xml").stat().st_mtime_ns, stamp)

    def test_corrupt_archive_does_not_install(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            archive, digest = self.archive(directory)
            archive.write_bytes(archive.read_bytes() + b"corruption")
            with patch.object(assets, "ARCHIVE", archive), patch.object(assets, "SHA256", digest), \
                 patch.dict(os.environ, RL_LAB_RMUC2027_STORAGE=str(directory / "cache")):
                with self.assertRaisesRegex(RuntimeError, "checksum mismatch"):
                    assets.ensure_pack()
                self.assertFalse(list((directory / "cache").rglob("pack")))

    def test_unsafe_member_aborts_without_partial_pack(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            archive, digest = self.archive(directory, ("terrain_rmuc2027/pack/../../escape", "bad"))
            with patch.object(assets, "ARCHIVE", archive), patch.object(assets, "SHA256", digest), \
                 patch.dict(os.environ, RL_LAB_RMUC2027_STORAGE=str(directory / "cache")):
                with self.assertRaisesRegex(ValueError, "Unsafe archive member"):
                    assets.ensure_pack()
                self.assertFalse(list((directory / "cache").rglob("install-*")))
                self.assertFalse(list(directory.rglob("escape")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
