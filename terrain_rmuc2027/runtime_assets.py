"""Install the committed, portable map on first use; no CAD tools or download."""
from pathlib import Path, PurePosixPath
import fcntl
import hashlib
import json
import os
import shutil
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent
ARCHIVE = ROOT / "assets/runtime.zip"
SHA256 = "051b1b4e31ebed427a12da33847d4db3eab58af9f123a4e6746a2d824b98ec39"
PREFIX = PurePosixPath("terrain_rmuc2027/pack")


def ensure_pack():
    """Use the existing workspace pack or atomically install the bundled one.

    RL_LAB_RMUC2027_STORAGE selects the cache parent on another disk. With no
    override, an existing pack is preserved; a fresh clone uses the user cache.
    Only pack members are extracted, never the archive's old scripts/docs.
    """
    storage = os.environ.get("RL_LAB_RMUC2027_STORAGE")
    existing = ROOT / "pack"
    if not storage and (existing / "field_collision.xml").is_file():
        return existing.resolve()
    cache = Path(storage).expanduser() if storage else Path(
        os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "rl-lab/rmuc2027"
    cache = cache.resolve() / SHA256[:16]
    cache.mkdir(parents=True, exist_ok=True)
    return _install(ARCHIVE, SHA256, cache,
                    ("field.xml", "field_collision.xml", "field_perimeter.xml", "manifest.json"))


def ensure_operator_visual():
    """The baked display LOD needs no geometry-conversion dependencies."""
    receipt = json.loads((ROOT / "operator_visual_receipt.json").read_text())
    digest = receipt["sha256"]
    cache = ensure_pack().parent / "operator-visual" / digest[:16]
    cache.mkdir(parents=True, exist_ok=True)
    return _install(ROOT / "assets/operator_visual.zip", digest, cache,
                    ("field.xml", "field_perimeter.xml", "report.json"))


def _install(archive_path, expected_digest, cache, required):
    pack = cache / "pack"
    with (cache / "install.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if (pack / ".archive-sha256").is_file() and (
                pack / ".archive-sha256").read_text().strip() == expected_digest:
            return pack
        if pack.exists():
            raise RuntimeError(f"Unverified map directory exists: {pack}; preserve it before retrying")
        digest = hashlib.sha256()
        with archive_path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        if digest.hexdigest() != expected_digest:
            raise RuntimeError(f"RMUC2027 archive checksum mismatch: {archive_path}")
        print("RMUC 2027: installing bundled map (first load)", flush=True)
        temporary = Path(tempfile.mkdtemp(prefix="install-", dir=cache))
        try:
            with zipfile.ZipFile(archive_path) as archive:
                for info in archive.infolist():
                    name = PurePosixPath(info.filename)
                    if ".." in name.parts or name.is_absolute() or "\\" in info.filename:
                        raise ValueError(f"Unsafe archive member: {info.filename}")
                    if not name.is_relative_to(PREFIX):
                        continue
                    relative = name.relative_to(PREFIX)
                    if info.is_dir():
                        continue
                    if (info.external_attr >> 16) & 0o170000 == 0o120000:
                        raise ValueError(f"Archive contains a symlink: {info.filename}")
                    target = temporary / relative
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with archive.open(info) as source, target.open("wb") as dest:
                        shutil.copyfileobj(source, dest, 128 * 1024)
            for name in required:
                if not (temporary / name).is_file():
                    raise ValueError(f"Map archive is missing {name}")
            (temporary / ".archive-sha256").write_text(expected_digest + "\n")
            temporary.rename(pack)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
    return pack


if __name__ == "__main__":
    print(ensure_pack())
