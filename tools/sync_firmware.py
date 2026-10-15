"""Import/check a byte-exact code_914 source snapshot; never modify the vehicle repo."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT.parent / "code_914" / "HNU_Embedded_Framework"
DESTINATION = ROOT / "firmware" / "vehicle"
MANIFEST = ROOT / "firmware" / "manifest.json"
SCHEDULER_FILES = ("Core/Src/main.c", "Core/Src/tim.c", "Core/Inc/FreeRTOSConfig.h",
                   "Core/Inc/stm32h7xx_hal_conf.h", "CtrBoard-H7_ALL.ioc")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(root: Path) -> dict[str, str]:
    files = {str(p.relative_to(root)): digest(p) for p in sorted((root / "src").rglob("*"))
             if p.is_file() and p.suffix in {".c", ".h"}}
    files.update({name: digest(root / name) for name in SCHEDULER_FILES if (root / name).is_file()})
    return files


def sync(source: Path, *, check: bool = False) -> dict:
    actual = inventory(source)
    if not actual or "src/rm_config.h" not in actual:
        raise ValueError(f"not a firmware source tree: {source}")
    mirrored = inventory(DESTINATION)
    if check:
        if actual != mirrored:
            changed = sorted(k for k in actual.keys() | mirrored.keys() if actual.get(k) != mirrored.get(k))
            raise ValueError("firmware differs: " + ", ".join(changed))
        manifest = json.loads(MANIFEST.read_text())
        if manifest["files"] != mirrored:
            raise ValueError("snapshot changed without updating its manifest")
        return manifest
    if MANIFEST.exists():
        previous = json.loads(MANIFEST.read_text())["files"]
        if previous != mirrored:
            raise ValueError("local vehicle snapshot has edits; preserve/export those before importing")
    for name in actual:
        dst = DESTINATION / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes((source / name).read_bytes())
    for name in mirrored.keys() - actual.keys():
        (DESTINATION / name).unlink()
    fingerprint = hashlib.sha256(json.dumps(actual, sort_keys=True).encode()).hexdigest()
    manifest = {"schema_version": 1, "source": str(source.resolve()),
                "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip(),
                "working_tree_status": subprocess.check_output(["git", "status", "--porcelain"], cwd=source, text=True),
                "fingerprint": fingerprint, "files": actual}
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    result = sync(args.source, check=args.check)
    print(f"{'verified' if args.check else 'imported'} {len(result['files'])} source files; {result['fingerprint']}")
