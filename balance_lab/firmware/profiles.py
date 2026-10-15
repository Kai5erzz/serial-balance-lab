"""Compile simulation-only C patches without editing the source checkout/mirror."""
from __future__ import annotations

import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def build_profile(base_library: Path, name: str) -> Path:
    from .build import FIRMWARE, ROOT, VEHICLE

    if name != "jump46":
        raise ValueError(f"unknown simulation controller profile: {name}")
    patch = FIRMWARE / "profiles" / f"{name}.patch"
    metadata_bytes = patch.with_suffix(".json").read_bytes()
    metadata = json.loads(metadata_bytes)
    patch_bytes = patch.read_bytes()
    key = hashlib.sha256(base_library.parent.name.encode() + patch_bytes + metadata_bytes +
                         Path(__file__).read_bytes()).hexdigest()
    target = ROOT / "out" / "firmware" / key / "vehicle.so"
    target.parent.mkdir(parents=True, exist_ok=True)
    with (target.parent / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if target.exists():
            return target
        # A disposable fork keeps firmware/vehicle byte-exact and auto-syncable.
        # Fail on changed patch context; never silently omit a tuning change.
        with tempfile.TemporaryDirectory(prefix="rl-lab-profile-") as temporary:
            vehicle = Path(temporary) / "vehicle"
            shutil.copytree(VEHICLE, vehicle)
            for relative in metadata["modified_files"]:
                source = vehicle / relative
                if not source.resolve().is_relative_to(vehicle):
                    raise ValueError("profile file must be inside its disposable source fork")
                # Some original headers use CRLF; the unified patch uses LF.
                source.write_text(source.read_text())
            applied = subprocess.run(
                ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(patch)],
                cwd=vehicle, capture_output=True, text=True)
            if applied.returncode:
                raise RuntimeError(f"simulation profile {name} no longer applies to source:\n"
                                   f"{applied.stdout}{applied.stderr}")
            base = json.loads((base_library.parent / "build.json").read_text())
            command = [part.replace(str(VEHICLE), str(vehicle)) for part in base["command"]]
            output = target.with_suffix(".building.so")
            command = [str(output) if part == str(base_library.with_suffix(".building.so"))
                       else part for part in command]
            result = subprocess.run(command, capture_output=True, text=True)
            (target.parent / "build.log").write_text(applied.stdout + result.stdout + result.stderr)
            (target.parent / "build.json").write_text(json.dumps({
                "fingerprint": key, "base_native_build": base_library.parent.name,
                "simulation_profile": {**metadata, "name": name,
                                       "patch_sha256": hashlib.sha256(patch_bytes).hexdigest()},
                "command": command,
            }, indent=2) + "\n")
            if result.returncode:
                output.unlink(missing_ok=True)
                raise RuntimeError(f"simulation profile build failed: {target.parent / 'build.log'}\n"
                                   f"{result.stderr[-4000:]}")
            output.replace(target)
    return target
