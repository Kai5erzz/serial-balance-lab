"""Cached physical robot in the locally installed RMUC 2026 Finals V2 field."""
from pathlib import Path
import fcntl
import hashlib
import os

import mujoco

from ..paths import MODEL_FAST, MODEL_FULL, REPO_ROOT

SPAWN_XY = (-11.25, -6.25)
# Measured against the installed CAD/contact map, whose floor is slightly tilted.
SPAWN_SURFACE_Z = -0.001847710538051217


def load_rmuc_model():
    from terrain_rmuc2026.compose import compose

    terrain = REPO_ROOT / "terrain_rmuc2026"
    pack = (terrain / "pack").resolve(strict=True)
    digest = hashlib.sha256(mujoco.__version__.encode())
    for path in (MODEL_FAST, MODEL_FULL, pack / "field_collision.xml",
                 pack / "field_perimeter.xml",
                 pack / "manifest.json", terrain / "compose.py",
                 REPO_ROOT / "terrain_rmuc2025/robot_contact.py", Path(__file__)):
        digest.update(path.read_bytes())
    # Follow the map's existing storage location (D盘2), not the system disk.
    cache = pack.parent / "operator-cache" / ("physics-" + digest.hexdigest()[:20])
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache / "scene.mjb"
    with (cache / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if binary.exists():
            print("RMUC 2026: loading cached arena", flush=True)
            return mujoco.MjModel.from_binary_path(str(binary))
        scene = cache / "scene.xml"
        if not scene.exists():
            # Physical map retains every packaged contact; display geometry
            # lives in a separate model and cannot affect feedback or forces.
            compose(MODEL_FAST, scene, profile="collision",
                    spawn=(*SPAWN_XY, SPAWN_SURFACE_Z + .6), perimeter=True)
        print("RMUC 2026: compiling robot and arena contacts (first load)", flush=True)
        model = mujoco.MjModel.from_xml_path(str(scene))
        temporary = cache / "scene.tmp.mjb"
        mujoco.mj_saveModel(model, str(temporary))
        os.replace(temporary, binary)
        return model
