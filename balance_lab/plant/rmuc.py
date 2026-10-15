"""Cached composition of the existing robot and official RMUC 2025 arena."""
from pathlib import Path
import fcntl
import hashlib
import os

import mujoco

from ..paths import MODEL_FAST, MODEL_FULL, REPO_ROOT

SPAWN_XY = (-8.25, -3.75)
SPAWN_SURFACE_Z = 0.0029995367855670807


def load_rmuc_model():
    from terrain_rmuc2025.compose import compose

    terrain = REPO_ROOT / "terrain_rmuc2025"
    pack = terrain / "pack"
    # Cache only the unmodified MJCF model. Identification and actuator limits
    # are reapplied by the caller, exactly as for the ordinary robot scene.
    digest = hashlib.sha256(mujoco.__version__.encode())
    for path in (MODEL_FAST, MODEL_FULL, pack / "field.xml", pack / "manifest.json",
                 terrain / "compose.py", terrain / "interactive_visual.py",
                 terrain / "robot_contact.py", Path(__file__)):
        digest.update(path.read_bytes())
    cache = REPO_ROOT / "out/models" / ("rmuc2025-" + digest.hexdigest()[:20])
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache / "scene.mjb"
    with (cache / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if binary.exists():
            print("RMUC 2025: loading cached arena", flush=True)
            return mujoco.MjModel.from_binary_path(str(binary))
        scene = cache / "scene.xml"
        if not scene.exists():
            compose(MODEL_FAST, scene, profile="interactive",
                    spawn=(*SPAWN_XY, SPAWN_SURFACE_Z + .6))
        print("RMUC 2025: compiling arena and robot; first load takes about 1–2 minutes", flush=True)
        model = mujoco.MjModel.from_xml_path(str(scene))
        temporary = cache / "scene.tmp.mjb"
        mujoco.mj_saveModel(model, str(temporary))
        os.replace(temporary, binary)
        return model
