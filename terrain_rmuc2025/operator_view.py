"""Render-only spatial tiles; contact/control use the separate original model."""
from pathlib import Path
import fcntl
import hashlib
import os
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def load_operator_view(variant="rmuc2025", *, prepare_only=False):
    from balance_lab.paths import MODEL_FAST, MODEL_FULL, REPO_ROOT

    if variant == "rmuc2025":
        from terrain_rmuc2025.compose import compose
        from terrain_rmuc2025.interactive_visual import interactive_field
        pack = REPO_ROOT / "terrain_rmuc2025/pack"
        field = interactive_field(pack)
        profile = "interactive"
        cache_root = REPO_ROOT / "out/models"
    elif variant == "rmuc2026":
        from terrain_rmuc2026.compose import compose
        pack = (REPO_ROOT / "terrain_rmuc2026/pack").resolve(strict=True)
        # The 2026 package already has reduced visual meshes. Tile those
        # directly, preserving every visible CAD triangle in this view.
        field = pack / "field.xml"
        profile = "full"
        cache_root = pack.parent / "operator-cache"
    elif variant == "rmuc2027":
        from terrain_rmuc2027.compose import compose
        from terrain_rmuc2027.runtime_assets import ensure_operator_visual
        pack = ensure_operator_visual()
        field = pack / "field.xml"
        profile = "full"
        cache_root = pack.parent / "operator-cache"
    else:
        raise ValueError(f"unknown arena {variant!r}")
    prefix = variant + "_"
    visual_prefix = variant + "_visual_"
    digest = hashlib.sha256(MODEL_FAST.read_bytes()+field.read_bytes()+
                            Path(__file__).read_bytes()+mujoco.__version__.encode()).hexdigest()[:20]
    digest = hashlib.sha256((digest + variant).encode() + MODEL_FULL.read_bytes() +
                            (REPO_ROOT / "terrain_rmuc2025/robot_contact.py").read_bytes() +
                            Path(compose.__code__.co_filename).read_bytes()).hexdigest()[:20]
    if variant in ("rmuc2026", "rmuc2027"):
        digest = hashlib.sha256(digest.encode() +
                                (pack / "field_perimeter.xml").read_bytes()).hexdigest()[:20]
    cache = cache_root / ("rmuc-view-"+digest)
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache / "view.mjb"
    with (cache / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if binary.exists():
            if prepare_only:
                return
            return mujoco.MjModel.from_binary_path(str(binary))
        source = cache / "composed.xml"
        if not source.exists():
            compose(MODEL_FAST, source, pack=pack, profile=profile)
        def write_tiles():
            root = ET.parse(source).getroot()
            # Mesh compilation normally runs in parallel, retaining several large
            # temporary BVHs at once. The display is compiled once and cached.
            root.find("compiler").set("usethread", "false")
            world, assets = root.find("worldbody"), root.find("asset")
            visual = {g.get("mesh"): g for g in world.findall("geom")
                      if g.get("name", "").startswith(visual_prefix)}
            for geom in list(world.findall("geom")):
                if geom.get("name", "").startswith(prefix):
                    world.remove(geom)
            for asset in list(assets.findall("mesh")):
                name = asset.get("name", "")
                if not name.startswith(prefix):
                    continue
                assets.remove(asset)
                if name not in visual:
                    continue
                with Path(asset.get("file")).open("rb") as stream:
                    nv, nn, nt, nf = struct.unpack("<4i", stream.read(16))
                    vertices = np.fromfile(stream, dtype="<f4", count=nv*3).reshape(-1, 3)
                    stream.seek(12*nn+8*nt, 1)
                    faces = np.fromfile(stream, dtype="<i4", count=nf*3).reshape(-1, 3)
                centers = vertices[faces].mean(axis=1)
                cells = np.floor(centers[:, :2]/3).astype(np.int32)
                keys = cells[:, 0]*100+cells[:, 1]
                order = np.argsort(keys, kind="stable")
                ends = np.r_[0, np.flatnonzero(np.diff(keys[order]))+1, nf]
                parts, small = [], []
                for a, b in zip(ends[:-1], ends[1:]):
                    part = faces[order[a:b]]
                    if len(part) < 4 or len(np.unique(part)) < 4:
                        small.append(part)
                    else:
                        parts.append(part)
                if small:
                    if parts:
                        parts[0] = np.concatenate([parts[0], *small])
                    else:
                        parts = [faces]
                for index, part in enumerate(parts):
                    used, inverse = np.unique(part, return_inverse=True)
                    tile = f"{name}_tile_{index:03d}"
                    path = cache / (tile+".msh")
                    with path.open("wb") as stream:
                        np.asarray([len(used), 0, 0, len(part)], dtype="<i4").tofile(stream)
                        vertices[used].astype("<f4").tofile(stream)
                        inverse.astype("<i4").tofile(stream)
                    ET.SubElement(assets, "mesh", name=tile, file=str(path), inertia="shell", maxhullvert="64")
                    attributes = dict(visual[name].attrib, name=tile, mesh=tile)
                    ET.SubElement(world, "geom", attributes)
            # Collision shapes are invisible in this display-only model. Their
            # CAD meshes would otherwise be compiled a second time for no
            # rendered pixels. Keep body/joint topology and all visual faces.
            for body in root.iter("worldbody"):
                for parent in body.iter():
                    for geom in list(parent.findall("geom")):
                        if geom.get("group") == "3":
                            parent.remove(geom)
            used_meshes = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
            for mesh in list(assets.findall("mesh")):
                if mesh.get("name") not in used_meshes:
                    assets.remove(mesh)
            # The display copy supplies transforms only. It never generates
            # feedback, contacts, forces, control output or a physical trajectory.
            for geom in root.iter("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
            scene = cache / "view.xml"
            ET.indent(root)
            ET.ElementTree(root).write(scene, encoding="utf-8", xml_declaration=True)
        write_tiles()
        scene = cache / "view.xml"
        # ElementTree/NumPy construction can leave large freed arenas in libc.
        # Return them before MuJoCo allocates its native compiler structures.
        import ctypes, gc
        gc.collect()
        ctypes.CDLL(None).malloc_trim(0)
        print("RMUC: compiling tiled display model", flush=True)
        model = mujoco.MjModel.from_xml_path(str(scene))
        temporary = cache / "view.tmp.mjb"
        mujoco.mj_saveModel(model, str(temporary))
        os.replace(temporary, binary)
        if not prepare_only:
            return model
