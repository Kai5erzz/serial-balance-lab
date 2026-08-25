"""Create a floating, contact-enabled MuJoCo model for the traditional controller."""

from __future__ import annotations

import os
from pathlib import Path
from copy import deepcopy
import struct
import xml.etree.ElementTree as ET

import mujoco


PROJECT = Path(__file__).resolve().parent
MODEL_DIR = PROJECT / "model"
SOURCE = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\mujoco_whole_body\whole_body_closed.xml"
)
FULL_RES_MESHES = SOURCE.parent.parent / "meshes"
OUTPUT = MODEL_DIR / "whole_body_lqr.xml"

# This is the complete-robot mass used by the embedded LQR/gas-spring model.
LQR_TOTAL_MASS_KG = 12.801
CAD_BASE_MASS_KG = 21.768640481954
CAD_BASE_DIAG_INERTIA = (0.0320035150805737, 0.0371693390431462, 0.0319986911694648)
CAD_BASE_COM = (-0.0528146148782616, -0.0138844980591804, -0.0528008040866336)
WHEEL_RADIUS_M = 0.0580
WHEEL_HALF_WIDTH_M = 0.02993
# CAD is Y-up, while MuJoCo's visual tools and the reference simulator use
# Z-up. Rotate the complete CAD body +90 degrees around X and lift it so the
# initial wheel cylinders have a small (3 mm) contact overlap with z=0.
CAD_TO_ZUP_QUAT = ".70710678 .70710678 0 0"
CAD_WORLD_Z_M = 0.2550
MESH_FACE_LIMIT = 180_000
# The only mechanical angular end stop belongs to the two passive ``leg``
# hinges.  The user verified that CAD q=0 is normal standing and that the
# 300 N gas springs are fully extended at the outward stop.  Their exported
# axes are mirrored: spring force pushes l_leg toward +q and r_leg toward
# -q, so q=0 is the opposite end of the two MuJoCo ranges.  Keeping both
# ranges symmetric (or widening them for an invalid recovery branch) deletes
# that real stop and lets the springs drive the closed links into a false
# configuration.
PASSIVE_LEG_LIMITS_RAD = {
    "l_leg": (-1.35, 0.0),
    "r_leg": (0.0, 1.35),
}
BASE_COLLISION_DIR = MODEL_DIR / "collision_meshes"
PART_COLLISION_DIR = MODEL_DIR / "collision_meshes_vhacd"
# A tyre's convex exterior is its true ground-contact envelope.  The gimbal
# is normally elevated and locked; its exterior hull is a conservative
# fall-contact envelope while all articulated leg members use CoACD pieces.
EXTERIOR_HULL_STEMS = {"l_wheel", "r_wheel", "gimbal_yaw"}


def relpath(path: Path, start: Path) -> str:
    return os.path.relpath(path, start).replace("\\", "/")


def find_body(root: ET.Element, name: str) -> ET.Element:
    body = next((item for item in root.findall(".//body") if item.get("name") == name), None)
    if body is None:
        raise ValueError(f"Missing body: {name}")
    return body


def stl_face_count(path: Path) -> int:
    """Return the triangle count of a binary STL and validate its layout."""
    with path.open("rb") as stream:
        stream.seek(80)
        count = struct.unpack("<I", stream.read(4))[0]
    expected_bytes = 84 + 50 * count
    if path.stat().st_size != expected_bytes:
        raise ValueError(f"Only binary STL is supported for lossless splitting: {path}")
    return count


def split_stl_losslessly(source: Path, output_dir: Path) -> list[Path]:
    """Split an over-limit binary STL on triangle boundaries without decimation."""
    faces = stl_face_count(source)
    if faces <= MESH_FACE_LIMIT:
        return [source]
    output_dir.mkdir(parents=True, exist_ok=True)
    parts: list[Path] = []
    with source.open("rb") as input_stream:
        header = input_stream.read(80)
        input_stream.read(4)
        remaining = faces
        index = 0
        while remaining:
            chunk_faces = min(remaining, MESH_FACE_LIMIT)
            destination = output_dir / f"{source.stem}_part_{index:03d}.STL"
            with destination.open("wb") as output_stream:
                part_header = (header[:60] + f" MuJoCo part {index}".encode("ascii"))[:80].ljust(80, b" ")
                output_stream.write(part_header)
                output_stream.write(struct.pack("<I", chunk_faces))
                output_stream.write(input_stream.read(50 * chunk_faces))
            parts.append(destination)
            remaining -= chunk_faces
            index += 1
    return parts


def expand_mesh_asset(root: ET.Element, asset: ET.Element, mesh: ET.Element, selected: Path) -> None:
    """Replace one large mesh asset and its geoms with lossless STL parts."""
    source_name = mesh.get("name")
    if not source_name:
        raise ValueError("Mesh asset has no name")
    parts = split_stl_losslessly(selected, MODEL_DIR / "full_resolution_meshes")
    if len(parts) == 1:
        mesh.set("file", relpath(parts[0].resolve(), MODEL_DIR))
        return

    part_names = [f"{source_name}_part_{index:03d}" for index in range(len(parts))]
    asset_index = list(asset).index(mesh)
    asset.remove(mesh)
    for index, (part_name, part_path) in enumerate(zip(part_names, parts)):
        replacement = deepcopy(mesh)
        replacement.set("name", part_name)
        replacement.set("file", relpath(part_path.resolve(), MODEL_DIR))
        asset.insert(asset_index + index, replacement)

    for parent in root.iter():
        children = list(parent)
        for index, child in enumerate(children):
            if child.tag != "geom" or child.get("mesh") != source_name:
                continue
            parent.remove(child)
            for part_index, part_name in enumerate(part_names):
                replacement = deepcopy(child)
                replacement.set("mesh", part_name)
                # Geom names must be unique. Visual geoms normally have no
                # name, but preserve named input meshes safely if present.
                if child.get("name"):
                    replacement.set("name", f"{child.get('name')}_part_{part_index:03d}")
                parent.insert(index + part_index, replacement)


def main() -> None:
    if not SOURCE.is_file():
        raise FileNotFoundError(f"Build the whole-body inspector first: {SOURCE}")
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    source_model = mujoco.MjModel.from_xml_path(str(SOURCE))
    base_id = mujoco.mj_name2id(source_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    nonbase_mass = float(source_model.body_mass.sum() - source_model.body_mass[base_id])
    base_mass = LQR_TOTAL_MASS_KG - nonbase_mass
    if base_mass <= 0.0:
        raise ValueError(f"LQR mass budget is invalid: non-base bodies already weigh {nonbase_mass:.3f} kg")

    tree = ET.parse(SOURCE)
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    for mesh in list(asset.findall("mesh")):
        original = mesh.get("file")
        if original:
            # The inspection model replaced base/gimbal meshes with simplified
            # previews. For physical contact use each original export mesh
            # when present, including the high-resolution base and gimbal.
            full_resolution = FULL_RES_MESHES / Path(original).name
            selected = full_resolution if full_resolution.is_file() else (SOURCE.parent / original)
            expand_mesh_asset(root, asset, mesh, selected.resolve())

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.attrib.update({
        "gravity": "0 0 -9.81",
        "timestep": ".001",
        "integrator": "implicitfast",
        "solver": "Newton",
        "iterations": "100",
    })

    for item in list(asset):
        if item.get("name") in {"lqr_ground_checker", "lqr_ground_material"}:
            asset.remove(item)
    ET.SubElement(asset, "texture", {
        "name": "lqr_ground_checker", "type": "2d", "builtin": "checker",
        "width": "512", "height": "512", "rgb1": ".055 .075 .11",
        "rgb2": ".36 .43 .52", "mark": "edge", "markrgb": ".98 .56 .10",
    })
    ET.SubElement(asset, "material", {
        "name": "lqr_ground_material", "texture": "lqr_ground_checker",
        "texrepeat": "12 12", "texuniform": "true", "reflectance": ".06",
        "specular": ".12", "shininess": ".35",
    })

    world = root.find("worldbody")
    if world is None:
        raise ValueError("MJCF has no worldbody")
    for geom in list(world.findall("geom")):
        if geom.get("name") in {"floor", "ground"}:
            world.remove(geom)
    ET.SubElement(world, "geom", {
        "name": "ground", "type": "plane", "pos": "0 0 0", "size": "5 5 .1",
        "friction": "1.2 .02 .001", "material": "lqr_ground_material",
        # Category 4 is reserved for ground. Every visual CAD mesh accepts
        # only category 4, so all base/leg/wheel/gimbal shapes contact the
        # ground while adjacent mechanical meshes do not self-collide.
        "contype": "4", "conaffinity": "3",
    })

    base = find_body(root, "base_link")
    # A free joint is required to be a top-level body.  Put the CAD-to-Z-up
    # transform into base_link's reference pose instead of adding a wrapper
    # parent; MuJoCo then records it in qpos0.
    if base not in list(world):
        raise ValueError("base_link must remain directly under worldbody for its free joint")
    base.set("pos", f"0 0 {CAD_WORLD_Z_M}")
    base.set("quat", CAD_TO_ZUP_QUAT)
    if base.find("freejoint") is None:
        base.insert(0, ET.Element("freejoint", {"name": "base_free"}))
    if base.find("inertial") is None:
        inertia_scale = base_mass / CAD_BASE_MASS_KG
        ET.SubElement(base, "inertial", {
            "pos": " ".join(f"{value:.12g}" for value in CAD_BASE_COM),
            "mass": f"{base_mass:.12g}",
            "diaginertia": " ".join(f"{value * inertia_scale:.12g}" for value in CAD_BASE_DIAG_INERTIA),
        })
    # Full-resolution CAD is visual only.  A non-convex dynamic mesh becomes
    # one *convex* hull inside MuJoCo, so using it directly makes open legs
    # and chassis frames collide through empty space.  The collision geoms
    # appended below are CoACD multi-hull pieces in the same CAD coordinates.
    for geom in root.findall(".//geom"):
        if geom.get("type") == "mesh":
            geom.attrib.update({
                "density": "0", "contype": "0", "conaffinity": "0",
            })

    # Keep the full-resolution base STL strictly visual.  MuJoCo regards a
    # dynamic mesh as one convex hull, which fills the chassis' open underside
    # and makes it touch the floor through empty space.  CoACD supplies an
    # accurate union of convex collision pieces in the same local frame.
    base_collision_parts = sorted(BASE_COLLISION_DIR.glob("base_link_collision_*.STL"))
    if not base_collision_parts:
        raise FileNotFoundError(
            "Missing chassis collision decomposition. Run decompose_base_collision.py before building."
        )
    for index, part in enumerate(base_collision_parts):
        mesh_name = f"base_link_collision_{index:03d}"
        ET.SubElement(asset, "mesh", {
            "name": mesh_name,
            "file": relpath(part.resolve(), MODEL_DIR),
        })
        ET.SubElement(base, "geom", {
            "type": "mesh", "mesh": mesh_name,
            "density": "0", "contype": "1", "conaffinity": "4",
            "condim": "3", "friction": "1.0 .02 .001",
            "solref": ".008 1", "solimp": ".92 .99 .002",
            "rgba": "0 0 0 0",
        })

    # Moving CAD members use their own high-fidelity convex unions.  The
    # base is handled separately above because it has an existing connected-
    # component decomposition; all other exported STL names match their body
    # names.  Do not silently fall back to a visual-mesh convex hull: a
    # missing collision asset must be explicit during model construction.
    collision_stems = [
        path.stem for path in sorted(FULL_RES_MESHES.glob("*.STL"))
        if path.stem not in {"base_link", *EXTERIOR_HULL_STEMS}
    ]
    missing_collision = [stem for stem in collision_stems if not list(PART_COLLISION_DIR.glob(f"{stem}_*.STL"))]
    if missing_collision:
        raise FileNotFoundError(
            "Missing convex collision assets for: " + ", ".join(missing_collision) +
            ". Run make_collision_meshes.py before building."
        )
    for stem in collision_stems:
        body = find_body(root, stem)
        for index, part in enumerate(sorted(PART_COLLISION_DIR.glob(f"{stem}_*.STL"))):
            mesh_name = f"collision_{stem}_{index:03d}"
            ET.SubElement(asset, "mesh", {
                "name": mesh_name,
                "file": relpath(part.resolve(), MODEL_DIR),
            })
            ET.SubElement(body, "geom", {
                "type": "mesh", "mesh": mesh_name,
                "density": "0", "contype": "1", "conaffinity": "4",
                "condim": "3", "friction": "1.0 .02 .001",
                "solref": ".008 1", "solimp": ".92 .99 .002",
                "rgba": "0 0 0 0",
            })

    for stem in sorted(EXTERIOR_HULL_STEMS):
        source = FULL_RES_MESHES / f"{stem}.STL"
        if not source.is_file():
            raise FileNotFoundError(f"Missing exterior collision STL: {source}")
        # MuJoCo caps a binary STL at 200k faces.  The high-resolution
        # gimbal export is therefore split losslessly for asset loading;
        # wheels stay as one exterior envelope.
        for index, part in enumerate(split_stl_losslessly(source, MODEL_DIR / "full_resolution_meshes")):
            mesh_name = f"collision_{stem}_exterior_{index:03d}"
            ET.SubElement(asset, "mesh", {
                "name": mesh_name,
                "file": relpath(part.resolve(), MODEL_DIR),
            })
            ET.SubElement(find_body(root, stem), "geom", {
                "type": "mesh", "mesh": mesh_name,
                "density": "0", "contype": "1", "conaffinity": "4",
                "condim": "3", "friction": "1.0 .02 .001",
                "solref": ".008 1", "solimp": ".92 .99 .002",
                "rgba": "0 0 0 0",
            })

    for joint_name, limits in PASSIVE_LEG_LIMITS_RAD.items():
        joint = next((item for item in root.findall(".//joint") if item.get("name") == joint_name), None)
        if joint is None:
            raise ValueError(f"Missing passive leg joint: {joint_name}")
        joint.set("range", f"{limits[0]} {limits[1]}")

    actuator = root.find("actuator")
    if actuator is not None:
        root.remove(actuator)
    actuator = ET.SubElement(root, "actuator")
    for name in ("l_wheel", "r_wheel"):
        ET.SubElement(actuator, "motor", {
            "name": f"{name}_motor", "joint": name, "gear": "1",
            "ctrllimited": "true", "ctrlrange": "-3.69 3.69",
        })
    for name in ("l_j1", "l_j2", "r_j1", "r_j2"):
        ET.SubElement(actuator, "motor", {
            "name": f"{name}_motor", "joint": name, "gear": "1",
            # Firmware recovery uses a brief 24 N.m breakaway allowance;
            # normal LQR is still clamped to 20 N.m in traditional_lqr.py.
            "ctrllimited": "true", "ctrlrange": "-24 24",
        })
    ET.SubElement(actuator, "motor", {
        "name": "gimbal_yaw_motor", "joint": "gimbal_yaw", "gear": "1",
        "ctrllimited": "true", "ctrlrange": "-1.2 1.2",
    })

    ET.indent(tree, space="  ")
    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(OUTPUT))
    print(f"Wrote {OUTPUT}")
    print(f"mass: base={base_mass:.3f} kg, total={model.body_mass.sum():.3f} kg")
    print(f"free base, full-resolution visual CAD + convex collision unions; actuators={model.nu}")


if __name__ == "__main__":
    main()
