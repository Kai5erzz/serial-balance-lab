"""Build a low-memory interactive model without changing rigid-body data."""

from __future__ import annotations

from pathlib import Path
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


PROJECT = Path(__file__).resolve().parent
SOURCE = PROJECT / "model" / "whole_body_lqr.xml"
OUTPUT = PROJECT / "model" / "whole_body_lqr_fast.xml"
PREVIEW_SOURCE = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\mujoco_whole_body\whole_body_closed.xml"
)
WHEEL_RADIUS_M = 0.058
WHEEL_HALF_WIDTH_M = 0.02993


def numbers(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def relative_asset(path: Path) -> str:
    return os.path.relpath(path, OUTPUT.parent).replace("\\", "/")


def preview_meshes() -> dict[str, Path]:
    """Reuse the CAD export's existing low-detail display meshes."""
    tree = ET.parse(PREVIEW_SOURCE)
    asset = tree.getroot().find("asset")
    if asset is None:
        raise ValueError("Preview model has no asset section")
    result: dict[str, Path] = {}
    for mesh in asset.findall("mesh"):
        name = mesh.get("name")
        filename = mesh.get("file")
        if not name or not filename:
            continue
        path = (PREVIEW_SOURCE.parent / filename).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        result[name] = path
    return result


def mesh_body_bounds(model: mujoco.MjModel) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Union full-resolution visual vertices in each body's local frame."""
    bounds: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for geom in range(model.ngeom):
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if model.geom_contype[geom] != 0:
            continue
        mesh = int(model.geom_dataid[geom])
        address = int(model.mesh_vertadr[mesh])
        count = int(model.mesh_vertnum[mesh])
        vertices = model.mesh_vert[address:address + count]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
        transformed = vertices @ rotation.reshape(3, 3).T + model.geom_pos[geom]
        low = transformed.min(axis=0)
        high = transformed.max(axis=0)
        body = int(model.geom_bodyid[geom])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if not name:
            continue
        if name in bounds:
            old_low, old_high = bounds[name]
            low = np.minimum(low, old_low)
            high = np.maximum(high, old_high)
        bounds[name] = (low, high)
    return bounds


def main() -> None:
    source_model = mujoco.MjModel.from_xml_path(str(SOURCE))
    bounds = mesh_body_bounds(source_model)
    display_mesh = preview_meshes()
    tree = ET.parse(SOURCE)
    root = tree.getroot()

    # Remove every triangle mesh geom and asset.  Body inertials, joints,
    # equality constraints, sites, actuators, and the ground plane remain
    # byte-for-byte equivalent to the validated model.
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("type") == "mesh":
                parent.remove(child)
    asset = root.find("asset")
    if asset is not None:
        for child in list(asset):
            if child.tag == "mesh":
                asset.remove(child)
    else:
        asset = ET.SubElement(root, "asset")

    for name, path in display_mesh.items():
        ET.SubElement(asset, "mesh", {
            "name": f"fast_visual_mesh_{name}",
            "file": relative_asset(path),
        })

    body_by_name = {
        body.get("name"): body
        for body in root.findall(".//body")
        if body.get("name")
    }
    for name, (low, high) in bounds.items():
        body = body_by_name[name]
        if name in display_mesh:
            ET.SubElement(body, "geom", {
                "name": f"fast_visual_{name}",
                "type": "mesh", "mesh": f"fast_visual_mesh_{name}",
                "density": "0", "contype": "0", "conaffinity": "0",
                "group": "1", "rgba": ".752941 .752941 .752941 1",
            })
        else:
            center = 0.5 * (low + high)
            half_size = np.maximum(0.5 * (high - low), 0.004)
            ET.SubElement(body, "geom", {
                "name": f"fast_visual_{name}",
                "type": "box", "pos": numbers(center),
                "size": numbers(half_size),
                "density": "0", "contype": "0", "conaffinity": "0",
                "group": "1", "rgba": ".45 .50 .58 1",
            })

        if name in {"l_wheel", "r_wheel"}:
            ET.SubElement(body, "geom", {
                "name": f"fast_collision_{name}",
                "type": "cylinder",
                "size": f"{WHEEL_RADIUS_M} {WHEEL_HALF_WIDTH_M}",
                "density": "0", "contype": "1", "conaffinity": "4",
                "condim": "3", "friction": "1.0 .02 .001",
                "solref": ".008 1", "solimp": ".92 .99 .002",
                "rgba": "0 0 0 0",
            })
            continue

        center = 0.5 * (low + high)
        half_size = np.maximum(0.5 * (high - low), 0.004)
        ET.SubElement(body, "geom", {
            "name": f"fast_collision_{name}",
            "type": "box", "pos": numbers(center),
            "size": numbers(np.maximum(0.75 * half_size, 0.004)),
            "density": "0", "contype": "1", "conaffinity": "4",
            "condim": "3", "friction": "1.0 .02 .001",
            "solref": ".008 1", "solimp": ".92 .99 .002",
            "rgba": "0 0 0 0",
        })

    ET.indent(tree, space="  ")
    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)
    fast_model = mujoco.MjModel.from_xml_path(str(OUTPUT))
    if not np.isclose(fast_model.body_mass.sum(), source_model.body_mass.sum()):
        raise ValueError("Fast model changed total rigid-body mass")
    if (fast_model.njnt, fast_model.nu, fast_model.neq) != (
        source_model.njnt, source_model.nu, source_model.neq
    ):
        raise ValueError("Fast model changed joints, actuators, or constraints")
    print(f"Wrote {OUTPUT}")
    print(
        f"geoms {source_model.ngeom} -> {fast_model.ngeom}, "
        f"meshes {source_model.nmesh} -> {fast_model.nmesh}, "
        f"mass={fast_model.body_mass.sum():.3f} kg"
    )


if __name__ == "__main__":
    main()
