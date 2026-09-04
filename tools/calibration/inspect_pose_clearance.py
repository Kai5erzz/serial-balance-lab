"""Report collision-geometry ground clearances at calibrated encoder zero."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import apply_encoder_zero_pose


ROOT = REPO_ROOT


def geom_min_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    geom_type = model.geom_type[geom_id]
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    position = data.geom_xpos[geom_id]
    size = model.geom_size[geom_id]
    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        address = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[address:address + count]
        return float(np.min(vertices @ rotation[2] + position[2]))
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        extent = float(np.abs(rotation[2]) @ size)
    elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        extent = float(size[0])
    elif geom_type in {
        mujoco.mjtGeom.mjGEOM_CYLINDER,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
    }:
        axis_z = abs(float(rotation[2, 2]))
        radial_z = float(np.sqrt(max(0.0, 1.0 - axis_z * axis_z)))
        extent = axis_z * float(size[1]) + radial_z * float(size[0])
        if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            extent += float(size[0])
    else:
        extent = float(np.max(size))
    return float(position[2] - extent)


def main() -> None:
    for filename in ("whole_body_lqr.xml", "whole_body_lqr_fast.xml"):
        model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / filename))
        data = mujoco.MjData(model)
        apply_encoder_zero_pose(model, data)
        entries = []
        for geom_id in range(model.ngeom):
            if model.geom_contype[geom_id] == 0:
                continue
            body_id = int(model.geom_bodyid[geom_id])
            if body_id == 0:
                continue
            entries.append((
                geom_min_z(model, data, geom_id),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
            ))
        print(filename)
        for height, body, geom in sorted(entries)[:12]:
            print(f"  z={height:+.6f} body={body} geom={geom}")


if __name__ == "__main__":
    main()
