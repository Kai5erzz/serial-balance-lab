"""Attribute height outliers to CAD solids and compare original triangles."""
from pathlib import Path
import json
import resource
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
source = json.loads((ROOT/"build/cad78/assembly.json").read_text())
manifest = json.loads((ROOT/"pack/manifest.json").read_text())
rotation = np.array(manifest["cad_to_world_rotation"])
origin = np.array(manifest["world_origin_before_shift_m"])
model = mujoco.MjModel.from_xml_path(str(ROOT/"pack/scene_collision.xml"))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
rows = json.loads((ROOT/"checks/height_rays.json").read_text())
result = []
for row in rows:
    if abs(row.get("error_m", 0)) <= .012:
        continue
    x, y = row["xy_m"]
    mask = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    geom = np.array([-1], dtype=np.int32)
    mujoco.mj_ray(model, data, np.array([x, y, 4.]), np.array([0., 0., -1.]), mask, 1, -1, geom)
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom[0]))
    instance = source["instances"][int(name.split("_")[-2])]
    record = source["unique_solids"][instance["solid_id"]]
    mesh = np.load(ROOT/"build/cad78"/record["mesh"])
    pose = np.array(instance["cad_transform_mm"])
    vertices = mesh["vertices_mm"]*.001@(rotation@pose[:3, :3]).T+rotation@pose[:3, 3]*.001-origin
    triangles = vertices[mesh["faces"]]
    a = triangles[:, 0]; u = triangles[:, 1]-a; v = triangles[:, 2]-a
    dx, dy = x-a[:, 0], y-a[:, 1]
    determinant = u[:, 0]*v[:, 1]-u[:, 1]*v[:, 0]
    valid = abs(determinant) > 1e-13
    first = np.zeros(len(a)); second = first.copy()
    first[valid] = (dx[valid]*v[valid, 1]-dy[valid]*v[valid, 0])/determinant[valid]
    second[valid] = (u[valid, 0]*dy[valid]-u[valid, 1]*dx[valid])/determinant[valid]
    valid &= (first >= -1e-8) & (second >= -1e-8) & (first+second <= 1+1e-8)
    heights = a[:, 2]+first*u[:, 2]+second*v[:, 2]
    raw_height = float(heights[valid].max()) if valid.any() else None
    item = dict(row, collision_geom=name, solid_id=record["id"], original_solid_height_m=raw_height,
                cad_faces=len(mesh["faces"]))
    result.append(item)
print(json.dumps(result, indent=2), flush=True)
(ROOT/"build/collision_outliers.json").write_text(json.dumps(result, indent=2)+"\n")
print("peak MiB", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
