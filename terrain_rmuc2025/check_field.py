"""Check asset identity, CAD/collision rays and wheel-sized contact probes."""
from pathlib import Path
import argparse
import copy
import gc
import ctypes
import hashlib
import json
import resource
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from compose import compose

ROOT = Path(__file__).resolve().parent


def ray_height(model, data, x, y, group):
    mask = np.zeros(6, dtype=np.uint8); mask[group] = 1
    geom = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, np.array([x, y, 4.]), np.array([0., 0., -1.]), mask, 1, -1, geom)
    return None if distance < 0 else 4.-float(distance)


def release_compiler_memory():
    if hasattr(mujoco, "mj_freeLastXML"):
        mujoco.mj_freeLastXML()
    if hasattr(mujoco, "mj_getCache") and hasattr(mujoco, "mj_clearCache"):
        mujoco.mj_clearCache(mujoco.mj_getCache())
    gc.collect()
    ctypes.CDLL(None).malloc_trim(0)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pack", type=Path, default=ROOT/"pack")
    p.add_argument("--output", type=Path, default=ROOT/"checks")
    a = p.parse_args()
    if a.output.exists():
        p.error("output exists; choose a new directory")
    a.output.mkdir(parents=True)
    started = time.monotonic()
    manifest = json.loads((a.pack/"manifest.json").read_text())
    for relative, digest in manifest["files"].items():
        if hashlib.sha256((a.pack/relative).read_bytes()).hexdigest() != digest:
            raise AssertionError(f"asset fingerprint differs: {relative}")
    reference_entry = "scene_cad_reference.xml" if (a.pack/"scene_cad_reference.xml").exists() else "scene.xml"
    model = mujoco.MjModel.from_xml_path(str(a.pack/reference_entry))
    # mj_ray ignores alpha-zero geoms, even when their group is enabled. The
    # full scene hides collision geoms for rendering; expose them in this
    # diagnostic model only, without altering files or contact parameters.
    model.geom_rgba[model.geom_group == 3, 3] = 1
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rows, errors, misses = [], [], []
    for x in np.arange(-13.75, 14., .5):
        for y in np.arange(-7.25, 7.5, .5):
            visual, collision = ray_height(model, data, x, y, 1), ray_height(model, data, x, y, 3)
            row = {"xy_m": [float(x), float(y)], "cad_visual_height_m": visual, "collision_height_m": collision}
            if visual is not None and collision is not None:
                row["error_m"] = collision-visual
                errors.append(collision-visual)
            elif visual is not None or collision is not None:
                misses.append(row)
            rows.append(row)
    (a.output/"height_rays.json").write_text(json.dumps(rows, indent=2)+"\n")
    print("CAD/collision ray samples", len(rows), "both hit", len(errors), "one sided", len(misses), flush=True)
    if misses or not errors or max(abs(np.asarray(errors))) > .012:
        raise AssertionError("CAD/collision height mismatch remains; inspect height_rays.json")
    # Select three clear flat CAD patches, covering ground and raised surfaces.
    candidates = []
    for row in rows:
        h = row["collision_height_m"]
        if h is None or not -.01 <= h <= .8 or abs(row.get("error_m", 100)) > .012:
            continue
        x, y = row["xy_m"]
        ring = [ray_height(model, data, x+dx, y+dy, 3)
                for dx, dy in ((-.35, 0), (.35, 0), (0, -.35), (0, .35), (-.25, -.25), (.25, .25))]
        if all(value is not None and abs(value-h) < .004 for value in ring):
            candidates.append(row)
    if not candidates:
        raise AssertionError("no clear flat spawn in the CAD field")
    selected = []
    for target, at in ((0., (-8, -4)), (.2, (8, 4)), (.6, (0, 0))):
        eligible = [r for r in candidates if all(np.linalg.norm(np.array(r["xy_m"])-s["xy_m"]) > .7 for s in selected)]
        selected.append(min(eligible, key=lambda r: 100*abs(r["collision_height_m"]-target)+np.linalg.norm(np.array(r["xy_m"])-at)))
    (a.output/"spawns.json").write_text(json.dumps(selected, indent=2)+"\n")
    del model, data
    release_compiler_memory()
    # Drop a sphere with the vehicle's 58 mm wheel radius on each selected patch.
    root = ET.parse(a.pack/"scene_collision.xml").getroot()
    root.find("include").set("file", str((a.pack/"field_collision.xml").resolve()))
    field = ET.parse(a.pack/"field_collision.xml").getroot()
    root.remove(root.find("include"))
    for asset in field.find("asset"):
        asset.set("file", str((a.pack/asset.get("file")).resolve()))
    root.append(copy.deepcopy(field.find("asset")))
    world = root.find("worldbody")
    for geom in field.find("worldbody"):
        world.append(copy.deepcopy(geom))
    for i, row in enumerate(selected):
        x, y = row["xy_m"]; h = row["collision_height_m"]
        body = ET.SubElement(world, "body", name=f"wheel_radius_probe_{i}", pos=f"{x} {y} {h+.24}")
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "geom", name=f"probe_{i}", type="sphere", size=".058", mass=".2",
                      contype="1", conaffinity="4", condim="3", friction="1.2 .02 .001")
    contact_model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    contact_data = mujoco.MjData(contact_model)
    for _ in range(2000):
        mujoco.mj_step(contact_model, contact_data)
        if any(contact_data.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,
               mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)):
            raise AssertionError("contact probe became numerically invalid")
    probe_rows = []
    for i, row in enumerate(selected):
        body = mujoco.mj_name2id(contact_model, mujoco.mjtObj.mjOBJ_BODY, f"wheel_radius_probe_{i}")
        position = contact_data.xpos[body].copy()
        error = float(position[2]-(row["collision_height_m"]+.058))
        if abs(error) > .004:
            raise AssertionError(f"wheel-radius contact height differs: {i}: {error}")
        probe_rows.append({"spawn": row, "settled_center_m": position.tolist(), "height_error_m": error})
    del contact_model, contact_data
    release_compiler_memory()
    # Composition is checked against a read-only copy of the current RL-lab robot.
    scene_path = a.output/"rl_lab_fast_scene.xml"
    robot = ROOT.parent/"model/whole_body_lqr_fast.xml"
    xy = selected[0]["xy_m"]
    composition = compose(robot, scene_path, pack=a.pack, spawn=(*xy, selected[0]["collision_height_m"]+.6))
    composed = mujoco.MjModel.from_xml_path(str(scene_path))
    composed_data = mujoco.MjData(composed)
    for _ in range(10):
        mujoco.mj_step(composed, composed_data)
    if any(composed_data.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,
           mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)):
        raise AssertionError("composed robot scene is numerically invalid")
    report = {"asset_hashes_verified": len(manifest["files"]), "ray_samples": len(rows),
              "reference_scene": reference_entry,
              "both_hit_samples": len(errors), "one_sided_hit_samples": len(misses),
              "height_error_abs_max_m": float(max(abs(np.asarray(errors)))),
              "height_error_abs_p95_m": float(np.percentile(abs(np.asarray(errors)), 95)),
              "height_error_over_12mm_count": int(np.sum(abs(np.asarray(errors)) > .012)),
              "wheel_radius_probes": probe_rows, "robot_composition": composition,
              "composed_actuators": int(composed.nu), "controller_behavior_tested": False,
              "wall_seconds": time.monotonic()-started, "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
    (a.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(report, flush=True)


if __name__ == "__main__":
    main()
