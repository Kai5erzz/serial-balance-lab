"""Check asset identity, CAD/collision rays and wheel-sized contact probes."""
from pathlib import Path
import argparse
import copy
import gc
import ctypes
import hashlib
import json
import resource
from cad_rays import heights
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from compose import compose

ROOT = Path(__file__).resolve().parent


RAY_TOP=8.

def ray_height(model, data, x, y, group):
    mask = np.zeros(6, dtype=np.uint8); mask[group] = 1
    geom = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, np.array([x, y, RAY_TOP]), np.array([0., 0., -1.]), mask, 1, -1, geom)
    return None if distance < 0 else RAY_TOP-float(distance)


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
    p.add_argument("--resume",action="store_true")
    a = p.parse_args()
    resource.setrlimit(resource.RLIMIT_AS,(int(5.5*1024**3),)*2)
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    if a.output.exists() and not a.resume:
        p.error("output exists; choose a new directory")
    a.output.mkdir(parents=True,exist_ok=a.resume)
    started = time.monotonic()
    manifest = json.loads((a.pack/"manifest.json").read_text())
    for relative, digest in manifest["files"].items():
        if hashlib.sha256((a.pack/relative).read_bytes()).hexdigest() != digest:
            raise AssertionError(f"asset fingerprint differs: {relative}")
    cad=ROOT/'build/cad78';source=json.loads((cad/'assembly.json').read_text())
    grid=np.array([[x,y] for x in np.arange(-13.75,14.,.5) for y in np.arange(-7.25,7.5,.5)])
    cache=a.output/'raw_cad_rays.json'
    if a.resume and cache.exists():
        raw=json.loads(cache.read_text())
        if raw['source_sha256']!=manifest['source']['sha256']:raise ValueError('Ray source differs')
        full_z=np.array(raw['height_m'],dtype=float);owners=np.array(raw['owners']);is_surface=np.array(raw['surface_owner']);closed_z=np.array(raw['closed_height_m'],dtype=float)
    else:
        full_z,owners,is_surface,counts,closed_z=heights(source,cad,np.array(manifest['cad_to_world_rotation']),np.array(manifest['world_origin_before_shift_m']),grid)
        cache.write_text(json.dumps({'source_sha256':source['source_sha256'],'xy_m':grid.tolist(),'height_m':[float(z) if np.isfinite(z) else None for z in full_z],'owners':owners.tolist(),'surface_owner':is_surface.tolist(),'closed_height_m':[float(z) if np.isfinite(z) else None for z in closed_z],'counts':counts})+'\n')
    reference_entry='raw per-solid B-rep tessellations (2 mm deflection), independent of MuJoCo mesh compiler'
    # Compile the contact probes with the field once. Their group is excluded
    # from the terrain rays, and their free joints are moved to selected patches
    # only after the ray comparison has passed.
    probe_scene=ET.parse(a.pack/'scene_collision.xml').getroot()
    probe_scene.find('include').set('file',str((a.pack/'field_collision.xml').resolve()))
    probe_scene.find('compiler').set('meshdir',str(a.pack.resolve()))
    world=probe_scene.find('worldbody')
    for i in range(3):
        body=ET.SubElement(world,'body',name=f'wheel_radius_probe_{i}',pos=f'0 0 {20+i}')
        ET.SubElement(body,'freejoint',name=f'wheel_probe_joint_{i}')
        ET.SubElement(body,'geom',name=f'probe_{i}',type='sphere',size='.058',mass='.2',
                      contype='1',conaffinity='4',condim='3',friction='1.2 .02 .001')
    probe_path=a.output/'collision_probes_scene.xml'
    ET.ElementTree(probe_scene).write(probe_path,encoding='utf-8',xml_declaration=True)
    print('Compiling collision field and three contact probes',flush=True)
    model=mujoco.MjModel.from_xml_path(str(probe_path));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    rows=[];errors=[];misses=[];facility=[]
    for i,(x,y) in enumerate(grid):
        geom=np.array([-1],dtype=np.int32);mask=np.zeros(6,dtype=np.uint8);mask[3]=1
        d=mujoco.mj_ray(model,data,np.array([x,y,RAY_TOP]),np.array([0.,0.,-1.]),mask,1,-1,geom)
        collision=None if d<0 else RAY_TOP-float(d)
        name='' if geom[0]<0 else mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(geom[0]))
        visual=float(full_z[i]) if np.isfinite(full_z[i]) else None
        equipment=bool(is_surface[i] or 'surface_contact' in name or 'facility_contact' in name or int(owners[i]) in manifest.get('closed_facility_proxy_ids',[]))
        row={'xy_m':[float(x),float(y)],'cad_visual_height_m':visual,'closed_cad_height_m':float(closed_z[i]) if np.isfinite(closed_z[i]) else None,
             'collision_height_m':collision,'cad_part_id':int(owners[i]),'source_surface_only':bool(is_surface[i]),'collision_geom':name,'facility_proxy_sample':equipment}
        if visual is not None and collision is not None:
            row['error_m']=collision-visual
            if equipment:facility.append(row)
            else:errors.append(collision-visual)
        elif visual is not None or collision is not None:
            if equipment:facility.append(row)
            else:misses.append(row)
        rows.append(row)
    (a.output/'height_rays.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('CAD/collision rays',len(rows),'terrain compared',len(errors),'facility proxies',len(facility),'one-sided',len(misses),
          'max error',max(abs(np.array(errors))),flush=True)
    if misses or not errors or max(abs(np.asarray(errors)))>.012:raise AssertionError('Terrain mismatch; inspect height_rays.json')
    # Select three clear flat CAD patches, covering ground and raised surfaces.
    candidates = []
    for row in rows:
        h = row["collision_height_m"]
        if row["facility_proxy_sample"] or h is None or not -.4 <= h <= 1.2 or abs(row.get("error_m", 100)) > .012:
            continue
        x, y = row["xy_m"]
        ring = [ray_height(model, data, x+dx, y+dy, 3)
                for dx, dy in ((-.35, 0), (.35, 0), (0, -.35), (0, .35), (-.25, -.25), (.25, .25))]
        if all(value is not None and abs(value-h) < .025 for value in ring):
            candidates.append(row)
    if not candidates:
        raise AssertionError("no clear flat spawn in the CAD field")
    selected = []
    for target, at in ((0., (-8, -4)), (.2, (8, 4)), (.6, (0, 0))):
        eligible = [r for r in candidates if all(np.linalg.norm(np.array(r["xy_m"])-s["xy_m"]) > .7 for s in selected)]
        selected.append(min(eligible, key=lambda r: 100*abs(r["collision_height_m"]-target)+np.linalg.norm(np.array(r["xy_m"])-at)))
    (a.output/"spawns.json").write_text(json.dumps(selected, indent=2)+"\n")
    # Drop a sphere with the vehicle's 58 mm wheel radius on each selected patch.
    for i, row in enumerate(selected):
        x, y = row["xy_m"]; h = row["collision_height_m"]
        joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,f'wheel_probe_joint_{i}')
        address=model.jnt_qposadr[joint]
        data.qpos[address:address+7]=[x,y,h+.24,1,0,0,0]
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    contact_model=model;contact_data=data
    for _ in range(2000):
        mujoco.mj_step(contact_model, contact_data)
        if any(contact_data.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,
               mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)):
            raise AssertionError("contact probe became numerically invalid")
    probe_rows = []
    for i, row in enumerate(selected):
        body = mujoco.mj_name2id(contact_model, mujoco.mjtObj.mjOBJ_BODY, f"wheel_radius_probe_{i}")
        position = contact_data.xpos[body].copy()
        local_height=ray_height(contact_model,contact_data,float(position[0]),float(position[1]),3)
        if local_height is None:raise AssertionError("Probe left terrain")
        error = float(position[2]-(local_height+.058))
        if abs(error) > .004:
            raise AssertionError(f"wheel-radius contact height differs: {i}: {error}")
        probe_rows.append({"spawn": row, "settled_center_m": position.tolist(), "height_error_m": error})
    del contact_model, contact_data, model, data
    release_compiler_memory()
    # Composition is checked against a read-only copy of the current RL-lab robot.
    scene_path = a.output/"rl_lab_fast_scene.xml"
    if a.resume and scene_path.exists():scene_path.unlink()
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
              "terrain_compared_samples": len(errors), "facility_proxy_samples": len(facility),
              "facility_proxy_max_abs_height_error_m": max((abs(r.get("error_m",0)) for r in facility),default=0),
              "both_hit_samples": sum(r["collision_height_m"] is not None and r["cad_visual_height_m"] is not None for r in rows), "one_sided_hit_samples": len(misses),
              "both_miss_samples": sum(r["collision_height_m"] is None and r["cad_visual_height_m"] is None for r in rows),
              "facility_one_sided_samples": sum((r["collision_height_m"] is None)!=(r["cad_visual_height_m"] is None) for r in facility),
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
