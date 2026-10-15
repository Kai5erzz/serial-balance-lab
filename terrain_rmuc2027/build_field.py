"""Build a colored MJCF field with independently decomposed CAD collisions.

The source remains the official 2027 V1.0.0 assembly. This script
does not generate a single convex hull or a heightfield for the whole arena.
"""
from pathlib import Path
import argparse
import copy
import ctypes
import gc
from collections import defaultdict
import hashlib
import json
import resource
import re
import shutil
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull, QhullError

ROOT = Path(__file__).resolve().parent
sys.path.append(str(ROOT.parent/"terrain_rmuc2025/.caddeps"))
# main() verifies the CAD dimensions and computes a proper rotation.
# The pinned 2027 assembly is Z-up, with its long direction along X.
CAD_TO_WORLD = np.eye(3)


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def numbers(values):
    return " ".join(f"{float(x):.10g}" for x in values)


def write_msh(mesh, path):
    """Stream MuJoCo's binary mesh format instead of formatting a giant OBJ.

    Header: vertex, normal, texcoord and triangle counts (little-endian int32).
    Normal/UV counts zero ask MuJoCo to compute normals; vertex float32 and
    face int32 rows follow. Conversion buffers are bounded to 8192 rows.
    """
    with Path(path).open("wb") as stream:
        np.asarray([len(mesh.vertices), 0, 0, len(mesh.faces)], dtype="<i4").tofile(stream)
        for array, dtype in ((mesh.vertices, "<f4"), (mesh.faces, "<i4")):
            for start in range(0, len(array), 8192):
                np.asarray(array[start:start+8192], dtype=dtype).tofile(stream)


def material_rgba(mesh):
    material = getattr(mesh.visual, "material", None)
    rgba = getattr(material, "main_color", np.array([178, 178, 178, 255]))
    return tuple(int(x) for x in rgba)


def clipped_road_cells(mesh,cell=.12):
    """Convexify bounded XY cells of the two closed, curved road solids.

    Clip source triangles, including their bottom and side faces, to each cell.
    This retains each solid's lower boundary; no column is extended to the floor.
    Only these two road parts use this policy, and their heights are checked
    independently against the original CAD tessellation.
    """
    start=mesh.bounds[0,:2];triangles=mesh.triangles
    buckets=defaultdict(list)
    for triangle in triangles:
        first=np.floor((triangle.min(0)[:2]-start)/cell).astype(int)
        last=np.floor((triangle.max(0)[:2]-start)/cell).astype(int)
        for i in range(first[0],last[0]+1):
            for j in range(first[1],last[1]+1):
                polygon=list(triangle)
                for axis,limit,sign in ((0,start[0]+i*cell,1),(0,start[0]+(i+1)*cell,-1),
                                        (1,start[1]+j*cell,1),(1,start[1]+(j+1)*cell,-1)):
                    clipped=[]
                    if not polygon:break
                    previous=polygon[-1];d0=sign*(previous[axis]-limit)
                    for current in polygon:
                        d1=sign*(current[axis]-limit)
                        if (d0>=0)!=(d1>=0):
                            clipped.append(previous+(current-previous)*(d0/(d0-d1)))
                        if d1>=0:clipped.append(current)
                        previous=current;d0=d1
                    polygon=clipped
                if len(polygon)>=3:buckets[i,j].extend(polygon)
    hulls=[]
    for points in buckets.values():
        points=np.unique(np.round(points,10),axis=0)
        if len(points)<4:continue
        try:hull=trimesh.Trimesh(vertices=points,faces=None).convex_hull
        except QhullError:continue
        if abs(hull.volume)>1e-12:hulls.append(hull)
    if not hulls:raise RuntimeError('Road cell clipping produced no solids')
    return hulls,{'cell_width_m':cell,'convex_parts':len(hulls),'bottom_boundary_retained':True,
                  'source_volume_m3':abs(float(mesh.volume)),'cell_hull_volume_m3':sum(abs(h.volume) for h in hulls)}


def planar_prisms(mesh, source_volume):
    """Preserve thin extruded plates and holes using their top triangulation.

    Convex adjacent polygons are merged only when their combined area equals
    their convex hull area. This avoids voxelizing millimetre-thick CAD sheets.
    """
    axis = int(np.argmin(mesh.extents))
    thickness = float(mesh.extents[axis])
    if not 1e-5 < thickness < .03:
        return None
    vertices = np.asarray(mesh.vertices)
    lo, hi = mesh.bounds[:, axis]
    triangles = vertices[mesh.faces]
    selected = np.all(abs(triangles[:, :, axis]-hi) < 1e-6, axis=1) & (mesh.area_faces > 1e-12)
    top_faces = mesh.faces[selected]
    if not len(top_faces):
        return None
    top_area = float(mesh.area_faces[selected].sum())
    if abs(top_area*thickness-source_volume) > max(source_volume*.003, 1e-10):
        return None
    plane_axes = [x for x in range(3) if x != axis]
    points = vertices[:, plane_axes]
    polygons = [list(face) for face in top_faces]
    areas = [float(value) for value in mesh.area_faces[selected]]
    for _ in range(32):
        edges = defaultdict(list)
        for i, poly in enumerate(polygons):
            if poly is not None:
                for j, vertex in enumerate(poly):
                    edges[tuple(sorted((vertex, poly[(j+1) % len(poly)])))].append(i)
        merged = 0
        for adjacent in edges.values():
            if len(adjacent) != 2:
                continue
            left, right = adjacent
            if polygons[left] is None or polygons[right] is None:
                continue
            indices = list(set(polygons[left]+polygons[right]))
            hull = ConvexHull(points[indices])
            combined = areas[left]+areas[right]
            if abs(hull.volume-combined) < max(1e-11, combined*1e-7):
                polygons[left] = [indices[x] for x in hull.vertices]
                areas[left] = combined
                polygons[right] = None
                merged += 1
        if not merged:
            break
    # Tiny drilled/etched components can contain thousands of little polygons.
    # They use the same 1 cm CoACD approximation as other hardware; large arena
    # plates keep the exact prisms so traversable holes remain open.
    if sum(poly is not None for poly in polygons) > 128 and mesh.extents.max() < 2:
        return None
    hulls = []
    for poly in polygons:
        if poly is not None:
            top = vertices[poly].copy(); top[:, axis] = hi
            bottom = top.copy(); bottom[:, axis] = lo
            hulls.append(trimesh.Trimesh(vertices=np.concatenate([top, bottom]), faces=None).convex_hull)
    return hulls


def exact_convex_parts(mesh):
    """Constrained tetrahedra, merged only when their union is convex."""
    import tetgen
    generator = tetgen.TetGen(np.asarray(mesh.vertices), np.asarray(mesh.faces, dtype=np.int32))
    cavities = []
    components = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), min_len=1)
    for component in components:
        shell = mesh.submesh([component], append=True, repair=False)
        if shell.volume < -1e-10:
            face = int(np.argmax(shell.area_faces))
            point = shell.triangles_center[face]+shell.face_normals[face]*1e-5
            generator.add_hole(point.tolist())
            cavities.append(point.tolist())
    nodes, elements, *_ = generator.tetrahedralize(switches="pzYQ")
    vertices = nodes[elements]
    volumes = abs(np.linalg.det(vertices[:, 1:]-vertices[:, :1]))/6
    total = float(volumes.sum())
    if abs(total-abs(mesh.volume)) > max(1e-9, abs(mesh.volume)*1e-7):
        raise RuntimeError(f"constrained tetrahedra do not reproduce the CAD volume: {total} vs {mesh.volume}")
    valid=volumes>1e-12
    discarded_volume=float(volumes[~valid].sum())
    elements=elements[valid];volumes=volumes[valid]
    parent = list(range(len(elements)))
    groups = [set(map(int, tet)) for tet in elements]
    facets = defaultdict(list)
    for i, tet in enumerate(elements):
        for face in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
            facets[tuple(sorted(int(tet[k]) for k in face))].append(i)
    adjacent = [values for values in facets.values() if len(values) == 2]
    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index
    for _ in range(32):
        merges = 0; attempted = set()
        for left, right in adjacent:
            left, right = root(left), root(right)
            pair = tuple(sorted((left, right)))
            if left == right or pair in attempted:
                continue
            attempted.add(pair)
            union = groups[left] | groups[right]
            try: hull = ConvexHull(nodes[list(union)])
            except QhullError: continue
            combined = volumes[left]+volumes[right]
            if abs(hull.volume-combined) < max(1e-13, combined*1e-9):
                parent[right] = left
                groups[left] = union
                groups[right] = set()
                volumes[left] = combined
                merges += 1
        if not merges:
            break
    hulls=[]
    for i,indices in enumerate(groups):
        if not indices or root(i)!=i or volumes[i]<=1e-12:continue
        try: hulls.append(trimesh.Trimesh(vertices=nodes[list(indices)],faces=None).convex_hull)
        except QhullError:
            if volumes[i]>1e-10:raise
            discarded_volume+=float(volumes[i])
    audit = {"switches": "pzYQ", "tetrahedra": len(elements), "convex_parts": len(hulls),
             "cad_cavity_markers": cavities,
             "discarded_degenerate_tetra_volume_m3": discarded_volume,
             "tetra_volume_m3": total, "mesh_volume_m3": abs(mesh.volume),
             "merged_volume_m3": sum(abs(hull.volume) for hull in hulls)}
    if abs(audit["merged_volume_m3"]-total) > max(1e-8, total*1e-6):
        raise RuntimeError("convex merging changed the CAD volume")
    return hulls, audit


def boundary_prisms(mesh):
    """Keep the CAD boundary with inward 1 mm shells, merging convex facets."""
    handled=np.zeros(len(mesh.faces),dtype=bool);hulls=[]
    for group in mesh.facets:
        vertices=mesh.vertices[np.unique(mesh.faces[group])]
        normal=mesh.face_normals[int(group[0])]
        if np.max(abs((vertices-vertices[0])@normal))>1e-6:continue
        axis=np.eye(3)[int(np.argmin(abs(normal)))];u=np.cross(normal,axis);u/=np.linalg.norm(u);v=np.cross(normal,u)
        points=np.column_stack((vertices@u,vertices@v));hull=ConvexHull(points)
        area=float(mesh.area_faces[group].sum())
        if abs(hull.volume-area)>max(1e-11,area*1e-6):continue
        polygon=vertices[hull.vertices]
        hulls.append(trimesh.Trimesh(vertices=np.concatenate([polygon,polygon-normal*.001]),faces=None).convex_hull)
        handled[group]=True
    for index in np.flatnonzero(~handled):
        triangle=mesh.triangles[index];normal=mesh.face_normals[index]
        if mesh.area_faces[index]<1e-10:continue
        hulls.append(trimesh.Trimesh(vertices=np.concatenate([triangle,triangle-normal*.001]),faces=None).convex_hull)
    return hulls,{'source_faces':len(mesh.faces),'convex_parts':len(hulls),'inward_shell_thickness_m':.001,
        'boundary_only':True,'solid_interior_not_filled':True}


def bounded_boundary_mesh(original):
    """Reduce dense road triangulation with topology and distance guards.

    Both directions test every vertex plus 1024 face centers using exact
    point/triangle distances. CAD height and tunnel tests remain independent.
    This is a sampled surface test, not a Hausdorff-distance proof.
    """
    if len(original.faces)<=16000:
        return original,None
    for target in (12000,16000,24000):
        if target>=len(original.faces):continue
        candidate=original.simplify_quadric_decimation(face_count=target,aggression=3)
        volume_error=abs(abs(candidate.volume)-abs(original.volume))/max(abs(original.volume),1e-12)
        bounds_error=float(np.max(abs(candidate.bounds-original.bounds)))
        if not candidate.is_watertight or candidate.euler_number!=original.euler_number or volume_error>.001 or bounds_error>.0005:
            continue
        maximum=0.;counts=[]
        for mesh,reference in ((original,candidate),(candidate,original)):
            face_ids=np.linspace(0,len(mesh.faces)-1,min(1024,len(mesh.faces)),dtype=int)
            points=np.concatenate([mesh.vertices,mesh.triangles_center[face_ids]])
            counts.append(len(points))
            for start in range(0,len(points),128):
                _,distance,_=trimesh.proximity.closest_point(reference,points[start:start+128])
                if not np.isfinite(distance).all():
                    maximum=float('inf');break
                maximum=max(maximum,float(distance.max()))
                if maximum>.0015:break
            if maximum>.0015:break
        if maximum>.0015:
            print('Rejected road boundary LOD',target,'distance',maximum,flush=True)
            continue
        audit={'source_faces':len(original.faces),'boundary_faces':len(candidate.faces),
               'all_vertices_and_face_centers_tested':counts,'max_tested_bidirectional_distance_m':maximum,
               'distance_limit_m':.0015,'volume_error_relative':volume_error,'bounds_error_m':bounds_error,
               'closed_and_euler_number_preserved':True,'surface_distance_is_sampled':True}
        print('Bounded terrain boundary simplification',audit,flush=True)
        return candidate,audit
    print('Road boundary simplification rejected; preserving original',flush=True)
    return original,{'reduced':False,'reason':'no candidate passed topology/volume/distance guards'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cad", type=Path, default=ROOT/"build/cad78")
    p.add_argument("--output", type=Path, default=ROOT/"pack")
    p.add_argument("--resume", action="store_true", help="resume this builder's partially generated directory")
    p.add_argument("--visual-only", action="store_true")
    args = p.parse_args()
    args.visual_detail="compact"
    if args.output.exists() and not args.resume:
        p.error("output exists; choose a new directory")
    resource.setrlimit(resource.RLIMIT_AS, (4*1024**3, 4*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    started = time.monotonic()
    source = json.loads((args.cad/"assembly.json").read_text())
    args.output.mkdir(parents=True, exist_ok=args.resume)
    (args.output/"meshes").mkdir(exist_ok=args.resume)
    state_path = args.output/"build_state.json"
    state = {"source_sha256": source["source_sha256"], "policy": "2027_chunked_cad_convex_v1", "parts": {}}
    if state_path.exists():
        previous = json.loads(state_path.read_text())
        if previous["source_sha256"] != state["source_sha256"] or previous["policy"] not in (
                state["policy"], "coacd_1cm_64_seed2027_v1", "coacd_1cm_64_seed2027_prisms_v2", "coacd_1cm_64_seed2027_prisms_v3"):
            raise ValueError("resume source or collision settings differ")
        previous["policy"] = state["policy"]
        state = previous
    # Interrupted legacy runs predate the checkpoint file. Every lower ID is
    # complete; the highest written ID is recomputed, including all its pieces.
    prior_files = defaultdict(list)
    if args.resume and not state_path.exists():
        for path in sorted((args.output/"meshes").glob("rmuc2027_solid_*.msh")):
            match = re.fullmatch(r"rmuc2027_solid_(\d+)_(\d+)\.msh", path.name)
            if match:
                prior_files[int(match[1])].append(path)
    last_prior = max(prior_files, default=-1)
    # The complete CAD is streamed by partition; collision checks use the
    # independent raw per-solid tessellation rather than a full CAD render model.
    global CAD_TO_WORLD
    bounds=np.asarray(source['cad_bounds_mm']).reshape(2,3)
    extents=bounds[1]-bounds[0]
    vertical=int(np.argmin(extents)); long=int(np.argmax(extents)); short=next(i for i in range(3) if i not in (vertical,long))
    CAD_TO_WORLD=np.eye(3)[[long,short,vertical]]
    if np.linalg.det(CAD_TO_WORLD)<0: CAD_TO_WORLD[1]*=-1
    size=extents[[long,short,vertical]]*.001
    if not (25<size[0]<35 and 13<size[1]<20 and 2<size[2]<6):
        raise ValueError(f'Unexpected official CAD dimensions: {size}')
    visual_state_path=args.output/'visual_state.json'
    if args.resume and visual_state_path.exists():
        visual_state=json.loads(visual_state_path.read_text())
        if visual_state['source_sha256']!=source['source_sha256']:raise ValueError('Visual checkpoint source differs')
        full=ET.parse(args.output/'field_visual.xml').getroot();assets=full.find('asset');world=full.find('worldbody')
        origin=np.asarray(visual_state['origin']);error=visual_state['error'];floor_area=visual_state['floor_area']
        visual_face_count=visual_state['visual_faces'];simplification=visual_state['simplification'];cad_face_count=visual_state['cad_faces']
        print('Reused verified-source visual checkpoint',visual_face_count,'faces',flush=True)
    else:
        from visual_builder import build_visuals
        full,assets,world,origin,error,floor_area,visual_face_count,simplification,cad_face_count=build_visuals(
            source,args.cad,args.output,CAD_TO_WORLD,write_msh,numbers,ET)
        write_xml(full,args.output/'field_visual.xml')
        visual_state={'source_sha256':source['source_sha256'],'origin':origin.tolist(),'error':error,'floor_area':floor_area,
            'visual_faces':visual_face_count,'simplification':simplification,'cad_faces':cad_face_count}
        visual_state_path.write_text(json.dumps(visual_state,indent=2)+'\n')
    if args.visual_only:
        root=ET.Element('mujoco',model='RMUC2027_visual_inspection')
        ET.SubElement(root,'compiler',angle='radian')
        v=ET.SubElement(root,'visual');ET.SubElement(v,'global',offwidth='1280',offheight='720')
        ET.SubElement(v,'headlight',ambient='.35 .35 .35',diffuse='.6 .6 .6')
        bg=ET.SubElement(root,'asset');ET.SubElement(bg,'texture',name='rmuc2027_sky',type='skybox',builtin='gradient',rgb1='.16 .21 .29',rgb2='.025 .035 .055',width='64',height='384')
        ET.SubElement(root,'include',file='field_visual.xml')
        wb=ET.SubElement(root,'worldbody');ET.SubElement(wb,'light',pos='0 0 12',dir='0 0 -1',directional='true',castshadow='false')
        write_xml(root,args.output/'scene_visual.xml')
        print('Visual inspection scene ready',flush=True)
        return
    collision = ET.Element("mujoco", model="rmuc2027_collision")
    cassets = ET.SubElement(collision, "asset")
    cworld = ET.SubElement(collision, "worldbody")
    parts = {}
    audit = []
    worker_dir = ROOT/"build/decomposition"
    worker_dir.mkdir(parents=True, exist_ok=True)
    exact_fail_path=ROOT/"build/exact_failures.json"
    exact_failures=json.loads(exact_fail_path.read_text()) if exact_fail_path.exists() else {}
    policy = json.loads((ROOT/"build/geometry_policy.json").read_text())
    if policy["source_sha256"] != source["source_sha256"]: raise ValueError("geometry policy source differs")
    road_cell_ids = set(policy["road_cell_ids"])
    boundary_shell_ids = set(policy["boundary_shell_ids"])
    fine_decomposition_ids = set(policy["fine_decomposition_ids"])
    boundary_terrain_ids = set(source.get("promoted_closed_terrain_ids", []))
    for record in source["unique_solids"]:
        data = np.load(args.cad/record["mesh"])
        mesh = trimesh.Trimesh(vertices=data["vertices_mm"]*.001, faces=data["faces"], process=True)
        volume = record["volume_mm3"]*1e-9
        index = record["id"]
        if index in boundary_shell_ids or mesh.extents.max()>.2:
            mesh.merge_vertices(digits_vertex=6);mesh.update_faces(mesh.nondegenerate_faces());mesh.update_faces(mesh.unique_faces())
            mesh.remove_unreferenced_vertices();mesh.fill_holes();mesh.fix_normals(multibody=False)
        # The arena-wide sloped foundation is a thin, closed CAD solid. Keep
        # its exact boundary rather than asking a volume optimizer to process
        # a 28 m sheet. Select by geometry, never a prior season's part number.
        extents_sorted = np.sort(mesh.extents)
        if extents_sorted[2] > 20 and extents_sorted[1] > 10 and extents_sorted[0] < .5:
            if not mesh.is_watertight:
                raise RuntimeError(f"arena foundation {index} is not closed")
            boundary_terrain_ids.add(index)
        if mesh.extents.max() < .025 or volume < 1e-8:
            parts[index] = []
            audit.append({"id": index, "collision": "omitted small hardware", "volume_m3": volume,
                          "max_extent_m": float(mesh.extents.max())})
            continue
        try:
            hull = mesh.convex_hull
        except Exception as exc:
            raise RuntimeError(f"solid {index} convex hull failed") from exc
        ratio = volume/max(abs(hull.volume), 1e-12)
        threshold = .9999 if mesh.extents.max() > 3 else .985
        if mesh.extents.max() > 3 and len(mesh.faces) > 2000 and ratio < threshold:
            volume_error = abs(abs(mesh.volume)-volume)/max(volume,1e-12)
            if not mesh.is_watertight and volume_error > .002:
                raise RuntimeError(f"large terrain {index} tessellation disagrees with CAD volume: {volume_error}")
            boundary_terrain_ids.add(index)
        facility_proxy = len(mesh.faces)>50000 and mesh.extents.max()<2.5
        decomposition = ratio < threshold and not facility_proxy
        hulls = [hull]
        method = "facility conservative convex hull (high-detail CAD hardware)" if facility_proxy else "convex hull"
        repair = None
        coacd_options = None
        exact_audit = None
        cached = state["parts"].get(str(index))
        if index in boundary_terrain_ids and cached is not None and cached['audit']['collision'] != 'CAD terrain boundary prisms (1 mm inward shell)':
            cached = None
        if index in boundary_terrain_ids and len(mesh.faces)>16000 and cached is not None and 'surface_simplification' not in (cached['audit'].get('exact_decomposition') or {}):
            cached = None
        road_cells=index in road_cell_ids
        refine = index in fine_decomposition_ids
        if road_cells and cached is not None and cached['audit']['collision']!='CAD clipped road cells (120 mm XY)':
            cached=None
        if refine and cached is not None and (cached["audit"].get("coacd_parameters") or {}).get("threshold_m",.01)>.005:
            cached=None
        if cached is not None:
            try:
                json.dumps(cached["audit"],allow_nan=False)
            except ValueError:
                cached=None
                print("Rebuilding nonfinite audit checkpoint",index,flush=True)
        oversized_prisms = cached is not None and cached["audit"]["collision"].startswith("planar") and len(cached["files"]) > 128 and mesh.extents.max() < 2
        oversized_hardware = cached is not None and cached["audit"]["collision"] == "tetgen PLC with convex merging" and len(cached["files"]) > 128 and mesh.extents.max() < .5
        exact_terrain = bool(str(index) not in exact_failures and decomposition and ((mesh.extents.max() > 1 and len(mesh.faces) < 2000) or (ratio < .2 and mesh.extents.max() >= .5 and len(mesh.faces) < 5000)) and mesh.is_watertight)
        requires_exact = exact_terrain and (cached is None or ((index in boundary_shell_ids or (mesh.extents.max()>1 and len(mesh.faces)<2000)) and cached["audit"]["collision"] not in ("tetgen PLC with convex merging","planar triangle prisms, convex adjacent polygons merged")))
        reused = not (oversized_prisms or requires_exact or oversized_hardware) and (cached is not None or (index < last_prior and index in prior_files))
        if oversized_prisms or requires_exact or oversized_hardware:
            cached = None
        if cached and cached["audit"]["collision"] == "coacd" and not cached["audit"].get("coacd_parameters"):
            worker_output = worker_dir/f"output_{index}.npz"
            if worker_output.exists():
                with np.load(worker_output) as checkpoint:
                    if "parameters" in checkpoint.files:
                        cached["audit"]["coacd_parameters"] = json.loads(str(checkpoint["parameters"]))
            if not cached["audit"].get("coacd_parameters"):
                repair_info = cached["audit"].get("repair")
                cached["audit"]["coacd_parameters"] = {"sampling": 1000, "mcts_iterations": 30,
                    "mcts_nodes": 8, "mcts_max_depth": 3, "preprocess_mode": "auto" if repair_info else "off",
                    "preprocess_resolution": repair_info["resolution"] if repair_info else 200}
        if reused:
            paths = [args.output/x for x in cached["files"]] if cached else prior_files[index]
            for path in paths:
                with path.open("rb") as stream:
                    nv, nn, nt, nf = struct.unpack("<4i", stream.read(16))
                if path.stat().st_size != 16+12*(nv+nn+nf)+8*nt or nv < 4:
                    raise ValueError(f"incomplete checkpoint mesh: {path}")
            hulls = [None]*len(paths)
        elif index in boundary_terrain_ids:
            boundary_mesh,reduction=bounded_boundary_mesh(mesh)
            hulls,exact_audit=boundary_prisms(boundary_mesh)
            if reduction is not None:exact_audit['surface_simplification']=reduction
            method='CAD terrain boundary prisms (1 mm inward shell)'
            if not mesh.is_watertight:
                repair={'method':'retain original CAD boundary despite non-manifold tessellation seams',
                        'source_tessellation_volume_error_relative':abs(abs(mesh.volume)-volume)/max(volume,1e-12)}
            print('Retained CAD terrain boundary',index,exact_audit,flush=True)
        elif road_cells:
            hulls,exact_audit=clipped_road_cells(mesh)
            method='CAD clipped road cells (120 mm XY)'
            print('Clipped CAD road cells',index,exact_audit,flush=True)
        elif index in boundary_shell_ids and str(index) in exact_failures:
            hulls,exact_audit=boundary_prisms(mesh)
            method="CAD boundary prisms (1 mm inward shell)"
        elif exact_terrain:
            try:
                hulls=planar_prisms(mesh,volume)
                if hulls is None:
                    hulls, exact_audit = exact_convex_parts(mesh)
                    method = "tetgen PLC with convex merging"
                else:
                    method="planar triangle prisms, convex adjacent polygons merged"
                print("exact terrain solid", index, exact_audit, flush=True)
            except RuntimeError as exc:
                exact_failures[str(index)]=str(exc)
                exact_fail_path.write_text(json.dumps(exact_failures,indent=2)+'\n')
                if index in boundary_shell_ids:
                    hulls,exact_audit=boundary_prisms(mesh)
                    method="CAD boundary prisms (1 mm inward shell)"
                    print("Bounded exact-boundary fallback",index,exact_audit,flush=True)
                else: hulls=None
                print("CAD surface rejected PLC; bounded CoACD fallback",index,str(exc),flush=True)
        elif decomposition:
            hulls = planar_prisms(mesh, volume)
            if hulls is not None:
                method = "planar triangle prisms, convex adjacent polygons merged"
                print("planar CAD solid", index, "prisms", len(hulls), flush=True)
        if not reused and decomposition and hulls is None:
            preprocess = "auto" if str(index) in exact_failures else "off"
            # CAD already gives outward solid normals, including inward normals
            # on cavity shells. Flipping every component would fill those voids.
            mesh.fix_normals(multibody=False)
            if not mesh.is_watertight:
                mesh.merge_vertices(digits_vertex=6)
                mesh.update_faces(mesh.nondegenerate_faces())
                mesh.update_faces(mesh.unique_faces())
                mesh.remove_unreferenced_vertices()
                mesh.fill_holes()
                mesh.fix_normals(multibody=False)
            if not mesh.is_watertight:
                relative_volume_error = abs(abs(mesh.volume)-volume)/max(volume, 1e-12)
                if relative_volume_error > .02:
                    raise RuntimeError(f"nonconvex solid {index} has an unclosed CAD volume ({relative_volume_error:.3%})")
                preprocess = "auto"
                repair = {"method": "coacd manifold preprocessing", "resolution": 200,
                          "source_tessellation_volume_error_relative": relative_volume_error}
            print("decomposing CAD solid", index, "ratio", round(ratio, 4), "extent", mesh.extents.tolist(), flush=True)
            input_file = worker_dir/f"input_{index}.npz"
            output_file = worker_dir/f"output_{index}.npz"
            np.savez(input_file, vertices=np.asarray(mesh.vertices), faces=np.asarray(mesh.faces, dtype=np.int32))
            gc.collect()
            available_mb = int(next(line.split()[1] for line in Path("/proc/meminfo").read_text().splitlines()
                                    if line.startswith("MemAvailable:")))/1024
            if available_mb < 3500:
                raise RuntimeError(f"RAM guard before worker: {available_mb:.0f} MiB; resume later")
            resolutions = (50, 25) if mesh.extents.max() < .25 else ((100, 50) if mesh.extents.max() < 1.0 else (200, 100))
            if refine:resolutions=(128,100)
            for preprocess_resolution in resolutions:
                command = [sys.executable, str(ROOT/"decompose_worker.py"), str(input_file), str(output_file),
                           "--preprocess", preprocess, "--preprocess-resolution", str(preprocess_resolution)]
                if refine:
                    command.extend(["--threshold",".005","--max-hulls","128","--sampling","750","--iterations","8","--fine-preprocess"])
                try:
                    result = subprocess.run(command, timeout=240 if refine else 90)
                except subprocess.TimeoutExpired:
                    print("CoACD worker time guard; retrying", index, flush=True)
                    continue
                if result.returncode == 0:
                    if repair:
                        repair["resolution"] = preprocess_resolution
                    break
                if preprocess == "off":
                    preprocess="auto"
                    repair={"method":"coacd manifold preprocessing after native input rejection", "resolution":preprocess_resolution}
            else:
                raise RuntimeError(f"bounded CoACD preprocessing failed: solid {index}")
            with np.load(output_file) as output:
                if "parameters" in output.files:
                    coacd_options = json.loads(str(output["parameters"]))
                    if repair:repair["resolution"]=coacd_options["preprocess_resolution"]
                hulls = [trimesh.Trimesh(vertices=output[f"v{k}"], faces=output[f"f{k}"], process=False)
                         for k in range(len(output.files)//2)]
            method = "coacd"
        names = []
        for k, hull in enumerate(hulls):
            name = f"rmuc2027_solid_{index:04d}_{k:02d}"
            file = f"meshes/{name}.msh"
            if not reused:
                write_msh(hull, args.output/file)
            ET.SubElement(cassets, "mesh", name=name, file=file, inertia="convex")
            names.append(name)
        parts[index] = names
        audit.append({"id": index, "collision": cached["audit"]["collision"] if cached else ("coacd" if reused and decomposition else method),
                      "source_volume_m3": volume, "source_hull_volume_ratio": ratio,
                      "pieces": len(names), "mesh_watertight": bool(mesh.is_watertight),
                      "repair": cached["audit"].get("repair") if cached else repair,
                      "coacd_parameters": cached["audit"].get("coacd_parameters") if cached else coacd_options,
                      "exact_decomposition": cached["audit"].get("exact_decomposition") if cached else exact_audit})
        state["parts"][str(index)] = {"files": [f"meshes/{name}.msh" for name in names], "audit": audit[-1]}
        state_path.write_text(json.dumps(state)+"\n")
        if len(audit) % 25 == 0:
            print("collision solids processed", len(audit), "/", len(source["unique_solids"]), flush=True)
        gc.collect()
    facility_ids={r["id"] for r in audit if r["collision"].startswith("facility conservative")}
    for i, instance in enumerate(source["instances"]):
        pose = np.asarray(instance["cad_transform_mm"])
        rotation = CAD_TO_WORLD@pose[:3, :3]
        quat = Rotation.from_matrix(rotation).as_quat()[[3, 0, 1, 2]]
        position = CAD_TO_WORLD@pose[:3, 3]*.001-origin
        for k, name in enumerate(parts[instance["solid_id"]]):
            ET.SubElement(cworld, "geom", name=f"rmuc2027_{'facility_' if instance['solid_id'] in facility_ids else ''}contact_{i:05d}_{k:02d}", type="mesh", mesh=name,
                          pos=numbers(position), quat=numbers(quat), contype="4", conaffinity="3", group="3",
                          density="0", condim="3", friction="1.2 .02 .001", solref=".008 1",
                          solimp=".92 .99 .002", rgba=".5 .55 .6 1")
    surface_audit=[];surface_parts={}
    for record in source.get('surface_only_shells',[]):
        with np.load(args.cad/record['mesh']) as d:
            mesh=trimesh.Trimesh(vertices=d['vertices_mm']*.001,faces=d['faces'],process=True)
        index=record['id']
        if mesh.area<.01 or mesh.extents.max()<.05:continue
        vertices=np.asarray(mesh.vertices)
        try: hull=mesh.convex_hull
        except Exception:
            normal=mesh.face_normals[int(np.argmax(mesh.area_faces))]
            hull=trimesh.Trimesh(vertices=np.concatenate([vertices+normal*.0005,vertices-normal*.0005]),faces=None).convex_hull
        if abs(hull.volume)<1e-12:continue
        name=f'rmuc2027_surface_{index:04d}';filename=f'meshes/{name}.msh';write_msh(hull,args.output/filename)
        ET.SubElement(cassets,'mesh',name=name,file=filename,inertia='convex');surface_parts[index]=name
        surface_audit.append({'id':index,'area_m2':float(mesh.area),'extents_m':mesh.extents.tolist(),'proxy':'conservative hull of source open shell; facility hardware, not terrain'})
    for i,instance in enumerate(source.get('surface_instances',[])):
        name=surface_parts.get(instance['surface_id'])
        if not name:continue
        pose=np.asarray(instance['cad_transform_mm']);rotation=CAD_TO_WORLD@pose[:3,:3]
        quat=Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]];position=CAD_TO_WORLD@pose[:3,3]*.001-origin
        ET.SubElement(cworld,'geom',name=f'rmuc2027_surface_contact_{i:05d}',type='mesh',mesh=name,pos=numbers(position),quat=numbers(quat),
            contype='4',conaffinity='3',group='3',density='0',condim='3',friction='1.2 .02 .001',solref='.008 1',solimp='.92 .99 .002',rgba='.5 .55 .6 1')
    # Full and collision-only entrypoints contain exactly the same contact geoms.
    write_xml(collision, args.output/"field_collision.xml")
    for a in cassets:
        assets.append(a)
    for g in cworld:
        visual_collision = copy.deepcopy(g)
        visual_collision.set("rgba", "0 0 0 0")
        world.append(visual_collision)
    write_xml(full, args.output/"field.xml")
    reference_path = args.output/"field_cad_reference.xml"
    if reference_path.exists():
        reference = ET.parse(reference_path).getroot()
        rassets, rworld = reference.find("asset"), reference.find("worldbody")
        for asset in list(rassets):
            if asset.get("name", "").startswith("rmuc2027_solid_"):
                rassets.remove(asset)
        for geom in list(rworld):
            if geom.get("name", "").startswith("rmuc2027_contact_"):
                rworld.remove(geom)
        for asset in cassets:
            rassets.append(copy.deepcopy(asset))
        for geom in cworld:
            item = copy.deepcopy(geom); item.set("rgba", "0 0 0 0"); rworld.append(item)
        write_xml(reference, reference_path)
    perimeter = ET.Element("mujoco", model="optional_rules_perimeter")
    pworld = ET.SubElement(perimeter, "worldbody")
    for name, position, size in (("west", (-14.025, 0, 1.2), (.025, 7.5, 1.2)),
                                 ("east", (14.025, 0, 1.2), (.025, 7.5, 1.2)),
                                 ("south", (0, -7.525, 1.2), (14.05, .025, 1.2)),
                                 ("north", (0, 7.525, 1.2), (14.05, .025, 1.2))):
        ET.SubElement(pworld, "geom", name=f"rmuc2027_perimeter_{name}", type="box",
                      pos=numbers(position), size=numbers(size), density="0", group="4",
                      contype="4", conaffinity="3", friction="1.2 .02 .001", rgba=".15 .18 .22 .2")
    write_xml(perimeter, args.output/"field_perimeter.xml")
    for profile, entry in (("full", "field.xml"), ("collision", "field_collision.xml")):
        root = ET.Element("mujoco", model=f"RMUC2027_{profile}")
        ET.SubElement(root, "compiler", angle="radian")
        ET.SubElement(root, "option", timestep=".001", gravity="0 0 -9.81", integrator="implicitfast")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="1280", offheight="720")
        ET.SubElement(visual, "headlight", ambient=".35 .35 .35", diffuse=".6 .6 .6", specular=".1 .1 .1")
        background = ET.SubElement(root, "asset")
        ET.SubElement(background, "texture", name="rmuc2027_sky", type="skybox", builtin="gradient",
                      rgb1=".16 .21 .29", rgb2=".025 .035 .055", width="64", height="384")
        ET.SubElement(root, "include", file=entry)
        wb = ET.SubElement(root, "worldbody")
        ET.SubElement(wb, "light", pos="0 0 12", dir="0 0 -1", directional="true", castshadow="false", diffuse=".5 .5 .5")
        write_xml(root, args.output/("scene.xml" if profile == "full" else "scene_collision.xml"))
        ET.SubElement(root, "include", file="field_perimeter.xml")
        write_xml(root, args.output/("scene_bounded.xml" if profile == "full" else "scene_collision_bounded.xml"))
    manifest = {"source": {"page": "https://bbs.robomaster.com/article/1970956?source=4", "sha256": source["source_sha256"],
                           "version": "RMUC2027 V1.0.0", "units": "m"},
                "official_core_size_m": [28, 15], "official_perimeter_height_m": 2.4,
                "cad_bounds_mm": source["cad_bounds_mm"], "world_origin_before_shift_m": origin.tolist(),
                "cad_to_world_rotation": CAD_TO_WORLD.tolist(), "glb_bounds_error_m": error,
                "dominant_floor_upward_area_m2": floor_area,
                "collision_instances": len(cworld), "collision_parts": sum(len(x) for x in parts.values()),
                "unique_cad_solids": len(source["unique_solids"]), "cad_solid_instances": source["solid_count"],
                "visual_detail": args.visual_detail, "visual_faces": visual_face_count,
                "visual_simplification": simplification,
                "cad_visual_faces": cad_face_count, "floor_color": "Official CAD colors and source marking/text geometry retained; visual meshes simplified by material",
                "optional_perimeter": "rules-based 28x15m inner rectangle, 2.4m high, 50mm assumed thickness; transparent for inspection; entrances not modeled",
                "collision_policy": "individual CAD solids; convex decomposition for concave parts and exact triangulated extrusion prisms for thin plates; no whole-map hull/heightfield",
                "coacd": {"threshold_m": .01, "max_hulls": 64, "seed": 2027},
                "exact_terrain_solids": [r["id"] for r in audit if r["collision"] == "tetgen PLC with convex merging"],
                "exact_decomposition_input_rejections": exact_failures,
                "friction": {"values": [1.2, .02, .001], "measured_official_material": False,
                             "meaning": "RL-lab baseline, editable by caller"},
                "scope": "static CAD snapshot; referee logic and moving mechanisms are not simulated",
                "geometry_policy": policy, "promoted_closed_terrain_ids":source.get("promoted_closed_terrain_ids", []),
                "boundary_terrain_ids":sorted(boundary_terrain_ids),
                "solid_collision_audit": audit, "closed_facility_proxy_ids": sorted(facility_ids), "open_surface_facility_proxies": surface_audit, "cad_surface_shell_instances": len(source.get("surface_instances", [])),
                "wall_seconds": time.monotonic()-started, "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                "files": {}}
    for path in sorted(args.output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            manifest["files"][str(path.relative_to(args.output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print("field ready", args.output, "contact geoms", len(cworld), "RSS MiB", manifest["peak_rss_mb"], flush=True)


if __name__ == "__main__":
    main()
