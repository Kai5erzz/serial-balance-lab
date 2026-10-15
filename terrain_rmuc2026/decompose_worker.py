"""One bounded CoACD job; invoked sequentially inside the heavy_run parent."""
from pathlib import Path
import argparse
import json
import resource
import sys

resource.setrlimit(resource.RLIMIT_AS, (int(2.5*1024**3), int(2.5*1024**3)))
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
import numpy as np
sys.path.append(str(Path(__file__).resolve().parent.parent/"terrain_rmuc2025/.caddeps"))
import coacd
import trimesh

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("input", type=Path)
p.add_argument("output", type=Path)
p.add_argument("--preprocess", choices=("off", "auto"), required=True)
p.add_argument("--preprocess-resolution", type=int, default=200)
p.add_argument("--sampling", type=int, default=500)
p.add_argument("--iterations", type=int, default=8)
p.add_argument("--threshold",type=float,default=.01)
p.add_argument("--max-hulls",type=int,default=64)
p.add_argument("--fine-preprocess",action="store_true")
a = p.parse_args()
data = np.load(a.input)
original=trimesh.Trimesh(vertices=data["vertices"],faces=data["faces"],process=False)
working=original
if a.preprocess=="auto" and not a.fine_preprocess and original.extents.max()<=1 and a.preprocess_resolution>100:
    a.preprocess_resolution=100
    print("Bounded hardware preprocessing resolution",100,flush=True)
reduction=None
if len(original.faces)>5000:
    for target in (5000,10000,20000,40000):
        if target>=len(original.faces):continue
        candidate=original.simplify_quadric_decimation(face_count=target,aggression=5)
        volume_error=abs(abs(candidate.volume)-abs(original.volume))/max(abs(original.volume),1e-12)
        bounds_error=float(np.max(abs(candidate.bounds-original.bounds)))
        if (a.preprocess=="off" and not candidate.is_watertight) or volume_error>.03 or bounds_error>.002:
            print("Rejected collision LOD",target,"watertight",candidate.is_watertight,"volume",volume_error,"bounds",bounds_error,flush=True)
            continue
        distances=[]
        for mesh,reference in ((original,candidate),(candidate,original)):
            reference=reference.copy()
            reference.update_faces(reference.nondegenerate_faces(height=1e-8))
            reference.remove_unreferenced_vertices()
            points=mesh.vertices[np.linspace(0,len(mesh.vertices)-1,min(256,len(mesh.vertices)),dtype=int)]
            for start in range(0,len(points),8):
                _,distance,_=trimesh.proximity.closest_point_naive(reference,points[start:start+8])
                distances.extend(distance.tolist())
        if not np.isfinite(distances).all() or max(distances,default=0)>.003:continue
        working=candidate
        reduction={"source_faces":len(original.faces),"input_faces":len(candidate.faces),
            "sampled_bidirectional_max_distance_m":max(distances),"volume_error_relative":volume_error,"bounds_error_m":bounds_error,"samples_each_direction":256}
        print("Bounded collision input simplification",reduction,flush=True)
        break
print("CoACD input",len(working.vertices),"vertices",len(working.faces),"faces",flush=True)
coacd.set_log_level("info" if a.max_hulls>64 else "error")
hulls = coacd.run_coacd(coacd.Mesh(np.asarray(working.vertices), np.asarray(working.faces,dtype=np.int32)),
                       threshold=a.threshold, max_convex_hull=a.max_hulls, preprocess_mode=a.preprocess,
                       preprocess_resolution=a.preprocess_resolution, resolution=a.sampling,
                       mcts_nodes=8, mcts_iterations=a.iterations, mcts_max_depth=3, merge=True,
                       decimate=True, max_ch_vertex=96, seed=2026, real_metric=True)
arrays = {}
for i, (vertices, faces) in enumerate(hulls):
    arrays[f"v{i}"] = vertices
    arrays[f"f{i}"] = faces
if not arrays:
    raise RuntimeError("CoACD returned no collision pieces")
arrays["parameters"] = np.asarray(json.dumps({"sampling": a.sampling, "mcts_iterations": a.iterations,
    "threshold_m":a.threshold,"max_hulls":a.max_hulls,
    "mcts_nodes": 8, "mcts_max_depth": 3, "preprocess_mode": a.preprocess,
    "preprocess_resolution": a.preprocess_resolution, "input_simplification": reduction},allow_nan=False))
np.savez_compressed(a.output, **arrays)
print("worker pieces", len(hulls), "peak MiB", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024, flush=True)
