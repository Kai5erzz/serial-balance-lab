"""One bounded CoACD job; invoked sequentially inside the heavy_run parent."""
from pathlib import Path
import argparse
import json
import resource
import sys

resource.setrlimit(resource.RLIMIT_AS, (int(2.5*1024**3), int(2.5*1024**3)))
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
import numpy as np
sys.path.append(str(Path(__file__).resolve().parent/".caddeps"))
import coacd

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("input", type=Path)
p.add_argument("output", type=Path)
p.add_argument("--preprocess", choices=("off", "auto"), required=True)
p.add_argument("--preprocess-resolution", type=int, default=200)
p.add_argument("--sampling", type=int, default=500)
p.add_argument("--iterations", type=int, default=8)
a = p.parse_args()
data = np.load(a.input)
coacd.set_log_level("error")
hulls = coacd.run_coacd(coacd.Mesh(data["vertices"], data["faces"].astype(np.int32)),
                       threshold=.01, max_convex_hull=64, preprocess_mode=a.preprocess,
                       preprocess_resolution=a.preprocess_resolution, resolution=a.sampling,
                       mcts_nodes=8, mcts_iterations=a.iterations, mcts_max_depth=3, merge=True,
                       decimate=True, max_ch_vertex=96, seed=2025, real_metric=True)
arrays = {}
for i, (vertices, faces) in enumerate(hulls):
    arrays[f"v{i}"] = vertices
    arrays[f"f{i}"] = faces
if not arrays:
    raise RuntimeError("CoACD returned no collision pieces")
arrays["parameters"] = np.asarray(json.dumps({"sampling": a.sampling, "mcts_iterations": a.iterations,
    "mcts_nodes": 8, "mcts_max_depth": 3, "preprocess_mode": a.preprocess,
    "preprocess_resolution": a.preprocess_resolution}))
np.savez_compressed(a.output, **arrays)
print("worker pieces", len(hulls), "peak MiB", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024, flush=True)
