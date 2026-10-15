"""Match exceptional collision policies by geometry, never by another year's IDs.

The 2026 converter needed local road clipping and an inward boundary shell for
specific CAD solids. A new STEP assembly has unrelated solid numbers. Transfer
those policies only when vertices, faces and exact CAD volume match byte for
byte after export through the same CAD kernel. Everything else takes the
generic per-solid decomposition path and is checked against independent rays.
"""
from pathlib import Path
import hashlib
import json
import resource
import numpy as np

ROOT = Path(__file__).resolve().parent


def signature(directory, record):
    digest = hashlib.sha256()
    with np.load(directory / record["mesh"]) as mesh:
        for name, dtype in (("vertices_mm", "<f8"), ("faces", "<i4")):
            values = np.asarray(mesh[name], dtype=dtype)
            digest.update(str(values.shape).encode("ascii"))
            for start in range(0, len(values), 8192):
                digest.update(np.ascontiguousarray(values[start:start + 8192]).tobytes())
    digest.update(float(record["volume_mm3"]).hex().encode("ascii"))
    return digest.hexdigest()


def main():
    resource.setrlimit(resource.RLIMIT_AS, (1024**3,) * 2)
    cad = ROOT / "build/cad78"
    source = json.loads((cad / "assembly.json").read_text())
    policy_ids = {"road_cell_ids": {153, 181}, "boundary_shell_ids": {420},
                  "fine_decomposition_ids": {616, 780}}
    seed_path = ROOT / "research/reference_geometry_policies.json"
    if seed_path.exists():
        seed = json.loads(seed_path.read_text())
    else:
        reference = ROOT.parent / "terrain_rmuc2026/build/cad78"
        old = json.loads((reference / "assembly.json").read_text())
        special = set.union(*policy_ids.values())
        seed = {"source_sha256": old["source_sha256"],
                "signatures": {signature(reference, record): record["id"]
                               for record in old["unique_solids"] if record["id"] in special}}
        seed_path.write_text(json.dumps(seed, indent=2) + "\n")
    signatures = seed["signatures"]
    matches = []
    for record in source["unique_solids"]:
        key = signature(cad, record)
        if key in signatures:
            matches.append({"new_id": record["id"], "reference_id": signatures[key],
                            "geometry_sha256": key})
    report = {"source_sha256": source["source_sha256"],
              "reference_source_sha256": seed["source_sha256"],
              "method": "exact ordered vertices/faces/volume SHA-256; no year-specific ID assumptions",
              "matches": matches}
    for name, ids in policy_ids.items():
        report[name] = [match["new_id"] for match in matches if match["reference_id"] in ids]
    (ROOT / "build/geometry_policy.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Geometry policy:", json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
