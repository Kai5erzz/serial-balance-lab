"""Export the pinned official 2025 CAD, one independent solid at a time.

Use run_heavy.sh; this stage has a 5.5 GiB address-space ceiling. Source units
are millimetres. It produces exact placed-solid records and CAD-colored glTF;
the next stage builds separate visual and collision resources in metres.
"""
from pathlib import Path
import argparse
import hashlib
import json
import resource
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.append(str(ROOT / ".caddeps78"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=ROOT/"research/rmuc2025_national_v2.stp")
    p.add_argument("--output", type=Path, default=ROOT/"build/cad78")
    args = p.parse_args()
    if args.output.exists():
        p.error("output exists; choose a new directory")
    resource.setrlimit(resource.RLIMIT_AS, (int(5.5*1024**3), int(5.5*1024**3)))
    import numpy as np
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString, TCollection_AsciiString
    try:
        from OCP.TColStd import TColStd_IndexedDataMapOfStringString
    except ImportError:  # OpenCascade 8 renamed template collections.
        from OCP.collections import IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString as TColStd_IndexedDataMapOfStringString
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    started = time.monotonic()
    digest = hashlib.sha256()
    with args.source.open("rb") as stream:
        while b := stream.read(1024*1024):
            digest.update(b)
    expected = "2655d4c2caad62d35cae989b6240ce7bba0172bcb675a9af3866100ace384e1c"
    if digest.hexdigest() != expected:
        raise ValueError("official STEP SHA256 differs")
    args.output.mkdir(parents=True)
    meshes = args.output/"solids"
    meshes.mkdir()
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    for setter in ("SetNameMode", "SetLayerMode", "SetPropsMode", "SetGDTMode", "SetMatMode", "SetSHUOMode", "SetViewMode"):
        getattr(reader, setter)(False)
    print("Reading verified official STEP", flush=True)
    if reader.ReadFile(str(args.source)) != IFSelect_RetDone:
        raise RuntimeError("STEP read failed")
    print("Transferring assembly; RSS MiB", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024, flush=True)
    if not reader.Transfer(doc):
        raise RuntimeError("STEP assembly transfer failed")
    shape = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).GetOneShape()
    bounds = Bnd_Box()
    BRepBndLib.Add_s(shape, bounds, False)
    print("CAD bounds mm", bounds.Get(), "RSS MiB", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024, flush=True)
    mesher = BRepMesh_IncrementalMesh(shape, 2.0, False, .4, False)
    if not mesher.IsDone():
        raise RuntimeError("CAD meshing failed")
    print("Exporting colored visual glTF", flush=True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(str(args.output/"official_visual.glb")), True)
    writer.SetParallel(False)
    writer.SetMergeFaces(True)
    if not writer.Perform(doc, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange()):
        raise RuntimeError("glTF export failed")
    unique = {}
    records = []
    instances = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        placed = explorer.Current()
        loc = placed.Location().Transformation()
        transform = [[loc.Value(i, j) for j in range(1, 5)] for i in range(1, 4)] + [[0, 0, 0, 1]]
        solid = placed.Located(TopLoc_Location())
        key = hash(solid)
        if key not in unique:
            index = len(records)
            unique[key] = index
            vertices, faces = [], []
            face_exp = TopExp_Explorer(solid, TopAbs_FACE)
            while face_exp.More():
                face = (TopoDS.Face_s(face_exp.Current()) if hasattr(TopoDS, "Face_s")
                        else TopoDS.Face(face_exp.Current()))
                face_loc = TopLoc_Location()
                triangulation = BRep_Tool.Triangulation_s(face, face_loc)
                if triangulation is not None:
                    start = len(vertices)
                    for k in range(1, triangulation.NbNodes()+1):
                        point = triangulation.Node(k).Transformed(face_loc.Transformation())
                        vertices.append((point.X(), point.Y(), point.Z()))
                    for k in range(1, triangulation.NbTriangles()+1):
                        tri = list(triangulation.Triangle(k).Get())
                        if face.Orientation() == TopAbs_REVERSED:
                            tri[1], tri[2] = tri[2], tri[1]
                        faces.append([start+x-1 for x in tri])
                face_exp.Next()
            if not vertices or not faces:
                raise RuntimeError(f"solid {index} has no tessellation")
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(solid, props)
            path = meshes/f"solid_{index:04d}.npz"
            np.savez_compressed(path, vertices_mm=np.asarray(vertices), faces=np.asarray(faces, dtype=np.int32))
            records.append({"id": index, "mesh": str(path.relative_to(args.output)),
                            "volume_mm3": abs(float(props.Mass())), "triangles": len(faces)})
        instances.append({"solid_id": unique[key], "cad_transform_mm": transform})
        explorer.Next()
        if len(instances) % 1000 == 0:
            print("solid instances", len(instances), "unique", len(records), flush=True)
    report = {"source_sha256": expected, "source_units": "mm", "cad_bounds_mm": list(bounds.Get()),
              "visual_glb": "official_visual.glb", "solid_count": len(instances), "unique_solids": records,
              "instances": instances, "linear_deflection_mm": 2.0, "angular_deflection_rad": .4,
              "wall_seconds": time.monotonic()-started,
              "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
              "address_space_limit_gib": 5.5}
    (args.output/"assembly.json").write_text(json.dumps(report, indent=2)+"\n")
    print("CAD export complete", len(instances), "solids", len(records), "unique; RSS MiB", report["peak_rss_mb"], flush=True)


if __name__ == "__main__":
    main()
