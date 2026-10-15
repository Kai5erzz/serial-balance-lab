"""Package the original C tree and its fingerprint for vehicle-project integration."""
import argparse
from pathlib import Path
import sys
import zipfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from balance_lab.firmware.build import FIRMWARE, build

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=Path("out/vehicle_controller.zip"))
    args=parser.parse_args()
    build()  # validate/import the authoritative current worktree first
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(args.output,"w",compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted((FIRMWARE/"vehicle").rglob("*")):
            if path.is_file(): archive.write(path,path.relative_to(FIRMWARE/"vehicle"))
        archive.write(FIRMWARE/"manifest.json","firmware_manifest.json")
        guide=FIRMWARE.parent/"docs/SHARED_VEHICLE_CONTROLLER.md"
        # Preserve repository-relative documentation links in docs/, while
        # keeping the original top-level integration-guide entry point.
        archive.write(guide,"docs/SHARED_VEHICLE_CONTROLLER.md")
        top_level_guide=guide.read_text()
        for name in ("NATIVE_FRICTION_AUDIT_20261014.md", "NATIVE_POLARITY_AUDIT_20261014.md", "NATIVE_WHEEL_HANDOFF_AUDIT_20261014.md"):
            top_level_guide=top_level_guide.replace(f"({name})",f"(docs/{name})")
            archive.write(FIRMWARE.parent/"docs"/name,"docs/"+name)
        archive.writestr("SHARED_VEHICLE_CONTROLLER.md",top_level_guide)
        figure=FIRMWARE.parent/"out/native/friction_audit/wheel_damping_comparison.png"
        if figure.is_file():
            archive.write(figure,"out/native/friction_audit/wheel_damping_comparison.png")
        yaw_figure=FIRMWARE.parent/"out/native/yaw_follow_audit_20261014/yaw_follow_comparison.png"
        if yaw_figure.is_file():
            archive.write(yaw_figure,"out/native/yaw_follow_audit_20261014/yaw_follow_comparison.png")
        coherent_figure=FIRMWARE.parent/"out/native/yaw_follow_coherent_20261014/yaw_follow_coherent_comparison.png"
        if coherent_figure.is_file():
            archive.write(coherent_figure,"out/native/yaw_follow_coherent_20261014/yaw_follow_coherent_comparison.png")
    print(args.output.resolve())
