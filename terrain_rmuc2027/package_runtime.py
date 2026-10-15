"""Stream a portable runtime archive; run through the shared heavy_run."""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parent
PACK = ROOT/"pack"
output = PACK.resolve().parent/"rmuc2027_mujoco.zip"
if output.exists():
    raise FileExistsError(output)
manifest = json.loads((PACK/"manifest.json").read_text())
runtime = {path.relative_to(PACK).as_posix() for path in PACK.glob("*.xml")}
for relative in list(runtime):
    root=ET.parse(PACK/relative).getroot()
    for include in root.iter('include'):
        target=(PACK/relative).parent/include.get('file')
        if Path(include.get('file')).is_absolute() or target.resolve().relative_to(PACK.resolve()).as_posix() not in runtime:
            raise AssertionError(f'nonportable include: {relative}: {include.attrib}')
    for asset in root.findall("asset/*"):
        if "file" in asset.attrib:
            if Path(asset.get('file')).is_absolute():
                raise AssertionError(f'absolute runtime asset: {relative}: {asset.attrib}')
            path = (PACK/asset.get("file")).resolve()
            runtime.add(path.relative_to(PACK.resolve()).as_posix())
packaged_manifest = copy.deepcopy(manifest)
packaged_manifest["files"] = {relative: manifest["files"][relative] for relative in sorted(runtime)}
packaged_manifest["packaging"] = "Runtime assets only; original STEP and conversion dependencies stay in the source workspace."
def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(128*1024):
            result.update(chunk)
    return result.hexdigest()
for relative, expected in packaged_manifest["files"].items():
    if digest(PACK/relative) != expected:
        raise AssertionError(f"runtime asset changed: {relative}")
with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=5) as archive:
    for relative in sorted(runtime):
        with (PACK/relative).open("rb") as source, archive.open("terrain_rmuc2027/pack/"+relative, "w") as target:
            shutil.copyfileobj(source, target, length=128*1024)
    archive.writestr("terrain_rmuc2027/pack/manifest.json", json.dumps(packaged_manifest, indent=2)+"\n")
    extras = [ROOT/"README.md", ROOT/"run_heavy.sh", *ROOT.glob("*.py"),
              *sorted((ROOT/"preview").glob("*")), ROOT/"checks/report.json", ROOT/"checks/spawns.json",
              ROOT/"checks/height_rays.json", ROOT/"research/source.json", ROOT/"research/rules_source.json"]
    extras.append(ROOT/'checks/refined_raw_convex_rays.json')
    extras.extend([ROOT/'checks/passages.json', ROOT/'checks/raw_cad_passages.json',
                   ROOT/'research/reference_geometry_policies.json'])
    extras.extend([ROOT/'research/terrain_promotion.json', ROOT/'research/geometry_policy.json'])
    for path in extras:
        if path.is_file():
            with path.open("rb") as source, archive.open("terrain_rmuc2027/"+path.relative_to(ROOT).as_posix(), "w") as target:
                shutil.copyfileobj(source, target, length=128*1024)
    for info in archive.infolist():
        info.create_system = 3
        info.external_attr = (0o100755 if info.filename.endswith("/run_heavy.sh") else 0o100644) << 16
with zipfile.ZipFile(output) as archive:
    invalid = archive.testzip()
    if invalid:
        raise AssertionError(f"archive CRC failed: {invalid}")
(ROOT/output.name).symlink_to(output)
receipt = {"archive": str(ROOT/output.name), "stored_at": str(output), "bytes": output.stat().st_size, "sha256": digest(output),
           "runtime_files_verified": len(runtime), "crc_verified": True,
           "relative_runtime_paths_verified":True,
           "contains_original_step_or_dependencies": False}
(ROOT/"package_receipt.json").write_text(json.dumps(receipt, indent=2)+"\n")
print(receipt, flush=True)
