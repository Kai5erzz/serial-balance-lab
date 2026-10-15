"""Retain large closed terrain delivered as STEP shells, not solid entities."""
from pathlib import Path
import json
import resource
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
resource.setrlimit(resource.RLIMIT_AS, (1024**3,) * 2)
cad = ROOT/'build/cad78'
path = cad/'assembly.json'
source = json.loads(path.read_text())
receipt_path = cad/'promoted_terrain.json'
if receipt_path.exists():
    receipt = json.loads(receipt_path.read_text())
    assert receipt['source_sha256'] == source['source_sha256']
    assert source.get('promoted_closed_terrain_ids') == receipt['solid_ids']
else:
    promoted = []
    for record in source['surface_only_shells']:
        with np.load(cad/record['mesh']) as data:
            mesh = trimesh.Trimesh(vertices=data['vertices_mm']*.001, faces=data['faces'], process=True)
        if mesh.extents.max() <= 3:
            continue
        if not mesh.is_watertight:
            raise RuntimeError(f'Large open architecture needs explicit collision treatment: {record["id"]}')
        index = len(source['unique_solids'])
        source['unique_solids'].append({**record, 'id':index, 'volume_mm3':abs(float(mesh.volume))*1e9,
                                       'original_shell_id':record['id'], 'volume_source':'closed CAD tessellation'})
        for instance in source['surface_instances']:
            if instance['surface_id'] == record['id']:
                source['instances'].append({'solid_id':index, 'cad_transform_mm':instance['cad_transform_mm']})
        promoted.append({'original_shell_id':record['id'], 'solid_id':index, 'triangles':len(mesh.faces),
                         'area_m2':float(mesh.area), 'volume_m3':abs(float(mesh.volume)),
                         'extents_m':mesh.extents.tolist(), 'collision':'inward 1 mm boundary prisms, no filled gaps'})
    removed = {row['original_shell_id'] for row in promoted}
    source['surface_only_shells'] = [r for r in source['surface_only_shells'] if r['id'] not in removed]
    source['surface_instances'] = [r for r in source['surface_instances'] if r['surface_id'] not in removed]
    source['solid_count'] = len(source['instances'])
    source['promoted_closed_terrain_ids'] = [r['solid_id'] for r in promoted]
    receipt = {'source_sha256':source['source_sha256'], 'solid_ids':source['promoted_closed_terrain_ids'],
               'reason':'large watertight CAD road shells must be terrain, never facility hull proxies',
               'parts':promoted}
    original = cad/'assembly_before_terrain_promotion.json'
    if original.exists():raise FileExistsError(original)
    original.write_text(path.read_text())
    path.write_text(json.dumps(source,indent=2)+'\n')
    receipt_path.write_text(json.dumps(receipt,indent=2)+'\n')
print('Closed terrain shell promotion:',json.dumps(receipt),flush=True)
