from pathlib import Path
import json,numpy as np,trimesh
ROOT=Path(__file__).resolve().parent;cad=ROOT/'build/cad78'
s=json.loads((cad/'assembly.json').read_text());rows=[]
for r in s['surface_only_shells']:
 d=np.load(cad/r['mesh']);m=trimesh.Trimesh(vertices=d['vertices_mm']*.001,faces=d['faces'],process=True)
 rows.append({'id':r['id'],'faces':len(m.faces),'area_m2':float(m.area),'extents_m':m.extents.tolist(),'watertight':bool(m.is_watertight),'volume_m3':float(m.volume),'bounds_m':m.bounds.tolist()})
report={'unique_open_shells':len(rows),'placed_open_shells':len(s['surface_instances']),'total_area_m2':sum(r['area_m2'] for r in rows),'large_area_count':sum(r['area_m2']>.05 for r in rows),'largest':sorted(rows,key=lambda r:r['area_m2'],reverse=True)[:25],'all':rows}
(ROOT/'build/surface_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='all'},indent=2))
