"""Export source partitions sequentially; reuse completed exports on resume."""
from pathlib import Path
import json,subprocess,sys,resource,time,os
ROOT=Path(__file__).resolve().parent
started=time.monotonic();info=json.loads((ROOT/'build/step_chunks/chunks.json').read_text())
chunks=ROOT/'build/cad_chunks';chunks.mkdir(exist_ok=True)
records=[];instances=[];bounds=[];visuals=[];chunk_reports=[];surfaces=[];surface_instances=[]
for i,row in enumerate(info['chunks']):
 out=chunks/f'chunk_{i:03d}'
 if (out/'assembly.json').exists() and json.loads((out/'assembly.json').read_text())['source_sha256'] != row['sha256']:
  out.rename(out.with_name(out.name+f'_previous_{int(time.time())}'))
 if not (out/'assembly.json').exists():
  if out.exists():out.rename(out.with_name(out.name+f'_failed_{int(time.time())}'))
  available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))/1024
  if available<5500:raise RuntimeError(f'RAM guard: {available:.0f} MiB available; resume later')
  print('CAD partition',i+1,'/',len(info['chunks']),row['bytes'],'bytes',flush=True)
  log=chunks/f'chunk_{i:03d}.log'
  with log.open('w') as stream:
   r=subprocess.run([sys.executable,'-u',str(ROOT/'export_cad.py'),'--source',str(ROOT/'build'/row['path']),'--output',str(out),'--expected-sha256',row['sha256']],stdout=stream,stderr=subprocess.STDOUT,timeout=300)
  if r.returncode:raise RuntimeError(f'CAD partition {i} failed: {r.returncode}; see {log}')
 data=json.loads((out/'assembly.json').read_text());offset=len(records)
 for record in data['unique_solids']:
  record=dict(record);record['id']+=offset;record['mesh']=f'../cad_chunks/chunk_{i:03d}/'+record['mesh'];record['source_chunk']=i;records.append(record)
 for instance in data['instances']:
  instance=dict(instance);instance['solid_id']+=offset;instances.append(instance)
 surface_offset=len(surfaces)
 for record in data.get('surface_only_shells',[]):
  record=dict(record);record['id']+=surface_offset;record['mesh']=f'../cad_chunks/chunk_{i:03d}/'+record['mesh'];surfaces.append(record)
 for instance in data.get('surface_instances',[]):
  instance=dict(instance);instance['surface_id']+=surface_offset;surface_instances.append(instance)
 bounds.append(data['cad_bounds_mm']);visuals.append(f'../cad_chunks/chunk_{i:03d}/'+data['visual_glb'])
 chunk_reports.append({'chunk':i,'solids':data['solid_count'],'unique_solids':len(data['unique_solids']),'peak_rss_mb':data['peak_rss_mb'],'seconds':data['wall_seconds']})
 print('Exported',i,'unique',len(data['unique_solids']),'placed',data['solid_count'],'peak MiB',data['peak_rss_mb'],flush=True)
low=[min(b[j] for b in bounds) for j in range(3)];high=[max(b[j+3] for b in bounds) for j in range(3)]
result={'source_sha256':info['source_sha256'],'source_units':'mm','cad_bounds_mm':low+high,'visual_glbs':visuals,'solid_count':len(instances),'unique_solids':records,'instances':instances,'surface_only_shells':surfaces,'surface_instances':surface_instances,'linear_deflection_mm':2.0,'angular_deflection_rad':.4,'partitions':chunk_reports,'wall_seconds':time.monotonic()-started,'closed_solid_collision_scope':True}
out=ROOT/'build/cad78';out.mkdir(exist_ok=True)
(out/'assembly.json').write_text(json.dumps(result,indent=2)+'\n')
print('Full CAD exported',len(records),'unique',len(instances),'placed; bounds mm',low+high,flush=True)
