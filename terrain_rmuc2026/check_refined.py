"""Quick raw-convex height audit for selected parts before whole-scene compilation."""
from pathlib import Path
import json,resource
import numpy as np

ROOT=Path(__file__).resolve().parent
resource.setrlimit(resource.RLIMIT_AS,(1024**3,)*2)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
source=json.loads((ROOT/'build/cad78/assembly.json').read_text())
state=json.loads((ROOT/'pack/build_state.json').read_text())
manifest=json.loads((ROOT/'pack/manifest.json').read_text())
raw=json.loads((ROOT/'checks/raw_cad_rays.json').read_text())
if raw['source_sha256']!=source['source_sha256']:raise ValueError('Different CAD source')
xy=np.array(raw['xy_m']);ref=np.array(raw['height_m'],dtype=float);owner=np.array(raw['owners'])
rotation=np.array(manifest['cad_to_world_rotation']);origin=np.array(manifest['world_origin_before_shift_m'])
results=[]
for index in (153,181,616,780):
    top=np.full(len(xy),-np.inf)
    for relative in state['parts'][str(index)]['files']:
        with (ROOT/'pack'/relative).open('rb') as stream:
            nv,nn,nt,nf=np.fromfile(stream,dtype='<i4',count=4)
            vertices=np.fromfile(stream,dtype='<f4',count=nv*3).reshape(-1,3)
            stream.seek(int(nn*12+nt*8),1)
            faces=np.fromfile(stream,dtype='<i4',count=nf*3).reshape(-1,3)
        for instance in source['instances']:
            if instance['solid_id']!=index:continue
            pose=np.array(instance['cad_transform_mm']);v=vertices@(rotation@pose[:3,:3]).T+rotation@pose[:3,3]*.001-origin
            hits=np.flatnonzero(np.all((xy>=v.min(0)[:2]-1e-8)&(xy<=v.max(0)[:2]+1e-8),axis=1))
            tri=v[faces];a=tri[:,0];u=tri[:,1]-a;w=tri[:,2]-a
            den=u[:,0]*w[:,1]-u[:,1]*w[:,0];valid=abs(den)>1e-14
            a=a[valid];u=u[valid];w=w[valid];den=den[valid]
            for j in hits:
                d=xy[j]-a[:,:2];b=(d[:,0]*w[:,1]-d[:,1]*w[:,0])/den;c=(u[:,0]*d[:,1]-u[:,1]*d[:,0])/den
                inside=(b>=-1e-8)&(c>=-1e-8)&(b+c<=1+1e-8)
                if inside.any():top[j]=max(top[j],np.max(a[inside,2]+b[inside]*u[inside,2]+c[inside]*w[inside,2]))
    selected=np.flatnonzero(owner==index);errors=top[selected]-ref[selected]
    row={'cad_part_id':index,'samples':len(selected),'max_abs_error_m':float(max(abs(errors),default=0)),
         'over_12mm':[{'xy_m':xy[j].tolist(),'error_m':float(top[j]-ref[j])} for j in selected if not np.isfinite(top[j]) or abs(top[j]-ref[j])>.012]}
    results.append(row);print(row,flush=True)
(ROOT/'checks/refined_raw_convex_rays.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
if any(row['over_12mm'] for row in results):raise AssertionError('Selected parts still differ from raw CAD')
