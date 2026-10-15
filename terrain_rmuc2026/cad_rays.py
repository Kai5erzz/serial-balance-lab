"""Independent vertical rays against raw B-rep tessellations, without MuJoCo."""
from pathlib import Path
from collections import defaultdict
import numpy as np

def heights(source,cad,rotation,origin,xy):
    z=np.full(len(xy),-np.inf);owners=np.full(len(xy),-1,dtype=int);surface=np.zeros(len(xy),dtype=bool)
    counts={'closed_instances':0,'surface_instances':0,'tested_rays':0}
    for key,instance_key,index_key,is_surface in (('unique_solids','instances','solid_id',False),('surface_only_shells','surface_instances','surface_id',True)):
        by_part=defaultdict(list)
        for i,instance in enumerate(source.get(instance_key,[])):by_part[instance[index_key]].append((i,instance))
        for record in source.get(key,[]):
            if record['id'] not in by_part:continue
            with np.load(cad/record['mesh']) as d:vertices=d['vertices_mm']*.001;faces=d['faces']
            for i,instance in by_part[record['id']]:
                pose=np.asarray(instance['cad_transform_mm']);r=rotation@pose[:3,:3];t=rotation@pose[:3,3]*.001-origin
                world=vertices@r.T+t
                low=world.min(0);high=world.max(0)
                hits=np.flatnonzero(np.all((xy>=low[:2]-1e-8)&(xy<=high[:2]+1e-8),axis=1))
                if not len(hits):continue
                triangles=world[faces];a=triangles[:,0];u=triangles[:,1]-a;v=triangles[:,2]-a
                denominator=u[:,0]*v[:,1]-u[:,1]*v[:,0]
                valid=abs(denominator)>1e-14
                a=a[valid];u=u[valid];v=v[valid];denominator=denominator[valid]
                for j in hits:
                    dx=xy[j,0]-a[:,0];dy=xy[j,1]-a[:,1]
                    b=(dx*v[:,1]-dy*v[:,0])/denominator;c=(u[:,0]*dy-u[:,1]*dx)/denominator
                    mask=(b>=-1e-8)&(c>=-1e-8)&(b+c<=1+1e-8)
                    if not mask.any():continue
                    top=float(np.max(a[mask,2]+b[mask]*u[mask,2]+c[mask]*v[mask,2]))
                    if top>z[j]:z[j]=top;owners[j]=record['id'];surface[j]=is_surface
                counts['surface_instances' if is_surface else 'closed_instances']+=1;counts['tested_rays']+=len(hits)
            if record['id']%100==0:print('CAD rays part',key,record['id'],flush=True)
        if not is_surface: closed_z=z.copy()
    return z,owners,surface,counts,closed_z
