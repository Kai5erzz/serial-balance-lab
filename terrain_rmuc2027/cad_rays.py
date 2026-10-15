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


def covered_gaps(source, cad, rotation, origin, xy, excluded_ids=()):
    """Find air between independent closed CAD volumes on vertical lines.

    Unlike a highest-surface ray, this detects ceilings above traversable gaps.
    Intersections with each closed solid are paired using even/odd parity, then
    unioned across solids. Open shells and conservative equipment proxies are
    excluded. Odd intersection counts are rejected rather than guessed.
    """
    intervals = [[] for _ in xy]
    excluded_ids = set(excluded_ids)
    by_part = defaultdict(list)
    for instance in source['instances']:
        by_part[instance['solid_id']].append(instance)
    rejected = 0
    for record in source['unique_solids']:
        if record['id'] in excluded_ids:
            continue
        with np.load(cad / record['mesh']) as mesh:
            vertices = mesh['vertices_mm'] * .001
            faces = mesh['faces']
        for instance in by_part[record['id']]:
            pose = np.asarray(instance['cad_transform_mm'])
            r = rotation @ pose[:3, :3]
            t = rotation @ pose[:3, 3] * .001 - origin
            world = vertices @ r.T + t
            low, high = world.min(0), world.max(0)
            if high[2] < -.4 or low[2] > 1.8:
                continue
            hits = np.flatnonzero(np.all((xy >= low[:2] - 1e-8) & (xy <= high[:2] + 1e-8), axis=1))
            if not len(hits):
                continue
            triangles = world[faces]
            a = triangles[:, 0]
            u = triangles[:, 1] - a
            v = triangles[:, 2] - a
            denominator = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
            valid = abs(denominator) > 1e-14
            a, u, v, denominator = a[valid], u[valid], v[valid], denominator[valid]
            for j in hits:
                dx, dy = xy[j, 0] - a[:, 0], xy[j, 1] - a[:, 1]
                b = (dx * v[:, 1] - dy * v[:, 0]) / denominator
                c = (u[:, 0] * dy - u[:, 1] * dx) / denominator
                mask = (b >= -1e-8) & (c >= -1e-8) & (b + c <= 1 + 1e-8)
                levels = np.unique(np.round(a[mask, 2] + b[mask] * u[mask, 2] + c[mask] * v[mask, 2], 7))
                if len(levels) % 2:
                    rejected += 1
                    continue
                for bottom, top in levels.reshape(-1, 2):
                    intervals[j].append([float(bottom), float(top), record['id'], record['id']])
    result = []
    for point, spans in zip(xy, intervals):
        union = []
        for span in sorted(spans):
            if union and span[0] <= union[-1][1] + 1e-5:
                if span[1] > union[-1][1]:
                    union[-1][1], union[-1][3] = span[1], span[3]
            else:
                union.append(span.copy())
        for floor, roof in zip(union, union[1:]):
            clearance = roof[0] - floor[1]
            if -.4 <= floor[1] <= 1.0 and .12 <= clearance <= 1.0 and roof[0] <= 1.5:
                result.append({'xy_m': point.tolist(), 'floor_z_m': floor[1], 'ceiling_z_m': roof[0],
                               'clearance_m': clearance, 'floor_part_id': floor[3], 'ceiling_part_id': roof[2]})
    return result, {'rejected_odd_intersections': rejected, 'grid_points': len(xy)}
