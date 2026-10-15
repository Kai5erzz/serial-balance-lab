"""Stream CAD partitions into compact visual palettes in a common CAD frame."""
from pathlib import Path
from collections import defaultdict
import ctypes,gc,json,itertools
import numpy as np
import trimesh


def build_visuals(source,cad,output,rotation,write_msh,numbers,ET):
    b=np.asarray(source['cad_bounds_mm']).reshape(2,3)*.001
    corners=np.array([[x,y,z] for x in b[:,0] for y in b[:,1] for z in b[:,2]])@rotation.T
    world_bounds=np.array([corners.min(0),corners.max(0)])
    bins=defaultdict(float);sums=defaultdict(float);bounds_error=0;floor_triangles=[]
    glb_to_cad={}
    def load(relative):
        nonlocal bounds_error
        scene=trimesh.load(cad/relative,force='scene',process=False)
        if relative not in glb_to_cad:
            # Exporter up-axis conventions are checked against B-rep bounds.
            part=json.loads((cad/Path(relative).parent/'assembly.json').read_text())
            expected=np.array(part['cad_bounds_mm']).reshape(2,3)*.001
            raw=scene.bounds
            raw_corners=np.array([[x,y,z] for x in raw[:,0] for y in raw[:,1] for z in raw[:,2]])
            candidates=[]
            for perm in itertools.permutations(range(3)):
                for signs in itertools.product((-1,1),repeat=3):
                    matrix=np.eye(3)[list(perm)]*np.array(signs)[:,None]
                    if np.linalg.det(matrix)<.5:continue
                    points=raw_corners@matrix.T
                    error=float(np.max(abs(np.array([points.min(0),points.max(0)])-expected)))
                    candidates.append((error,matrix))
            error,matrix=min(candidates,key=lambda x:x[0])
            if error>.015:raise ValueError(f'glTF/CAD unit/axis mismatch {error} m')
            glb_to_cad[relative]=matrix;bounds_error=max(bounds_error,error)
            print('Verified glTF-to-CAD rotation',matrix.tolist(),'bounds error m',error,flush=True)
        return scene,glb_to_cad[relative]
    # Only one partition and one placed visual mesh are live at a time.
    for relative in source['visual_glbs']:
        scene,glb_rotation=load(relative)
        for node in scene.graph.nodes_geometry:
            transform,name=scene.graph[node];mesh=scene.geometry[name].copy();mesh.apply_transform(transform)
            mesh.vertices=np.asarray(mesh.vertices)@glb_rotation.T@rotation.T
            selected=mesh.face_normals[:,2]>.999
            heights=mesh.triangles_center[selected,2];areas=mesh.area_faces[selected]
            broad=(mesh.face_normals[:,2]>.98)&(mesh.area_faces>2)
            for centre,normal,area in zip(mesh.triangles_center[broad],mesh.face_normals[broad],mesh.area_faces[broad]):
                floor_triangles.append((centre.copy(),normal.copy(),float(area)))
            keys=np.rint(heights/.005).astype(int)
            for key in np.unique(keys):
                mask=keys==key;bins[int(key)]+=float(areas[mask].sum());sums[int(key)]+=float((heights[mask]*areas[mask]).sum())
        del scene,mesh;gc.collect();ctypes.CDLL(None).malloc_trim(0)
    if not bins:raise ValueError('No upward CAD surfaces')
    winner=max(bins,key=bins.get);floor=sums[winner]/bins[winner]
    origin=np.array([*world_bounds.mean(axis=0)[:2],floor])
    floor_planes=[(centre-origin,normal) for centre,normal,area in floor_triangles if area>10 and abs(centre[2]-floor)<.3]
    if not floor_planes:raise ValueError('No broad ground panels for visual floor classification')
    full=ET.Element('mujoco',model='rmuc2026_field');assets=ET.SubElement(full,'asset');world=ET.SubElement(full,'worldbody')
    simplification=[];face_count=0;cad_faces=0;palette_count=0
    for chunk,relative in enumerate(source['visual_glbs']):
        scene,glb_rotation=load(relative)
        colored=defaultdict(list);cache={}
        for node in scene.graph.nodes_geometry:
            transform,name=scene.graph[node];original=scene.geometry[name]
            material=getattr(original.visual,'material',None)
            rgba=tuple(int(x) for x in getattr(material,'main_color',np.array([178,178,178,255])))
            if name not in cache:
                cleaned=original.copy()
                cleaned.merge_vertices();cleaned.update_faces(cleaned.nondegenerate_faces());cleaned.update_faces(cleaned.unique_faces());cleaned.remove_unreferenced_vertices()
                simplified=cleaned
                for ratio in (.15,.35,.6):
                    target=min(len(cleaned.faces),max(128,int(len(cleaned.faces)*ratio)))
                    if target>=len(cleaned.faces):break
                    candidate=cleaned.simplify_quadric_decimation(face_count=target,aggression=5)
                    if len(candidate.faces)>=4 and np.max(abs(candidate.bounds-original.bounds))<=.015 and abs(candidate.area-cleaned.area)<=max(1e-10,cleaned.area*.05):
                        simplified=candidate;break
                cache[name]=simplified
                simplification.append({'chunk':chunk,'geometry':name,'cad_faces':len(original.faces),'visual_faces':len(simplified.faces),'bounds_error_m':float(np.max(abs(simplified.bounds-original.bounds)))})
            mesh=cache[name].copy();mesh.apply_transform(transform);mesh.vertices=np.asarray(mesh.vertices)@glb_rotation.T@rotation.T-origin
            face_count+=len(mesh.faces);cad_faces+=len(original.faces)
            # Display-only material correction. Original colored CAD stays in
            # the source GLBs; geometry and collision samples remain untouched.
            if min(rgba[:3])>160 and max(rgba[:3])-min(rgba[:3])<35:
                centres=mesh.triangles_center
                up=(mesh.face_normals[:,2]>.3)&(mesh.area_faces>.05)
                distance=np.full(len(mesh.faces),np.inf)
                for point,normal in floor_planes:
                    ground=point[2]-((centres[:,0]-point[0])*normal[0]+(centres[:,1]-point[1])*normal[1])/normal[2]
                    distance=np.minimum(distance,abs(centres[:,2]-ground))
                ground=up&(distance<.015)
                raised=up&~ground&(centres[:,2]<1.3)
                remaining=~(ground|raised)
                for selected,colour in ((ground,(30,34,39,255)),(raised,(103,111,121,255)),(remaining,rgba)):
                    if selected.any():colored[colour].append(mesh.submesh([np.flatnonzero(selected)],append=True,repair=False))
            else:colored[rgba].append(mesh)
        for index,(rgba,meshes) in enumerate(sorted(colored.items())):
            mesh=trimesh.util.concatenate(meshes);name=f'rmuc2026_visual_{chunk:02d}_{index:03d}';filename=f'meshes/{name}.msh'
            write_msh(mesh,output/filename)
            ET.SubElement(assets,'mesh',name=name,file=filename,inertia='shell',maxhullvert='64')
            ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0',group='1',density='0',rgba=numbers(np.array(rgba)/255))
            palette_count+=1
        print('Visual partition',chunk,'total faces',face_count,'CAD faces',cad_faces,flush=True)
        del scene,colored,cache,mesh,meshes,original,simplified;gc.collect();ctypes.CDLL(None).malloc_trim(0)
    print('Visuals ready',palette_count,'palettes',face_count,'faces; floor/origin',floor,origin,flush=True)
    (output/'visual_axis_contract.json').write_text(json.dumps({k:v.tolist() for k,v in glb_to_cad.items()},indent=2)+'\n')
    return full,assets,world,origin,bounds_error,bins[winner],face_count,simplification,cad_faces
