"""Bake display-only mesh LOD; full packaged contact and CAD visuals stay intact."""
from pathlib import Path
import argparse,hashlib,json,struct,sys,tempfile,zipfile
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dependencies',type=Path,help='optional isolated fast_simplification installation')
    a=p.parse_args()
    if a.dependencies:sys.path.insert(0,str(a.dependencies.resolve()))
    import fast_simplification
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from terrain_rmuc2027.runtime_assets import ensure_pack
    pack=ensure_pack();output=ROOT/'assets/operator_visual.zip'
    rows=[]
    with tempfile.TemporaryDirectory(prefix='operator-lod-',dir=pack.parent) as temporary:
        directory=Path(temporary);(directory/'meshes').mkdir()
        source=ET.parse(pack/'field_visual.xml').getroot()
        for asset in source.findall('asset/mesh'):
            original=pack/asset.get('file')
            with original.open('rb') as f:
                nv,nn,nt,nf=struct.unpack('<4i',f.read(16))
                points=np.fromfile(f,dtype='<f4',count=nv*3).reshape(-1,3)
                f.seek(12*nn+8*nt,1)
                faces=np.fromfile(f,dtype='<i4',count=nf*3).reshape(-1,3)
            welded,inverse=np.unique(np.rint(points.astype(float)*1e5).astype(np.int64),axis=0,return_inverse=True)
            vertices=welded.astype(float)*1e-5
            triangles=inverse[faces].astype(np.int32)
            triangles=triangles[(triangles[:,0]!=triangles[:,1])&(triangles[:,1]!=triangles[:,2])&(triangles[:,0]!=triangles[:,2])]
            x=np.concatenate((triangles[:,0],triangles[:,1],triangles[:,2]));y=np.concatenate((triangles[:,1],triangles[:,2],triangles[:,0]))
            graph=coo_matrix((np.ones(len(x),np.uint8),(x,y)),shape=(len(vertices),)*2).tocsr()
            _,labels=connected_components(graph,directed=False)
            face_labels=labels[triangles[:,0]];order=np.argsort(face_labels,kind='stable')
            ends=np.r_[0,np.flatnonzero(np.diff(face_labels[order]))+1,len(order)]
            chunks=[];indices=[];offset=0;fallbacks=0
            for start,end in zip(ends[:-1],ends[1:]):
                component=triangles[order[start:end]]
                used,remap=np.unique(component,return_inverse=True)
                v=vertices[used];t=remap.reshape(-1,3).astype(np.int32)
                target=min(len(t),max(16,round(len(t)*.2)))
                if target<len(t):
                    reduced_v,reduced_t=fast_simplification.simplify(v,t,target_count=target,agg=5,preserve_border=True)
                    error=np.max(abs(np.array([reduced_v.min(0),reduced_v.max(0)])-np.array([v.min(0),v.max(0)])))
                    if error<=.003:v,t=reduced_v,reduced_t
                    else:fallbacks+=1
                chunks.append(v);indices.append(t+offset);offset+=len(v)
            result_v=np.concatenate(chunks);result_t=np.concatenate(indices)
            relative='meshes/'+original.name
            with (directory/relative).open('wb') as f:
                np.asarray([len(result_v),0,0,len(result_t)],dtype='<i4').tofile(f)
                result_v.astype('<f4').tofile(f);result_t.astype('<i4').tofile(f)
            asset.set('file',relative)
            error=float(np.max(abs(np.array([result_v.min(0),result_v.max(0)])-np.array([points.min(0),points.max(0)]))))
            row=dict(mesh=original.name,original_faces=nf,display_faces=len(result_t),components=len(ends)-1,bounds_error_m=error,fallbacks=fallbacks)
            rows.append(row);print(row,flush=True)
        ET.indent(source);ET.ElementTree(source).write(directory/'field.xml',encoding='utf-8',xml_declaration=True)
        (directory/'field_perimeter.xml').write_bytes((pack/'field_perimeter.xml').read_bytes())
        report=dict(original_visual_faces=sum(r['original_faces'] for r in rows),display_visual_faces=sum(r['display_faces'] for r in rows),components_preserved=True,collision_resources_modified=False,meshes=rows)
        (ROOT/'operator_visual_receipt.json').write_text(json.dumps(report,indent=2)+'\n')
        (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        with zipfile.ZipFile(output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as z:
            for file in sorted(directory.rglob('*')):
                if file.is_file():z.write(file,'terrain_rmuc2027/pack/'+file.relative_to(directory).as_posix())
    report.update(archive='assets/operator_visual.zip',bytes=output.stat().st_size,sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (ROOT/'operator_visual_receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='meshes'}),flush=True)

if __name__=='__main__':main()
