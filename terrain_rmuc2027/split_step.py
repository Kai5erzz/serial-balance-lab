"""Partition STEP geometry while preserving original assembly transforms/styles.

Each partition retains the complete assembly metadata. Inactive shape items are
removed from representation item lists, and only the dependency closure of the
remaining geometry and its original styles is emitted. No coordinates change.
"""
from pathlib import Path
from array import array
import mmap,re,json,hashlib,resource,time
ROOT=Path(__file__).resolve().parent
resource.setrlimit(resource.RLIMIT_AS,(3*1024**3,)*2)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
started=time.monotonic(); source=ROOT/'research/RMUC2027_V1.0.0.stp'
identity=json.loads((ROOT/'research/source.json').read_text())
h=hashlib.sha256();md5=hashlib.md5()
with source.open('rb') as f:
 while b:=f.read(1024*1024):h.update(b);md5.update(b)
if h.hexdigest()!=identity['sha256'] or md5.hexdigest()!=identity['source']['md5']:raise ValueError('Official STEP identity mismatch')
idxdir=ROOT/'build/step_index'; meta=json.loads((idxdir/'index.json').read_text()); kinds=meta['kinds']; n=meta['index_size']
off=array('Q');length=array('I')
with (idxdir/'offsets.bin').open('rb') as f:off.fromfile(f,n)
with (idxdir/'lengths.bin').open('rb') as f:length.fromfile(f,n)
f=source.open('rb');mm=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
refpat=re.compile(rb'#(\d+)'); typat=re.compile(rb'#\d+\s*=\s*([A-Z0-9_]+)')
strpat=re.compile(rb"'(?:[^']|'')*'")
def statement(i):
 if not 0<=i<n or not length[i]:raise ValueError(f'Missing STEP dependency #{i}')
 return mm[off[i]:off[i]+length[i]]
def refs(s):return list(map(int,refpat.findall(strpat.sub(b"''",s.split(b'=',1)[1]))))
def kind(i):
 m=typat.match(statement(i));return m[1] if m else b'COMPLEX'
roots=kinds.get('SHAPE_DEFINITION_REPRESENTATION',[])+kinds.get('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION',[])+kinds.get('SHAPE_REPRESENTATION_RELATIONSHIP',[])
# STEP representation relationships are reverse links, independent of SDR.
roots += [i for i in kinds.get('COMPLEX',[]) if b'REPRESENTATION_RELATIONSHIP(' in statement(i)]
leaves=kinds.get('MANIFOLD_SOLID_BREP',[])+kinds.get('BREP_WITH_VOIDS',[])
# Seven open surface models remain as source visual geometry; they are audited
# separately from closed solid collision parts by the CAD exporter.
for k,ids in kinds.items():
 if k=='MANIFOLD_SURFACE_SHAPE_REPRESENTATION':
  for i in ids:
   leaves.extend(r for r in refs(statement(i)) if kind(r)==b'SHELL_BASED_SURFACE_MODEL')
leaves=sorted(set(leaves)); leafset=set(leaves)
repids=set(sum([ids for k,ids in kinds.items() if k in ('SHAPE_REPRESENTATION','ADVANCED_BREP_SHAPE_REPRESENTATION','MANIFOLD_SURFACE_SHAPE_REPRESENTATION')],[]))
repitems=re.compile(rb'^(#\d+\s*=\s*\w+\(.*?,\()([#\d,\s]+)(\),\s*#\d+\s*\);\s*)$',re.S)
def rewrite(i,active):
 s=statement(i)
 if i in repids:
  m=repitems.match(s)
  if not m:raise ValueError(f'Cannot parse representation #{i}')
  ids=list(map(int,refpat.findall(m[2])))
  keep=[r for r in ids if r not in leafset or r in active]
  s=m[1]+b','.join(b'#'+str(r).encode() for r in keep)+m[3]
 return s
# STEP contains uninstantiated geometry. Retain every leaf reachable from
# the original product/assembly graph; record all unused leaf IDs separately.
all_leaves=list(leaves);seen_full=bytearray(n);todo=list(roots)
while todo:
    i=todo.pop()
    if seen_full[i]:continue
    seen_full[i]=1;todo.extend(refs(statement(i)))
unused=[i for i in leaves if not seen_full[i]]
leaves=[i for i in leaves if seen_full[i]]
print('Assembly-reachable geometry',len(leaves),'uninstantiated source leaves',len(unused),flush=True)
del seen_full
# Estimate independent solid closures, then keep chunks around 55 MiB of STEP.
groups=[];active=[];cost=0;visited=bytearray(n)
for count,i in enumerate(leaves):
 todo=[i];seen=set();weight=0
 while todo:
  r=todo.pop()
  if r in seen:continue
  seen.add(r);s=statement(r);weight+=len(s);todo.extend(refs(s))
 if active and cost+weight>55*1024**2:groups.append(active);active=[];cost=0
 active.append(i);cost+=weight
 if count%100==0:print('Partition weights',count,'/',len(leaves),flush=True)
if active:groups.append(active)
print('Geometry leaves',len(leaves),'partitions',len(groups),flush=True)
out=ROOT/'build/step_chunks';out.mkdir(exist_ok=True)
header=mm[:mm.find(b'DATA;')+5]+b'\n'; reports=[]
style_ids=kinds.get('STYLED_ITEM',[])+kinds.get('OVER_RIDING_STYLED_ITEM',[])
styles=[]
for i in style_ids:
 s=statement(i);m=re.search(rb'\),\s*#(\d+)',s)
 if m:styles.append((i,int(m[1])))
for chunk_id,group in enumerate(groups):
 active=set(group);seen=bytearray(n);todo=list(roots);kept=[]
 def walk():
  while todo:
   i=todo.pop()
   if seen[i]:continue
   seen[i]=1;kept.append(i);todo.extend(refs(rewrite(i,active)))
 walk()
 selected_styles=[i for i,target in styles if seen[target]]
 todo.extend(selected_styles);walk()
 replacements={}
 for i in kinds.get('MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION',[]):
  s=statement(i);m=repitems.match(s)
  if not m:raise ValueError('Cannot parse style presentation')
  replacements[i]=m[1]+b','.join(b'#'+str(r).encode() for r in selected_styles)+m[3]
  if not seen[i]:seen[i]=1;kept.append(i)
  todo.extend(refs(replacements[i]));walk()
 path=out/f'chunk_{chunk_id:03d}.stp'
 with path.open('wb') as target:
  target.write(header)
  for i in sorted(kept): target.write(replacements.get(i) or rewrite(i,active))
  target.write(b'ENDSEC;\nEND-ISO-10303-21;\n')
 digest=hashlib.sha256()
 with path.open('rb') as stream:
  while b:=stream.read(1024*1024):digest.update(b)
 reports.append({'path':str(path.relative_to(ROOT/'build')),'sha256':digest.hexdigest(),'bytes':path.stat().st_size,'leaf_ids':group,'entities':len(kept),'styles':len(selected_styles)})
 print('Chunk',chunk_id,len(group),'leaves',path.stat().st_size,'bytes',len(kept),'entities',flush=True)
 mm.madvise(mmap.MADV_DONTNEED)
report={'source_sha256':h.hexdigest(),'source_md5':md5.hexdigest(),'source_bytes':source.stat().st_size,'geometry_leaves':len(leaves),'uninstantiated_source_leaves':unused,'chunks':reports,'seconds':time.monotonic()-started,'peak_rss_mb':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'method':'Unmodified STEP entity dependency closure; full assembly transforms and relevant original styles; inactive geometry removed only from representation item lists.'}
(out/'chunks.json').write_text(json.dumps(report,indent=2)+'\n')
print('Split complete',report['seconds'],'seconds',report['peak_rss_mb'],'MiB',flush=True)
