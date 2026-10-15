"""Index STEP statements in bounded RAM; run with the shared heavy runner."""
from pathlib import Path
from collections import Counter,defaultdict
from array import array
import json,re,resource,time,hashlib
ROOT=Path(__file__).resolve().parent
source=ROOT/'research/RMUC2027_V1.0.0.stp'
out=ROOT/'build/step_index'
out.mkdir(parents=True,exist_ok=True)
resource.setrlimit(resource.RLIMIT_AS,(3*1024**3,)*2)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
started=time.monotonic()
offsets=array('Q'); lengths=array('I'); counts=Counter(); kinds=defaultdict(list)
count=0
with source.open('rb') as f:
 while True:
  offset=f.tell(); line=f.readline()
  if not line: break
  if not line.startswith(b'#'): continue
  statement=bytearray(line)
  while not statement.rstrip().endswith(b';'):
   line=f.readline()
   if not line: raise RuntimeError('Incomplete STEP statement')
   statement.extend(line)
  m=re.match(rb'#(\d+)\s*=\s*([A-Z0-9_]+)?',statement)
  idx=int(m[1]); kind=m[2].decode() if m[2] else 'COMPLEX'
  if idx>=len(offsets):
   extra=((idx-len(offsets))//100000+1)*100000
   offsets.extend(array('Q',[0])*extra);lengths.extend(array('I',[0])*extra)
  if lengths[idx]: raise RuntimeError(f'duplicate #{idx}')
  offsets[idx]=offset;lengths[idx]=len(statement);counts[kind]+=1;count+=1
  if ('REPRESENTATION' in kind or kind in ('MANIFOLD_SOLID_BREP','BREP_WITH_VOIDS','STYLED_ITEM','OVER_RIDING_STYLED_ITEM','PRODUCT_DEFINITION','NEXT_ASSEMBLY_USAGE_OCCURRENCE','COMPLEX')):
   kinds[kind].append(idx)
  if count%1000000==0: print('Indexed',count,'entities',f.tell(),'bytes',flush=True)
with (out/'offsets.bin').open('wb') as f: offsets.tofile(f)
with (out/'lengths.bin').open('wb') as f: lengths.tofile(f)
report={'entity_count':count,'index_size':len(offsets),'type_counts':dict(counts.most_common()),'kinds':dict(kinds),'seconds':time.monotonic()-started,'peak_rss_mb':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
(out/'index.json').write_text(json.dumps(report)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='kinds'},indent=2),flush=True)
