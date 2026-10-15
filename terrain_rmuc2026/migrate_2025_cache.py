"""Move this terrain task's reproducible caches to D, preserving old paths."""
from pathlib import Path
import hashlib,json,shutil,time
root=Path(__file__).resolve().parent.parent/'terrain_rmuc2025'
destination=Path('/media/kaiser/D盘2/rl-lab-terrain/rmuc2025')
destination.mkdir(parents=True,exist_ok=True)
receipt=[]
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  while chunk:=f.read(1024*1024): h.update(chunk)
 return h.hexdigest()
for name in ('build','.caddeps','.caddeps78','research'):
 src=root/name; dst=destination/name
 if src.is_symlink():
  print('Already relocated',src,flush=True);continue
 if dst.exists(): raise FileExistsError(dst)
 shutil.copytree(src,dst,copy_function=shutil.copy2,symlinks=True)
 files=0;size=0
 for p in src.rglob('*'):
  if p.is_file() and not p.is_symlink():
   q=dst/p.relative_to(src)
   if q.stat().st_size!=p.stat().st_size or sha(p)!=sha(q): raise AssertionError(p)
   files+=1;size+=p.stat().st_size
 backup=src.with_name(name+'.relocating')
 src.rename(backup)
 try: src.symlink_to(dst,target_is_directory=True)
 except BaseException:
  backup.rename(src);raise
 shutil.rmtree(backup)
 receipt.append({'original':str(src),'destination':str(dst),'files_sha256_verified':files,'bytes':size})
 print('Relocated and verified',name,files,'files',size,'bytes',flush=True)
(Path(__file__).resolve().parent/'storage_migration.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('Freed bytes',sum(r['bytes'] for r in receipt),flush=True)
