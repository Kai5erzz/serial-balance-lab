from pathlib import Path
import ast,urllib.request,hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parent
source=ROOT.parent/'terrain_rmuc2025/research/reference__conversion.py'
values={}
for node in ast.parse(source.read_text()).body:
 if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('OFFICIAL_RULEBOOK_URL','OFFICIAL_RULEBOOK_SIZE','OFFICIAL_RULEBOOK_SHA256'):
  values[node.targets[0].id]=ast.literal_eval(node.value)
official_url=values['OFFICIAL_RULEBOOK_URL'];url=official_url.replace('bbs-web-static.robomaster.com','hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com');dest=ROOT/'research/rules_v2.pdf'
subprocess.run(['curl','-L','--fail','--silent','--show-error','--retry','2','--max-time','80','-o',str(dest),url],check=True)
h=hashlib.sha256()
with dest.open('rb') as f:
 while b:=f.read(128*1024):h.update(b)
if dest.stat().st_size!=values['OFFICIAL_RULEBOOK_SIZE'] or h.hexdigest()!=values['OFFICIAL_RULEBOOK_SHA256']:raise ValueError('Rulebook identity differs')
(ROOT/'research/rules_source.json').write_text(json.dumps({'url':official_url,'download_url':url,'version':'V2.0.0 (2026-06-26), matching CAD publication','bytes':dest.stat().st_size,'sha256':h.hexdigest()},indent=2)+'\n')
print('Rulebook verified',dest.stat().st_size,'bytes')
