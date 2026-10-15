"""Prepare pinned official CAD through bounded, restartable subprocess stages."""
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parent
if not (ROOT/'research/RMUC2027_V1.0.0.stp').is_file():
 raise FileNotFoundError('Download the pinned official STEP listed in research/source.json first.')
for script,checkpoint in [('index_step.py','build/step_index/index.json'),('split_step.py','build/step_chunks/chunks.json'),('export_chunks.py','build/cad78/assembly.json')]:
 if not (ROOT/checkpoint).is_file():
  subprocess.run([sys.executable,'-u',str(ROOT/script)],check=True)

subprocess.run([sys.executable,"-u",str(ROOT/"promote_terrain.py")],check=True)
subprocess.run([sys.executable,"-u",str(ROOT/"geometry_policy.py")],check=True)
