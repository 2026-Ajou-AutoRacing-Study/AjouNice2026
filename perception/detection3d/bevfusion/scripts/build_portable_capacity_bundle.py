#!/usr/bin/env python3
"""Export minimal standalone latency input/weights/code, outside Git delivery."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    root=Path(__file__).resolve().parents[4]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=root/'artifacts/handoff'/('capacity_4090_'+datetime.now().strftime('%Y%m%d_%H%M%S')))
    a=p.parse_args();dest=a.output/'ajounice_capacity_4090'
    if dest.exists():raise FileExistsError(dest)
    dest.mkdir(parents=True);payload=dest/'payload'
    scripts='perception/detection3d/bevfusion/scripts/'
    files=[scripts+'profile_base5_stages.py',scripts+'benchmark_base5_capacity.py',
           'perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py',
           'perception/detection3d/bevfusion/view_geometry.py',
           'configs/sensors/Sensor_competition_local.json',
           'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth']
    folders=['perception/detection3d/bevfusion/plugin','perception/detection3d/common','docker/bevfusion/app',
             'third_party/mmdetection3d/configs','third_party/mmdetection3d/projects/BEVFusion/configs',
             'artifacts/evaluation/bevfusion/capacity_261006/samples']
    for rel in files:
        target=payload/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,target)
    for rel in folders:
        shutil.copytree(root/rel,payload/rel,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
    for name in ('run.sh','run_benchmark.py','README.md'):
        shutil.copy2(root/scripts/'portable_capacity'/name,dest/name)
    # Provide normal packages rather than relying on implicit namespace merging.
    for rel in ('perception/__init__.py','perception/detection3d/__init__.py','perception/detection3d/bevfusion/__init__.py'):
        target=payload/rel
        if (root/rel).exists():shutil.copy2(root/rel,target)
        else:target.touch()
    manifest={'built_at':datetime.now().isoformat(),
              'source_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
              'scope':'Fixed E25 sampling intervals; four cost controls; no ROS or accuracy evaluation.',
              'sha256':{str(f.relative_to(dest)):sha(f) for f in sorted(dest.rglob('*')) if f.is_file()}}
    (dest/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    archive=a.output/'ajounice_capacity_4090.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
        for f in sorted(dest.rglob('*')):
            if f.is_file():z.write(f,str(Path(dest.name)/f.relative_to(dest)))
    print(json.dumps({'directory':str(dest),'archive':str(archive),'bytes':archive.stat().st_size,'sha256':sha(archive)},indent=2))


if __name__=='__main__':main()
