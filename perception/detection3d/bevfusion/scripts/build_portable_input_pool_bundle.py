#!/usr/bin/env python3
"""Versioned standalone GPU probe, no model checkpoint or full dataset transfer."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import pickle
import shutil
import subprocess
import zipfile


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root=Path(__file__).resolve().parents[4]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--infos',type=Path,required=True);a=p.parse_args()
    dest=a.output/'ajounice_input_pool_gpu';payload=dest/'payload'
    if dest.exists():raise FileExistsError(dest)
    payload.mkdir(parents=True)
    scripts='perception/detection3d/bevfusion/scripts/'
    files=[scripts+name for name in ('profile_base5_stages.py','benchmark_base5_capacity.py',
        'benchmark_base5_input_optimization.py','validate_base5_pool_model.py','test_pool_optimization.py')]
    files+=['configs/sensors/Sensor_competition_local.json','perception/detection3d/bevfusion/view_geometry.py',
        'dataset/morai_od/scripts/dynamic_capture_metadata.py',
        'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py']
    folders=['perception/detection3d/bevfusion/configs','perception/detection3d/bevfusion/plugin',
        'perception/detection3d/common','docker/bevfusion/app','third_party/mmdetection3d/configs',
        'third_party/mmdetection3d/projects/BEVFusion/configs']
    for rel in files:
        target=payload/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,target)
    for rel in folders:
        shutil.copytree(root/rel,payload/rel,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
    shutil.copytree(a.samples,payload/'samples')
    ids={v['id'] for v in json.loads((a.samples/'manifest.json').read_text())['samples']}
    records=[]
    for file in a.infos.glob('*infos_*.pkl'):
        records += [v for v in pickle.loads(file.read_bytes())['data_list'] if v['scene_id'] in ids]
    if len(records)!=len(ids):raise ValueError('GT sample subset missing/duplicate')
    info=payload/'infos';info.mkdir()
    (info/'probe_infos_train.pkl').write_bytes(pickle.dumps(dict(data_list=records)))
    for rel in ('perception/__init__.py','perception/detection3d/__init__.py','perception/detection3d/bevfusion/__init__.py'):
        target=payload/rel
        if (root/rel).exists():shutil.copy2(root/rel,target)
        else:target.touch()
    for name in ('run.sh','run_inside.py','README.md'):
        shutil.copy2(root/scripts/'portable_input_pool'/name,dest/name)
    manifest=dict(built_at=datetime.now().astimezone().isoformat(),
        source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        scope='Fresh four-class cost/correctness; rect-axis fix; no learned AP or fullstack.',
        sha256={str(f.relative_to(dest)):sha(f) for f in sorted(dest.rglob('*')) if f.is_file()})
    (dest/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    archive=a.output/'ajounice_input_pool_gpu.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
        for f in sorted(dest.rglob('*')):
            if f.is_file():z.write(f,str(Path(dest.name)/f.relative_to(dest)))
    print(json.dumps(dict(directory=str(dest),archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive)),indent=2))


if __name__=='__main__':main()
