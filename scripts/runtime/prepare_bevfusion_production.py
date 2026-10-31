#!/usr/bin/env python3
"""Install pinned production weights without starting ROS/GPU or changing a branch."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from bevfusion_production_profile import digest, profile

DRIVE_ROOT='12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP'


def prepare(args):
    p=profile(args.model)
    destination=Path(p['checkpoint_path'])
    if destination.exists():
        if digest(destination)!=p['sha256']:
            raise ValueError('Existing destination has a different hash; preserve it and choose a new path')
        print(json.dumps(dict(event='checkpoint_ready',path=str(destination),sha256=p['sha256'])))
        return
    if args.check_only:raise FileNotFoundError(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='production_download_',dir=destination.parent) as temporary:
        target=Path(temporary)/p['checkpoint']
        if args.source_checkpoint:
            if digest(args.source_checkpoint)!=p['sha256']:raise ValueError('Source checkpoint hash differs')
            shutil.copyfile(args.source_checkpoint,target)
        else:
            if args.model!='e24':
                raise ValueError('Selected model requires existing weights or --source-checkpoint; use install_base5_production.sh for the E12 ZIP or the legacy E25 setup')
            # Tokens stay in the user's rclone config, never in a command or receipt.
            remote_path='model/bevfusion/production/base5_e24_261013/E24.pth'
            subprocess.run(['rclone','copyto',args.remote+':'+remote_path,str(target),
                '--drive-root-folder-id',args.drive_root,'--progress','--transfers','1',
                '--tpslimit','2','--retries','2','--low-level-retries','2'],check=True)
        if digest(target)!=p['sha256']:raise ValueError('Downloaded checkpoint hash differs; not installed')
        if destination.exists():raise FileExistsError('Destination appeared during preparation; preserve it')
        # Same filesystem: atomically fail if another process created the target.
        os.link(target,destination)
    print(json.dumps(dict(event='checkpoint_ready',path=str(destination),sha256=p['sha256'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=('e12','e24','e25'),default='e12')
    parser.add_argument('--source-checkpoint',type=Path)
    parser.add_argument('--remote',default='ajou_drive')
    parser.add_argument('--drive-root',default=DRIVE_ROOT)
    parser.add_argument('--check-only',action='store_true')
    prepare(parser.parse_args())
