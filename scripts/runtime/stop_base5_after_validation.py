#!/usr/bin/env python3
"""Explicitly authorized, PID-scoped stop after a complete validation row."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import time


def complete_validation_row(row, epoch):
    match = re.search(r'Epoch\(val\)\s+\[' + str(epoch) +
                      r'\](?:\[(?P<done>\d+)/(?P<total>\d+)\])?\s+.*val/center_mAP:', row)
    return bool(match and (match.group('done') is None or
                           int(match.group('done')) == int(match.group('total')) > 0))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work', type=Path, required=True)
    p.add_argument('--log', type=Path, required=True)
    p.add_argument('--pid', type=int, required=True)
    p.add_argument('--epoch', type=int, required=True)
    p.add_argument('--timeout', type=int, default=1200)
    a = p.parse_args()
    cmd = Path(f'/proc/{a.pid}/cmdline').read_bytes().split(b'\0')
    tokens = [x.decode() for x in cmd if x]
    if '--work-dir' not in tokens or tokens[tokens.index('--work-dir') + 1] != str(a.work):
        raise ValueError('PID does not belong to the requested training work directory')
    if '--run' not in tokens:
        raise ValueError('PID is not an active training invocation')
    parent = int(Path(f'/proc/{a.pid}/stat').read_text().split()[3])
    if b'--remote-worker' not in Path(f'/proc/{parent}/cmdline').read_bytes().split(b'\0'):
        raise ValueError('Refuse to signal a data worker or unrelated process')
    start = Path(f'/proc/{a.pid}/stat').read_text().split()[21]
    deadline = time.monotonic() + a.timeout
    while time.monotonic() < deadline:
        if not Path(f'/proc/{a.pid}').exists():
            raise RuntimeError('Training exited before the authorized stop')
        if any(complete_validation_row(row, a.epoch) for row in a.log.read_text(errors='replace').splitlines()):
            checkpoint = a.work / f'epoch_{a.epoch}.pth'
            if not checkpoint.is_file():
                raise RuntimeError('Completed validation has no epoch checkpoint')
            # Give the post-validation checkpoint hook time to finish its atomic rename.
            time.sleep(2)
            if Path(f'/proc/{a.pid}/stat').read_text().split()[21] != start:
                raise RuntimeError('PID was reused; refuse to signal')
            os.kill(a.pid, signal.SIGINT)
            receipt = dict(reason='user_requested_distance_diagnostics', pid=a.pid,
                           completed_epoch=a.epoch, checkpoint=str(checkpoint),
                           checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                           signal='SIGINT', time=time.time())
            (a.work / 'distance_diagnostics_stop.json').write_text(json.dumps(receipt, indent=2)+'\n')
            print(json.dumps(receipt), flush=True)
            return
        time.sleep(.5)
    raise TimeoutError('No complete validation row; training was not stopped')


if __name__ == '__main__':
    main()
