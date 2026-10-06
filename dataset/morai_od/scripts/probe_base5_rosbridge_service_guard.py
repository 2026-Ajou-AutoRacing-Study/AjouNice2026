#!/usr/bin/env python3
"""Reproduce installed rosbridge cross-generation response theft, then test opt-in guard."""
import argparse
from pathlib import Path
from base5_capture_safety import atomic_json
from base5_rosbridge_service_guard import install,reproduce


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Evidence overwrite refused')
    import rosbridge_library.capabilities.advertise_service as module
    before=reproduce(module);install(module);after=reproduce(module)
    report=dict(before=before,after=after,passed=before['old_consumed_new_response'] and not after['shared_response_store'] and not after['ids_equal'] and not after['old_consumed_new_response'])
    atomic_json(args.output,report);print(report,flush=True)
    raise SystemExit(0 if report['passed'] else 1)
