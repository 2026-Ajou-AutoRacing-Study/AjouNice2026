#!/usr/bin/env python3
"""Bounded scheduler for the frozen one-shot supplement, with shared map caches.

The original collector and supplement journal contracts remain unchanged. This
additional entrypoint records its own guard. Cache prewarming changes no pose,
GT or QA threshold. A time budget requests PAUSE at a safe attempt boundary.
"""
import argparse
import fcntl
from pathlib import Path
import threading
import time

import run_base5_failure_aware_replenishment as fa
from base5_map_surface import configure
from base5_region_contract import regions


def prewarm(ctx,config):
    # configure_static_geometry returns a shallow context copy. Without these
    # keys on the shared original, every candidate rebuilds the road index and
    # region union and discards its height cache at the end of gen.build.
    configure(ctx,config.get('placement_ground'))
    if config.get('regional_policy'):regions(fa.gen,ctx)
    return ctx


def run(args):
    if not 30<=args.time_budget_sec<=1800:raise ValueError('Time budget must be 30–1800 seconds')
    args.output.mkdir(parents=True,exist_ok=True)
    # Do not create a budget PAUSE in an already-running collection.
    with (args.output/'.supplement.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard=args.output/'fast_scheduler_contract.json'
        contract=dict(schema=1,wrapper_sha256=fa.sha256(__file__),
            supplement_driver_sha256=fa.sha256(fa.__file__),
            generator_sha256=fa.sha256(fa.gen.__file__),
            policy='shared pure caches; bounded safe-boundary PAUSE; no collector/GT changes')
        if guard.exists() and fa.read(guard)!=contract:raise ValueError('Fast scheduler changed; inspect before resume')
        if not guard.exists():fa.atomic_json(guard,contract)
        pause=args.output/'PAUSE'
        if args.resume and pause.exists():
            try:owned=fa.read(pause).get('owner')=='failure_aware_time_budget'
            except (ValueError,OSError):owned=False
            if owned:pause.unlink()  # Explicit --resume clears only our own budget request.
    original_context=fa.gen.context
    config=fa.read(args.manifest)['config']
    def cached_context(*a,**kw):return prewarm(original_context(*a,**kw),config)
    # Dependency injection only around this call; the exact gen.build and
    # capture runtime are untouched, and wrapper code is separately guarded.
    fa.gen.context=cached_context
    def pause_at_budget():
        if not pause.exists():fa.atomic_json(pause,dict(owner='failure_aware_time_budget',
            reason='Finish current attempt and pause',time=time.time()))
    timer=threading.Timer(args.time_budget_sec,pause_at_budget);timer.daemon=True
    if args.run:timer.start()
    try:return fa.execute(args.manifest,args.baseline,args.output,args.resume,args.run,
        args.stop_after_cells,args.max_attempts,args.candidates,getattr(args,'import_supplement',None))
    finally:
        timer.cancel();fa.gen.context=original_context


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','baseline','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--resume',action='store_true');p.add_argument('--run',action='store_true')
    p.add_argument('--stop-after-cells',type=int,default=0)
    p.add_argument('--max-attempts',type=int,default=2);p.add_argument('--candidates',type=int,default=16)
    p.add_argument('--time-budget-sec',type=int,default=900)
    p.add_argument('--import-supplement',type=Path)
    args=p.parse_args();raise SystemExit(run(args))
