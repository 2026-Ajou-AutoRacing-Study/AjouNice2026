#!/usr/bin/env python3
"""Static-only bounded read-only service preparation concurrent with pose waiting.

The actual EventCmd uses the original fresh proxy and reply validation AFTER
fresh post-load pose/topics. No control request is sent by the preparation
thread. Animation, file completeness, stationary/contact/Park QA stay native.
"""
import argparse
import ast
from pathlib import Path
import sys
import threading
import time

import run_base5_capture_optimized as delay
from base5_capture_safety import sha256
from base5_static_ground_policy import lightweight_static


def eligible(meta):
    return not meta['runtime_actors'] or lightweight_static(meta)


def start_probe(meta,wait):
    if not eligible(meta):return None
    done=threading.Event();result={};started=time.monotonic()
    def invoke():
        try:wait('/Service_MoraiEventCmd',timeout=4.);result['ok']=True
        except Exception as exc:result['error']=str(exc)
        finally:result['elapsed_sec']=time.monotonic()-started;done.set()
    thread=threading.Thread(target=invoke,daemon=True);thread.start()
    return done,result,thread


def finish_probe(probe,meta,wait):
    if probe is None:return wait('/Service_MoraiEventCmd',timeout=4.)
    done,result,thread=probe
    if not done.wait(4.):raise RuntimeError('Bounded read-only service preparation timeout')
    thread.join(timeout=.05)
    if not result.get('ok'):raise RuntimeError('Read-only service preparation failed: '+result.get('error','unknown'))
    meta['service_preparation_overlap']=dict(result,policy='read-only bounded probe concurrent with pose; actual command remains fresh/verified')


def compiled_source():
    tree=ast.parse(delay.SOURCE.read_text());counts=dict(load=0,wait=0,delay=0,options=0)
    class Adapter(ast.NodeTransformer):
        def visit_Expr(self,node):
            self.generic_visit(node)
            call=node.value
            if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Attribute) or not isinstance(call.func.value,ast.Name):return node
            name,attr=call.func.value.id,call.func.attr
            if (name,attr)==('loader','publish'):
                counts['load']+=1
                return [node,ast.parse('_service_probe = _start_probe(meta, rospy.wait_for_service)').body[0]]
            if (name,attr)==('rospy','wait_for_service'):
                counts['wait']+=1
                return ast.copy_location(ast.parse('_finish_probe(_service_probe, meta, rospy.wait_for_service)').body[0],node)
            if (name,attr)==('time','sleep') and len(call.args)==1 and isinstance(call.args[0],ast.Constant) and call.args[0].value==.7:
                counts['delay']+=1
                return ast.copy_location(ast.If(test=ast.parse('not _static_fast_ready(meta,state,load_time)',mode='eval').body,body=[node],orelse=[]),node)
            return node
        def visit_Assign(self,node):
            self.generic_visit(node)
            if any(isinstance(t,ast.Name) and t.id=='runner_options' for t in node.targets):
                counts['options']+=1
                return [node,ast.parse('runner_options.update(overlap_adapter=_adapter_contract)').body[0]]
            return node
    tree=Adapter().visit(tree)
    if set(counts.values())!={1}:raise ValueError('Frozen collector AST changed; cannot apply overlap adapter')
    return compile(ast.fix_missing_locations(tree),str(delay.SOURCE),'exec')


def main():
    p=argparse.ArgumentParser(add_help=False);p.add_argument('--setup-profile',choices=['baseline','static-ready'],default='static-ready')
    a,rest=p.parse_known_args()
    if a.setup_profile=='baseline':return delay.main()
    namespace=dict(__file__=str(delay.SOURCE),__name__='base5_overlap_capture',
        _start_probe=start_probe,_finish_probe=finish_probe,_static_fast_ready=delay.static_fast_ready,
        _adapter_contract=dict(adapter_sha256=sha256(__file__),source_sha256=sha256(delay.SOURCE),
            delay_adapter_sha256=sha256(delay.__file__),policy='static bounded read-only service probe overlap'))
    exec(compiled_source(),namespace);sys.argv=[str(delay.SOURCE)]+rest;namespace['main']()


if __name__=='__main__':main()
