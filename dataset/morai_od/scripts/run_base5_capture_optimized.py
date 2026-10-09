#!/usr/bin/env python3
"""Opt-in static setup experiment over the frozen collector; all QA stays native.

Only the pre-EventCmd 0.7s static delay is removed. Fresh proxy, verified reply,
bounded retry, per-load transport, stationary/contact gates and file stability
are unchanged. Animated scenes keep the original delay. This is not a claim of
bulk qualification or a persistent worker across separately supervised chunks.
"""
import argparse
import ast
from pathlib import Path
import sys

from base5_capture_safety import sha256


SOURCE = Path(__file__).with_name('run_base5_capture.py')


def compiled_source(profile):
    if profile not in ('baseline','static-ready'): raise ValueError('Unknown setup profile')
    tree = ast.parse(SOURCE.read_text()); matches = 0; options = 0
    class Adapter(ast.NodeTransformer):
        def visit_Expr(self,node):
            nonlocal matches
            self.generic_visit(node)
            if (isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
                    and isinstance(node.value.func.value,ast.Name) and node.value.func.value.id=='time'
                    and node.value.func.attr=='sleep' and len(node.value.args)==1
                    and isinstance(node.value.args[0],ast.Constant) and node.value.args[0].value==.7):
                matches += 1
                if profile == 'static-ready':
                    condition = ast.parse('not _static_fast_ready(meta, state, load_time)',mode='eval').body
                    return ast.copy_location(ast.If(test=condition,body=[node],orelse=[]),node)
            return node
        def visit_Assign(self,node):
            nonlocal options
            self.generic_visit(node)
            if any(isinstance(t,ast.Name) and t.id=='runner_options' for t in node.targets):
                options += 1
                return [node,ast.parse('runner_options.update(optimized_adapter=_adapter_contract)').body[0]]
            return node
    tree = Adapter().visit(tree)
    if matches != 1 or options != 1: raise ValueError('Frozen collector structure changed; inspect adapter before use')
    return compile(ast.fix_missing_locations(tree),str(SOURCE),'exec')


def static_fast_ready(meta,state,load_time):
    import time
    from base5_static_ground_policy import lightweight_static
    from base5_capture_timing import post_load_topics_ready, deferred_static_secondary
    now = time.monotonic()
    # Background has no actor: requires the same fresh post-load transport.
    static = lightweight_static(meta) or not meta['runtime_actors']
    return static and (post_load_topics_ready(state,meta['runtime_actors'],load_time,now)
        or bool(deferred_static_secondary(state,meta['runtime_actors'],load_time,now,allow_missing_primary=True)))


def main():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--setup-profile',choices=['baseline','static-ready'],default='baseline')
    a,rest = p.parse_known_args()
    namespace = dict(__file__=str(SOURCE),__name__='base5_optimized_capture',
        _static_fast_ready=static_fast_ready,
        _adapter_contract=dict(profile=a.setup_profile,adapter_sha256=sha256(__file__),
            source_sha256=sha256(SOURCE),policy='static conditional setup delay only; native gates'))
    exec(compiled_source(a.setup_profile),namespace)
    sys.argv = [str(SOURCE)]+rest
    namespace['main']()


if __name__ == '__main__': main()
