"""Explicit, reversible single-frame Base5 inference experiments.

No checkpoint, training forward, global torch function or upstream source is
changed. SDPA/topk can change floating-point results or tied proposal ordering;
this helper is not an accuracy or tracker qualification certificate.
"""
import ast
import hashlib
import inspect
import textwrap
import types

import torch
from torch import nn


# Python 3.11 AST of dedented inspect.getsource on the frozen upstream method.
# A different Python AST schema/source requires an explicit qualification.
FORWARD_AST_SHA256 = 'ff24ebde9a33a958ee25859d58b2b75b58d039a6d5bc7f21d5b0f2569e5c0eef'
MMCV_FORWARD_AST_SHA256 = 'd10f011fa2fada2751dfb595e4c161417281f00a5f8f3edaeec8aec34747250b'
HEAD_MODULE = 'perception.detection3d.bevfusion.plugin.morai_rectangular_bev'
UPSTREAM_MODULE = 'projects.BEVFusion.bevfusion.transfusion_head'
MMCV_MODULE = 'mmcv.cnn.bricks.transformer'
_MISSING = object()


class HeadOptimizationUnavailableError(ValueError):
    """Audited startup/source conditions failed before any wrapper is attached."""


def _tree(function):
    try:
        return ast.parse(textwrap.dedent(inspect.getsource(function)))
    except (OSError, TypeError, SyntaxError) as error:
        raise HeadOptimizationUnavailableError(
            'Cannot inspect the audited head source: ' + str(error)) from error


def _dump(node):
    return ast.dump(node, include_attributes=False)


def _guard_eval(head):
    if head.training or torch.is_grad_enabled():
        raise RuntimeError('Head runtime optimization requires eval and inference_mode/no_grad')


def _validate_head(head):
    if (type(head).__name__ != 'MoraiRectangularTransFusionHead' or
            type(head).__module__ != HEAD_MODULE or
            getattr(head, 'query_memory', None) is not None or
            getattr(head, 'num_classes', None) != 3 or
            getattr(head, 'num_proposals', None) != 200):
        raise HeadOptimizationUnavailableError('Only the audited three-class single-frame Base5 head is supported')
    if head.training:
        raise HeadOptimizationUnavailableError('Head runtime optimization requires eval')
    original = head.forward_single
    function = getattr(original, '__func__', None)
    if (function is None or function.__module__ != UPSTREAM_MODULE or
            function.__qualname__ != 'TransFusionHead.forward_single'):
        raise HeadOptimizationUnavailableError('Head forward_single is not the audited inherited upstream binding')
    tree = _tree(function)
    digest = hashlib.sha256(_dump(tree.body[0]).encode()).hexdigest()
    if digest != FORWARD_AST_SHA256:
        raise HeadOptimizationUnavailableError('Upstream head forward AST changed; qualify the new source explicitly')
    return original, tree, digest


class _Rewrite(ast.NodeTransformer):
    TOPK = ast.parse('top_proposals = heatmap.view(batch_size, -1).argsort('
                     'dim=-1, descending=True)[..., :self.num_proposals]').body[0]
    POSITION = ast.parse('bev_pos = self.bev_pos.repeat(batch_size, 1, 1).to('
                         'fusion_feat.device)').body[0]

    def __init__(self, topk, position):
        self.topk, self.position = topk, position
        self.counts = {'topk': 0, 'position': 0}

    def visit_Assign(self, node):
        if self.topk and _dump(node) == _dump(self.TOPK):
            self.counts['topk'] += 1
            new = ast.parse('top_proposals = heatmap.view(batch_size, -1).topk('
                            'k=self.num_proposals, dim=-1, largest=True, sorted=True).indices').body[0]
            return ast.copy_location(new, node)
        if self.position and _dump(node) == _dump(self.POSITION):
            self.counts['position'] += 1
            new = ast.parse('bev_pos = _base5_cached_position(fusion_feat, batch_size)').body[0]
            return ast.copy_location(new, node)
        return self.generic_visit(node)


def _validate_attention_wrapper(wrapper):
    if (type(wrapper).__module__ != MMCV_MODULE or type(wrapper).__name__ != 'MultiheadAttention' or
            type(wrapper.attn) is not nn.MultiheadAttention or
            getattr(wrapper.attn.forward, '__func__', None) is not nn.MultiheadAttention.forward):
        raise HeadOptimizationUnavailableError('Attention must be an unmodified MMCV wrapper of torch MultiheadAttention')
    # Only qualify wrappers which discard weights. Do not silently change a
    # caller that consumes attention weights for another purpose.
    tree = _tree(wrapper.forward)
    if hashlib.sha256(_dump(tree.body[0]).encode()).hexdigest() != MMCV_FORWARD_AST_SHA256:
        raise HeadOptimizationUnavailableError('MMCV attention wrapper AST changed; qualify the new source explicitly')
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
             isinstance(node.func, ast.Attribute) and node.func.attr == 'attn' and
             isinstance(node.func.value, ast.Name) and node.func.value.id == 'self']
    uses = [node for node in ast.walk(tree) if isinstance(node, ast.Subscript) and
            node.value in calls and isinstance(node.slice, ast.Constant) and node.slice.value == 0]
    if len(calls) != 1 or len(uses) != 1 or any(k.arg == 'need_weights' for k in calls[0].keywords):
        raise HeadOptimizationUnavailableError('MMCV wrapper must exclusively use attention output [0]')


class HeadRuntimeOptimization:
    def __init__(self, head, proposal_topk=False, attention_sdpa=False, cache_bev_pos=False):
        if not any((proposal_topk, attention_sdpa, cache_bev_pos)):
            raise HeadOptimizationUnavailableError('Select at least one explicit head optimization')
        if hasattr(head, '_base5_head_runtime_optimization'):
            raise RuntimeError('Head runtime optimization is already installed; preserve its owner')
        original, tree, digest = _validate_head(head)
        self.head, self.original = head, original
        self.original_local = head.__dict__.get('forward_single', _MISSING)
        self.attention = []
        self.positions = {}
        self.closed = False
        self.flags = dict(proposal_topk=bool(proposal_topk), attention_sdpa=bool(attention_sdpa),
                          cache_bev_pos=bool(cache_bev_pos))
        if cache_bev_pos and not torch.is_tensor(getattr(head, 'bev_pos', None)):
            raise HeadOptimizationUnavailableError('Cached BEV position requires the original head position tensor')
        wrappers = []
        if attention_sdpa:
            decoder = getattr(head, 'decoder', None)
            if not isinstance(decoder, (list, tuple, nn.ModuleList)) or len(decoder) != 1:
                raise HeadOptimizationUnavailableError('Only the audited single decoder layer is supported')
            for layer in decoder:
                for name in ('self_attn', 'cross_attn'):
                    wrapper = getattr(layer, name, None)
                    _validate_attention_wrapper(wrapper)
                    wrappers.append(wrapper)
        rewrite = _Rewrite(proposal_topk, cache_bev_pos)
        if proposal_topk or cache_bev_pos:
            tree = rewrite.visit(tree)
            for key, enabled in (('topk', proposal_topk), ('position', cache_bev_pos)):
                if enabled and rewrite.counts[key] != 1:
                    raise HeadOptimizationUnavailableError('Audited head AST assignment not found exactly once: ' + key)
            ast.fix_missing_locations(tree)
            namespace = dict(original.__func__.__globals__, _base5_cached_position=self._position)
            try:
                exec(compile(tree, inspect.getsourcefile(original), 'exec'), namespace)
            except (SyntaxError, TypeError, ValueError) as error:
                raise HeadOptimizationUnavailableError(
                    'Cannot compile the audited head source: ' + str(error)) from error
            selected = types.MethodType(namespace['forward_single'], head)
        else:
            selected = original

        def forward_single(*args, **kwargs):
            _guard_eval(head)
            inputs = args[0] if args else kwargs.get('inputs')
            if getattr(inputs, 'shape', (0,))[0] != 1:
                raise ValueError('Head runtime optimization requires single-frame batch size one')
            return selected(*args, **kwargs)

        self.installed_forward = forward_single
        # All validation/AST compilation completes before attaching anything.
        try:
            for wrapper in wrappers:
                inner = wrapper.attn
                before = inner.forward
                installed = self._without_weights(before)
                self.attention.append((inner, inner.__dict__.get('forward', _MISSING), installed))
                inner.forward = installed
            head.forward_single = self.installed_forward
            head._base5_head_runtime_optimization = self
        except Exception:
            self._restore()
            raise
        self.report = dict(schema='base5_head_runtime_optimization_v1', **self.flags,
            upstream_forward_ast_sha256=digest, attention_modules=len(self.attention),
            scope='eval only; existing checkpoint and training source are unchanged',
            precision_note='SDPA kernel selection and tied topk ordering may change numerical outputs',
            validated_accuracy=False, validated_tracker=False)

    def _without_weights(self, original):
        def forward(*args, **kwargs):
            _guard_eval(self.head)
            if len(args) > 4:
                if args[4] is not False:
                    raise ValueError('Optimized attention does not return attention weights')
            else:
                if 'need_weights' in kwargs and kwargs['need_weights'] is not False:
                    raise ValueError('Optimized attention does not return attention weights')
                kwargs['need_weights'] = False
            result = original(*args, **kwargs)
            if not isinstance(result, tuple) or len(result) != 2 or result[1] is not None:
                raise RuntimeError('Optimized attention must return (output, None)')
            return result
        return forward

    def _position(self, fusion_feat, batch_size):
        if batch_size != 1:
            raise ValueError('Cached BEV position requires batch size one')
        source = self.head.bev_pos
        key = (id(source), source.data_ptr(), source._version, tuple(source.shape),
               source.dtype, source.device, fusion_feat.device)
        if key not in self.positions:
            self.positions.clear()
            self.positions[key] = source.repeat(1, 1, 1).to(fusion_feat.device)
        return self.positions[key]

    def _restore(self):
        if self.original_local is _MISSING:
            self.head.__dict__.pop('forward_single', None)
        else:
            self.head.forward_single = self.original_local
        for inner, local, _ in reversed(self.attention):
            if local is _MISSING:
                inner.__dict__.pop('forward', None)
            else:
                inner.forward = local
        if getattr(self.head, '_base5_head_runtime_optimization', None) is self:
            del self.head._base5_head_runtime_optimization
        self.positions.clear()

    def close(self):
        if self.closed:
            return
        if self.head.forward_single is not self.installed_forward or any(
                inner.forward is not installed for inner, _, installed in self.attention):
            raise RuntimeError('Head optimization ownership changed; refusing to overwrite another wrapper')
        self._restore()
        self.closed = True

    disable = close


def install_head_runtime_optimization(head, proposal_topk=False, attention_sdpa=False,
                                      cache_bev_pos=False):
    return HeadRuntimeOptimization(head, proposal_topk, attention_sdpa, cache_bev_pos)
