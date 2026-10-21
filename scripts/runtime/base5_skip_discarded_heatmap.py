"""Opt-in removal of a discarded legacy target, not a new loss or head.

The rectangular head calls the legacy parent's full target builder and then
replaces its last (dense heatmap) result with its own correct XY heatmap. This
qualified adapter skips only the discarded parent heatmap. Assignment, all
seven used parent outputs, the rectangular heatmap, parameters and RNG remain
unchanged. Frozen upstream files and unrelated/legacy heads are not edited.
"""
import ast
import copy
import hashlib
import inspect
from pathlib import Path

VERSION = 'base5_skip_discarded_heatmap_v1'
PARENT_SHA256 = '7712b2c65649f2fcd5f9bd852a8f4406e4081af8e9137001a565261d9d3699f1'
RECTANGULAR_SHA256 = '92f7611c6f7cf11738fe86a1352340ebc7e35a8717dd5ed45212b717419d3581'
_installed = None


def _require(condition, message):
    if not condition: raise ValueError(message)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strip_discarded_block(function):
    """Strict structural guard in addition to the complete upstream file SHA."""
    function = copy.deepcopy(function)
    _require(function.name == 'get_targets_single', 'Unexpected parent target function')
    starts = [i for i, node in enumerate(function.body) if isinstance(node, ast.Assign)
              and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
              and node.targets[0].id == 'device']
    _require(len(starts) == 1, 'Expected one legacy dense-target block')
    start = starts[0]
    block = function.body[start:start+8]
    names = [node.targets[0].id if isinstance(node, ast.Assign)
             and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) else None
             for node in block[:7]]
    _require(names == ['device', 'gt_bboxes_3d', 'grid_size', 'pc_range',
                       'voxel_size', 'feature_map_size', 'heatmap']
             and len(block) == 8 and isinstance(block[-1], ast.For),
             'Legacy dense-target statements changed')
    _require(ast.dump(block[0].value) == ast.dump(ast.parse('labels.device', mode='eval').body),
             'Placeholder device must be the original target-label device')
    tail = function.body[start+8:]
    _require(len(tail) == 2 and isinstance(tail[0], ast.Assign)
             and isinstance(tail[1], ast.Return) and isinstance(tail[1].value, ast.Tuple)
             and len(tail[1].value.elts) == 8
             and ast.dump(tail[1].value.elts[-1]) == ast.dump(ast.parse('heatmap[None]', mode='eval').body),
             'Only the last discarded tuple element may change')
    retained_tail_names = {node.id for node in ast.walk(tail[0]) if isinstance(node, ast.Name)}
    _require(not retained_tail_names & {'device', 'gt_bboxes_3d', 'grid_size', 'pc_range',
                                       'voxel_size', 'feature_map_size', 'heatmap'},
             'A retained target depends on the discarded heatmap block')
    # The rectangular caller reads only targets[-1].device before replacing it.
    # empty() is deterministic, does not consume RNG, and has no autograd edge.
    placeholder = ast.parse('heatmap = labels.new_empty((0,), dtype=torch.float32)').body
    function.body = function.body[:start] + placeholder + tail
    function.name = '_qualified_parent_without_discarded_heatmap'
    function.decorator_list = []
    return ast.fix_missing_locations(function)


def specialize_parent(source, namespace, *, check_sha=True):
    if check_sha:
        _require(hashlib.sha256(source.encode()).hexdigest() == PARENT_SHA256,
                 'Qualified parent source SHA differs')
    tree = ast.parse(source)
    parents = [node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'TransFusionHead']
    _require(len(parents) == 1, 'Expected one original TransFusionHead')
    functions = [node for node in parents[0].body if isinstance(node, ast.FunctionDef)
                 and node.name == 'get_targets_single']
    _require(len(functions) == 1, 'Expected one original target method')
    candidate = strip_discarded_block(functions[0])
    scope = dict(namespace)
    exec(compile(ast.Module(body=[candidate], type_ignores=[]),
                 '<qualified-skip-discarded-dense-target>', 'exec'), scope)
    return scope[candidate.name]


def guarded_parent(original, candidate, rectangular_class):
    """A legacy/E25 or other nonrectangular head always takes its original path."""
    def get_targets_single(self, *args, **kwargs):
        function = candidate if isinstance(self, rectangular_class) else original
        return function(self, *args, **kwargs)
    return get_targets_single


def receipt():
    return dict(version=VERSION, script_sha256=_sha(__file__),
                parent_source_sha256=PARENT_SHA256,
                rectangular_source_sha256=RECTANGULAR_SHA256,
                operation='skip discarded legacy heatmap only; retain rectangular XY target')


def install():
    global _installed
    from projects.BEVFusion.bevfusion.transfusion_head import TransFusionHead
    from perception.detection3d.bevfusion.plugin.morai_transfusion_head import MoraiTransFusionHead
    from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import MoraiRectangularTransFusionHead
    if _installed is not None: return receipt()
    parent_path = Path(inspect.getsourcefile(TransFusionHead))
    rectangular_path = Path(inspect.getsourcefile(MoraiRectangularTransFusionHead))
    _require(_sha(parent_path) == PARENT_SHA256 and _sha(rectangular_path) == RECTANGULAR_SHA256,
             'Exact qualified source files required; do not patch another head revision')
    original = MoraiTransFusionHead.get_targets_single
    _require(original is TransFusionHead.get_targets_single,
             'Another parent-target patch is already installed')
    candidate = specialize_parent(parent_path.read_text(), original.__globals__)
    own_attribute = 'get_targets_single' in MoraiTransFusionHead.__dict__
    MoraiTransFusionHead.get_targets_single = guarded_parent(
        original, candidate, MoraiRectangularTransFusionHead)
    _installed = (MoraiTransFusionHead, original, own_attribute)
    return receipt()


def uninstall():
    """For sequential isolated A/B qualification; not a training-time toggle."""
    global _installed
    if _installed is None: return
    parent, original, own_attribute = _installed
    if own_attribute: parent.get_targets_single = original
    else: delattr(parent, 'get_targets_single')
    _installed = None
