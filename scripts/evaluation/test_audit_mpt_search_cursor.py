import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('cursor_audit',Path(__file__).with_name('audit_mpt_search_cursor.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_aligned_but_blocked_previous_seed_starves_other_side_under_single_attempt_budget():
    args=dict(mask_count=2,preferred_mask=0,feasible_masks={1},budget_masks=1,callbacks=16)
    old=module.budgeted_search(**args,candidate=False)
    new=module.budgeted_search(**args,candidate=True)
    assert all(row['examined']==[0] and row['winner'] is None for row in old)
    assert [row['examined'] for row in new]==[[0],[1]]
    assert new[-1]['winner']==1


def test_multiple_sides_are_explored_across_callbacks_without_exceeding_budget():
    args=dict(mask_count=8,preferred_mask=0,feasible_masks={6},budget_masks=2,callbacks=16)
    old=module.budgeted_search(**args,candidate=False)
    new=module.budgeted_search(**args,candidate=True)
    assert all(row['examined']==[0,1] for row in old)
    assert [row['examined'] for row in new]==[[0,1],[2,3],[4,5],[6]]
    assert all(len(row['examined'])<=2 for row in new)


def test_validated_incumbent_keeps_the_same_spatial_side_preference():
    args=dict(mask_count=4,preferred_mask=3,feasible_masks={3},budget_masks=1,callbacks=4,incumbent=True)
    assert module.budgeted_search(**args,candidate=False)==module.budgeted_search(**args,candidate=True)
    assert module.budgeted_search(**args,candidate=True)[0]['winner']==3


def test_no_previous_seed_preserves_existing_cursor_behavior():
    args=dict(mask_count=4,preferred_mask=0,feasible_masks={3},budget_masks=1,callbacks=8,previous_seed=False)
    assert module.budgeted_search(**args,candidate=False)==module.budgeted_search(**args,candidate=True)


def test_exhausted_search_never_fabricates_a_feasible_solution():
    result=module.budgeted_search(mask_count=4,preferred_mask=0,feasible_masks=set(),
        budget_masks=1,callbacks=8,candidate=True)
    assert all(row['winner'] is None for row in result)
    assert [row['examined'][0] for row in result]==[0,1,2,3,0,1,2,3]


def test_saved_cursor_wraps_after_scene_size_changes():
    assert module.first_mask(7,2,True,0,False,True)==1
    assert module.first_mask(7,2,True,0,True,True)==0


def test_runtime_source_gates_side_hint_on_revalidated_incumbent_and_retains_cursor():
    root=Path(__file__).resolve().parents[2]
    source=(root/'planning_ws/src/autoware_path_optimizer/src/node.cpp').read_text()
    contract=module.source_contract(source)
    assert contract['candidate_incumbent_gated_override']
    assert not contract['unconditional_previous_seed_override']
    assert contract['cursor_advanced']
