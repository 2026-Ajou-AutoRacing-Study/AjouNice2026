import copy
from unittest.mock import patch

import pytest

from test_base5_shadow_ground_reuse import registered, reuse, atomic_json


def test_materialized_ground_is_not_prepared_twice(tmp_path):
    f=registered(tmp_path)
    prepared=reuse.adopt(f.manifest,tmp_path/'prepared')
    before={p:p.read_bytes() for p in prepared.parent.iterdir()}
    assert reuse.adopt(prepared,tmp_path/'unused')==prepared
    assert not (tmp_path/'unused').exists()
    assert before=={p:p.read_bytes() for p in prepared.parent.iterdir()}


@pytest.mark.parametrize('prepared', [False,True])
def test_explicit_verified_unit_tag_does_not_reject_unchanged_ground(tmp_path,prepared):
    f=registered(tmp_path)
    source=reuse.adopt(f.manifest,tmp_path/'prepared') if prepared else f.manifest
    deck=reuse.read(source)
    original=deck['scenes'][0]
    meta=reuse.read(original['metadata'])
    ref=dict(path='/pinned/review.json',sha256='expected')
    meta['native_unit_geometry_reference']=ref
    target=tmp_path/'derived_metadata.json';atomic_json(target,meta)
    deck['config']['native_unit_geometry_reference']=ref
    deck['scenes'][0]=dict(original,metadata=str(target))
    with patch('base5_unit_geometry_contract.validate_entry') as verify:
        reuse.validate_manifest(deck)
        verify.assert_called_once_with(deck['scenes'][0],deck)
    # This patch exercises only verified tag plumbing. Unit contract source,
    # catalog and byte hashes are exercised separately without a mock.
    meta['runtime_actors'][0]['position']['z']+=.1
    atomic_json(target,meta)
    with patch('base5_unit_geometry_contract.validate_entry'):
        with pytest.raises(reuse.ShadowReuseContractError):reuse.validate_manifest(deck)


def test_unknown_derived_field_does_not_bypass_exact_metadata(tmp_path):
    f=registered(tmp_path)
    deck=reuse.read(f.manifest)
    meta=reuse.read(deck['scenes'][0]['metadata'])
    meta['unregistered_qa_override']=True
    target=tmp_path/'foreign.json';atomic_json(target,meta)
    deck['scenes'][0]['metadata']=str(target)
    with pytest.raises(reuse.ShadowReuseContractError):reuse.validate_manifest(deck)


def test_missing_training_overlay_does_not_authorize_derived_quota(tmp_path):
    f=registered(tmp_path)
    deck=reuse.read(f.manifest)
    deck['split_overlay']=dict(path='/missing.json',sha256='missing')
    deck['scenes'][0]['derived_training_split']='train'
    with pytest.raises(reuse.ShadowReuseContractError):reuse.validate_manifest(deck)
