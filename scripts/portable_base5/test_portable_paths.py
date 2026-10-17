"""Temporary local-only tests; no frozen sources, ROS, mounts or network used."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location("portable_paths", Path(__file__).with_name("portable_paths.py"))
paths = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(paths)


@pytest.fixture
def source(tmp_path):
    repo, raw, launcher = (tmp_path / name for name in ("repo", "raw", "launcher"))
    for root in (repo, raw, launcher):
        root.mkdir()
    runtime = repo / "collector.py"
    runtime.write_text("# no imports or calls: immutable fixture\n")
    runtime.chmod(0o755)
    proof = launcher / "catalog.json"
    proof.write_text('{"asset":"WoodBox","size":[1,1,1]}\n')
    native = raw / "Instance.bin"
    native.write_bytes(b"\x00\xffnative-geometry\x00")
    # Nested path+hash and a content digest must stay byte-identical, not be
    # replaced with installation paths or a newly minted approval.
    overlay = repo / "overlay.json"
    overlay.write_text(json.dumps(dict(source=str(native), sha256=paths.sha256(native),
        overlay_sha256="2" * 64, metadata_reference=dict(path=str(proof), sha256=paths.sha256(proof)))))
    manifest = repo / "manifest.json"
    manifest.write_text(json.dumps(dict(count=5000, overlay=dict(path=str(overlay), sha256=paths.sha256(overlay)))))
    release = repo / "release.json"
    release.write_text(json.dumps(dict(schema="base5_obstacle_support_release_v1", bulk_ready=True,
        bulk_blockers=[], bulk_started=False, training_started=False, completion_live_qualified=False,
        full_manifest=str(manifest), full_manifest_sha256=paths.sha256(manifest),
        qualified_environments=["sunny13"], runtime_hashes={str(runtime): paths.sha256(runtime)},
        input_hashes={str(p): paths.sha256(p) for p in (native, overlay, proof, manifest)},
        environments=dict(sunny13=dict(manifest=str(manifest),manifest_sha256=paths.sha256(manifest),
            capture_output=str(raw / "old-output"))))))
    return dict(release=release, roots=[str(repo), str(raw), str(launcher)], runtime=runtime,
                native=native, overlay=overlay, manifest=manifest, proof=proof)


def make_plan(source, **kwargs):
    return paths.release_pin_plan(source["release"], logical_roots=source["roots"], **kwargs)


def write_release(source, change):
    doc = json.loads(source["release"].read_text())
    change(doc)
    source["release"].write_text(json.dumps(doc))


def resign(plan):
    plan["graph_sha256"] = paths._digest({k: v for k, v in plan.items() if k != "graph_sha256"})
    return plan


def test_copy_to_different_installation_preserves_nested_json_and_raw_bytes(source, tmp_path):
    plan = make_plan(source)
    original = {p: p.read_bytes() for p in (source["release"], source["overlay"], source["native"])}
    bundle = tmp_path / "different-installation" / "bundle"
    copied = paths.copy_graph(plan, bundle)
    assert copied["files_checked"] == plan["file_count"] == 6
    for source_path, payload in original.items():
        assert (bundle / paths.bundle_relative_path(source_path)).read_bytes() == payload
        assert source_path.read_bytes() == payload
    assert paths.verify_graph(plan, bundle)["file_hashes_verified"]
    assert not plan["binding_enforced"] and not plan["new_host_capture_qualified"]
    assert json.loads((bundle / "portable_graph.json").read_text()) == plan


def test_all_roles_and_explicit_extra_pins_retained(source):
    extra = Path(source["roots"][0]) / "extra.json"
    extra.write_text("{}")
    plan = make_plan(source, extra_pins={str(extra): paths.sha256(extra),
                                         str(source["runtime"]): paths.sha256(source["runtime"])})
    assert plan["files"][paths.bundle_relative_path(source["runtime"])]["roles"] == ["extra", "runtime"]
    assert plan["files"][paths.bundle_relative_path(source["manifest"])]["roles"] == ["input", "manifest"]
    assert plan["file_count"] == 7
    assert plan == make_plan(source, extra_pins={str(extra): paths.sha256(extra),
                                                str(source["runtime"]): paths.sha256(source["runtime"])})


@pytest.mark.parametrize("value", ["relative/file", "/a/../b", "/a//b", "//a/b", "/a\\b", "/"])
def test_noncanonical_paths_rejected(value):
    with pytest.raises(paths.PortablePathError):
        paths.bundle_relative_path(value)


def test_conflicting_pins_rejected(source):
    with pytest.raises(paths.PortablePathError, match="Conflicting"):
        make_plan(source, extra_pins={str(source["runtime"]): "0" * 64})


def test_outside_explicit_root_rejected(source, tmp_path):
    outside = tmp_path / "foreign.txt"
    outside.write_text("private")
    with pytest.raises(paths.PortablePathError, match="outside"):
        make_plan(source, extra_pins={str(outside): paths.sha256(outside)})


def test_nested_roots_rejected(source):
    with pytest.raises(paths.PortablePathError, match="overlap"):
        paths.release_pin_plan(source["release"], logical_roots=source["roots"] + [str(source["runtime"].parent / "nested")])


@pytest.mark.parametrize("private", [".ssh/id_ed25519", ".aws/credentials", "rclone.conf", "client_secret_x.json"])
def test_private_state_is_not_packaged(source, private):
    p = Path(source["roots"][0]) / private
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("never disclose")
    with pytest.raises(paths.PortablePathError, match="Credential"):
        make_plan(source, extra_pins={str(p): paths.sha256(p)})


def test_missing_pinned_file_rejected(source):
    source["native"].unlink()
    with pytest.raises(FileNotFoundError):
        make_plan(source)


def test_existing_wrong_bytes_rejected(source):
    source["native"].write_bytes(b"changed")
    with pytest.raises(paths.PortablePathError, match="hash differs"):
        make_plan(source)


def test_directory_pin_rejected(source):
    p = Path(source["roots"][0]) / "directory"
    p.mkdir()
    with pytest.raises(paths.PortablePathError, match="regular"):
        make_plan(source, extra_pins={str(p): "0" * 64})


@pytest.mark.parametrize("kind", ["file", "ancestor"])
def test_source_symlink_rejected(source, kind):
    target = source["native"]
    link = target.parent / "alias.bin" if kind == "file" else target.parent / "alias"
    link.symlink_to(target if kind == "file" else target.parent, target_is_directory=kind == "ancestor")
    p = link if kind == "file" else link / target.name
    with pytest.raises(paths.PortablePathError, match="Symlink"):
        make_plan(source, extra_pins={str(p): paths.sha256(p)})


def test_unqualified_environment_or_source_rejected(source):
    with pytest.raises(paths.PortablePathError, match="Environment"):
        make_plan(source, environment="foggy13")
    write_release(source, lambda doc: doc.update(bulk_ready=False))
    with pytest.raises(paths.PortablePathError, match="qualification"):
        make_plan(source)


def test_environment_manifest_mismatch_rejected(source):
    write_release(source, lambda doc: doc["environments"]["sunny13"].update(manifest_sha256="0" * 64))
    with pytest.raises(paths.PortablePathError, match="manifest differs"):
        make_plan(source)


def test_copy_always_checks_hash_even_for_inventory_proposal(source, tmp_path):
    plan = make_plan(source, verify_hashes=False)
    source["runtime"].write_text("# same-length corruption for proposal\n")
    assert not plan["source_hashes_verified"]
    with pytest.raises(paths.PortablePathError, match="changed"):
        paths.copy_graph(plan, tmp_path / "bundle")


def test_existing_destination_is_never_overwritten(source, tmp_path):
    p = tmp_path / "existing"
    p.mkdir()
    with pytest.raises(paths.PortablePathError, match="fresh"):
        paths.copy_graph(make_plan(source), p)


def test_copied_bytes_mode_missing_and_extra_files_rejected(source, tmp_path):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    p = root / paths.bundle_relative_path(source["runtime"])
    original = p.read_bytes()
    p.write_bytes(b"X" * len(original))
    with pytest.raises(paths.PortablePathError, match="hash/size"):
        paths.verify_graph(plan, root)
    p.write_bytes(original)
    p.chmod(0o644)
    with pytest.raises(paths.PortablePathError, match="mode"):
        paths.verify_graph(plan, root)
    p.chmod(0o755)
    extra = root / "payload" / "extra.bin"
    extra.write_bytes(b"unregistered")
    with pytest.raises(paths.PortablePathError, match="inventory"):
        paths.verify_graph(plan, root)
    extra.unlink()
    p.unlink()
    with pytest.raises(paths.PortablePathError, match="inventory"):
        paths.verify_graph(plan, root)


def test_bundle_symlink_rejected(source, tmp_path):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    p = root / paths.bundle_relative_path(source["runtime"])
    p.unlink()
    p.symlink_to(source["runtime"])
    with pytest.raises(paths.PortablePathError, match="Symlink"):
        paths.verify_graph(plan, root)


def test_plan_tampering_and_resigned_nonbijective_mapping_rejected(source, tmp_path):
    plan = make_plan(source)
    modified = deepcopy(plan)
    modified["bulk_started"] = True
    with pytest.raises(paths.PortablePathError, match="digest"):
        paths.verify_graph(modified, tmp_path)
    modified = deepcopy(plan)
    relative = paths.bundle_relative_path(source["native"])
    modified["files"][relative]["source"] = str(source["runtime"])
    with pytest.raises(paths.PortablePathError, match="Nonbijective"):
        paths.verify_graph(resign(modified), tmp_path)


def test_release_pin_coverage_cannot_be_removed_by_resigning_the_plan(source, tmp_path):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    relative = paths.bundle_relative_path(source["native"])
    removed = plan["files"].pop(relative)
    (root / relative).unlink()
    plan["file_count"] -= 1
    plan["payload_bytes"] -= removed["bytes"]
    with pytest.raises(paths.PortablePathError, match="release pin"):
        paths.verify_graph(resign(plan), root)


def test_packed_release_environment_binding_is_checked_not_just_manifest_sha(source, tmp_path):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    plan["original_capture_output"] = "/different/output"
    with pytest.raises(paths.PortablePathError, match="binding"):
        paths.verify_graph(resign(plan), root)


def test_moved_bundle_verification_does_not_need_original_inputs(source, tmp_path):
    plan = make_plan(source)
    first = tmp_path / "first"
    paths.copy_graph(plan, first)
    moved = tmp_path / "elsewhere" / "bundle"
    moved.parent.mkdir()
    first.rename(moved)
    source["native"].unlink()
    source["release"].unlink()
    assert paths.verify_graph(plan, moved)["file_hashes_verified"]


def test_mount_plan_is_explicit_readonly_requirement_not_an_executor(source, tmp_path):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    output = tmp_path / "new-capture"
    mounted = paths.host_mount_plan(plan, root, output)
    assert not output.exists()
    assert mounted["capture_binding"]["logical_path"] == "/capture"
    assert all(b["read_only_required"] for b in mounted["input_bindings"])
    assert mounted["launcher_arguments"][-2:] == ["--output", "/capture"]
    assert "--run" not in mounted["launcher_arguments"]
    assert not mounted["bindings_applied"] and not mounted["read_only_enforced"]
    assert mounted["executor_selected"] is None and not mounted["new_host_capture_qualified"]


@pytest.mark.parametrize("bad", ["bundle-child", "ancestor", "logical-input", "nonempty"])
def test_writable_output_does_not_overlap_or_reuse_inputs(source, tmp_path, bad):
    plan = make_plan(source)
    root = tmp_path / "bundle"
    paths.copy_graph(plan, root)
    output, logical = tmp_path / "new-output", "/capture"
    if bad == "bundle-child":
        output = root / "output"
    elif bad == "ancestor":
        output = tmp_path
    elif bad == "logical-input":
        logical = source["roots"][0] + "/output"
    else:
        output.mkdir()
        (output / "old-state").write_text("preserve")
    with pytest.raises(paths.PortablePathError):
        paths.host_mount_plan(plan, root, output, logical_capture=logical)
