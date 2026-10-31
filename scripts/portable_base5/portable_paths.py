"""Preserve frozen Base5 bytes in an explicitly mapped logical namespace.

This module copies only named, hashed files. It never rewrites JSON paths,
updates source digests, mounts a filesystem, contacts a host, or starts capture.
An installation at a different host path needs an executor which actually
implements the returned logical bindings. Read-only is a requirement in that
plan, not a claim that this module has enforced it.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
from typing import Mapping, Sequence


SCHEMA = "base5_portable_paths_v1"
DEFAULT_LOGICAL_ROOTS = (
    "/home/luv/AjouNice2026",
    "/home/luv/morai_cap",
    "/home/luv/MoraiLauncher_Lin",
)
_FORBIDDEN_PARTS = {".git", ".ssh", ".aws", ".codex", ".agents"}


class PortablePathError(ValueError):
    """A pin, namespace boundary, or copied payload does not match."""


def _require(condition, message):
    if not condition:
        raise PortablePathError(message)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _digest(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def _absolute(value):
    text = os.fspath(value)
    _require(isinstance(text, str) and text.startswith("/") and not text.startswith("//")
             and "\\" not in text and "\x00" not in text, "An absolute POSIX path is required")
    path = PurePosixPath(text)
    _require(str(path) == text and ".." not in path.parts, "Noncanonical absolute path: " + text)
    return path


def _relative(value):
    text = os.fspath(value)
    _require(isinstance(text, str) and text and "\\" not in text and "\x00" not in text,
             "A relative bundle path is required")
    path = PurePosixPath(text)
    _require(not path.is_absolute() and path.parts and ".." not in path.parts
             and str(path) == text and text != ".", "Unsafe bundle path: " + text)
    return path


def _sha(value):
    _require(isinstance(value, str) and len(value) == 64
             and all(c in "0123456789abcdef" for c in value), "Expected lowercase SHA256")
    return value


def bundle_relative_path(original):
    """A reversible mapping; not a string replacement in any source file."""
    path = _absolute(original)
    _require(str(path) != "/", "Cannot package the filesystem root")
    return str(PurePosixPath("payload") / path.relative_to("/"))


def _roots(values):
    roots = tuple(sorted((_absolute(value) for value in values), key=str))
    _require(bool(roots) and len(set(roots)) == len(roots), "Unique logical roots required")
    for index, root in enumerate(roots):
        _require(str(root) != "/", "The filesystem root is not a logical binding")
        for other in roots[index + 1:]:
            _require(root not in other.parents and other not in root.parents,
                     "Logical root bindings must not overlap")
    return roots


def _source_root(path, roots):
    matches = [root for root in roots if root in path.parents]
    _require(len(matches) == 1, "Pinned file is outside the explicit roots: " + str(path))
    return matches[0]


def _no_symlinks(path):
    path = Path(path)
    for item in (path, *path.parents):
        _require(not item.is_symlink(), "Symlink paths are not portable inputs: " + str(item))


def _regular_file(path):
    path = Path(path)
    _no_symlinks(path)
    info = path.stat()
    _require(stat.S_ISREG(info.st_mode), "Pinned input is not a regular file: " + str(path))
    return info


def _not_credential(path):
    name = path.name.lower()
    _require(not (_FORBIDDEN_PARTS & set(path.parts))
             and name not in {".env", "rclone.conf", "id_rsa", "id_ed25519"}
             and not name.startswith("client_secret") and not name.endswith(".pem"),
             "Credential/private state cannot be included: " + str(path))


def _validate_plan(plan):
    _require(isinstance(plan, dict) and plan.get("schema") == SCHEMA, "Portable path plan required")
    expected = plan.get("graph_sha256")
    _sha(expected)
    _require(_digest({key: value for key, value in plan.items() if key != "graph_sha256"}) == expected,
             "Portable path plan digest differs")
    roots = _roots([row["logical_root"] for row in plan["bindings"]])
    _require(plan["bindings"] == [dict(logical_root=str(root),
                                      bundle_path=bundle_relative_path(root),
                                      read_only_required=True) for root in roots],
             "Logical bindings changed")
    files = plan.get("files")
    _require(isinstance(files, dict) and bool(files), "A nonempty pinned file inventory is required")
    seen = set()
    for relative, entry in files.items():
        _relative(relative)
        source = _absolute(entry["source"])
        _not_credential(source)
        _source_root(source, roots)
        _require(relative == bundle_relative_path(source) and str(source) not in seen,
                 "Nonbijective source/bundle mapping")
        seen.add(str(source))
        _sha(entry["sha256"])
        _require(type(entry["bytes"]) is int and entry["bytes"] >= 0, "Invalid file size")
        _require(type(entry["mode"]) is int and 0 <= entry["mode"] <= 0o777, "Invalid file mode")
        _require(isinstance(entry["roles"], list) and entry["roles"]
                 and entry["roles"] == sorted(set(entry["roles"]))
                 and set(entry["roles"]) <= {"runtime", "input", "release", "manifest", "extra"},
                 "Invalid pin roles")
    release = plan["release"]
    _require(release["bundle_path"] == bundle_relative_path(release["source"]), "Release mapping differs")
    record = files[release["bundle_path"]]
    _require(record["sha256"] == release["sha256"] and "release" in record["roles"],
             "Release is not bound to the inventory")
    manifest = plan["manifest"]
    _require(manifest["bundle_path"] == bundle_relative_path(manifest["source"]), "Manifest mapping differs")
    record = files[manifest["bundle_path"]]
    _require(record["sha256"] == manifest["sha256"] and "manifest" in record["roles"],
             "Manifest is not bound to the inventory")
    _require(plan["file_count"] == len(files) and plan["payload_bytes"] == sum(e["bytes"] for e in files.values()),
             "File/byte counts differ")
    _absolute(plan["original_capture_output"])
    _require(plan.get("json_or_raw_rewritten") is False and plan.get("binding_enforced") is False
             and plan.get("new_host_capture_qualified") is False and plan.get("bulk_started") is False,
             "A path plan cannot claim execution or host qualification")
    return roots


def release_pin_plan(release_path, *, extra_pins: Mapping[str, str] | None = None,
                     logical_roots: Sequence[str] = DEFAULT_LOGICAL_ROOTS,
                     environment="sunny13", verify_hashes=True):
    """Collect the frozen release pins, preserving every file's original bytes.

    ``extra_pins`` is for explicitly audited runtime/config dependencies which
    are not release pins. Missing files, conflicts, credentials and symlinks
    fail closed. ``verify_hashes=False`` makes an inventory proposal only;
    copy_graph and verify_graph always check actual file hashes themselves.
    """
    release_path = Path(_absolute(release_path))
    _regular_file(release_path)
    original = release_path.read_bytes()
    release = json.loads(original)
    _require(release.get("schema") == "base5_obstacle_support_release_v1", "Frozen support release required")
    _require(environment in release.get("qualified_environments", []), "Environment is not qualified")
    _require(release.get("bulk_ready") is True and release.get("bulk_blockers") == [],
             "Source release qualification incomplete")
    _require(release.get("bulk_started") is False and release.get("training_started") is False,
             "Only a pre-capture frozen release is supported")
    roots = _roots(logical_roots)
    pins, roles = {}, {}

    def pin(path, digest, role):
        path = str(_absolute(path))
        _source_root(PurePosixPath(path), roots)
        _not_credential(PurePosixPath(path))
        _sha(digest)
        _require(path not in pins or pins[path] == digest, "Conflicting file pins: " + path)
        pins[path] = digest
        roles.setdefault(path, set()).add(role)

    for field, role in (("runtime_hashes", "runtime"), ("input_hashes", "input")):
        values = release.get(field)
        _require(isinstance(values, dict) and bool(values), "Missing release pin set: " + field)
        for path, digest in values.items():
            pin(path, digest, role)
    for path, digest in (extra_pins or {}).items():
        pin(path, digest, "extra")
    release_sha = hashlib.sha256(original).hexdigest()
    pin(release_path, release_sha, "release")
    env = release["environments"][environment]
    _require(env["manifest"] == release["full_manifest"]
             and env["manifest_sha256"] == release["full_manifest_sha256"], "Environment/full manifest differs")
    pin(env["manifest"], env["manifest_sha256"], "manifest")
    files = {}
    for source, digest in sorted(pins.items()):
        info = _regular_file(source)
        if verify_hashes:
            _require(sha256(source) == digest, "Frozen input hash differs: " + source)
        files[bundle_relative_path(source)] = dict(source=source, sha256=digest,
            bytes=info.st_size, mode=stat.S_IMODE(info.st_mode) & 0o777, roles=sorted(roles[source]))
    _require(release_path.read_bytes() == original, "Source release changed while making the plan")
    result = dict(schema=SCHEMA, relocation_strategy="unchanged-bytes-original-logical-namespace",
        environment=environment, release=dict(source=str(release_path), sha256=release_sha,
            bundle_path=bundle_relative_path(release_path)),
        manifest=dict(source=env["manifest"], sha256=env["manifest_sha256"],
            bundle_path=bundle_relative_path(env["manifest"])),
        bindings=[dict(logical_root=str(root), bundle_path=bundle_relative_path(root),
                       read_only_required=True) for root in roots],
        files=files, file_count=len(files), payload_bytes=sum(e["bytes"] for e in files.values()),
        role_counts=dict(Counter(role for entry in files.values() for role in entry["roles"])),
        source_hashes_verified=bool(verify_hashes), original_capture_output=env["capture_output"],
        json_or_raw_rewritten=False, binding_enforced=False, new_host_capture_qualified=False,
        bulk_started=False,
        limitations=["The executor must implement the logical namespace and read-only bindings.",
                     "This is not a native runtime, network, annotation, collision or Park qualification.",
                     "Original output/contracts are historical; create a fresh capture output and host smoke.",
                     "JSON path/SHA/digest dependencies are preserved, never repaired by string replacement."])
    result["graph_sha256"] = _digest(result)
    _validate_plan(result)
    return result


def verify_graph(plan, bundle_root):
    """Verify every packed file plus the exact payload inventory; no source access."""
    _validate_plan(plan)
    root = Path(_absolute(bundle_root))
    _no_symlinks(root)
    _require(root.is_dir(), "Bundle root does not exist")
    payload = root / "payload"
    _require(payload.is_dir() and not payload.is_symlink(), "Bundle payload directory required")
    actual = set()
    for path in payload.rglob("*"):
        _no_symlinks(path)
        if path.is_dir():
            continue
        _regular_file(path)
        actual.add(str(path.relative_to(root)))
    _require(actual == set(plan["files"]), "Bundle inventory contains missing or unlisted files")
    for relative, entry in plan["files"].items():
        path = root / _relative(relative)
        info = _regular_file(path)
        _require(info.st_size == entry["bytes"] and sha256(path) == entry["sha256"],
                 "Packed input hash/size differs: " + relative)
        _require(stat.S_IMODE(info.st_mode) & 0o777 == entry["mode"], "Packed file mode differs: " + relative)
    # A graph checksum alone must not turn an accidentally truncated inventory
    # into a complete release. Derive the mandatory set independently from the
    # preserved release bytes, rather than trusting copied count/role fields.
    release = json.loads((root / plan["release"]["bundle_path"]).read_text())
    _require(release.get("schema") == "base5_obstacle_support_release_v1"
             and release.get("bulk_ready") is True and release.get("bulk_blockers") == []
             and release.get("bulk_started") is False and release.get("training_started") is False,
             "Packed release is not the qualified pre-capture source")
    _require(plan["environment"] in release.get("qualified_environments", []),
             "Packed release does not qualify the planned environment")
    env = release["environments"][plan["environment"]]
    _require(plan["manifest"]["source"] == release["full_manifest"] == env["manifest"]
             and plan["manifest"]["sha256"] == release["full_manifest_sha256"] == env["manifest_sha256"]
             and plan["original_capture_output"] == env["capture_output"],
             "Packed release/manifest/environment binding differs")
    required_count = 0
    for field, role in (("runtime_hashes", "runtime"), ("input_hashes", "input")):
        values = release.get(field)
        _require(isinstance(values, dict) and bool(values), "Packed release pin set is missing")
        for source, digest in values.items():
            entry = plan["files"].get(bundle_relative_path(source))
            _require(entry is not None and entry["source"] == source and entry["sha256"] == digest
                     and role in entry["roles"], "Packed graph omits/changes a release pin: " + source)
            required_count += 1
    return dict(schema="base5_portable_graph_verification_v1", graph_sha256=plan["graph_sha256"],
                bundle_root=str(root), files_checked=len(actual), payload_bytes=plan["payload_bytes"],
                required_release_pin_occurrences=required_count,
                exact_payload_inventory=True, file_hashes_verified=True, json_or_raw_rewritten=False,
                binding_enforced=False, new_host_capture_qualified=False, bulk_started=False)


def copy_graph(plan, destination):
    """Copy to a new local bundle directory; do not change sources or retry outputs."""
    _validate_plan(plan)
    root = Path(_absolute(destination))
    _no_symlinks(root)
    _require(not root.exists(), "A fresh bundle directory is required")
    _require(all(root != Path(e["source"]) and root not in Path(e["source"]).parents
                 for e in plan["files"].values()), "Bundle output overlaps a source input")
    root.mkdir(parents=True)
    for relative, entry in plan["files"].items():
        source = Path(entry["source"])
        before = _regular_file(source)
        _require(before.st_size == entry["bytes"], "Source file size changed: " + str(source))
        target = root / _relative(relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256()
        with source.open("rb") as original, target.open("xb") as copied:
            for block in iter(lambda: original.read(4 * 1024 * 1024), b""):
                copied.write(block)
                digest.update(block)
        after = _regular_file(source)
        _require(digest.hexdigest() == entry["sha256"] and
                 (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
                 (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
                 "Source input changed during copy: " + str(source))
        target.chmod(entry["mode"])
    receipt = verify_graph(plan, root)
    with (root / "portable_graph.json").open("x") as handle:
        json.dump(plan, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    return receipt


def host_mount_plan(plan, bundle_root, capture_output, *, logical_capture="/capture"):
    """Describe, but never apply, a new host installation's required bindings."""
    roots = _validate_plan(plan)
    verified = verify_graph(plan, bundle_root)
    root = Path(verified["bundle_root"])
    output = Path(_absolute(capture_output))
    _no_symlinks(output)
    _require(output != root and output not in root.parents and root not in output.parents,
             "Capture output must not overlap the immutable bundle")
    _require(not output.exists() or (output.is_dir() and not any(output.iterdir())),
             "Use a fresh, empty capture output; never relocate an existing capture contract")
    target = _absolute(logical_capture)
    _require(str(target) != "/" and all(target != source and target not in source.parents
             and source not in target.parents for source in roots),
             "Writable capture binding overlaps frozen logical inputs")
    bindings = []
    for entry in plan["bindings"]:
        source = root / _relative(entry["bundle_path"])
        _require(source.is_dir(), "Logical input root is empty or absent: " + str(source))
        bindings.append(dict(host_path=str(source), logical_path=entry["logical_root"],
                             read_only_required=True))
    return dict(schema="base5_portable_host_mount_plan_v1", graph_sha256=plan["graph_sha256"],
        bundle_verification=verified, input_bindings=bindings,
        capture_binding=dict(host_path=str(output), logical_path=str(target), read_write_required=True),
        release_path=plan["release"]["source"], manifest_path=plan["manifest"]["source"],
        launcher_arguments=["--release", plan["release"]["source"], "--environment", plan["environment"],
                            "--output", str(target)],
        executor_selected=None, bindings_applied=False, read_only_enforced=False,
        json_or_raw_rewritten=False, new_host_capture_qualified=False, bulk_started=False,
        required_before_capture=["Implement and validate logical bindings with a chosen executor.",
            "Verify current native runtime, sensor/network/output permissions and available disk.",
            "Run separately authorized native smoke with annotation, collision and Park checks.",
            "Use a new capture contract; --run requires explicit collection authorization."])
