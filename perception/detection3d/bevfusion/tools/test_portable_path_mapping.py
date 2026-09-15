#!/usr/bin/env python3
"""Dependency-light regression test for Base3 handoff path remapping."""

from pathlib import Path

from build_base3_bevfusion_infos import remap_capture_path


def main():
    recorded = Path("/home/source/morai_cap/session")
    actual = Path("/mnt/team/base3")
    serialized = Path("/datasets/base3")
    read_path, info_path = remap_capture_path(
        "/home/source/morai_cap/session/batch001/frame.bin",
        recorded, actual, serialized)
    assert read_path == Path("/mnt/team/base3/batch001/frame.bin")
    assert info_path == Path("/datasets/base3/batch001/frame.bin")
    try:
        remap_capture_path("/tmp/unrelated.bin", recorded, actual, serialized)
    except ValueError:
        pass
    else:
        raise AssertionError("outside-root path must be rejected")
    print("portable path mapping: PASS")


if __name__ == "__main__":
    main()
