"""Detect missing native annotation export without retrying other scenes.

This is a configuration/transport stop, never an ordinary scene deficit.
Normal export incurs no extra wait. An incomplete set is preserved by caller.
"""
from pathlib import Path


class AnnotationExportUnavailable(RuntimeError):
    """Raw sensor output exists, but native annotation export is unavailable."""


def inventory(paths):
    paths = list(map(Path, paths))
    rgb = {p.parent.name for p in paths if p.name.endswith('_Intensity.png')}
    lidar = [p for p in paths if p.parent.name.startswith('LIDAR') and p.name.endswith('_Intensity.bin')]
    native = [p for p in paths if p.name.endswith(('_Instance.txt', '_Instance_8Points.txt',
        '_Instance.png', '_BBox2D.txt', '_BBox3D.txt', '_Instance.bin'))]
    return dict(camera_rgb=sorted(rgb), lidar_intensity_files=len(lidar),
                native_annotation_files=len(native), files=sorted(str(p) for p in paths))


def annotation_export_missing(paths, elapsed_sec, stable_sec):
    observed = inventory(paths)
    raw_ready = set(observed['camera_rgb']) == {'CAMERA_1','CAMERA_2','CAMERA_3','CAMERA_4'} and observed['lidar_intensity_files'] == 1
    return bool(raw_ready and not observed['native_annotation_files']
                and elapsed_sec >= 6. and stable_sec >= 2.), observed
