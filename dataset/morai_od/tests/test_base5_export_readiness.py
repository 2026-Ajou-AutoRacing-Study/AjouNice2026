import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_export_readiness import annotation_export_missing,AnnotationExportUnavailable


def raw():
    return [Path(f'CAMERA_{i}/timestamp_Intensity.png') for i in range(1,5)] + [Path('LIDAR_5/timestamp_Intensity.bin')]


def test_stable_raw_only_is_configuration_stop_not_scene_quality():
    missing, observed = annotation_export_missing(raw(),6.,2.)
    assert missing
    assert observed['native_annotation_files'] == 0
    assert issubclass(AnnotationExportUnavailable,RuntimeError)


def test_normal_native_export_does_not_add_a_wait():
    missing,_=annotation_export_missing(raw()+[Path('LIDAR_5/timestamp_Instance.txt')],6.,2.)
    assert not missing


def test_in_progress_or_incomplete_raw_not_called_disabled_export():
    assert not annotation_export_missing(raw(),5.99,8.)[0]
    assert not annotation_export_missing(raw(),20.,1.99)[0]
    assert not annotation_export_missing(raw()[:-1],20.,8.)[0]
    assert not annotation_export_missing(raw()[1:],20.,8.)[0]


def test_partial_native_export_keeps_existing_completeness_timeout():
    assert not annotation_export_missing(raw()+[Path('CAMERA_1/timestamp_Instance.png')],20.,8.)[0]
