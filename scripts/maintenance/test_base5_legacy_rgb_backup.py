import json
from pathlib import Path

from PIL import Image
import pytest

import base5_legacy_rgb_backup as backup


def test_remote_mismatch_cannot_be_accepted():
    backup.require_remote_md5('a'*32+'  rgb_q90.tar.zst\n','a'*32)
    for value in ('', 'b'*32+'  rgb_q90.tar.zst', 'a'*32+'  one\n'+'a'*32+'  two'):
        with pytest.raises(ValueError):backup.require_remote_md5(value,'a'*32)


def test_prepare_preserves_native_gt_and_png_until_remote_verification(tmp_path,monkeypatch):
    root=tmp_path/'source'; camera=root/'capture/CAMERA_1'; camera.mkdir(parents=True)
    png=camera/'frame_Intensity.png'; Image.new('RGB',(32,24),(90,150,210)).save(png)
    mask=camera/'frame_Instance.png'; Image.new('L',(32,24),42).save(mask)
    gt=root/'native.txt';gt.write_text('untouched GT')
    originals={p:p.read_bytes() for p in (png,mask,gt)}
    monkeypatch.setattr(backup,'SOURCE',root)
    monkeypatch.setattr(backup,'OUTPUT_BASE',tmp_path/'artifacts')
    output=tmp_path/'artifacts/backup'
    backup.prepare(output,1)
    for p,data in originals.items():assert p.read_bytes()==data
    assert png.with_suffix('.jpg').is_file()
    assert not mask.with_suffix('.jpg').exists()
    assert (output/'pack.json').is_file()
    # Simulate failed remote integrity after upload, before any PNG removal.
    monkeypatch.setattr(backup,'rclone',lambda *args: 'b'*32+'  rgb_q90.tar.zst' if args[0]=='md5sum' else '')
    with pytest.raises(ValueError,match='MD5 mismatch'):backup.upload_and_prune(output)
    for p,data in originals.items():assert p.read_bytes()==data
    row=json.loads((output/'metadata/rgb_mapping.jsonl').read_text())
    backup.verify_predecoded_jpeg(root,row)
    png.with_suffix('.jpg').write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='JPEG bytes changed'):
        backup.verify_predecoded_jpeg(root,row)
    assert png.read_bytes()==originals[png]


def test_output_cannot_be_inside_capture_or_reused(tmp_path,monkeypatch):
    monkeypatch.setattr(backup,'OUTPUT_BASE',tmp_path/'artifacts')
    with pytest.raises(ValueError):backup.checked_output(tmp_path/'capture')
    existing=tmp_path/'artifacts/existing';existing.mkdir(parents=True)
    with pytest.raises(ValueError):backup.checked_output(existing,fresh=True)
