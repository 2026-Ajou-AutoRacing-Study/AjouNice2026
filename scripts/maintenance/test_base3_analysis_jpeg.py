"""Disposable CPU fixtures for the explicitly authorized Base3 JPEG migration."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

import base3_analysis_jpeg as migration


class MigrationTests(unittest.TestCase):
    def fixture(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        base=Path(temporary.name);root=base/'raw';root.mkdir()
        for i in range(1,5):
            folder=root/'run'/('CAMERA_'+str(i));folder.mkdir(parents=True)
            Image.fromarray(np.full((12,18,3),[80,120,160],dtype=np.uint8)).save(folder/'frame_f00_Intensity.png')
            Image.fromarray(np.full((12,18),i,dtype=np.uint8)).save(folder/'frame_f00_Semantic.png')
        lidar=root/'run/LIDAR_5';lidar.mkdir()
        (lidar/'frame_f00_Intensity.bin').write_bytes(b'preserve native lidar')
        (lidar/'frame_f00_Instance_8Points.txt').write_bytes(b'preserve original corners')
        (root/'metadata.json').write_text('{"original":true}\n')
        return root,base/'analysis'

    def test_read_only_plan(self):
        root,out=self.fixture()
        with patch.object(migration,'ROOT',root):
            result=migration.run(root,out)
        self.assertEqual(result['rgb_images'],4)
        self.assertFalse(out.exists())
        self.assertEqual(len(list(root.rglob('*_Intensity.png'))),4)
        self.assertEqual(len(list(root.rglob('*.jpg'))),0)

    def test_only_rgb_removed_after_verified_complete_index(self):
        root,out=self.fixture();native={str(p.relative_to(root)):p.read_bytes()
            for p in root.rglob('*') if p.is_file() and not migration.rgb_source(p)}
        with patch.object(migration,'ROOT',root):
            result=migration.run(root,out,execute=True,delete_png=True,workers=2)
        self.assertTrue(result['passed']);self.assertEqual(result['deleted_png'],4)
        self.assertEqual(result['complete_four_camera_lidar_frames'],1)
        self.assertEqual(len(list(root.rglob('*.jpg'))),4)
        self.assertEqual(len(list(root.rglob('*_Semantic.png'))),4)
        self.assertEqual(native,{str(p.relative_to(root)):p.read_bytes()
            for p in root.rglob('*') if p.is_file() and p.suffix!='.jpg'})
        frames=[json.loads(line) for line in (out/'frames.jsonl').read_text().splitlines()]
        self.assertTrue(frames[0]['lidar'].endswith('_Intensity.bin'))
        self.assertEqual(len(frames[0]['images']),4)

    def test_encoding_failure_never_deletes_png(self):
        root,out=self.fixture()
        with patch.object(migration,'ROOT',root),patch.object(migration.cv2,'imencode',return_value=(False,None)):
            with self.assertRaisesRegex(ValueError,'encoding failed'):
                migration.run(root,out,execute=True,delete_png=True)
        self.assertEqual(len(list(root.rglob('*_Intensity.png'))),4)

    def test_preserved_payload_verification_failure_keeps_png(self):
        root,out=self.fixture()
        with patch.object(migration,'ROOT',root),patch.object(migration,'verify_retained',side_effect=ValueError('native changed')):
            with self.assertRaisesRegex(ValueError,'native changed'):
                migration.run(root,out,execute=True,delete_png=True)
        self.assertEqual(len(list(root.rglob('*_Intensity.png'))),4)

    def test_current_release_target_is_never_deleted(self):
        root,out=self.fixture();release=root.parent/'release.json'
        source=next(root.rglob('*_Intensity.png'))
        release.write_text(json.dumps({'input_hashes':{str(source):'pinned'}}))
        with patch.object(migration,'ROOT',root),self.assertRaisesRegex(ValueError,'qualified release'):
            migration.run(root,out,execute=True,delete_png=True,releases=[release])
        self.assertFalse(out.exists());self.assertTrue(source.exists())

    def test_foreign_scope_and_existing_output_are_rejected(self):
        root,out=self.fixture()
        with self.assertRaisesRegex(ValueError,'approved Base3'):
            migration.run(root,out,execute=True,delete_png=True)
        out.mkdir()
        with patch.object(migration,'ROOT',root),self.assertRaisesRegex(ValueError,'Fresh external'):
            migration.run(root,out,execute=True,delete_png=True)

    def test_nonmatching_existing_jpeg_is_preserved(self):
        root,out=self.fixture();source=next(root.rglob('*_Intensity.png'))
        jpeg=source.with_suffix('.jpg');jpeg.write_bytes(b'foreign existing JPEG')
        with patch.object(migration,'ROOT',root),self.assertRaisesRegex(ValueError,'never overwrite'):
            migration.run(root,out,execute=True,delete_png=True)
        self.assertEqual(jpeg.read_bytes(),b'foreign existing JPEG');self.assertTrue(source.exists())

    def test_symlink_source_is_rejected(self):
        root,out=self.fixture();source=next(root.rglob('*_Intensity.png'))
        source.rename(source.with_suffix('.original'));source.symlink_to(source.with_suffix('.original'))
        with patch.object(migration,'ROOT',root),self.assertRaisesRegex(ValueError,'symlink'):
            migration.run(root,out,execute=True,delete_png=True)


if __name__=='__main__':unittest.main()
