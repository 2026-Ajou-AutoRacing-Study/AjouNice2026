"""CPU-only regression checks for the sampled native JPEG cache benchmark."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from PIL import Image

from benchmark_base5_jpeg_cache import build_cache, choose_indices
from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import (
    cache_train_pipeline, MoraiUseNativeJPEGCache, validate_jpeg_cache)


class FakeDataset:
    def __init__(self, records):
        self.records = records

    def __len__(self):
        return len(self.records)

    def get_data_info(self, index):
        return self.records[index]


class CacheTest(unittest.TestCase):
    def fixture(self, root):
        rgb = np.random.RandomState(1).randint(0, 256, (32, 64, 3), np.uint8)
        source = root / 'native.png'
        Image.fromarray(rgb).save(source)
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        (root / 'manifest.json').write_text(json.dumps(dict(files={
            'native.png': dict(sha256=digest)})))
        records = [dict(images={'front': dict(img_path=str(source), height=32, width=64)})]
        return rgb, source, records

    def test_cache_is_native_single_encode_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            rgb, source, records = self.fixture(root)
            before = source.read_bytes()
            cache = root / 'cache'
            manifest = build_cache(root, records + deepcopy(records), cache)
            self.assertEqual(manifest['image_count'], 1)
            self.assertEqual(source.read_bytes(), before)
            cached = cache / manifest['files']['native.png']['path']
            ok, blob = cv2.imencode('.jpg', rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.assertTrue(ok)
            self.assertEqual(cached.read_bytes(), blob.tobytes())
            with Image.open(cached) as image:
                self.assertEqual(image.size, (64, 32))
            with self.assertRaises(FileExistsError):
                build_cache(root, records, cache)

    def test_changed_source_is_rejected_before_completed_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, source, records = self.fixture(root)
            source.write_bytes(b'changed')
            cache = root / 'cache'
            with self.assertRaisesRegex(ValueError, 'hash differs'):
                build_cache(root, records, cache)
            self.assertFalse((cache / 'manifest.json').exists())

    def test_native_dimensions_must_match(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, _, records = self.fixture(root)
            records[0]['images']['front']['height'] = 20
            with self.assertRaisesRegex(ValueError, 'dimensions changed'):
                build_cache(root, records, root / 'cache')

    def test_families_are_balanced_and_no_duplicate_indices(self):
        rows = [dict(token=str(i)) for i in range(60)]
        ledger = {str(i): dict(family=str(i % 3)) for i in range(60)}
        dataset = FakeDataset(rows)
        result = choose_indices(dataset, ledger, 32)
        self.assertEqual(len(set(result)), 32)
        self.assertEqual(result, choose_indices(dataset, ledger, 32))
        counts = [sum(i % 3 == family for i in result) for family in range(3)]
        self.assertLessEqual(max(counts) - min(counts), 1)

    def test_cache_missing_source_and_corruption_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, source, records = self.fixture(root)
            cache = root / 'cache'
            manifest = build_cache(root, records, cache)
            extra = root / 'extra.png'
            extra.write_bytes(b'placeholder')
            with self.assertRaisesRegex(ValueError, 'missing required'):
                validate_jpeg_cache(root, cache, [extra])
            target = cache / manifest['files']['native.png']['path']
            value = target.read_bytes()
            target.write_bytes(b'X' + value[1:])
            with self.assertRaisesRegex(ValueError, 'checksum differs'):
                validate_jpeg_cache(root, cache, [source], verify_hashes=True)

    def test_redirect_changes_only_path_and_rejects_wrong_dimensions(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, _, records = self.fixture(root)
            build_cache(root, records, root / 'cache')
            transform = MoraiUseNativeJPEGCache(root, root / 'cache')
            data = deepcopy(records[0])
            data['gt_bboxes_3d'] = 'sentinel'
            result = transform.transform(data)
            self.assertEqual(result['gt_bboxes_3d'], 'sentinel')
            self.assertEqual(result['images']['front']['height'], 32)
            self.assertTrue(result['images']['front']['img_path'].endswith('.jpg'))
            wrong = deepcopy(records[0])
            wrong['images']['front']['height'] = 22
            with self.assertRaisesRegex(ValueError, 'dimensions differ'):
                transform.transform(wrong)

    def test_pipeline_never_double_encodes_and_rejects_other_aug(self):
        domain = dict(type='MoraiOnlineImageDomainAug3D', jpeg_prob=1., jpeg_quality=[90,90],
                      native_resolution=True, color_order='rgb', blur_prob=0.,
                      exposure_prob=0., noise_prob=0.)
        pipeline = [dict(type='LoadPointsFromFile'), dict(type='BEVLoadMultiViewImageFromFiles'),
                    domain, dict(type='MoraiImageAug3D')]
        converted = cache_train_pipeline(pipeline, '/tmp/source', '/tmp/cache')
        self.assertEqual(converted[1]['type'], 'MoraiUseNativeJPEGCache')
        self.assertFalse(any(t['type']=='MoraiOnlineImageDomainAug3D' for t in converted))
        self.assertEqual(pipeline[2], domain)
        for key, value in (('jpeg_prob', .8), ('jpeg_quality', [80,95]), ('noise_prob', .1)):
            changed = deepcopy(pipeline)
            changed[2][key] = value
            with self.assertRaisesRegex(ValueError, 'requires native Q90'):
                cache_train_pipeline(changed, '/tmp/source', '/tmp/cache')


if __name__ == '__main__':
    unittest.main()
