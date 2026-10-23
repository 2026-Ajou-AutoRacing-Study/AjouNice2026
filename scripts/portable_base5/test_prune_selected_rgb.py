"""CPU fixtures only; no live capture, external data deletion or networking."""
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import pickle
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

SPEC = importlib.util.spec_from_file_location('prune_selected_rgb',
    Path(__file__).with_name('prune_selected_rgb.py'))
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)


class PruneTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw_allow = self.root / 'raw'
        self.raw = self.raw_allow / 'capture'
        self.png, self.cache, self.jpeg = [self.root / n for n in ('png', 'cache', 'jpeg')]
        for root in (self.raw, self.png, self.cache, self.jpeg):
            root.mkdir(parents=True)
        self.scenario = self.put(self.raw / 'scenario.json', dict(scene='chosen'))
        self.selected = self.root / 'selected.jsonl'
        row = dict(id='chosen', source_cell_id='slot', status='accepted_candidate',
                   capture_root=str(self.raw), scenario_sha256=p.digest(self.scenario))
        self.selected.write_text(json.dumps(row) + '\n')
        self.pm = dict(scenes=1, profiles=['512x1056'], files={},
                       source_ledger_sha256=p.digest(self.selected),
                       provenance=dict(chosen=dict(original_capture_root=str(self.raw))))
        self.info = dict(metainfo=dict(classes=('vehicle', 'pedestrian', 'obstacle', 'animal'),
                                      box_geometry_policy='upright'), data_list=[])
        record = dict(token='chosen', scene_group_id='group', images={},
                      lidar_points=dict(lidar_path='data/chosen/LIDAR_5/0_Intensity.bin'),
                      instances=[dict(bbox_label_3d=3, bbox_3d=[1, 2, 3, 4, 5, 6, 0],
                                      asset='animal', component_id='a')])
        self.links = []
        for i in range(1, 5):
            relative = 'data/chosen/CAMERA_%s/0_Intensity.png' % i
            source = self.raw / ('CAMERA_%s/0_Intensity.png' % i)
            source.parent.mkdir()
            Image.new('RGB', (8, 6), (i * 30, 20, 100)).save(source)
            self.transfer_png(source, relative)
            record['images']['front' + str(i)] = dict(img_path=relative, width=8, height=6,
                                                    cam2img=[[1, 0], [0, 1]])
            self.links.append(source)
            self.put(self.raw / ('CAMERA_%s/0_Instance.png' % i), b'UNTOUCHED NATIVE MASK')
            self.transfer_png(self.raw / ('CAMERA_%s/0_Instance.png' % i),
                              'data/chosen/CAMERA_%s/0_Instance.png' % i)
        for source, relative in [(self.scenario, 'data/chosen/scenario.json'),
            (self.put(self.raw / 'LIDAR_5/0_Intensity.bin', b'POINTS'),
             'data/chosen/LIDAR_5/0_Intensity.bin'),
            (self.put(self.raw / 'LIDAR_5/0_Annotation.json', dict(native='GT')),
             'data/chosen/LIDAR_5/0_Annotation.json'),
            (self.selected, 'provenance/original_selected_ledger.jsonl')]:
            self.transfer_png(source, relative)
        self.info['data_list'] = [record]
        for split in ('train', 'val', 'test'):
            obj = deepcopy(self.info)
            if split != 'train':
                obj['data_list'] = []
            path = self.put(self.png / ('infos/512x1056/morai_base5_infos_%s.pkl' % split),
                            pickle.dumps(obj))
            self.pm['files'][str(path.relative_to(self.png))] = self.meta(path)
        self.put(self.png / 'manifest.json', self.pm)
        self.cm = dict(complete=True, quality=90, native_resolution=True,
                       source_color_order='rgb', image_count=4,
                       source_manifest_sha256=p.digest(self.png / 'manifest.json'), files={})
        for image in record['images'].values():
            relative = image['img_path']
            name = p.hashlib.sha256(relative.encode()).hexdigest() + '.jpg'
            target = self.cache / 'images' / name
            target.parent.mkdir(exist_ok=True)
            Image.open(self.png / relative).save(target, quality=90)
            self.cm['files'][relative] = dict(path='images/' + name,
                **self.meta(target), source_bytes=(self.png / relative).stat().st_size,
                source_sha256=p.digest(self.png / relative), height=6, width=8)
        self.put(self.cache / 'manifest.json', self.cm)
        self.jm = deepcopy(self.pm)
        self.jm.update(image_policy=dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
                       training_label_policy='animal_to_pedestrian_v1')
        self.jm['files'] = {}
        for name in self.pm['files']:
            if name in self.cm['files'] or name.startswith('infos/'):
                continue
            target = self.jpeg / name
            target.parent.mkdir(parents=True, exist_ok=True)
            os.link(self.png / name, target)
            self.jm['files'][name] = self.meta(target)
        for image in self.cm['files'].values():
            target = self.jpeg / 'jpeg_rgb' / Path(image['path']).name
            target.parent.mkdir(exist_ok=True)
            os.link(self.cache / image['path'], target)
            self.jm['files'][str(target.relative_to(self.jpeg))] = self.meta(target)
        for split in ('train', 'val', 'test'):
            info = deepcopy(self.info)
            info['metainfo']['classes'] = ('vehicle', 'pedestrian', 'obstacle')
            info['data_list'] = [p.merge_record(self.info['data_list'][0])] if split == 'train' else []
            for rec in info['data_list']:
                for camera in rec['images'].values():
                    camera['img_path'] = 'jpeg_rgb/' + Path(self.cm['files'][camera['img_path']]['path']).name
            target = self.put(self.jpeg / ('infos/512x1056/morai_base5_infos_%s.pkl' % split),
                              pickle.dumps(info))
            self.jm['files'][str(target.relative_to(self.jpeg))] = self.meta(target)
        self.put(self.jpeg / 'manifest.json', self.jm)
        self.rejected = self.put(self.raw_allow / 'rejected/CAMERA_1/0_Intensity.png', b'FAILED RAW')

    def put(self, path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data if isinstance(data, bytes) else (json.dumps(data) + '\n').encode())
        return path

    def meta(self, path):
        return dict(bytes=path.stat().st_size, sha256=p.digest(path))

    def transfer_png(self, source, relative):
        target = self.png / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        os.link(source, target)
        self.pm['files'][relative] = self.meta(target)

    def inspect(self, **kwargs):
        return p.inspect(self.selected, self.png, self.cache, self.jpeg, [self.raw_allow],
                         expected_scenes=1, **kwargs)

    def plan(self, **kwargs):
        plan = self.inspect(**kwargs)
        path = self.root / 'approved_plan.json'
        p.write_new(path, plan)
        return path, p.digest(path)

    def test_dry_run_keeps_files_and_records_four_bindings(self):
        result = self.inspect()
        self.assertEqual(len(result['candidates']), 4)
        self.assertEqual(len(result['scenes']['chosen']['cameras']), 4)
        self.assertTrue(all(path.exists() for path in self.links))
        self.assertFalse((self.root / 'approved_plan.json').exists())

    def test_removes_exact_selected_rgb_and_hardlinks_only(self):
        path, sha = self.plan(delete_png_package_rgb=True)
        result = p.delete(path, sha, self.root / 'receipt', ack_lossy=True)
        self.assertEqual(result['removed_files'], 8)
        self.assertTrue(result['complete'])
        self.assertFalse(any(path.exists() for path in self.links))
        self.assertTrue(self.rejected.exists())
        self.assertTrue((self.raw / 'CAMERA_1/0_Instance.png').exists())
        self.assertTrue((self.raw / 'LIDAR_5/0_Annotation.json').exists())
        self.assertTrue((self.jpeg / 'manifest.json').exists())
        self.assertEqual(json.loads((self.root / 'receipt/COMPLETE.json').read_text())['removed_files'], 8)

    def test_acknowledgement_required(self):
        path, sha = self.plan()
        with self.assertRaisesRegex(ValueError, 'acknowledgement'):
            p.delete(path, sha, self.root / 'receipt')
        self.assertTrue(all(path.exists() for path in self.links))

    def test_plan_sha_mismatch_refuses_all_deletes(self):
        path, _ = self.plan()
        with self.assertRaisesRegex(ValueError, 'plan SHA'):
            p.delete(path, '0' * 64, self.root / 'receipt', True)

    def test_changed_jpeg_after_plan_refuses_all_deletes(self):
        path, sha = self.plan()
        jpeg = next((self.jpeg / 'jpeg_rgb').iterdir())
        jpeg.write_bytes(b'BROKEN JPEG')
        with self.assertRaisesRegex(ValueError, 'manifest'):
            p.delete(path, sha, self.root / 'receipt', True)
        self.assertTrue(all(path.exists() for path in self.links))

    def test_symlinked_native_png_refused(self):
        original = self.links[0]
        original.unlink()
        original.symlink_to(self.png / 'data/chosen/CAMERA_1/0_Intensity.png')
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            self.inspect()

    def test_raw_scope_cannot_select_rejected_or_other_data(self):
        with self.assertRaisesRegex(ValueError, 'allowlist'):
            p.inspect(self.selected, self.png, self.cache, self.jpeg,
                      [self.raw_allow / 'rejected'], expected_scenes=1)

    def test_corrupt_jpeg_with_updated_manifest_still_needs_full_decode(self):
        target = next((self.jpeg / 'jpeg_rgb').iterdir())
        relative = next(k for k, value in self.cm['files'].items()
                        if Path(value['path']).name == target.name)
        target.write_bytes(b'NOT JPEG')
        self.cm['files'][relative].update(self.meta(target))
        self.jm['files'][str(target.relative_to(self.jpeg))] = self.meta(target)
        self.put(self.cache / 'manifest.json', self.cm)
        self.put(self.jpeg / 'manifest.json', self.jm)
        with self.assertRaises(Exception):
            self.inspect()
        self.assertTrue(all(path.exists() for path in self.links))

    def test_gt_box_change_is_not_permitted(self):
        target = self.jpeg / 'infos/512x1056/morai_base5_infos_train.pkl'
        info = pickle.loads(target.read_bytes())
        info['data_list'][0]['instances'][0]['bbox_3d'][0] = 999
        target.write_bytes(pickle.dumps(info))
        self.jm['files'][str(target.relative_to(self.jpeg))] = self.meta(target)
        self.put(self.jpeg / 'manifest.json', self.jm)
        with self.assertRaisesRegex(ValueError, 'GT boxes'):
            self.inspect()

    def test_changed_annotation_is_never_deleted(self):
        (self.raw / 'LIDAR_5/0_Annotation.json').write_bytes(b'CHANGED GT')
        with self.assertRaisesRegex(ValueError, 'manifest'):
            self.inspect()
        self.assertTrue(all(path.exists() for path in self.links))

    def test_edited_plan_cannot_add_unselected_path(self):
        path, _ = self.plan()
        plan = json.loads(path.read_text())
        plan['candidates'][str(self.rejected)] = p.snapshot(self.rejected)
        path.write_text(json.dumps(plan))
        with self.assertRaisesRegex(ValueError, 'approved dry-run'):
            p.delete(path, p.digest(path), self.root / 'receipt', True)
        self.assertTrue(self.rejected.exists())

    def test_duplicate_quota_is_rejected(self):
        row = json.loads(self.selected.read_text())
        self.selected.write_text(json.dumps(row) + '\n' + json.dumps(row) + '\n')
        with self.assertRaisesRegex(ValueError, 'count'):
            self.inspect()

    def test_hardlinked_images_decode_once_per_unique_inode(self):
        with patch.object(p, 'image_shape', wraps=p.image_shape) as decode:
            self.inspect()
        # Four native PNG inodes plus four JPEG inodes. The source/export and
        # cache/final aliases must not double the number of full decodes.
        self.assertEqual(decode.call_count, 8)

    def test_hardlinked_payload_hashes_once(self):
        reads = p.ReadCache()
        native = self.links[0]
        portable = self.png / 'data/chosen/CAMERA_1/0_Intensity.png'
        with patch.object(p, 'digest', wraps=p.digest) as sha:
            self.assertEqual(reads.snapshot(native), reads.snapshot(portable))
        self.assertEqual(sha.call_count, 1)

    def test_cache_is_not_shared_across_inspection_calls(self):
        with patch.object(p, 'image_shape', wraps=p.image_shape) as decode:
            self.inspect()
            self.inspect()
        self.assertEqual(decode.call_count, 16)

    def test_same_inode_changed_mtime_forces_hash_and_rejects_corruption(self):
        reads = p.ReadCache()
        native = self.links[0]
        expected = self.pm['files']['data/chosen/CAMERA_1/0_Intensity.png']
        original = reads.snapshot(native)
        data = native.read_bytes()
        native.write_bytes(bytes([data[0] ^ 1]) + data[1:])  # Same inode and size.
        os.utime(native, ns=(native.stat().st_atime_ns, original['mtime_ns'] + 1_000_000))
        self.assertEqual(native.stat().st_ino, original['inode'])
        with patch.object(p, 'digest', wraps=p.digest) as sha:
            with self.assertRaisesRegex(ValueError, 'immutable manifest'):
                p.checked_payload(self.png, 'data/chosen/CAMERA_1/0_Intensity.png', expected, reads)
        self.assertEqual(sha.call_count, 1)

    def test_same_inode_changed_mtime_forces_full_decode(self):
        reads = p.ReadCache()
        native = self.links[0]
        reads.image_shape(native, 'PNG', 8, 6)
        original = native.stat()
        native.write_bytes(b'X' * original.st_size)
        os.utime(native, ns=(native.stat().st_atime_ns, original.st_mtime_ns + 1_000_000))
        with self.assertRaises(Exception):
            reads.image_shape(native, 'PNG', 8, 6)

    def test_cached_path_does_not_allow_replacement_symlink(self):
        reads = p.ReadCache()
        native = self.links[0]
        alias = self.png / 'data/chosen/CAMERA_1/0_Intensity.png'
        reads.snapshot(native)
        native.unlink()
        native.symlink_to(alias)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            reads.snapshot(native)

    def update_png_binding(self, native, relative):
        """Keep this tiny fixture's immutable inventories bound to test pixels."""
        self.pm['files'][relative] = self.meta(native)
        self.put(self.png / 'manifest.json', self.pm)
        self.cm['source_manifest_sha256'] = p.digest(self.png / 'manifest.json')
        self.cm['files'][relative].update(source_bytes=native.stat().st_size,
                                          source_sha256=p.digest(native))
        self.put(self.cache / 'manifest.json', self.cm)

    def test_rgba_intensity_with_nonopaque_alpha_is_accepted(self):
        native = self.links[0]
        with Image.open(native) as source:
            rgba = source.convert('RGBA')
        alpha = Image.new('L', rgba.size)
        alpha.putdata([246 + index % 10 for index in range(rgba.width * rgba.height)])
        rgba.putalpha(alpha)
        rgba.save(native)
        self.update_png_binding(native, 'data/chosen/CAMERA_1/0_Intensity.png')
        result = self.inspect()
        camera = result['scenes']['chosen']['cameras'][0]
        self.assertEqual(camera['png_mode'], 'RGBA')
        self.assertTrue(camera['png_alpha_discarded'])
        self.assertIn('without compositing', camera['png_color_policy'])
        self.assertTrue(native.exists())

    def test_grayscale_intensity_png_is_rejected(self):
        native = self.links[0]
        Image.new('L', (8, 6), 30).save(native)
        self.update_png_binding(native, 'data/chosen/CAMERA_1/0_Intensity.png')
        with self.assertRaisesRegex(ValueError, 'RGB/RGBA PNG or RGB JPEG'):
            self.inspect()
        self.assertTrue(native.exists())

    def test_grayscale_jpeg_is_rejected(self):
        relative = 'data/chosen/CAMERA_1/0_Intensity.png'
        cache = self.cache / self.cm['files'][relative]['path']
        Image.new('L', (8, 6), 30).save(cache, quality=90)
        self.cm['files'][relative].update(self.meta(cache))
        target = self.jpeg / 'jpeg_rgb' / cache.name
        self.jm['files'][str(target.relative_to(self.jpeg))] = self.meta(target)
        self.put(self.cache / 'manifest.json', self.cm)
        self.put(self.jpeg / 'manifest.json', self.jm)
        with self.assertRaisesRegex(ValueError, 'RGB/RGBA PNG or RGB JPEG'):
            self.inspect()
        self.assertTrue(all(path.exists() for path in self.links))


if __name__ == '__main__':
    unittest.main()
