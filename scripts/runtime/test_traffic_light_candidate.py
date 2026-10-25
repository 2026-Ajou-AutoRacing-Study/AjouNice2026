"""Pure pinned-asset/installer contract tests. No network/GPU/ROS required."""
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import traffic_light_candidate as assets


class AssetsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.archive = self.root / 'bundle.tar.gz'
        self.doc = assets.load_release()
        self.payload = {'yolo_final/selected.pt': b'fake yolo for pure test',
                        'cnn_v5/best.pt': b'fake cnn for pure test'}
        self.pack()

    def pack(self, extras=(), missing=None):
        if self.archive.exists():self.archive.unlink()
        with tarfile.open(self.archive, 'w:gz') as tar:
            for name, value in self.payload.items():
                if name == missing:continue
                member = tarfile.TarInfo(name); member.size = len(value)
                tar.addfile(member, io.BytesIO(value))
            for member, value in extras:
                tar.addfile(member, None if value is None else io.BytesIO(value))
        self.doc['archive_bytes'] = self.archive.stat().st_size
        self.doc['archive_sha256'] = assets.digest(self.archive)
        for item in self.doc['models'].values():
            value = self.payload[item['member']]
            item.update(bytes=len(value), sha256=hashlib.sha256(value).hexdigest())

    def test_real_release_and_default_path_are_explicit_candidate(self):
        doc = assets.load_release()
        self.assertEqual(doc['version'], 'v5_3d29ee7f')
        self.assertIn('.local/share/ajounice/traffic_light_models/v5_', str(assets.default_models_dir(doc)))

    def test_install_is_exact_idempotent_and_does_not_start_runtime(self):
        destination = self.root / 'models'
        with patch.object(assets.subprocess, 'run') as network:
            first = assets.install(self.archive, destination, release=self.doc)
            second = assets.install(None, destination, release=self.doc)
        network.assert_not_called()
        self.assertFalse(first['runtime_qualified']); self.assertFalse(second['production_changed'])
        self.assertEqual((destination/'cnn_v5/best.pt').read_bytes(), self.payload['cnn_v5/best.pt'])
        self.assertEqual(assets.validate_install(destination, self.doc)['yolo'], str(destination/'yolo_final/selected.pt'))

    def test_existing_unmanaged_or_corrupt_models_are_not_replaced(self):
        destination = self.root / 'models'; destination.mkdir()
        own = destination/'keep.txt';own.write_text('user file')
        with self.assertRaises(ValueError):assets.install(self.archive, destination, release=self.doc)
        self.assertEqual(own.read_text(), 'user file')
        destination2 = self.root/'installed';assets.install(self.archive, destination2, release=self.doc)
        model=destination2/'cnn_v5/best.pt';model.write_bytes(b'user replacement')
        with self.assertRaises(ValueError):assets.install(self.archive, destination2, release=self.doc)
        self.assertEqual(model.read_bytes(), b'user replacement')

    def test_corrupt_archive_publishes_no_models(self):
        self.archive.write_bytes(b'bad transfer')
        with self.assertRaises(ValueError):assets.install(self.archive, self.root/'models', release=self.doc)
        self.assertFalse((self.root/'models').exists())

    def test_checksum_alone_does_not_authorize_changed_model(self):
        self.payload['cnn_v5/best.pt'] = b'wrong bytes';self.pack()
        self.doc['models']['cnn']['sha256'] = '0'*64
        with self.assertRaises(ValueError):assets.install(self.archive, self.root/'models', release=self.doc)
        self.assertFalse((self.root/'models').exists())

    def test_unsafe_duplicate_and_symlink_tar_members_are_rejected(self):
        for name, kind in (('../escape', tarfile.REGTYPE), ('/absolute', tarfile.REGTYPE),
                           ('symlink', tarfile.SYMTYPE), ('hardlink', tarfile.LNKTYPE),
                           ('yolo_final/selected.pt', tarfile.REGTYPE)):
            member=tarfile.TarInfo(name);member.type=kind;member.linkname='../outside'
            self.pack([(member, b'' if kind==tarfile.REGTYPE else None)])
            with self.subTest(name=name), self.assertRaises(ValueError):assets.validate_archive(self.archive,self.doc)

    def test_missing_model_and_changed_receipt_are_rejected(self):
        self.pack(missing='cnn_v5/best.pt')
        with self.assertRaises(ValueError):assets.validate_archive(self.archive,self.doc)
        self.pack();destination=self.root/'models';assets.install(self.archive,destination,release=self.doc)
        (destination/'models_ready.json').write_text('{}')
        with self.assertRaises(ValueError):assets.validate_install(destination,self.doc)

    def test_atomic_publish_never_replaces_existing_empty_directory(self):
        source=self.root/'staged';source.mkdir();(source/'new').write_text('model')
        destination=self.root/'existing';destination.mkdir()
        with self.assertRaises(OSError):assets.publish_no_replace(source,destination)
        self.assertTrue(source.exists());self.assertEqual(list(destination.iterdir()),[])

    def test_release_rejects_unsafe_model_paths_and_sizes(self):
        for key,value in (('member','../escape'),('bytes',True),('sha256','invalid')):
            doc=copy.deepcopy(self.doc);doc['models']['cnn'][key]=value
            path=self.root/'descriptor.json';path.write_text(json.dumps(doc))
            with self.assertRaises(ValueError):assets.load_release(path)

    def test_legacy_defaults_and_candidate_launch_match_release(self):
        package=assets.ROOT/'base_ws/src/perception/traffic_light_detection/launch'
        legacy={n.attrib['name']:n.attrib.get('default') for n in ET.parse(package/'traffic_light_node.launch').getroot().findall('arg')}
        self.assertEqual((legacy['selection_mode'],legacy['classifier_mode'],legacy['detector_imgsz']),('legacy_center','hsv','640'))
        candidate=ET.parse(package/'traffic_light_v5_candidate.launch').getroot()
        args={n.attrib['name']:n.attrib['value'] for n in candidate.find('include').findall('arg')}
        for name,value in assets.load_release()['runtime'].items():
            if name=='max_rate_hz':continue
            self.assertEqual(args[name],str(value))

    def test_shell_usage_does_not_install_or_download(self):
        for name in ('install_traffic_light_candidate.sh','setup_traffic_light_env.sh'):
            result=subprocess.run(['bash',str(assets.ROOT/'scripts/runtime'/name),'--help'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)


if __name__=='__main__':unittest.main()
