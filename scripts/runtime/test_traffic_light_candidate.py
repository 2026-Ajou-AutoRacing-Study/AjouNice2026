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


class PublicResponse(io.BytesIO):
    def __init__(self, payload, headers=None, url=None, status=200):
        super().__init__(payload)
        self.headers = {'Content-Type': 'application/octet-stream'} if headers is None else headers
        self.url = url or 'https://drive.usercontent.google.com/download?id=testPublicFile1234'
        self.status = status

    def geturl(self):
        return self.url


class AssetsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.archive = self.root / 'bundle.tar.gz'
        self.doc = assets.load_release()
        # Legacy/offline fixtures remain meaningful after an approved release
        # opts into public delivery; tests never depend on live sharing state.
        self.doc.pop('public_file_id', None)
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

    def test_public_file_id_is_opt_in_and_never_a_url_or_token(self):
        self.assertNotIn('public_file_id', self.doc)
        for value in (None, True, 123, '', 'short', 'a' * 201,
                      'https://drive.google.com/file/d/1234567890/view',
                      '1234567890?access_token=secret', '../1234567890', '1234567890\n'):
            doc=copy.deepcopy(self.doc);doc['public_file_id']=value
            path=self.root/'descriptor.json';path.write_text(json.dumps(doc))
            with self.subTest(value=value),self.assertRaises(ValueError):assets.load_release(path)
        doc['public_file_id']='testPublicFile1234567890'
        path.write_text(json.dumps(doc))
        self.assertEqual(assets.load_release(path)['public_file_id'],doc['public_file_id'])

    def test_public_success_uses_no_rclone_credentials_and_keeps_hash_checks(self):
        doc=copy.deepcopy(self.doc);doc['public_file_id']='testPublicFile1234567890'
        response=PublicResponse(self.archive.read_bytes(),
                                {'Content-Type':'application/gzip','Content-Length':str(doc['archive_bytes'])})
        with patch.object(assets,'urlopen',return_value=response) as fetch, \
             patch.object(assets.Path,'home',return_value=self.root), \
             patch.object(assets.subprocess,'run') as rclone:
            result=assets.install(None,self.root/'public-models',release=doc)
        rclone.assert_not_called()
        request=fetch.call_args.args[0]
        self.assertEqual(request.full_url,'https://drive.usercontent.google.com/download?id='+doc['public_file_id']+'&export=download&confirm=t')
        self.assertNotIn('Authorization',dict(request.header_items()))
        self.assertNotIn('Cookie',dict(request.header_items()))
        self.assertEqual(fetch.call_args.kwargs['timeout'],assets.PUBLIC_DOWNLOAD_SOCKET_TIMEOUT_SEC)
        self.assertTrue(result['downloaded']);self.assertFalse(result['production_changed'])
        self.assertEqual(assets.validate_install(self.root/'public-models',doc),result['models'])
        # A fully received gzip is still untrusted until its frozen SHA matches.
        damaged=bytearray(self.archive.read_bytes());damaged[-1]^=1
        with patch.object(assets,'urlopen',return_value=PublicResponse(bytes(damaged))), \
             patch.object(assets.Path,'home',return_value=self.root):
            with self.assertRaisesRegex(ValueError,'checksum'):
                assets.install(None,self.root/'bad-public-models',release=doc)
        self.assertFalse((self.root/'bad-public-models').exists())

    def test_public_html_size_redirect_and_network_failures_publish_nothing(self):
        doc=copy.deepcopy(self.doc);doc['public_file_id']='testPublicFile1234567890'
        payload=self.archive.read_bytes()
        responses=[PublicResponse(b'<html>confirm</html>',{'Content-Type':'text/html'}),
                   PublicResponse(b'<html>permission</html>'),
                   PublicResponse(payload,{'Content-Length':str(len(payload)+1)}),
                   PublicResponse(payload+b'extra'),PublicResponse(payload[:-1]),
                   PublicResponse(payload,url='https://example.com/other'),
                   PublicResponse(payload,status=403)]
        for number,response in enumerate(responses):
            destination=self.root/('failed-public-'+str(number))
            with self.subTest(number=number), \
                 patch.object(assets,'urlopen',return_value=response), \
                 patch.object(assets.Path,'home',return_value=self.root):
                with self.assertRaisesRegex(RuntimeError,'--archive'):
                    assets.install(None,destination,release=doc)
            self.assertFalse(destination.exists())
        with patch.object(assets,'urlopen',side_effect=assets.URLError('offline')), \
             patch.object(assets.Path,'home',return_value=self.root):
            with self.assertRaisesRegex(RuntimeError,'--archive'):
                assets.install(None,self.root/'offline-public',release=doc)
        self.assertFalse((self.root/'offline-public').exists())

    def test_public_download_honors_total_deadline_and_split_gzip_signature(self):
        destination=self.root/'deadline.tar.gz'
        with patch.object(assets,'urlopen',return_value=PublicResponse(self.archive.read_bytes())), \
             patch.object(assets.time,'monotonic',side_effect=[0,assets.PUBLIC_DOWNLOAD_MAX_SEC+1]):
            with self.assertRaisesRegex(RuntimeError,'time budget'):
                assets.download_public_archive('testPublicFile1234',destination,self.doc['archive_bytes'])
        class FragmentedResponse(PublicResponse):
            def read1(self, size=-1):
                return super().read1(min(size,1))
        with patch.object(assets,'urlopen',return_value=FragmentedResponse(self.archive.read_bytes())):
            assets.download_public_archive('testPublicFile1234',self.root/'fragmented.tar.gz',self.doc['archive_bytes'])
        self.assertEqual((self.root/'fragmented.tar.gz').read_bytes(),self.archive.read_bytes())

    def test_no_public_field_preserves_authorized_rclone_path(self):
        doc=copy.deepcopy(self.doc)
        def copy_archive(command,**kwargs):
            self.assertEqual(command[:3],['rclone','copyto','team_remote:'+doc['archive']])
            Path(command[3]).write_bytes(self.archive.read_bytes())
        with patch.object(assets,'urlopen') as public, \
             patch.object(assets.Path,'home',return_value=self.root), \
             patch.object(assets.subprocess,'run',side_effect=copy_archive):
            result=assets.install(None,self.root/'rclone-models',remote='team_remote',release=doc)
        public.assert_not_called();self.assertTrue(result['downloaded'])

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
