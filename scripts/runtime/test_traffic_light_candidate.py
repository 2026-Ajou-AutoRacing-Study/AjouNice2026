"""Pure pinned-asset/installer contract tests. No network/GPU/ROS required."""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import traffic_light_candidate as assets


MOCK_ELF_SOURCE = r'''import struct
def plugin_elf(strip_rpath=False, tamper='', old=False):
    names=['','.note.gnu.build-id','.text','.rodata','.dynstr','.dynsym','.dynamic','.shstrtab']
    shstr=b'\0';name_offsets={}
    for name in names[1:]:name_offsets[name]=len(shstr);shstr+=name.encode()+b'\0'
    strings=b'\0symbol\0libmock.so\0';path_offset=len(strings);path=b'/mock/build/rpath\0';strings+=path
    if strip_rpath:strings=strings[:path_offset]+b'\0'*len(path)
    entries=[(1,8),(29,path_offset),(12,0x1234)]
    if strip_rpath:entries=[entry for entry in entries if entry[0]!=29]
    dynamic=b''.join(struct.pack('<qQ',*entry) for entry in entries).ljust(80,b'\0')
    markers=b'enable_signal_compliance_guard signal_compliance_front_axle_offset_m enable_yellow_passthrough'
    data=[b'',struct.pack('<III',4,20,3)+b'GNU\0'+b'b'*20,b'\x90\xc3',
          b'old plugin' if old else markers,strings,b'\0'*24+struct.pack('<IBBHQQ',1,0,0,0,0,0),dynamic,shstr]
    if tamper=='text':data[2]=b'\xcc'+data[2][1:]
    if tamper=='rodata':data[3]=b'X'+data[3][1:]
    if tamper=='dynsym':data[5]=data[5][:-1]+b'\x01'
    if tamper=='build_id':data[1]=data[1][:-1]+b'c'
    kinds=[0,7,1,1,3,11,6,3];flags=[0,2,6,2,2,2,3,0]
    links=[0,0,0,0,0,4,4,0];sizes=[0,0,0,0,0,24,16,0]
    body=bytearray(64);headers=[(0,)*10]
    for index in range(1,len(names)):
        body+=b'\0'*(-len(body)%8);offset=len(body);body+=data[index]
        headers.append((name_offsets[names[index]],kinds[index],flags[index],0,offset,len(data[index]),links[index],0,8,sizes[index]))
    body+=b'\0'*(-len(body)%8);section_offset=len(body)
    for header in headers:body+=struct.pack('<IIQQQQIIQQ',*header)
    body[:64]=struct.pack('<16sHHIQQQIHHHHHH',b'\x7fELF\x02\x01\x01'+b'\0'*9,
                          3,62,1,0,0,section_offset,0,64,0,0,64,len(headers),7)
    return bytes(body)
'''


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
        defaults={n.attrib['name']:n.attrib.get('default') for n in ET.parse(package/'traffic_light_node.launch').getroot().findall('arg')}
        self.assertEqual((defaults['selection_mode'],defaults['classifier_mode'],defaults['detector_imgsz']),('map_association','cnn','960'))
        legacy={n.attrib['name']:n.attrib['value'] for n in ET.parse(package/'traffic_light_legacy.launch').getroot().find('include').findall('arg')}
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


class InstallerRuntimeTest(unittest.TestCase):
    """Run the shell in a tiny fake checkout; no real build/model/ROS/network."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)/'repo';self.root.mkdir()
        self.bin=self.root/'mock-bin';self.bin.mkdir()
        self.log=self.root/'commands.jsonl'
        self.cpp='autoware_behavior_velocity_traffic_light_module'
        self.traffic=self.root/'base_ws/src/perception/traffic_light_detection'
        self.integration=self.root/'base_ws/src/planning_addons/ajounice_planning_integration'
        self.validator=self.root/'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator'
        self.validator_installed=self.root/'base_ws/install_aw149/lib/python3/dist-packages/planning_scenario_validator'
        self.plugin=self.root/'planning_ws/install_isolated/lib'/('lib'+self.cpp+'.so')
        self.receipt=self.root/'base_ws/install_aw149/share/traffic_light_detection/oneclick_runtime_receipt.json'
        self.write('scripts/runtime/install_traffic_light_candidate.sh',
            (assets.ROOT/'scripts/runtime/install_traffic_light_candidate.sh').read_text())
        for relative in ('base_ws/src/perception/traffic_light_detection/CMakeLists.txt',
                         'base_ws/src/planning_addons/ajounice_planning_integration/CMakeLists.txt',
                         'planning_ws/src/'+self.cpp+'/CMakeLists.txt'):
            self.write(relative,'# mock source\n')
        self.write('planning_ws/src/'+self.cpp+'/src/scene.cpp','// mock compliance source\n')
        self.write('planning_ws/src/'+self.cpp+'/include/guard.hpp','// mock guard header\n')
        self.write('planning_ws/src/'+self.cpp+'/plugins.xml','<library/>\n')
        self.write('planning_ws/src/'+self.cpp+'/config/policy.yaml','guard: false\n')
        self.write('base_ws/src/perception/traffic_light_detection/scripts/node.py','#!/usr/bin/env python3\n# node body\n')
        self.write('base_ws/src/perception/traffic_light_detection/launch/traffic.launch','<launch/>\n')
        self.write('base_ws/src/perception/traffic_light_detection/config/policy.yaml','guard: true\n')
        self.write('base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch','<launch/>\n')
        self.write('base_ws/src/planning_addons/ajounice_planning_integration/config/planning.yaml','value: 1\n')
        self.write('base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/__init__.py','"""Validator package."""\n')
        self.write('base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/runner.py','"""Runner first line."""\nVALUE = 1\n')
        self.write('base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/nested/options.py','"""Nested module."""\nVALUE = 2\n')
        self.write('scripts/workspace/source_aw149.sh','# fake source; never calls ROS\n')
        self.write('planning_ws/tools/build.sh','#!/usr/bin/env bash\nexec mock_planning_build "$@"\n')
        self.write('scripts/runtime/setup_traffic_light_env.sh','#!/usr/bin/env bash\nexec mock_env "$@"\n')
        self.write('scripts/runtime/check_traffic_light_env.py','import os,json\nwith open(os.environ["MOCK_LOG"],"a") as f:f.write(json.dumps(["check_env"])+"\\n")\n')
        self.write('scripts/runtime/traffic_light_candidate.py', '''import json,os
def load_release():return {'models':{'yolo':{'sha256':'a'*64},'cnn':{'sha256':'b'*64}}}
def paths(directory,doc):return {'yolo':'/mock/yolo.pt','cnn':'/mock/cnn.pt'}
if __name__=='__main__':
    with open(os.environ['MOCK_LOG'],'a') as f:f.write(json.dumps(['models'])+'\\n')
''')
        self.write('mock_elf.py',MOCK_ELF_SOURCE)
        # One mock executable handles only commands the installer may invoke.
        self.write('mock-bin/dispatch', '''#!/usr/bin/python3
import json,os,shutil,sys
from pathlib import Path
r=Path(os.environ['MOCK_ROOT']);name=Path(sys.argv[0]).name
sys.path.insert(0,str(r))
from mock_elf import plugin_elf
with open(os.environ['MOCK_LOG'],'a') as f:f.write(json.dumps([name]+sys.argv[1:])+'\\n')
cpp='autoware_behavior_velocity_traffic_light_module'
def install_assets():
    for package,source in [('traffic_light_detection',r/'base_ws/src/perception/traffic_light_detection'),
                           ('ajounice_planning_integration',r/'base_ws/src/planning_addons/ajounice_planning_integration')]:
        for folder in ('launch','config'):
            shutil.copytree(source/folder,r/'base_ws/install_aw149/share'/package/folder,dirs_exist_ok=True)
        if package=='traffic_light_detection':
            target=r/'base_ws/install_aw149/lib'/package;target.mkdir(parents=True,exist_ok=True)
            for p in (source/'scripts').glob('*.py'):shutil.copy2(p,target/p.name)
    (r/'base_ws/install_aw149/share/planning_scenario_validator').mkdir(parents=True,exist_ok=True)
    shutil.copytree(r/'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator',r/'base_ws/install_aw149/lib/python3/dist-packages/planning_scenario_validator',dirs_exist_ok=True)
if name=='mock_planning_build':
    if os.environ.get('MOCK_BUILD_FAIL'):raise SystemExit(9)
    old=bool(os.environ.get('MOCK_OLD_PLUGIN'))
    built=plugin_elf(old=old)
    installed=plugin_elf(strip_rpath=bool(os.environ.get('MOCK_STRIP_RPATH')),tamper=os.environ.get('MOCK_TAMPER',''),old=old)
    for p,body in [(r/'planning_ws/install_isolated/lib'/('lib'+cpp+'.so'),installed),
                   (r/'planning_ws/devel_isolated'/cpp/'lib'/('lib'+cpp+'.so'),built)]:
        p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(body)
    (r/'planning_ws/install_isolated/share'/cpp).mkdir(parents=True,exist_ok=True)
    shutil.copy2(r/'planning_ws/src'/cpp/'plugins.xml',r/'planning_ws/install_isolated/share'/cpp/'plugins.xml')
    shutil.copytree(r/'planning_ws/src'/cpp/'config',r/'planning_ws/install_isolated/share'/cpp/'config',dirs_exist_ok=True)
elif name=='cmake':
    if 'install' in sys.argv:install_assets()
elif name=='catkin_make_isolated':install_assets()
elif name=='rospack':
    package=sys.argv[2]
    if os.environ.get('MOCK_WRONG_OVERLAY'):print('/other/share/'+package)
    else:print(r/('planning_ws/install_isolated/share' if package==cpp else 'base_ws/install_aw149/share')/package)
elif name=='ldd':
    print('libmock.so => not found' if os.environ.get('MOCK_BAD_LINK') else 'libmock.so => /mock/libmock.so')
elif name!='mock_env':raise SystemExit('Unexpected external command '+name)
''',executable=True)
        for name in ('cmake','catkin_make_isolated','rospack','ldd','mock_planning_build','mock_env'):
            (self.bin/name).symlink_to(self.bin/'dispatch')
        self.env=dict(os.environ,MOCK_ROOT=str(self.root),MOCK_LOG=str(self.log),
                      PATH=str(self.bin)+os.pathsep+os.environ['PATH'])
        self.caches()

    def write(self,relative,text,executable=False):
        path=self.root/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
        if executable:path.chmod(0o755)
        return path

    def caches(self):
        for package in ('traffic_light_detection','planning_scenario_validator','ajounice_planning_integration',self.cpp):
            planning=package==self.cpp
            workspace='planning_ws' if planning else 'base_ws'
            build='build_isolated' if planning else 'build_aw149'
            source=('planning_ws/src/'+package if planning else 'base_ws/src/'+
                    ('perception/' if package=='traffic_light_detection' else 'planning_addons/')+package)
            prefix=self.root/workspace/('install_isolated' if planning else 'install_aw149')
            self.write(workspace+'/'+build+'/'+package+'/CMakeCache.txt',
                'CMAKE_HOME_DIRECTORY:INTERNAL='+str(self.root/source)+'\nCMAKE_INSTALL_PREFIX:PATH='+str(prefix)+'\n')

    def run_installer(self,*arguments,**environment):
        return subprocess.run(['bash',str(self.root/'scripts/runtime/install_traffic_light_candidate.sh'),
                               *arguments],env=dict(self.env,**environment),text=True,capture_output=True)

    def commands(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_default_rebuilds_only_plugin_and_three_base_packages_then_verifies(self):
        result=self.run_installer()
        self.assertEqual(result.returncode,0,result.stderr)
        calls=self.commands()
        build=next(c for c in calls if c[0]=='mock_planning_build')
        self.assertEqual(build[1:],['--pkg',self.cpp,'--force-cmake','--make-args','-j2','-l2'])
        cmake=[c for c in calls if c[0]=='cmake']
        self.assertEqual(len(cmake),4)
        self.assertIn('clean',cmake[0]);self.assertIn(self.cpp,cmake[0][2])
        self.assertEqual([Path(c[2]).name for c in cmake[1:]],
                         ['traffic_light_detection','planning_scenario_validator','ajounice_planning_integration'])
        self.assertTrue(self.receipt.is_file())
        self.assertIn('[RUNTIME VERIFIED]',result.stdout)
        self.assertIn('[V5 READY]',result.stdout)

    def test_check_is_build_free_and_rejects_stale_cpp_source(self):
        self.assertEqual(self.run_installer().returncode,0)
        before=self.receipt.read_bytes();self.log.unlink()
        checked=self.run_installer('--check')
        self.assertEqual(checked.returncode,0,checked.stderr)
        self.assertFalse(any(c[0] in ('cmake','mock_planning_build','catkin_make_isolated') for c in self.commands()))
        self.assertEqual(self.receipt.read_bytes(),before)
        (self.root/'planning_ws/src'/self.cpp/'src/scene.cpp').write_text('// newer source\n')
        stale=self.run_installer('--check')
        self.assertNotEqual(stale.returncode,0)
        self.assertIn('Runtime receipt differs',stale.stderr)
        self.assertNotIn('[V5 READY]',stale.stdout)

    def test_validator_source_and_nested_modules_are_in_fingerprint_and_install_audit(self):
        installed=self.run_installer()
        self.assertEqual(installed.returncode,0,installed.stderr)
        receipt=json.loads(self.receipt.read_text())
        for module in ('__init__.py','runner.py','nested/options.py'):
            source=self.validator/module;target=self.validator_installed/module
            self.assertIn(str(source.relative_to(self.root)),receipt['source_sha256'])
            self.assertIn(str(target.relative_to(self.root)),receipt['installed_asset_sha256'])
        checked=self.run_installer('--check')
        self.assertEqual(checked.returncode,0,checked.stderr)
        # Package first lines are not catkin script shebangs; preserve them.
        (self.validator_installed/'runner.py').write_text('"""Tampered first line."""\nVALUE = 1\n')
        tampered=self.run_installer('--check')
        self.assertNotEqual(tampered.returncode,0)
        self.assertIn('Stale installed runtime asset',tampered.stderr)
        self.assertNotIn('[V5 READY]',tampered.stdout)

    def test_validator_source_change_and_missing_installed_module_cannot_pass(self):
        self.assertEqual(self.run_installer().returncode,0)
        source=self.validator/'nested/options.py';target=self.validator_installed/'nested/options.py'
        source.write_text('"""New nested source."""\nVALUE = 3\n')
        target.write_bytes(source.read_bytes())
        # Even byte-matching ad-hoc copies do not update the build receipt.
        stale=self.run_installer('--check')
        self.assertNotEqual(stale.returncode,0)
        self.assertIn('Runtime receipt differs',stale.stderr)
        target.unlink()
        missing=self.run_installer('--check')
        self.assertNotEqual(missing.returncode,0)
        self.assertIn('Missing installed runtime asset',missing.stderr)

    def test_build_failure_keeps_installed_plugin_and_never_claims_ready(self):
        self.plugin.parent.mkdir(parents=True);self.plugin.write_bytes(b'user existing installed plugin')
        result=self.run_installer(MOCK_BUILD_FAIL='1')
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.plugin.read_bytes(),b'user existing installed plugin')
        self.assertFalse(self.receipt.exists())
        self.assertNotIn('[V5 READY]',result.stdout)

    def test_old_plugin_missing_dependency_and_wrong_overlay_cannot_pass_install(self):
        for environment in ({'MOCK_OLD_PLUGIN':'1'},{'MOCK_BAD_LINK':'1'},{'MOCK_WRONG_OVERLAY':'1'}):
            with self.subTest(environment=environment):
                result=self.run_installer(**environment)
                self.assertNotEqual(result.returncode,0)
                self.assertFalse(self.receipt.exists())
                self.assertNotIn('[V5 READY]',result.stdout)

    def test_cmake_rpath_removal_is_accepted_with_distinct_raw_hashes(self):
        result=self.run_installer(MOCK_STRIP_RPATH='1')
        self.assertEqual(result.returncode,0,result.stderr)
        receipt=json.loads(self.receipt.read_text())
        self.assertNotEqual(receipt['built_plugin_sha256'],receipt['installed_plugin_sha256'])
        elf=receipt['plugin_elf_identity']
        self.assertEqual(elf['gnu_build_id'],(b'b'*20).hex())
        self.assertEqual(set(elf['normalized_alloc_section_sha256']),
                         {'.note.gnu.build-id','.text','.rodata','.dynstr','.dynsym','.dynamic'})
        checked=self.run_installer('--check')
        self.assertEqual(checked.returncode,0,checked.stderr)

    def test_code_data_symbol_tampering_with_reused_build_id_is_rejected(self):
        for section in ('text','rodata','dynsym'):
            with self.subTest(section=section):
                result=self.run_installer(MOCK_STRIP_RPATH='1',MOCK_TAMPER=section)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('differs outside expected RPATH relocation',result.stderr)
                self.assertIn('.'+section,result.stderr)
                self.assertFalse(self.receipt.exists())
                self.assertNotIn('[V5 READY]',result.stdout)

    def test_different_gnu_build_id_is_rejected(self):
        result=self.run_installer(MOCK_TAMPER='build_id')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('GNU build-id differs',result.stderr)
        self.assertFalse(self.receipt.exists())

    def test_models_only_and_explicit_skip_build_preserve_their_narrow_scope(self):
        result=self.run_installer('--models-only')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(self.commands(),[['models']])
        self.log.unlink()
        result=self.run_installer('--skip-build','--existing-python','/usr/bin/python3')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(self.commands(),[['models'],['check_env']])
        self.assertIn('[MODEL/ENV ONLY]',result.stdout)
        self.assertNotIn('[V5 READY]',result.stdout)

    def test_missing_base_cache_uses_only_scoped_base_fallback(self):
        (self.root/'base_ws/build_aw149/ajounice_planning_integration/CMakeCache.txt').unlink()
        result=self.run_installer()
        self.assertEqual(result.returncode,0,result.stderr)
        command=next(c for c in self.commands() if c[0]=='catkin_make_isolated')
        index=command.index('--pkg')
        self.assertEqual(command[index+1:index+4],
                         ['traffic_light_detection','planning_scenario_validator','ajounice_planning_integration'])

    def test_foreign_planning_cache_is_not_cleaned_or_reused(self):
        path=self.root/'planning_ws/build_isolated'/self.cpp/'CMakeCache.txt'
        original=path.read_text().replace(str(self.root/'planning_ws/src'/self.cpp),'/other/source')
        path.write_text(original)
        result=self.run_installer()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('Refusing foreign/moved CMake cache',result.stderr)
        self.assertEqual(path.read_text(),original)
        self.assertFalse(any(c[0] in ('cmake','mock_planning_build') for c in self.commands()))

    def test_foreign_base_cache_with_missing_cache_is_rejected_before_any_build(self):
        (self.root/'base_ws/build_aw149/ajounice_planning_integration/CMakeCache.txt').unlink()
        path=self.root/'base_ws/build_aw149/traffic_light_detection/CMakeCache.txt'
        original=path.read_text().replace(str(self.traffic),'/other/traffic/source')
        path.write_text(original)
        result=self.run_installer()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('Refusing foreign/moved CMake cache',result.stderr)
        self.assertEqual(path.read_text(),original)
        self.assertFalse(any(c[0] in ('cmake','mock_planning_build','catkin_make_isolated') for c in self.commands()))


if __name__=='__main__':unittest.main()
