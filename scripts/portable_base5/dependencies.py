#!/usr/bin/env python3
"""Assemble an isolated, pure-Python ROS overlay from trusted local files.

No apt/pip install, network, ROS node, generated-message import or source edits.
Host Noetic core remains a prerequisite. Platform wheels are assembled elsewhere.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
PACKAGES = ('rosbridge_library', 'rosbridge_server', 'rosbridge_msgs', 'rosauth')
MESSAGE_NAMES = ('CtrlCmd', 'EgoVehicleStatus', 'ObjectStatusList',
                 'SaveSensorData', 'ScenarioLoad', 'CollisionData')
SERVICE_NAMES = ('MoraiEventCmdSrv', 'MoraiEventCmdSrvRequest')
HEX32 = re.compile(r'^[0-9a-f]{32}$')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def generated_attributes(path, class_name):
    """Inspect genpy wire identities without importing ROS or generated code."""
    tree = ast.parse(Path(path).read_text(), filename=str(path))
    matches = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name]
    if len(matches) != 1:
        raise ValueError('Generated ROS class missing or ambiguous: ' + class_name)
    attrs = {}
    for node in matches[0].body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in ('_md5sum', '_full_text', '_type'):
                    attrs[target.id] = ast.literal_eval(node.value)
    if not HEX32.fullmatch(attrs.get('_md5sum', '')):
        raise ValueError('Generated ROS class has invalid wire MD5: ' + class_name)
    return attrs


def normalized_definition(text):
    return [line.split('#', 1)[0].strip() for line in text.splitlines()
            if line.split('#', 1)[0].strip()]


def validate_morai(generated, source):
    md5 = {}
    for name in MESSAGE_NAMES:
        attrs = generated_attributes(generated / 'msg' / ('_' + name + '.py'), name)
        definition = attrs.get('_full_text', '').split('=' * 80, 1)[0]
        if normalized_definition(definition) != normalized_definition((source / 'msg' / (name + '.msg')).read_text()):
            raise ValueError('MORAI generated/source message definitions differ: ' + name)
        if attrs.get('_type') != 'morai_msgs/' + name:
            raise ValueError('Unexpected MORAI generated type: ' + name)
        md5[name] = attrs['_md5sum']
    service = generated / 'srv/_MoraiEventCmdSrv.py'
    for name in SERVICE_NAMES:
        md5[name] = generated_attributes(service, name)['_md5sum']
    request = generated_attributes(service, 'MoraiEventCmdSrvRequest')
    response = generated_attributes(service, 'MoraiEventCmdSrvResponse')
    pieces = (source / 'srv/MoraiEventCmdSrv.srv').read_text().split('---')
    if len(pieces) != 2:
        raise ValueError('MORAI service source must have one request/response separator')
    for attrs, definition in ((request, pieces[0]), (response, pieces[1])):
        if normalized_definition(attrs.get('_full_text', '').split('=' * 80, 1)[0]) != normalized_definition(definition):
            raise ValueError('MORAI generated/source service definitions differ')
    # genpy request/response message MD5s can differ from the combined service
    # MD5. Pin each actual generated identity; do not require their equality.
    return md5


def regular_files(folder):
    folder = Path(folder)
    if folder.is_symlink() or not folder.is_dir():
        raise ValueError('Expected real source directory: ' + str(folder))
    for path in sorted(folder.rglob('*')):
        relative = path.relative_to(folder)
        if '__pycache__' in relative.parts or path.suffix in ('.pyc', '.pyo'):
            continue
        if path.is_symlink():
            raise ValueError('Symlink in local dependency source: ' + str(path))
        if path.is_file():
            yield path, relative


def package_metadata(path, expected):
    xml = ET.parse(path).getroot()
    if xml.findtext('name') != expected:
        raise ValueError('ROS package name differs: ' + str(path))
    return dict(name=expected, version=xml.findtext('version'),
                declared_licenses=[node.text for node in xml.findall('license')])


def assemble(output, *, repository=REPO, ros_prefix=Path('/opt/ros/noetic'),
             doc_root=Path('/usr/share/doc'), generated_directory=None, source_directory=None):
    output, repository, ros_prefix, doc_root = map(Path, (output, repository, ros_prefix, doc_root))
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError('Fresh overlay path required; existing dependencies are preserved')
    generated = Path(generated_directory) if generated_directory is not None else (
        repository / 'base_ws/install_aw149/lib/python3/dist-packages/morai_msgs')
    source = Path(source_directory) if source_directory is not None else repository / 'base_ws/src/morai/morai_msgs'
    for protected in (generated, source, ros_prefix, doc_root):
        if output.resolve() == protected.resolve() or protected.resolve() in output.resolve().parents:
            raise ValueError('Overlay output overlaps immutable dependency source')
    md5 = validate_morai(generated, source)
    plan, metadata, warnings = {}, {}, []

    def include(original, relative):
        original, relative = Path(original), Path(relative)
        if original.is_symlink() or not original.is_file():
            raise ValueError('Expected regular dependency file: ' + str(original))
        if relative.is_absolute() or '..' in relative.parts or str(relative) in plan:
            raise ValueError('Unsafe or duplicate overlay destination: ' + str(relative))
        basename = original.name.lower()
        if (basename.startswith(('.env', 'id_rsa', 'id_ed25519', 'client_secret', 'credentials'))
                or basename in ('rclone.conf', 'config.json', 'token.json')):
            raise ValueError('Credentials/config are not dependency payload: ' + str(original))
        plan[str(relative)] = dict(source=original, sha256=sha256(original))

    def include_python(folder, prefix):
        count = 0
        for original, relative in regular_files(folder):
            if original.suffix != '.py':
                raise ValueError('Unexpected non-Python file in generated/runtime ROS package: ' + str(original))
            ast.parse(original.read_text(), filename=str(original))
            include(original, Path(prefix) / relative)
            count += 1
        if count == 0:
            raise ValueError('Empty ROS Python package: ' + str(folder))

    python_relative = Path('lib/python3/dist-packages')
    include_python(generated, python_relative / 'morai_msgs')
    include(source / 'package.xml', 'share/morai_msgs/package.xml')
    metadata['morai_msgs'] = package_metadata(source / 'package.xml', 'morai_msgs')
    for kind in ('msg', 'srv'):
        for original, relative in regular_files(source / kind):
            if original.suffix != '.' + kind:
                raise ValueError('Unexpected MORAI interface payload: ' + str(original))
            include(original, Path('share/morai_msgs') / kind / relative)
    include(source / 'package.xml', 'licenses/morai_msgs/package.xml')
    warnings.append('morai_msgs source declares license TODO; no license grant inferred')
    for name in PACKAGES:
        include_python(ros_prefix / python_relative / name, python_relative / name)
        share = ros_prefix / 'share' / name
        include(share / 'package.xml', Path('share') / name / 'package.xml')
        metadata[name] = package_metadata(share / 'package.xml', name)
        for kind in ('msg', 'srv'):
            if (share / kind).is_dir():
                for original, relative in regular_files(share / kind):
                    if original.suffix != '.' + kind:
                        raise ValueError('Unexpected ROS interface file: ' + str(original))
                    include(original, Path('share') / name / kind / relative)
        copyright = doc_root / ('ros-noetic-' + name.replace('_', '-')) / 'copyright'
        if copyright.is_file():
            include(copyright, Path('licenses') / name / 'copyright')
        else:
            warnings.append(name + ': installed copyright text unavailable; package declaration and source headers preserved')
        include(share / 'package.xml', Path('licenses') / name / 'package.xml')
    executable = ros_prefix / 'lib/rosbridge_server/rosbridge_websocket'
    ast.parse(executable.read_text(), filename=str(executable))
    include(executable, 'lib/rosbridge_server/rosbridge_websocket')

    output.mkdir(parents=True)
    files = {}
    for relative, item in sorted(plan.items()):
        original = item['source']
        if sha256(original) != item['sha256']:
            raise ValueError('Dependency source changed while assembling: ' + str(original))
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, target)
        if sha256(target) != item['sha256']:
            raise ValueError('Dependency copy checksum differs: ' + relative)
        files[relative] = dict(sha256=item['sha256'], bytes=target.stat().st_size)
    # Retain a copy rather than symlink even on a shared local filesystem.
    for relative, item in plan.items():
        if sha256(item['source']) != item['sha256']:
            raise ValueError('Dependency source changed during assembly: ' + relative)
    receipt = dict(schema='base5_portable_ros_overlay_v1', packages=metadata, ros_md5=md5,
                   files=files, file_count=len(files), bytes=sum(item['bytes'] for item in files.values()),
                   copied=True, symlinks=False, compiled_payload=False, source_modified=False,
                   network_used=False, packages_installed=False, ros_started=False,
                   licensing_warnings=warnings,
                   host_prerequisites=['Python 3.10 compatible ROS1 Noetic core',
                       'std_msgs', 'geometry_msgs', 'rospy', 'roslaunch', 'genpy', 'rospkg'],
                   python_wheels_required=['scipy', 'shapely', 'autobahn', 'Twisted',
                       'tornado', 'pymongo (bson)', 'pyOpenSSL/service_identity and resolved transitives'])
    (output / 'overlay_receipt.json').write_text(json.dumps(receipt, indent=2, sort_keys=True) + '\n')
    return receipt


def assemble_overlay(output, install_prefix, ros_prefix, morai_source):
    """Stable builder API; all arguments are local input/output paths only."""
    return assemble(output, ros_prefix=ros_prefix,
                    generated_directory=Path(install_prefix) / 'lib/python3/dist-packages/morai_msgs',
                    source_directory=morai_source)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repository', type=Path, default=REPO)
    parser.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/noetic'))
    parser.add_argument('--doc-root', type=Path, default=Path('/usr/share/doc'))
    args = parser.parse_args(argv)
    result = assemble(args.output, repository=args.repository, ros_prefix=args.ros_prefix, doc_root=args.doc_root)
    print(json.dumps({key: value for key, value in result.items() if key != 'files'}, indent=2))


if __name__ == '__main__':
    main()
