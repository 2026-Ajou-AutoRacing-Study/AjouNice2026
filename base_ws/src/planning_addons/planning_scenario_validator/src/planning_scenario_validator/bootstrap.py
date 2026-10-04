"""Opt-in Ubuntu 20.04 Docker/Drive inference setup, using only stdlib.

Never removes existing Docker packages, upgrades GPU drivers, runs downloaded
shell scripts, or changes training/local environment configuration.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[6]
IMAGE = 'ajounice/bevfusion:team-cu128-v1'
IMAGE_FILE = 'ajounice-bevfusion-team-cu128-v1.image.tar.zst'
IMAGE_REMOTE = 'model/bevfusion/bevfusion_team_v1_20260916_050457/' + IMAGE_FILE
IMAGE_SHA = 'c52bf20f9edcd2bfe6221fb487519a964b51b18e6bccee7bf8c301c3245e2df7'
E25_FILE = 'base3_bf0_f40_e25_production.tar.zst'
E25_REMOTE = 'model/bevfusion/' + E25_FILE
E25_ARCHIVE_SHA = 'd95ff046ff3ba6a298e6d5e68f42f7fc81f664f5d798f94e183fd7de5bd7928b'
E25_NAME = 'base3_bf0_f40_e25_inference.pth'
E25_SHA = '36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011'
DRIVE_ID = '12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP'
CONFLICTS = ('docker.io', 'docker-compose', 'docker-compose-v2', 'podman-docker', 'containerd', 'runc')


class SetupError(RuntimeError):
    pass


def run(args, *, capture=False, check=True, timeout=None, **kwargs):
    if not capture:
        print('+ ' + ' '.join(str(arg) for arg in args), flush=True)
    environment = dict(kwargs.pop('env', os.environ), LC_ALL='C')
    result = subprocess.run([str(arg) for arg in args], text=True,
                            stdout=kwargs.pop('stdout', subprocess.PIPE if capture else None),
                            stderr=kwargs.pop('stderr', subprocess.STDOUT if capture else None),
                            timeout=timeout, env=environment, **kwargs)
    if check and result.returncode:
        raise SetupError('명령 실패 ({}): {}{}'.format(result.returncode, args[0],
            '\n' + (result.stdout or '')[-1200:] if capture else ' — 위 로그를 확인하세요'))
    return result


def checksum(path, algorithm='sha256'):
    value = hashlib.new(algorithm)
    with Path(path).open('rb') as handle:
        for data in iter(lambda: handle.read(1024**2), b''):
            value.update(data)
    return value.hexdigest()


def installed(package):
    result = run(['dpkg-query', '-W', '-f=${db:Status-Status}', package], capture=True, check=False)
    return result.returncode == 0 and result.stdout.strip() == 'installed'


def ubuntu_check():
    release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '20.04':
        raise SetupError('자동 설치는 Ubuntu 20.04 전용입니다. 다른 OS에 focal 패키지를 설치하지 않습니다')
    if run(['dpkg', '--print-architecture'], capture=True).stdout.strip() != 'amd64':
        raise SetupError('팀 공용 이미지는 amd64 PC용입니다')


def managed_system_file(source, target):
    """Do not overwrite a different existing apt source/key."""
    target = Path(target)
    if target.exists():
        if target.read_bytes() != Path(source).read_bytes():
            raise SetupError('기존 설정을 덮어쓰지 않습니다: {} — 저장소/키 중복을 먼저 확인하세요'.format(target))
        return
    run(['sudo', 'install', '-D', '-m', '0644', source, target])


def repository(kind):
    needle = 'download.docker.com/linux/ubuntu' if kind == 'docker' else 'nvidia.github.io/libnvidia-container'
    destination = '/etc/apt/sources.list.d/ajounice-docker.list' if kind == 'docker' else '/etc/apt/sources.list.d/ajounice-nvidia-container-toolkit.list'
    others = [path for path in Path('/etc/apt/sources.list.d').glob('*')
              if path.suffix in ('.list', '.sources') and str(path) != destination
              and needle in path.read_text()]
    if Path('/etc/apt/sources.list').exists() and needle in Path('/etc/apt/sources.list').read_text():
        others.append(Path('/etc/apt/sources.list'))
    if others:
        print('기존 apt 저장소 사용: ' + ', '.join(str(path) for path in others))
        run(['sudo', 'apt-get', 'update'])
        return
    with tempfile.TemporaryDirectory(prefix='ajou-apt-') as directory:
        directory = Path(directory)
        if kind == 'docker':
            key = directory / 'docker.asc'
            run(['curl', '-fL', '--retry', '3', 'https://download.docker.com/linux/ubuntu/gpg', '-o', key])
            managed_system_file(key, '/etc/apt/keyrings/docker.asc')
            text = 'deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu focal stable\n'
            destination = '/etc/apt/sources.list.d/ajounice-docker.list'
        else:
            key = directory / 'nvidia.asc'
            run(['curl', '-fL', '--retry', '3', 'https://nvidia.github.io/libnvidia-container/gpgkey', '-o', key])
            binary = directory / 'nvidia.gpg'
            run(['gpg', '--batch', '--dearmor', '--output', binary, key])
            managed_system_file(binary, '/usr/share/keyrings/ajounice-nvidia-container-toolkit.gpg')
            listing = run(['curl', '-fsSL', '--retry', '3', 'https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list'], capture=True).stdout
            text = listing.replace('deb https://', 'deb [signed-by=/usr/share/keyrings/ajounice-nvidia-container-toolkit.gpg] https://')
            destination = '/etc/apt/sources.list.d/ajounice-nvidia-container-toolkit.list'
        source = directory / 'source.list'
        source.write_text(text)
        managed_system_file(source, destination)
    run(['sudo', 'apt-get', 'update'])


def apt_install(packages):
    for package in packages:
        policy = run(['apt-cache', 'policy', package], capture=True).stdout
        if not any(line.strip().startswith('Candidate:') and '(none)' not in line for line in policy.splitlines()):
            raise SetupError('설치 가능한 패키지가 없습니다: {}. Ubuntu 20.04 저장소를 확인하세요'.format(package))
    run(['sudo', 'apt-get', 'install', '-y', '--no-upgrade', *packages])


def docker_command(allow_sudo):
    if run(['docker', 'info'], capture=True, check=False).returncode == 0:
        return ['docker']
    if not allow_sudo:
        raise SetupError('sudo 없이 docker info가 안 됩니다. Docker 그룹 재로그인/daemon 상태부터 확인하세요')
    if os.environ.get('DOCKER_HOST') or os.environ.get('DOCKER_CONTEXT'):
        raise SetupError('사용자 Docker endpoint가 지정됐습니다. 이를 무시하고 root daemon으로 바꾸지 않습니다')
    context = run(['docker', 'context', 'show'], capture=True).stdout.strip()
    if context != 'default':
        raise SetupError('기본 Docker context가 아닙니다: ' + context)
    if run(['sudo', 'docker', 'info'], capture=True, check=False).returncode:
        raise SetupError('Docker daemon에 연결할 수 없습니다. systemctl status docker를 확인하세요')
    return ['sudo', 'docker']


def system_setup():
    # Refuse destructive package replacement. Preserve working existing Docker.
    existing = shutil.which('docker') is not None
    if os.environ.get('DOCKER_HOST') or os.environ.get('DOCKER_CONTEXT'):
        raise SetupError('Docker endpoint 환경변수가 있어 시스템 설정을 자동 변경하지 않습니다. 기본 로컬 context를 확인하세요')
    if existing and run(['docker', 'context', 'show'], capture=True).stdout.strip() != 'default':
        raise SetupError('시스템 자동 설치는 기본 로컬 Docker context만 지원합니다')
    if not existing or run(['docker', 'compose', 'version'], capture=True, check=False).returncode:
        conflicts = [package for package in CONFLICTS if installed(package)]
        if conflicts:
            raise SetupError('기존 Docker 관련 패키지는 자동 삭제하지 않습니다: ' + ', '.join(conflicts) + '\n팀원 가이드의 수동 Docker 설치 부분을 확인하세요')
    tools = ('ca-certificates', 'curl', 'gnupg', 'rclone', 'zstd',
             'ffmpeg', 'x11-utils', 'python3-yaml', 'python3-tk', 'python3-gi', 'gir1.2-wnck-3.0')
    missing = [package for package in tools if not installed(package)]
    if missing:
        run(['sudo', 'apt-get', 'update'])
        apt_install(missing)
    if not existing:
        repository('docker')
        apt_install(['docker-ce', 'docker-ce-cli', 'containerd.io', 'docker-compose-plugin'])
        run(['sudo', 'systemctl', 'enable', '--now', 'docker'])
    elif run(['docker', 'compose', 'version'], capture=True, check=False).returncode:
        repository('docker')
        apt_install(['docker-compose-plugin'])
    docker = docker_command(True)
    runtimes = run(docker + ['info', '--format', '{{json .Runtimes}}'], capture=True).stdout
    if 'nvidia' not in json.loads(runtimes):
        if 'rootless' in run(docker + ['info', '--format', '{{json .SecurityOptions}}'], capture=True).stdout:
            raise SetupError('Rootless Docker의 GPU 설정은 자동 변경하지 않습니다. NVIDIA Rootless 안내를 확인하세요')
        if run(docker + ['ps', '-q'], capture=True).stdout.strip():
            raise SetupError('실행 중인 컨테이너가 있어 Docker를 재시작하지 않습니다. 학습/추론을 종료한 뒤 다시 실행하세요')
        repository('nvidia')
        apt_install(['nvidia-container-toolkit'])
        daemon = Path('/etc/docker/daemon.json')
        if daemon.exists():
            # sudo copy preserves permissions; a timestamped backup is never overwritten.
            import datetime
            backup = str(daemon) + '.ajounice.' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.bak'
            run(['sudo', 'cp', '-a', daemon, backup])
        run(['sudo', 'nvidia-ctk', 'runtime', 'configure', '--runtime=docker'])
        run(['sudo', 'systemctl', 'restart', 'docker'])
    run(['docker', 'compose', 'version'])
    if docker[0] == 'sudo':
        username = run(['id', '-un'], capture=True).stdout.strip()
        run(['sudo', 'usermod', '-aG', 'docker', username])
        print('주의: docker 그룹은 관리자 수준 권한입니다. 본인 계정만 추가했습니다.\n설치 후 로그아웃/로그인하고 sudo 없이 docker ps를 확인하세요.', flush=True)
    return docker


def ensure_remote(remote, drive_id):
    remotes = run(['rclone', 'listremotes'], capture=True).stdout.splitlines()
    if remote + ':' not in remotes:
        print('Google 로그인은 본인 계정으로 한 번 해야 합니다.\n'
              'rclone: n → 이름 ' + remote + ' → Google Drive → client_id/secret 비움 → scope drive → 브라우저 인증.\n'
              '일반 공유 AjouNice2026 폴더이며 Shared Drive로 지정하지 않습니다.', flush=True)
        if not sys.stdin.isatty():
            raise SetupError('대화형 터미널에서 rclone config로 로그인한 뒤 같은 설치 명령을 다시 실행하세요')
        run(['rclone', 'config'])
        if remote + ':' not in run(['rclone', 'listremotes'], capture=True).stdout.splitlines():
            raise SetupError(remote + ' remote가 없습니다. 로그인 설정 후 다시 실행하세요')
    run(['rclone', 'lsf', remote + ':model/bevfusion', '--max-depth', '1',
         '--drive-root-folder-id', drive_id], capture=True, timeout=60)


def download(remote, drive_id, source, destination, expected, algorithm='sha256'):
    destination = Path(destination)
    if destination.exists():
        if checksum(destination, algorithm) != expected:
            raise SetupError('기존 압축의 checksum이 다릅니다. 삭제/덮어쓰지 않았습니다: ' + str(destination))
        print('검증된 다운로드 재사용: ' + str(destination), flush=True)
        return destination
    partial = destination.with_suffix(destination.suffix + '.part')
    run(['rclone', 'copyto', remote + ':' + source, partial, '--drive-root-folder-id', drive_id,
         '--checksum', '--progress', '--contimeout', '15s', '--timeout', '1m', '--retries', '3'])
    if checksum(partial, algorithm) != expected:
        raise SetupError('다운로드 checksum 불일치. .part를 보존했고 등록하지 않았습니다: ' + str(partial))
    run(['zstd', '-t', partial])
    partial.replace(destination)
    return destination


def extract_checkpoint(tar_path, destination, expected=E25_SHA):
    """Read only the unique regular checkpoint; never extract paths/symlinks."""
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    candidate = None
    count = 0
    try:
        with tarfile.open(tar_path, 'r|') as archive:
            for member in archive:
                count += 1
                if count > 10000:
                    raise SetupError('E25 압축에 파일이 지나치게 많습니다')
                if Path(member.name).name != E25_NAME:
                    continue
                if candidate is not None or not member.isfile() or not 0 < member.size <= 2 * 1024**3:
                    raise SetupError('E25는 압축 안에 regular file 하나여야 합니다')
                with tempfile.NamedTemporaryFile(prefix='.e25-', dir=destination.parent, delete=False) as output:
                    candidate = Path(output.name)
                    with archive.extractfile(member) as source:
                        shutil.copyfileobj(source, output)
                    output.flush()
                    os.fsync(output.fileno())
        if candidate is None or checksum(candidate) != expected:
            raise SetupError('압축의 E25 가중치가 없거나 SHA256이 다릅니다')
        # Exclusive install avoids replacing an existing checkpoint after a race.
        os.link(candidate, destination)
    finally:
        if candidate is not None:
            candidate.unlink(missing_ok=True)


def load_image(path, docker):
    print('압축을 스트리밍으로 풀어 docker load합니다. 거대한 .tar 사본은 만들지 않습니다.', flush=True)
    process = subprocess.Popen(['zstd', '-dc', str(path)], stdout=subprocess.PIPE)
    try:
        loaded = subprocess.run(docker + ['load'], stdin=process.stdout)
        process.stdout.close()
        code = process.wait()
        if code or loaded.returncode:
            raise SetupError('이미지 압축 해제 또는 docker load 실패')
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait()


def setup(args):
    if os.geteuid() == 0:
        raise SetupError('스크립트 전체를 sudo로 실행하지 마세요. 필요한 시스템 명령만 내부에서 sudo를 사용합니다')
    ubuntu_check()
    if args.dry_run:
        print('실행 계획만 표시합니다. sudo/apt/Drive 다운로드/GPU 연산/설정 변경은 하지 않습니다.\n'
              '1. 기존 Docker 보존, 없으면 focal 공식 apt에서 Engine/Compose 설치\n'
              '2. NVIDIA Container Toolkit 필요 시 설치·설정 (컨테이너 실행 중 재시작 금지)\n'
              '3. rclone 본인 로그인/공유 폴더 접근 확인\n'
              '4. ' + args.remote + ':' + IMAGE_REMOTE + '\n'
              '5. ' + args.remote + ':' + E25_REMOTE + '\n'
              '6. checksum → 스트리밍 docker load → E25 단일 파일 설치 → 실제 GPU 검사\n'
              '다운로드 폴더: ' + str(args.download_dir) + '\n가중치: ' + str(args.checkpoint_path))
        return
    if not shutil.which('nvidia-smi') or run(['nvidia-smi'], capture=True, check=False).returncode:
        raise SetupError('호스트 NVIDIA 드라이버가 작동하지 않습니다. 드라이버는 자동 설치/업그레이드하지 않습니다')
    if args.checkpoint_path.exists() and checksum(args.checkpoint_path) != E25_SHA:
        raise SetupError('기존 가중치가 E25 기준과 다릅니다. 덮어쓰지 않습니다: ' + str(args.checkpoint_path))
    args.download_dir.mkdir(parents=True, exist_ok=True)
    with (args.download_dir / '.setup.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise SetupError('같은 다운로드 폴더의 설치가 이미 실행 중입니다') from error
        docker = docker_command(False) if args.assets_only else system_setup()
        for tool in ('rclone', 'zstd'):
            if not shutil.which(tool):
                raise SetupError(tool + '가 없습니다. 기본 설치 모드로 실행하세요')
        image_ready = run(docker + ['image', 'inspect', IMAGE], capture=True, check=False).returncode == 0
        checkpoint_ready = args.checkpoint_path.exists()
        if not image_ready or not checkpoint_ready:
            ensure_remote(args.remote, args.drive_root_folder_id)
            for directory, threshold in ((args.download_dir, 10 if not image_ready else 1), (args.checkpoint_path.parent, 1)):
                parent = directory
                while not parent.exists():
                    parent = parent.parent
                if shutil.disk_usage(parent).free < threshold * 1024**3:
                    raise SetupError('다운로드/가중치 공간 부족: ' + str(parent))
            if not image_ready:
                docker_root = Path(run(docker + ['info', '--format', '{{.DockerRootDir}}'], capture=True).stdout.strip())
                probe = run(docker[:-1] + ['df', '-Pk', str(docker_root)], capture=True, check=False)
                if probe.returncode or int(probe.stdout.splitlines()[-1].split()[3]) < 30 * 1024**2:
                    raise SetupError('Docker data-root에 최소 30GiB 여유를 확인할 수 없습니다: ' + str(docker_root))
                image = download(args.remote, args.drive_root_folder_id, IMAGE_REMOTE, args.download_dir / IMAGE_FILE, IMAGE_SHA)
                load_image(image, docker)
            if not checkpoint_ready:
                archive = download(args.remote, args.drive_root_folder_id, E25_REMOTE, args.download_dir / E25_FILE, E25_ARCHIVE_SHA)
                # Small release tar only; the multi-GB image is never expanded to disk.
                with tempfile.TemporaryDirectory(prefix='.e25-extract-', dir=args.download_dir) as temporary:
                    tar_path = Path(temporary) / 'release.tar'
                    with tar_path.open('wb') as output:
                        run(['zstd', '-dc', archive], stdout=output)
                    extract_checkpoint(tar_path, args.checkpoint_path)
        print('이미지/가중치 준비됨. 실제 모듈·CUDA 연산을 검사합니다.', flush=True)
        environment = dict(os.environ, BEVFUSION_IMAGE=IMAGE)
        # Existing verifier is shared; a sudo-required first install gets one
        # transient docker wrapper for this check, not a changed shell profile.
        with tempfile.TemporaryDirectory(prefix='ajou-docker-check-') as temporary:
            if docker[0] == 'sudo':
                wrapper = Path(temporary) / 'docker'
                wrapper.write_text('#!/bin/sh\nexec /usr/bin/sudo /usr/bin/docker "$@"\n')
                wrapper.chmod(0o755)
                environment['PATH'] = temporary + ':' + environment.get('PATH', '')
            run(['bash', ROOT / 'docker/bevfusion/scripts/verify_team_image.sh'], env=environment, timeout=300)
        print('PASS: 공용 Docker + E25 + 실제 GPU 검사 완료\n'
              '가중치: ' + str(args.checkpoint_path) + '\n'
              '학습 데이터/기존 .env/NVIDIA 드라이버는 변경하지 않았습니다.\n'
              '기존 .env.bevfusion.local이 있으면 이미지·checkpoint 경로를 GUI 환경 검사로 확인하세요.\n'
              '다음: install.sh --gui → MORAI Connect → ./planning_validator_gui', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='팀 Ubuntu 20.04: Docker 설치부터 Drive 이미지/E25 등록까지')
    parser.add_argument('--dry-run', action='store_true', help='변경/다운로드 없는 실행 계획')
    parser.add_argument('--assets-only', action='store_true', help='apt/그룹/daemon 변경 없이 기존 Docker에 자료 준비')
    parser.add_argument('--remote', default='ajou_drive', help='본인 rclone remote 이름 (콜론 없이)')
    parser.add_argument('--drive-root-folder-id', default=DRIVE_ID)
    parser.add_argument('--download-dir', type=lambda value: Path(value).expanduser().resolve(), default=Path.home() / 'bevfusion_downloads')
    parser.add_argument('--checkpoint-path', type=lambda value: Path(value).expanduser().resolve(), default=Path.home() / 'bevfusion_artifacts/checkpoints' / E25_NAME)
    args = parser.parse_args(argv)
    if not args.remote or ':' in args.remote or '/' in args.remote:
        parser.error('--remote에는 콜론/경로 없이 rclone 이름만 지정하세요')
    try:
        setup(args)
        return 0
    except (SetupError, OSError, ValueError, subprocess.TimeoutExpired, tarfile.TarError) as error:
        print('설치 중단: ' + str(error) + '\n기존 파일은 보존했습니다. 문제를 해결한 뒤 같은 명령을 다시 실행하세요.', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('사용자 중단. 완료한 자료는 재사용하고 미완료 다운로드는 다시 시도합니다.', file=sys.stderr)
        return 130
