#!/usr/bin/env bash
# Existing ROS workspace -> pinned models + isolated env + scoped runtime install.
# No nodes, scenario loads, driving commands, public sharing or production switch.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ARGS=()
PREFIX="$HOME/.local/share/ajounice/traffic_light_gpu"
EXISTING_PYTHON=""
DEVICE=0
CHECK=false
MODELS_ONLY=false
SKIP_BUILD=false
usage() {
  echo '사용: bash scripts/runtime/install_traffic_light_candidate.sh [--archive TAR] [--remote RCLONE_NAME] [--models-dir DIR] [--env-prefix DIR] [--existing-python PYTHON] [--device 0|cpu] [--models-only] [--skip-build] [--check]'
  echo '기존 ROS/풀스택 workspace가 필요합니다. 기본값은 모델·환경 검사 + 신호등 C++ 1개 패키지 재빌드 + traffic/Validator/planning integration 증분 설치입니다.'
  echo '승인된 모델 압축본은 Drive 로그인/rclone 없이 받습니다. 오프라인은 Downloads/final_weights_bundle.tar.gz 또는 --archive를 사용하세요. 공유 권한·시스템 Python·기존 모델 기본값은 바꾸지 않습니다.'
  echo '--models-only: 모델 파일만 준비, 환경 설치/추론/빌드 없음. --existing-python: 지정 interpreter를 변경하지 않고 검사.'
  echo '--skip-build: 모델/환경만 검사하며 새 runtime 배포 완료로 판정하지 않음. --check: 다운로드/설치/빌드 없이 기존 모델·환경·설치 runtime 일치 검사.'
}
while (($#)); do
  case "$1" in
    --archive|--remote|--models-dir) MODEL_ARGS+=("$1" "${2:?Missing option value}"); shift 2 ;;
    --env-prefix) PREFIX="${2:?Missing prefix}"; shift 2 ;;
    --existing-python) EXISTING_PYTHON="${2:?Missing Python}"; shift 2 ;;
    --device) DEVICE="${2:?Missing device}"; shift 2 ;;
    --models-only) MODELS_ONLY=true; SKIP_BUILD=true; shift ;;
    --skip-build) SKIP_BUILD=true; shift ;;
    --check) CHECK=true; SKIP_BUILD=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

source_runtime() {
  export AJOUNICE_ROOT="$ROOT"
  set +u
  source "$ROOT/scripts/workspace/source_aw149.sh"
  set -u
}

# No roscore, plugin/node startup or runtime parameter change. --check consumes
# the receipt written only after a successful scoped build and install audit.
runtime_audit() {
  /usr/bin/python3 - "$ROOT" "$@" <<'PY'
import hashlib,json,os,struct,subprocess,sys
from pathlib import Path

root=Path(sys.argv[1]).resolve();mode=sys.argv[2]
package='autoware_behavior_velocity_traffic_light_module'
planning=root/'planning_ws';base=root/'base_ws'
cpp=planning/'src'/package
traffic=base/'src/perception/traffic_light_detection'
integration=base/'src/planning_addons/ajounice_planning_integration'
receipt=base/'install_aw149/share/traffic_light_detection/oneclick_runtime_receipt.json'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def files(path):return sorted(p for p in path.rglob('*') if p.is_file())

def elf_identity(payload):
    """Bounded ELF64-LE/x86_64 parser; never rewrites a library on disk."""
    def reject(message):raise RuntimeError('Unsupported/malformed traffic plugin ELF: '+message)
    def span(offset,size):
        if offset<0 or size<0 or offset>len(payload) or size>len(payload)-offset:reject('out-of-bounds payload')
        return payload[offset:offset+size]
    if len(payload)<64:reject('short header')
    header=struct.unpack_from('<16sHHIQQQIHHHHHH',payload)
    ident,kind,machine,version=header[:4]
    if ident[:7]!=b'\x7fELF\x02\x01\x01' or (kind,machine,version)!=(3,62,1):
        reject('requires little-endian ELF64 x86_64 ET_DYN')
    section_offset,entry_size,count,names_index=header[6],header[11],header[12],header[13]
    if header[8]!=64 or entry_size!=64 or not 0<count<0xff00 or not 0<names_index<count:
        reject('unsupported section header layout')
    span(section_offset,count*entry_size)
    sections=[struct.unpack_from('<IIQQQQIIQQ',payload,section_offset+i*entry_size) for i in range(count)]
    names=span(sections[names_index][4],sections[names_index][5])
    def string(table,offset):
        if not 0<=offset<len(table):reject('invalid string offset')
        end=table.find(b'\0',offset)
        if end<0:reject('unterminated string')
        return table[offset:end],end+1
    named={}
    occupied=[]
    for section in sections[1:]:
        name=string(names,section[0])[0].decode('ascii')
        if name in named:reject('duplicate section name')
        if section[1]!=8:
            span(section[4],section[5])
            if section[5]:occupied.append((section[4],section[4]+section[5]))
        named[name]=section
    if any(left[1]>right[0] for left,right in zip(sorted(occupied),sorted(occupied)[1:])):
        reject('overlapping file sections')
    for name in ('.text','.rodata','.dynsym','.dynstr','.dynamic','.note.gnu.build-id'):
        if name not in named:reject('missing '+name)
    dynamic,dynstr,dynsym=(named[name] for name in ('.dynamic','.dynstr','.dynsym'))
    if dynamic[1]!=6 or dynamic[9]!=16 or dynamic[5]%16 or sections[dynamic[6]]!=dynstr or dynstr[1]!=3:
        reject('invalid dynamic/string section linkage')
    if dynsym[1]!=11 or dynsym[9]!=24 or dynsym[5]%24 or sections[dynsym[6]]!=dynstr:
        reject('invalid dynamic symbol linkage')
    table=span(dynstr[4],dynstr[5]);entries=[];terminated=False
    for offset in range(dynamic[4],dynamic[4]+dynamic[5],16):
        tag,value=struct.unpack_from('<qQ',payload,offset)
        if terminated and (tag or value):reject('nonzero data after DT_NULL')
        if tag==0:terminated=True
        else:entries.append((tag,value))
    if not terminated:reject('missing DT_NULL')
    paths=[(tag,value) for tag,value in entries if tag in (15,29)]
    if len(paths)>1:reject('multiple RPATH/RUNPATH entries')
    protected=[]
    for offset in range(dynsym[4],dynsym[4]+dynsym[5],24):
        value=struct.unpack_from('<I',payload,offset)[0]
        protected.append((value,string(table,value)[1]))
    for tag,value in entries:
        if tag in (1,14,0x7ffffffd,0x7fffffff,0x6ffffefa,0x6ffffefb,0x6ffffefc):
            protected.append((value,string(table,value)[1]))
    # GNU version requirements also reference .dynstr; do not mask their names.
    for section in sections:
        if section[1]!=0x6ffffffe:continue
        if sections[section[6]]!=dynstr:reject('invalid version requirement linkage')
        data=span(section[4],section[5]);offset=0;seen=set()
        while offset<len(data):
            if offset in seen or offset+16>len(data):reject('invalid version requirement')
            seen.add(offset)
            _,aux_count,file_name,aux,next_offset=struct.unpack_from('<HHIII',data,offset)
            protected.append((file_name,string(table,file_name)[1]));aux_offset=offset+aux
            for _ in range(aux_count):
                if aux_offset+16>len(data):reject('invalid version auxiliary')
                _,_,_,name,next_aux=struct.unpack_from('<IHHII',data,aux_offset)
                protected.append((name,string(table,name)[1]));aux_offset+=next_aux
            if not next_offset:break
            offset+=next_offset
    note=named['.note.gnu.build-id'];data=span(note[4],note[5]);offset=0;build_ids=[]
    while offset<len(data):
        if offset+12>len(data):reject('short build-id note')
        name_size,value_size,note_type=struct.unpack_from('<III',data,offset);offset+=12
        name=data[offset:offset+name_size];offset+=(name_size+3)&~3
        value=data[offset:offset+value_size];offset+=(value_size+3)&~3
        if offset>len(data):reject('out-of-bounds build-id note')
        if name==b'GNU\0' and note_type==3:build_ids.append(value.hex())
    if len(build_ids)!=1 or not 16<=len(build_ids[0])//2<=64:reject('missing/ambiguous GNU build-id')
    return dict(sections=named,build_id=build_ids[0],dynamic=dynamic,dynstr=dynstr,
                dynamic_entries=entries,rpaths=paths,protected_strings=protected)

def compare_plugin_elf(built_bytes,installed_bytes):
    """Allow only CMake's RPATH removal; protect code AND all remaining bytes."""
    left,right=map(elf_identity,(built_bytes,installed_bytes))
    if left['build_id']!=right['build_id']:raise RuntimeError('Traffic plugin GNU build-id differs')
    if left['sections']!=right['sections'] or len(built_bytes)!=len(installed_bytes):
        raise RuntimeError('Traffic plugin ELF layout differs from the scoped build')
    normalized=[bytearray(built_bytes),bytearray(installed_bytes)]
    if left['rpaths']:
        tag,value=left['rpaths'][0];strings=left['dynstr'];start=strings[4]+value
        end=built_bytes.find(b'\0',start,strings[4]+strings[5])
        if end<0 or end+1!=strings[4]+strings[5]:
            raise RuntimeError('Traffic plugin RPATH is not an isolated trailing .dynstr string')
        if any(a<end+1-strings[4] and b>value for a,b in left['protected_strings']+right['protected_strings']):
            raise RuntimeError('Traffic plugin RPATH overlaps a runtime symbol/dependency string')
        unchanged=right['rpaths']==[(tag,value)] and installed_bytes[start:end+1]==built_bytes[start:end+1]
        removed=not right['rpaths'] or right['rpaths']==[(tag,value)]
        cleared=installed_bytes[start]==0 and all(after in (0,before) for before,after in zip(built_bytes[start:end+1],installed_bytes[start:end+1]))
        if not unchanged and not (removed and cleared):
            raise RuntimeError('Unexpected installed traffic plugin RPATH change')
        for payload in normalized:payload[start:end+1]=b'\0'*(end+1-start)
    elif right['rpaths']:
        raise RuntimeError('Installed traffic plugin unexpectedly adds an RPATH')
    for payload,identity in zip(normalized,(left,right)):
        dynamic=identity['dynamic']
        entries=[entry for entry in identity['dynamic_entries'] if entry[0] not in (15,29)]
        canonical=b''.join(struct.pack('<qQ',*entry) for entry in entries)
        payload[dynamic[4]:dynamic[4]+dynamic[5]]=canonical+b'\0'*(dynamic[5]-len(canonical))
    hashes=[hashlib.sha256(payload).hexdigest() for payload in normalized]
    if hashes[0]!=hashes[1]:
        changed=[name for name,s in left['sections'].items() if s[1]!=8 and
                 normalized[0][s[4]:s[4]+s[5]]!=normalized[1][s[4]:s[4]+s[5]]]
        raise RuntimeError('Traffic plugin differs outside expected RPATH relocation: '+', '.join(changed or ['ELF header/padding']))
    return dict(gnu_build_id=left['build_id'],normalized_elf_sha256=hashes[0],
                normalized_alloc_section_sha256={name:hashlib.sha256(normalized[0][s[4]:s[4]+s[5]]).hexdigest()
                    for name,s in left['sections'].items() if s[2]&2 and s[1]!=8},
                allowed_difference='RPATH/RUNPATH removal only')

sources=[p for p in files(cpp) if p.suffix in ('.cpp','.hpp','.h','.xml','.txt','.yaml')]
sources += files(traffic/'scripts')+files(traffic/'launch')+files(traffic/'config')
sources += files(integration/'launch')+files(integration/'config')
sources += [traffic/'CMakeLists.txt',integration/'CMakeLists.txt',
            root/'scripts/runtime/install_traffic_light_candidate.sh',
            root/'planning_ws/tools/build.sh',root/'scripts/workspace/source_aw149.sh']
sources=[p for p in sources if '__pycache__' not in p.parts and p.suffix != '.pyc']
manifest={str(p.relative_to(root)):digest(p) for p in sorted(set(sources))}
if mode=='fingerprint':
    print(json.dumps(manifest,sort_keys=True));raise SystemExit(0)
if mode not in ('record','check'):raise RuntimeError('Unknown runtime audit mode')
if mode=='record' and json.loads(sys.argv[3]) != manifest:
    raise RuntimeError('Runtime source changed during build; rerun the installer before using the new stack')

prefix=planning/'install_isolated'
library=prefix/'lib'/('lib'+package+'.so')
built=planning/'devel_isolated'/package/'lib'/library.name
if not library.is_file() or not built.is_file():
    raise RuntimeError('Missing installed/scoped-build traffic C++ plugin')
elf=compare_plugin_elf(built.read_bytes(),library.read_bytes())
payload=library.read_bytes()
if not payload.startswith(b'\x7fELF') or any(marker not in payload for marker in (
        b'enable_signal_compliance_guard',b'signal_compliance_front_axle_offset_m',
        b'enable_yellow_passthrough')):
    raise RuntimeError('Traffic C++ plugin lacks the new compliance interface; old binary is not qualified')
linked=subprocess.run(['ldd',str(library)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
if linked.returncode or 'not found' in linked.stdout:
    raise RuntimeError('Traffic C++ plugin has unresolved shared-library dependencies:\n'+linked.stdout)

pairs=[(cpp/'plugins.xml',prefix/'share'/package/'plugins.xml')]
pairs += [(p,prefix/'share'/package/p.relative_to(cpp)) for p in files(cpp/'config')]
for folder in ('launch','config'):
    pairs += [(p,base/'install_aw149/share/traffic_light_detection'/p.relative_to(traffic))
              for p in files(traffic/folder)]
    pairs += [(p,base/'install_aw149/share/ajounice_planning_integration'/p.relative_to(integration))
              for p in files(integration/folder)]
for source in files(traffic/'scripts'):
    if source.suffix=='.py':
        pairs.append((source,base/'install_aw149/lib/traffic_light_detection'/source.name))
for source,target in pairs:
    if not target.is_file():raise RuntimeError('Missing installed runtime asset: '+str(target))
    left,right=source.read_bytes(),target.read_bytes()
    if source.suffix=='.py':
        # catkin_install_python may rewrite ONLY the interpreter shebang.
        left,right=left.split(b'\n',1)[1],right.split(b'\n',1)[1]
    if left!=right:raise RuntimeError('Stale installed runtime asset: '+str(target))

expected={package:prefix/'share'/package,
          'traffic_light_detection':base/'install_aw149/share/traffic_light_detection',
          'planning_scenario_validator':base/'install_aw149/share/planning_scenario_validator',
          'ajounice_planning_integration':base/'install_aw149/share/ajounice_planning_integration'}
for name,path in expected.items():
    found=Path(subprocess.check_output(['rospack','find',name],text=True).strip()).resolve()
    if found!=path.resolve():raise RuntimeError('Wrong runtime overlay for '+name+': '+str(found))
report=dict(schema='traffic_light_oneclick_runtime_v2',source_sha256=manifest,
            built_plugin_sha256=digest(built),plugin_elf_identity=elf,
            installed_plugin_sha256=digest(library),installed_plugin=str(library),
            installed_asset_sha256={str(target.relative_to(root)):digest(target) for _,target in pairs},
            package_paths={name:str(path) for name,path in expected.items()},
            limitations=['Build/install identity only; no nodes, live driving, permission or latency qualification.'])
if mode=='check':
    if not receipt.is_file():raise RuntimeError('No verified runtime install receipt; run the installer without --check')
    previous=json.loads(receipt.read_text())
    for key in ('schema','source_sha256','built_plugin_sha256','installed_plugin_sha256','plugin_elf_identity','installed_asset_sha256','package_paths'):
        if previous.get(key)!=report[key]:raise RuntimeError('Runtime receipt differs from source/install: '+key)
else:
    staged=receipt.with_name(receipt.name+'.tmp.'+str(os.getpid()))
    with staged.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    os.replace(staged,receipt)
print('[RUNTIME VERIFIED] scoped C++ plugin, Python helpers and installed launch/config match this checkout.')
PY
}

check_build_cache() {
  /usr/bin/python3 - "$1" "$2" "$3" <<'PY'
import sys
from pathlib import Path
cache,source,prefix=map(Path,sys.argv[1:])
values={line.split(':',1)[0]:line.split('=',1)[1] for line in cache.read_text().splitlines()
        if not line.startswith(('#','//')) and ':' in line and '=' in line}
for key,expected in (('CMAKE_HOME_DIRECTORY',source),('CMAKE_INSTALL_PREFIX',prefix)):
    if Path(values.get(key,'' )).resolve()!=expected.resolve():
        raise RuntimeError('Refusing foreign/moved CMake cache '+str(cache)+' ('+key+'); existing files were not removed')
PY
}

[[ "$DEVICE" == cpu || "$DEVICE" =~ ^[0-9]+$ ]] || { echo '장치는 cpu 또는 GPU 번호입니다' >&2; exit 2; }
if $CHECK; then MODEL_ARGS+=(--check); fi
/usr/bin/python3 "$ROOT/scripts/runtime/traffic_light_candidate.py" "${MODEL_ARGS[@]}"
if $MODELS_ONLY; then
  echo '[MODELS VERIFIED ONLY] GPU/ROS runtime은 아직 검사하지 않았습니다. 기존 운영 설정은 유지됩니다.'
  exit 0
fi
mapfile -t MODEL_INFO < <(/usr/bin/python3 - "$ROOT" "${MODEL_ARGS[@]}" <<'PY'
import json,sys
sys.path.insert(0,sys.argv[1]+'/scripts/runtime')
from traffic_light_candidate import load_release,paths
args=sys.argv[2:];directory=args[args.index('--models-dir')+1] if '--models-dir' in args else None
doc=load_release();models=paths(directory,doc)
for value in (models['yolo'],models['cnn'],doc['models']['yolo']['sha256'],doc['models']['cnn']['sha256']):print(value)
PY
)
[[ ${#MODEL_INFO[@]} == 4 ]] || { echo '모델 경로 확인 실패' >&2; exit 1; }
CHECK_ARGS=(--device "$DEVICE" --weights "${MODEL_INFO[0]}" --cnn-checkpoint "${MODEL_INFO[1]}" --imgsz 960
  --expected-weights-sha256 "${MODEL_INFO[2]}" --expected-cnn-sha256 "${MODEL_INFO[3]}")
if [[ -n "$EXISTING_PYTHON" ]]; then
  source_runtime
  PYTHONNOUSERSITE=1 "$EXISTING_PYTHON" -s "$ROOT/scripts/runtime/check_traffic_light_env.py" \
    "${CHECK_ARGS[@]}" --check-ros --require-team-versions --isolate-python
else
  ENV_ARGS=(--prefix "$PREFIX")
  if $CHECK; then ENV_ARGS+=(--check); fi
  bash "$ROOT/scripts/runtime/setup_traffic_light_env.sh" "${ENV_ARGS[@]}" "${CHECK_ARGS[@]}"
fi
if ! $SKIP_BUILD; then
  source_runtime
  RUNTIME_INPUTS="$(runtime_audit fingerprint)"
  PLANNING_PACKAGE=autoware_behavior_velocity_traffic_light_module
  PLANNING_BUILD="$ROOT/planning_ws/build_isolated/$PLANNING_PACKAGE"
  if [[ -f "$PLANNING_BUILD/CMakeCache.txt" ]]; then
    check_build_cache "$PLANNING_BUILD/CMakeCache.txt" "$ROOT/planning_ws/src/$PLANNING_PACKAGE" "$ROOT/planning_ws/install_isolated"
  fi
  # Validate every existing cache before touching either workspace, including
  # the mixed cached/missing case that takes the scoped catkin fallback.
  for package in traffic_light_detection planning_scenario_validator ajounice_planning_integration; do
    if [[ "$package" == traffic_light_detection ]]; then SOURCE="$ROOT/base_ws/src/perception/$package"; else SOURCE="$ROOT/base_ws/src/planning_addons/$package"; fi
    if [[ -f "$ROOT/base_ws/build_aw149/$package/CMakeCache.txt" ]]; then
      check_build_cache "$ROOT/base_ws/build_aw149/$package/CMakeCache.txt" "$SOURCE" "$ROOT/base_ws/install_aw149"
    fi
  done
  if [[ -f "$PLANNING_BUILD/CMakeCache.txt" ]]; then
    # Package-local clean avoids trusting old object timestamps/header history.
    # The last installed plugin is left in place unless the new build succeeds.
    cmake --build "$PLANNING_BUILD" --target clean -- -j2
  fi
  bash "$ROOT/planning_ws/tools/build.sh" --pkg "$PLANNING_PACKAGE" --force-cmake --make-args -j2 -l2
  if [[ -f "$ROOT/base_ws/build_aw149/traffic_light_detection/CMakeCache.txt" && -f "$ROOT/base_ws/build_aw149/planning_scenario_validator/CMakeCache.txt" && -f "$ROOT/base_ws/build_aw149/ajounice_planning_integration/CMakeCache.txt" ]]; then
    for package in traffic_light_detection planning_scenario_validator ajounice_planning_integration; do
      cmake --build "$ROOT/base_ws/build_aw149/$package" --target install -- -j2
    done
  else
    catkin_make_isolated --install --source "$ROOT/base_ws/src" --build "$ROOT/base_ws/build_aw149" \
      --devel "$ROOT/base_ws/devel_aw149" --install-space "$ROOT/base_ws/install_aw149" \
      --pkg traffic_light_detection planning_scenario_validator ajounice_planning_integration -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 --make-args -j2 -l2
  fi
  runtime_audit record "$RUNTIME_INPUTS"
elif $CHECK; then
  source_runtime
  runtime_audit check
else
  echo '[MODEL/ENV ONLY] --skip-build 선택: C++/Python/launch 설치를 갱신·검증하지 않았습니다. 새 runtime 배포 완료가 아닙니다.'
  exit 0
fi
echo '[V5 READY] 두 모델과 선택 장치 추론·ROS import 검사 완료. 주행은 시작하지 않았습니다.'
echo '[GUI/traffic] v5 지도+CNN이 기본입니다. 새 실험/새 터미널을 사용하고 기존 실행 사본은 수정하지 마세요.'
