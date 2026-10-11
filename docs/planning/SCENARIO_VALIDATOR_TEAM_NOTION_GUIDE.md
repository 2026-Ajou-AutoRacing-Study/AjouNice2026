# 시나리오 자동 주행 검증기 — 처음 세팅부터 실행까지

**설치가 끝났으면 [실제 작업 순서만 적은 문서](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md)를 보세요.** 예제 복사 → 두 장면 확인 → 시나리오 수정 → 배치 → 중단·재개 → 결과 공유 순서입니다. 버튼 위치는 [빨간 표시를 붙인 화면 5장](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_GUI_HIGHLIGHTS.md)에 있습니다. 이 문서는 첫 설치·업데이트·빠진 환경 준비가 필요할 때 참고합니다.

일단 이거는 시나리오를 전부 성공시키려는 도구가 아니라, **우리 시스템이 어디서 실패하는지 찾고 기록하려고 만든 도구입니다.** 실패해도 괜찮습니다. 실패한 시나리오랑 bag, 영상, 실패 근거를 같이 남기는 게 목적이에요.

처음 PC에 세팅하는 경우만 좀 길고, 세팅 끝나면 MORAI 연결 → GUI에서 내 미션 선택 → 실행으로 갑니다. 시나리오마다 사람이 녹화 켰다 껐다 할 필요는 없습니다.

현재 공유·실행 브랜치는 **planning/autoware-universe-to-ros1**입니다. 시나리오 검증기도 판단·제어·인지·측위와 같은 브랜치에 합칩니다. 팀원은 자동화 개발 브랜치로 따로 이동하지 않고, 이 브랜치를 pull하고 증분 빌드해서 사용하면 됩니다. 아래 내용은 개인 PC에서 MORAI와 알고리즘을 같이 켜는 기준입니다. 현장 LAN 설정과 섞지 말아 주세요.

**이미 Docker에 BEVFusion 이미지를 등록했고 E25 가중치도 준비한 분은 다시 설치하거나 다운로드하지 마세요.** 우리 풀스택도 준비돼 있으면 **1장 브랜치 갱신 → 4장 업데이트 빌드 → 7장 검증기 Network 준비 → 8장 GUI 실행**으로 가면 됩니다. **5·6장의 Docker 설치, 이미지 다운로드·docker load, E25 압축 해제는 건너뜁니다.** 7장의 `install.sh --gui`는 도구 검사와 검증기 Network 생성이므로 기존 준비자도 실행합니다. 신호등도 켤 사람만 7-1의 환경 설치/선택을 추가 확인해 주세요.

기존 환경은 지우지 말고 GUI의 **환경 검사**에서 실제 사용하는 이미지·E25 경로를 확인하세요. `.env.bevfusion.local`에 본인 경로가 있으면 기본 경로보다 우선합니다. GPU 사용 확인이 필요하면 설치기의 GPU 검사 또는 planning_validator_doctor.py --gpu 검사를 사용합니다. Docker만 설치됐거나 학습용 이미지만 있는 경우는 BEVFusion 추론/E25까지 준비됐다는 뜻이 아니니 아래 5장의 구분을 참고하세요. GUI·녹화 도구가 빠졌다면 그 도구만 추가하면 됩니다.

**첫 실행 전에 담당 미션의 `base/README.md`를 한 번 읽어 주세요.** 시작점·목적지는 왜 그곳인지, 어떤 조건을 더 만들어 볼지, 무엇이 아직 미검증인지 적어뒀습니다. [미션별 README 목차](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/base_ws/src/planning_addons/planning_scenario_validator/config/missions/README.md)에서 본인 미션을 고르면 됩니다. YAML만 열고 바로 24개를 돌리기보다는 이 안내를 보고 baseline 두 장면부터 확인해 주세요.

## 0. 시작 전에 이것만 확인

- 풀스택 기준 환경은 Ubuntu 20.04 + ROS Noetic + NVIDIA GPU입니다. 화면 녹화는 X11 기준입니다.
- Ubuntu 22.04에서 BEVFusion Docker가 도는 것과, ROS1 planning·제어까지 바로 도는 것은 별개입니다. 22.04인 분은 기존 Noetic 풀스택 환경이 있는지 먼저 확인해 주세요. 이 문서 때문에 OS부터 바꾸지는 마세요.
- MORAI 대회용 MR이 설치되어 있고 K-City 맵을 실행할 수 있어야 합니다. MR 설치·라이선스는 별도입니다.
- GitHub 저장소와 공유 Google Drive AjouNice2026 폴더 접근 권한이 필요합니다. 없으면 먼저 요청해 주세요.
- Docker 압축이 약 6.9 GiB이고, 로드하면 추가 공간을 씁니다. 첫 빌드·맵·녹화까지 고려해 **가능하면 60 GiB 이상** 비워두고 시작해 주세요. 이 숫자는 준비 여유 권장값이지 전체 설치 용량 실측치는 아닙니다.

```bash
lsb_release -ds
echo "$XDG_SESSION_TYPE"
nvidia-smi
df -h "$HOME"
```

XDG_SESSION_TYPE이 wayland면 로그인 화면에서 Ubuntu on Xorg를 선택해 주세요. nvidia-smi 자체가 안 되면 Docker보다 드라이버부터 해결해야 합니다.

## 1. 저장소 받기 / 이미 있으면 브랜치 옮기기

**저장소가 없는 분은 이것만 실행합니다.** Git이 없으면 먼저 sudo apt install git을 해 주세요.

```bash
cd ~
git clone --branch planning/autoware-universe-to-ros1 \
  https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026.git
cd ~/AjouNice2026
git submodule update --init --recursive
```

GitHub 로그인 오류가 나면 팀 저장소 권한부터 확인해 주세요. HTTPS 인증에는 GitHub 토큰/인증 도구가 필요할 수 있습니다. SSH를 이미 설정한 분은 git@github.com:2026-Ajou-AutoRacing-Study/AjouNice2026.git 주소를 써도 됩니다.

**이미 저장소가 있는 분은 아래 순서입니다.**

```bash
cd ~/AjouNice2026
git status --short
```

여기서 수정 파일이 나오면 일단 멈춰 주세요. 본인 작업을 날리면서 브랜치를 바꿀 필요는 없습니다. 본인 AI와 변경을 보존한 뒤 진행하고, git reset --hard 같은 건 쓰지 마세요.

깨끗한 상태면 이어서 실행합니다.

```bash
git fetch origin
git switch planning/autoware-universe-to-ros1
git pull --ff-only origin planning/autoware-universe-to-ros1
git submodule update --init --recursive
```

switch에서 로컬 브랜치가 없다고 나오면 대신 아래 명령을 한 번 쓰고, 위 pull·submodule 명령부터 이어가면 됩니다.

```bash
git switch --track origin/planning/autoware-universe-to-ros1
```

pull이 갈라진 이력 때문에 거절되면 강제 push/rebase부터 하지 말고 알려 주세요. 개인 YAML·시나리오는 뒤에 나오는 workspace에서 만들면 Git 변경으로 안 뜹니다.

## 2. ROS와 기본 도구 준비

이미 /opt/ros/noetic/setup.bash가 있는 분은 **2-1을 건너뛰고 2-2부터** 하면 됩니다. Conda/ROS2가 켜져 있지 않은 새 터미널에서 해 주세요.

### 2-1. ROS Noetic이 아예 없는 Ubuntu 20.04 PC만

Noetic은 지원 종료된 ROS입니다. 기존 대회 스택을 맞추는 용도로 사용하며, 다른 Ubuntu 버전에 focal 패키지를 억지로 설치하면 안 됩니다. ROS apt 설정은 [공식 ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source)의 focal 패키지를 사용합니다. 아래는 확인한 1.3.0 릴리스 기준입니다.

```bash
sudo apt update
sudo apt install -y curl ca-certificates gnupg
curl -fL \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros-apt-source_1.3.0.focal_all.deb \
  -o /tmp/ajou-ros-apt-source.deb
sudo apt install -y /tmp/ajou-ros-apt-source.deb
sudo apt update
sudo apt install -y ros-noetic-desktop-full python3-rosdep
source /opt/ros/noetic/setup.bash
```

이미 ROS apt 설정이 있는데 키·저장소 중복 오류가 나면 기존 설정을 먼저 확인해야 합니다. 패키지 설치가 실패한 상태에서 다음 빌드로 넘어가지 마세요.

### 2-2. 빌드·GUI·녹화·다운로드 도구

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config python3-dev python3-pip python3-venv \
  python3-rosdep python3-rospkg python3-nose python3-pytest \
  python3-numpy python3-scipy python3-opencv python3-yaml \
  python3-tk python3-gi gir1.2-wnck-3.0 \
  ffmpeg x11-utils rclone zstd \
  libeigen3-dev libpcl-dev libgeographic-dev libpugixml-dev \
  libyaml-cpp-dev libfmt-dev libboost-all-dev \
  ros-noetic-jsk-recognition-msgs ros-noetic-robot-localization

source /opt/ros/noetic/setup.bash
```

rosdep 초기화는 PC당 한 번입니다. 이미 initialized라고 나오면 다시 지우거나 초기화하지 않고 update만 합니다.

```bash
sudo rosdep init
rosdep update --include-eol-distros
cd ~/AjouNice2026
rosdep install --from-paths planning_ws/src base_ws/src \
  --ignore-src --rosdistro noetic -y \
  --skip-keys 'osqp proxsuite acados rospkg motion_velocity_planner mission_planner behavior_path_planner behavior_velocity_planner'
```

skip 목록은 아무 의존성이나 무시하라는 뜻은 아닙니다. osqp/proxsuite/acados는 저장소 빌드 도구가 따로 준비하고, rospkg는 위 python3-rospkg로 설치합니다. 마지막 네 개는 구 bringup에 남은 이름이며 현재 AW149의 autoware_* 패키지를 사용합니다. 이 목록 외의 unresolved dependency가 나오면 오류를 공유해 주세요. 이 명령은 현재 PC에서 설치 없이 simulate해 의존성 이름 처리를 확인했습니다. 완전히 빈 팀원 PC의 빌드까지 검증한 것은 아닙니다.

신호등은 이번 미션 예제에서 기본 off입니다. 켜서 시험할 사람은 아래 7-1의 공용 가상환경도 설치해 주세요. BEVFusion은 Docker, 신호등은 호스트 Python으로 별개로 돌아갑니다. BEVFusion만 쓰려고 호스트에 PyTorch나 CUDA Toolkit을 설치할 필요는 없습니다.

## 3. rclone 로그인하고 맵 받기

공유 Drive 경로를 터미널에서 받으려고 rclone을 사용합니다. **본인 Google 계정으로 로그인**하면 됩니다. 제 설정 파일이나 토큰을 복사해서 쓰는 게 아닙니다.

```bash
rclone config
```

새 remote를 만들 때:

1. n으로 새 remote를 만듭니다.
2. 이름은 ajou_drive로 해 주세요.
3. Storage는 Google Drive를 선택합니다. 번호는 rclone 버전마다 다르니 이름을 보고 고르면 됩니다.
4. client_id/client_secret은 따로 발급한 게 없다면 비워둡니다.
5. 다운로드와 결과 업로드를 둘 다 하려면 drive 전체 접근 scope를 선택합니다.
6. 고급 설정은 기본값으로 두고, 브라우저 인증을 선택해 공유 폴더에 접근 가능한 본인 계정으로 로그인합니다.
7. AjouNice2026이 일반 공유 폴더라면 Shared Drive로 따로 지정하지 않습니다. 공유받은 폴더와 Google Workspace의 Shared Drive는 다른 겁니다.

[rclone Google Drive 설정 안내](https://rclone.org/drive/)도 참고하면 됩니다. 기존 ajou_drive가 있으면 새로 만들지 말고 아래 조회부터 해 주세요.

```bash
rclone lsf ajou_drive: \
  --drive-root-folder-id 12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP
```

kcity_map, model 등이 보이면 공유 폴더 접근이 됩니다. 목록이 안 보이거나 권한 오류가 나면 로그인 계정과 공유 권한부터 확인합니다.

맵은 아래 스크립트로 받으면 됩니다.

```bash
cd ~/AjouNice2026
bash scripts/map/download_kcity_map_assets.sh
/usr/bin/python3 scripts/map/check_runtime_map_version.py
```

현재 브랜치의 기준 맵은 lanelet2_map_v85.osm입니다. 다운로드 스크립트는 맵을 받은 뒤 원격과 비교하고, 위 검사는 런타임 참조와 맵 해시를 확인합니다. 다운로드 도중 껐으면 같은 명령으로 다시 받으면 됩니다.

기존 파일이 원격과 다르면 immutable 오류가 날 수 있습니다. 기존 맵을 무조건 덮어쓰거나 폴더 전체를 지우지 말고, 충돌한 파일을 백업한 뒤 다시 받아 주세요.

## 4. 우리 workspace 빌드

여기는 Docker 빌드가 아니라 **ROS planning·제어·측위·메시지 빌드**입니다. 다른 PC의 build/devel/install 폴더를 가져오는 게 아니라 본인 PC에서 빌드해야 합니다.

처음 clone한 PC는:

```bash
cd ~/AjouNice2026
bash scripts/workspace/setup_ws.sh
```

이미 우리 풀스택을 빌드했던 PC는 아래로 업데이트합니다. setup_ws.sh는 base 빌드 결과를 clean하는 초기 세팅 명령이라 매번 쓰지 않습니다. **planning과 base 모두 증분 빌드를 호출**해야 판단·제어와 검증기 업데이트가 함께 적용됩니다.

```bash
cd ~/AjouNice2026
bash planning_ws/tools/build.sh --providers
bash scripts/workspace/build_all.sh
```

build_all.sh만 실행하면 완성된 planning 설치를 재사용해서 **새 planning 소스가 빌드되지 않을 수 있습니다.** 위 두 명령을 순서대로 실행하세요. 첫 명령이 실패했다면 두 번째로 넘어가지 않습니다.

매번 전체를 처음부터 컴파일하는 명령이 아닙니다. 기존 build/devel/install과 CMake·Make 결과를 유지하며 변경된 파일과 영향받는 의존 대상을 다시 컴파일합니다. 다만 패키지 순회·설정 확인·install 복사는 수행하므로 변경 없는 패키지도 로그에 나올 수 있습니다. 공용 header·메시지·빌드 설정이 바뀌면 관련 패키지까지 빌드될 수 있습니다. Python·launch·config도 install 경로에 반영해야 하므로 업데이트 빌드를 생략하지 않습니다. Docker 이미지나 모델 가중치를 다시 만드는 과정은 아닙니다.

빌드가 끝나면 확인합니다.

```bash
cd ~/AjouNice2026
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh
rospack find ajounice_planning_integration
rospack find planning_scenario_validator
rospack find morai_msgs
```

앞의 두 패키지가 base_ws/install_aw149 쪽에서 찾아져야 합니다. source 실패를 예전 base_ws/devel로 우회하지 마세요. 빌드 마지막 오류가 나면 마지막 30줄 정도 공유해 주세요.

### 4-1. 다음부터는 pull 후 증분 빌드

작업 시작 전이나 팀원이 판단·제어·인지·측위·검증기를 업데이트했다고 알려주면 갱신하세요. 실행 중인 캠페인은 GUI로 중단해서 녹화를 마감하고, GUI와 수동 풀스택 노드도 종료한 뒤 **Conda가 꺼진 새 터미널**에서 진행합니다. 실행 중 pull/build하면 현재 프로세스와 새 파일이 섞일 수 있습니다.

아래는 중간 명령이 실패하면 멈춥니다. 개인 작업이 있으면 자동 stash/reset/rebase하지 않고 보존하라는 메시지를 냅니다.

```bash
(
  set -e
  cd ~/AjouNice2026
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "본인 Git 변경이 있습니다. 작업을 보존한 뒤 다시 진행하세요."
    exit 1
  fi
  git switch planning/autoware-universe-to-ros1
  git pull --ff-only origin planning/autoware-universe-to-ros1
  git submodule update --init --recursive
  bash planning_ws/tools/build.sh --providers
  bash scripts/workspace/build_all.sh
)
```

성공한 뒤 새 터미널에서 `source scripts/workspace/source_aw149.sh`로 환경을 읽고 GUI를 다시 켭니다. **빌드는 이미 실행 중인 노드를 바꾸지 않으므로 재실행이 필요합니다.** 본인 workspace YAML·JSON과 녹화 결과는 Git 제외라 pull로 자동 공유되거나 지워지지 않습니다. 과거 실행 결과는 보존하되 코드 업데이트 전후 비교는 `새 실험 시작`으로 분리하세요. 입력 해시가 같다고 실행 코드까지 동일하다는 뜻은 아닙니다.

평상시에는 setup_ws.sh나 --clean을 쓰지 않습니다. ff-only가 거절되면 로컬 커밋을 강제로 지우지 말고 본인 AI와 이력을 확인하세요. 빌드 오류 때도 build/install을 먼저 통째로 삭제하지 않습니다.

## 5. BEVFusion Docker 준비

**GT 객체로 먼저 돌려볼 분은 이 장과 6장을 건너뛰고, planning 빠른 테스트 탭에서 od_gt로 대조하거나 개인 YAML의 stack.detector를 od_gt로 저장하면 됩니다.** 실제 BEVFusion 시험은 Docker 이미지와 E25 가중치가 둘 다 필요합니다.

**이미 이미지 등록·E25 준비가 끝났으면 이 장의 설치 명령을 반복하지 말고 7장으로 넘어가세요.** 준비 상태에 따라 필요한 부분만 진행합니다.

| 현재 준비 상태 | 할 일 |
| --- | --- |
| Docker·Compose·GPU 사용·BEVFusion 추론 이미지·E25가 준비됨 | 5·6장 건너뛰고 7장. GUI 환경 검사에서 실제 이미지/가중치 경로 확인 |
| Docker는 되지만 이미지나 E25가 없음 | Docker 재설치 없이 5-0의 `--assets-only` 또는 6장에서 없는 자료만 준비 |
| Docker/Compose/GPU Toolkit 준비가 안 됨 | 5-0 자동 준비 또는 아래 수동 안내. 기존 설치는 먼저 확인 |

### 5-0. 처음 준비하거나 빠진 항목이 있을 때만

Ubuntu 20.04 팀 PC에서 Docker·추론 준비가 아직 안 됐을 때 사용하세요. **이미 준비된 분은 실행할 필요 없습니다.** 앞의 ROS·빌드·맵 준비와 rclone 본인 로그인이 끝난 상태 기준입니다.

```bash
cd ~/AjouNice2026
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --bevfusion
```

이 명령이 필요한 다운로드·GUI 도구, Docker Engine/Compose, NVIDIA Container Toolkit을 준비하고, 공유 Drive의 공용 이미지와 E25를 받아 등록합니다. 이미지 압축은 체크섬 확인 후 바로 docker load로 흘려 보내서 큰 tar 사본을 만들지 않습니다. 가중치는 압축에서 E25 파일만 꺼내 SHA256을 확인하고 기본 경로에 넣습니다. 마지막에 기존 verify_team_image.sh로 실제 모듈 import·GPU 연산까지 검사합니다.

sudo 비밀번호는 본인 PC 비밀번호를 입력합니다. rclone 로그인 안 했으면 설치 중 config 화면을 열어 안내하니 본인 Google 계정으로 인증하면 됩니다. 인증 토큰을 Git이나 팀 채팅에 올리지 마세요.

기존 Docker와 정상 이미지·가중치는 재사용합니다. 기존 가중치가 다른 파일이면 덮어쓰지 않고 멈춥니다. 실행 중 컨테이너가 있으면 GPU 설정을 위해 Docker를 재시작하지 않습니다. docker.io 같은 충돌 패키지를 지우거나 NVIDIA 드라이버를 업그레이드하지도 않습니다. 이런 경우만 아래 수동 안내를 보거나 오류를 보내 주세요.

처음 Docker 그룹에 계정이 추가되면 이미지 준비는 sudo로 마무리하지만, **GUI 사용 전 로그아웃/로그인이 한 번 필요합니다.** 재로그인 후 docker ps가 sudo 없이 되면 됩니다. docker 그룹은 관리자 수준 권한입니다.

설치 전에 실행 계획만 보고 싶으면:

```bash
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --bevfusion --dry-run
```

설치 완료 후에는 **아래 5-1·5-2·6의 수동 명령을 반복하지 않고 7의 MORAI 연결로 넘어가세요.** ROS 설치·workspace 빌드·맵 다운로드·신호등 Python 환경까지 한꺼번에 설치하는 명령은 아닙니다. Ubuntu 20.04는 현재 Docker 공식 지원 목록에서 빠져 있어 focal 저장소의 설치 가능한 패키지를 확인하고, 후보가 없으면 다른 OS 패키지를 섞지 않고 중단합니다.

이미 Docker와 GPU 사용이 되고 **이미지나 E25만 빠졌다면**, 시스템 설정을 건드리지 않는 아래 명령을 사용합니다. 있는 이미지와 정상 E25는 재사용하고 없는 자료만 준비합니다. 둘 다 이미 있으면 이 명령도 생략하세요. Drive remote가 다른 경우 --remote 이름으로 바꿀 수 있습니다.

```bash
bash scripts/runtime/setup_bevfusion_inference.sh --assets-only
# 선택: --remote 내remote --download-dir ~/bevfusion_downloads
```

기본 다운로드는 ~/bevfusion_downloads, 기본 E25는 ~/bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth입니다. .env.bevfusion.local은 자동 수정하지 않으니 이전 경로를 지정했던 분은 GUI 환경 검사로 확인해 주세요. 다운로드 완료 압축은 보존·재사용하고, 중단된 .part는 다음에 다시 시도합니다. 중간 파일의 바이트 단위 이어받기를 보장하는 것은 아닙니다.

이번 로컬 검증은 기존 Docker가 있는 5060 Ti에서 실제 Drive E25 다운로드·압축 검증·설치·GPU 검사까지 완료한 것입니다. 빈 팀원 PC의 apt 설치와 다른 GPU 실기 설치까지 검증한 뜻은 아닙니다.

### 5-1. Docker가 이미 있으면 먼저 확인

```bash
docker version
docker compose version
nvidia-smi
```

여기서 쓰는 건 띄어쓰기 있는 docker compose입니다. 구버전 docker-compose만 있다고 준비된 건 아닙니다.

Docker가 없다면 [Docker 공식 Ubuntu 설치 안내](https://docs.docker.com/engine/install/ubuntu/)의 apt 방식으로 Engine과 Compose plugin을 설치합니다. 기존 Docker가 잘 되면 재설치하지 마세요. 현재 최신 지원 목록에는 Ubuntu 20.04가 빠져 있지만 [focal 배포 저장소](https://download.docker.com/linux/ubuntu/dists/focal/)는 남아 있습니다. 아래는 focal에서 제공되는 패키지를 설치하는 경로이며, 최신 Docker의 공식 지원을 보장한다는 뜻은 아닙니다. Candidate가 없으면 다른 OS 패키지를 섞지 말고 알려 주세요.

**Docker가 없는 Ubuntu 20.04 PC에서만 실행합니다.** 기존 docker.io/containerd 등이 있으면 먼저 공식 안내의 충돌 패키지를 확인해야 합니다.

```bash
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu focal stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list
sudo apt update
apt-cache policy docker-ce docker-compose-plugin
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

마지막 명령 후에는 **로그아웃했다가 로그인**해 주세요. GUI 실행기가 sudo 없이 Docker를 써야 합니다. docker 그룹은 관리자 수준 권한을 주므로 팀 본인 계정에만 추가합니다. 다시 로그인한 뒤 docker ps와 docker compose version이 sudo 없이 되는지 봅니다.

### 5-2. Docker에서 GPU 사용 가능하게 하기

호스트 nvidia-smi는 되는데 Docker에서 GPU가 안 잡히면 NVIDIA Container Toolkit이 필요합니다. 이미 설정된 PC는 아래 설치를 반복하지 않고 다음 장의 이미지 검사로 넘어가면 됩니다.

```bash
sudo apt install -y curl gnupg ca-certificates
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Docker 재시작은 실행 중 컨테이너에 영향을 줍니다. 학습·추론을 돌리는 도중에는 하지 마세요. 설정 기준은 [NVIDIA 공식 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)입니다.

## 6. Drive에서 공용 이미지 + E25 받기

**이미 BEVFusion 이미지 등록과 E25 준비를 마쳤으면 이 장 전체를 건너뛰고 7장으로 가세요.** 다시 다운로드·docker load·압축 해제할 필요 없습니다. 이미지나 가중치 한쪽만 없다면 5-0의 `--assets-only`가 없는 자료만 준비하므로 그 방법을 권장합니다. 아래 수동 명령은 둘 다 없는 경우의 순서입니다.

AjouNice2026/model/bevfusion/는 **Google Drive 공유 폴더 안의 경로**입니다. http://ajounice2026/...를 인터넷 다운로드 주소로 쓰는 건 아닙니다.

2026-10-02에 실제 업로드된 파일명을 확인했습니다. 아래 **두 압축만** 받으면 검증용 추론 준비가 됩니다.

- bevfusion_team_v1_20260916_050457/ajounice-bevfusion-team-cu128-v1.image.tar.zst — 약 6.9 GiB, Docker 이미지
- base3_bf0_f40_e25_production.tar.zst — 약 141 MiB, E25 추론 가중치

이미지 압축이 E25 모델까지 포함하는 건 아닙니다. 서로 따로예요. base3-static-10k나 학습용 checkpoints 전체 묶음은 이번 검증기에 필요 없습니다. W&B 로그인도 필요 없습니다.

```bash
mkdir -p "$HOME/bevfusion_downloads" "$HOME/bevfusion_artifacts/releases" \
  "$HOME/bevfusion_artifacts/checkpoints"

rclone copyto \
  ajou_drive:model/bevfusion/bevfusion_team_v1_20260916_050457/ajounice-bevfusion-team-cu128-v1.image.tar.zst \
  "$HOME/bevfusion_downloads/ajounice-bevfusion-team-cu128-v1.image.tar.zst" \
  --drive-root-folder-id 12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP --progress

rclone copyto \
  ajou_drive:model/bevfusion/base3_bf0_f40_e25_production.tar.zst \
  "$HOME/bevfusion_downloads/base3_bf0_f40_e25_production.tar.zst" \
  --drive-root-folder-id 12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP --progress

zstd -t "$HOME/bevfusion_downloads/ajounice-bevfusion-team-cu128-v1.image.tar.zst"
zstd -t "$HOME/bevfusion_downloads/base3_bf0_f40_e25_production.tar.zst"

# 공유 묶음의 SHA256SUMS에서 확인한 이미지 해시입니다. OK여야 합니다.
printf '%s  %s\n' \
  c52bf20f9edcd2bfe6221fb487519a964b51b18e6bccee7bf8c301c3245e2df7 \
  "$HOME/bevfusion_downloads/ajounice-bevfusion-team-cu128-v1.image.tar.zst" | sha256sum -c -
```

Drive를 브라우저로 열어서 직접 받아도 됩니다. 이 파일명으로 ~/bevfusion_downloads에 놓으면 이후 과정은 같습니다.

Docker 이미지 로드와 가중치 압축 풀기:

```bash
zstd -dc "$HOME/bevfusion_downloads/ajounice-bevfusion-team-cu128-v1.image.tar.zst" | docker load
tar --use-compress-program=unzstd -xf \
  "$HOME/bevfusion_downloads/base3_bf0_f40_e25_production.tar.zst" \
  -C "$HOME/bevfusion_artifacts/releases"

# 압축 안의 E25 파일을 찾아 기본 checkpoint 경로에 놓습니다.
mapfile -t E25_FILES < <(find "$HOME/bevfusion_artifacts/releases" -type f \
  -name base3_bf0_f40_e25_inference.pth)
if [ "${#E25_FILES[@]}" -eq 1 ]; then
  cp -n "${E25_FILES[0]}" "$HOME/bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth"
else
  echo "E25 파일이 정확히 하나가 아닙니다. releases 폴더를 확인해 주세요."
fi

sha256sum "$HOME/bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth"
```

마지막 해시는 아래와 같아야 합니다. 다른 값이면 GUI에서 실행하지 말고 먼저 확인해 주세요. 기존 파일을 cp -n으로 보호하므로, 옛날 파일이 같은 이름으로 있던 경우에도 해시를 확인해야 합니다.

```text
36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011
```

이제 실제 본인 GPU에서 이미지 검사를 합니다. MORAI·학습은 잠깐 꺼두고 검사하는 게 좋습니다.

```bash
cd ~/AjouNice2026
BEVFUSION_IMAGE=ajounice/bevfusion:team-cu128-v1 \
  bash docker/bevfusion/scripts/verify_team_image.sh
```

마지막에 PASS가 나오면 이미지 모듈·GPU 사용 검사는 통과입니다. 이것만으로 MORAI 풀스택 성능까지 확인한 건 아닙니다. 2080 Ti/3090/4090/5060 Ti용 연산 코드를 포함한 이미지지만, 제가 실측한 GPU는 5060 Ti라 각자 이 검사부터 해 주세요. GTX 1060은 이번 공용 이미지 대상이 아닙니다.

CUDA/driver 오류가 나면 에러와 nvidia-smi 결과를 같이 보내 주세요. 이미지 안은 CUDA 12.8이므로 호스트 드라이버 호환을 확인해야 합니다. 호스트 CUDA Toolkit부터 아무 버전이나 설치하는 것으로 해결되는 건 아닙니다.

기본값은 방금 둔 checkpoint 경로와 공용 이미지입니다. **이미 .env.bevfusion.local을 쓰던 PC는 그 파일의 BEVFUSION_IMAGE와 BEVFUSION_CHECKPOINT도 확인해 주세요.** 파일 설정이 기본값을 덮어쓸 수 있습니다. 지금은 새 .env를 만들 필요 없고, 이 문서에서 학습용 설정 파일을 덮어쓰지도 않습니다.

## 7. 검증기 도구 확인 + MORAI 연결

```bash
cd ~/AjouNice2026
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --gui
```

기본 install.sh와 --gui는 도구 확인·validator용 Ego Network 생성을 합니다. **--bevfusion을 선택했을 때만 Docker·Drive 이미지·E25 준비를 추가합니다.** ROS 설치 / 빌드 / 맵 다운로드는 여전히 앞에서 해야 합니다. --gui를 붙이면 tkinter가 없는 것도 오류로 알려줍니다. 자동 준비에 성공했으면 여기서는 JSON Load·Connect만 이어가면 됩니다.

MR을 켜서 K-City 맵을 연 뒤 다음 두 파일을 Load하고 Connect합니다.

| MORAI에서 로드할 항목 | 파일 |
| --- | --- |
| Sensor | ~/AjouNice2026/configs/sensors/Sensor_competition_local.json |
| Ego Network | ~/AjouNice2026/base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json |

일반 ego_network_competition_local.json 대신 **validator가 붙은 파일**을 써야 합니다. 자동 ScenarioLoad용 UDP 9095가 있어야 시나리오를 알아서 바꿉니다. 포트 번호는 임의로 바꾸지 말아 주세요.

기본 MR 설치 위치가 ~/MoraiLauncher_Lin이면 install.sh가 Ego Network를 MR SaveFile/Network/25.S4.MolitComp03에도 복사합니다. Sensor 파일은 직접 위 경로에서 고르면 됩니다. MR에서 저장된 Sensor 목록으로만 고를 수 있다면 아래로 복사합니다. 기존 같은 이름의 파일은 번호를 붙여 백업합니다.

```bash
cd ~/AjouNice2026
MR_SENSOR_DIR="$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03"
if [ -d "$MR_SENSOR_DIR" ]; then
  cp --backup=numbered configs/sensors/Sensor_competition_local.json "$MR_SENSOR_DIR/"
else
  echo "기본 MR Sensor 폴더가 없습니다. 본인 MR의 SaveFile/Sensor 위치를 먼저 확인해 주세요."
fi
```

복사 후 MR에서 Sensor_competition_local을 다시 Load합니다. 파일 선택창에서 ~가 안 풀리면 실제 홈 경로로 찾아가 주세요. /home/luv를 넣는 게 아닙니다.

MR이 다른 폴더에 설치되어 있으면 두 JSON을 직접 골라 Load하고, GUI 고급 설정의 MORAI scenario 폴더도 본인의 SaveFile/Scenario/R_KR_PR_K-city_2025로 지정해 주세요. 여러 MR/MRP 설치가 섞이면 auto 탐색이 실패할 수 있습니다.

네트워크 최초 Connect는 사람이 한 번 합니다. 이후 시나리오 로드와 주행은 검증기가 합니다. 센서는 켜둬야 합니다. **LiDAR·카메라 녹화 제외는 bag 용량 옵션이지 센서를 끄는 옵션이 아닙니다.**

## 7-1. 신호등도 켜려면 — 팀원은 공용 가상환경 추천

팀장님처럼 시스템 Python에서 이미 잘 돌아가면 그대로 쓰면 됩니다. 새로 세팅하는 팀원은 아래로 별도 환경을 만드는 쪽으로 가 주세요. 시스템 Python이나 기존 Conda 환경을 덮어쓰지 않습니다. 신호등 모델 best.pt는 Git에 있어서 별도로 받을 필요 없습니다. BEVFusion E25 가중치와는 다른 모델입니다.

ROS 빌드까지 끝낸 다음 새 터미널에서 최초 한 번 실행합니다.

```bash
cd ~/AjouNice2026
nvidia-smi
bash scripts/runtime/setup_traffic_light_env.sh
```

Conda가 있으면 그 실행 파일을 쓰고, 없으면 홈 폴더 아래 Miniforge를 설치합니다. conda init이나 bashrc 변경은 하지 않습니다. 패키지 다운로드에 인터넷이 필요하고, 제 PC에서는 새 환경 자체가 약 7.2 GiB를 사용했습니다. 다운로드 캐시·Miniforge 공간도 별도라 신호등 설치용으로 15 GiB 정도 여유를 권장합니다. nvidia-smi가 안 되면 NVIDIA 드라이버부터 해결해야 합니다. GPU 드라이버는 이 스크립트가 설치하지 않습니다.

설치 완료 후 아래 환경이 생깁니다.

- Python 경로: ~/.local/share/ajounice/traffic_light_gpu/bin/python
- Python 3.11 / torch 2.7.1+cu128 / torchvision 0.22.1+cu128
- ultralytics 8.4.143 / numpy 1.26.4 / opencv-python 4.8.1.78

2080 Ti, 3090, 4090, 5060 Ti 모두 같은 설치 명령을 사용합니다. 다만 드라이버와 실제 GPU에서의 작동은 각 PC에서 마지막 검사를 통과해야 합니다. **현재 실제 설치·GPU 추론 검증은 제 5060 Ti에서 완료했고 나머지 GPU 실기 검증은 아직입니다.** 설치 마지막에 GPU 연산, 신호등 가중치 추론, ROS 메시지 import를 검사해서 PASS인지 보여줍니다. CUDA Toolkit을 호스트에 따로 깔 필요는 없습니다. 버전 조합은 [PyTorch 공식 설치 안내](https://pytorch.org/get-started/previous-versions/)의 2.7.1 CUDA 12.8 조합을 사용합니다. Conda가 없는 경우 설치하는 도구는 [Miniforge](https://github.com/conda-forge/miniforge)입니다.

신호등 on/off는 GUI 실행 탭에서 바꾸고 YAML 저장을 누르면 됩니다. 실행 환경·Python 경로·장치는 개인 YAML의 stack에서 설정하고 VSCode에서 저장합니다. GPU가 하나면 장치는 '0', CPU 시험만 cpu입니다. GUI의 설정 확인 탭은 읽기 전용이고, planning 빠른 테스트 탭에서는 저장된 환경을 그대로 사용해 on/off 대조만 합니다. 기존 실험은 당시 환경의 사본으로 재개할 수 있습니다.

매번 conda activate를 할 필요 없습니다. 검증기가 선택한 Python을 직접 실행합니다. GUI에서 몰래 설치하거나 sudo를 실행하지는 않습니다.

나중에 설치 없이 환경만 다시 검사하려면:

```bash
cd ~/AjouNice2026
bash scripts/runtime/setup_traffic_light_env.sh --check
```

기존 시스템 Python을 유지할 사람은 YAML에서 traffic_light_runtime: system을 사용합니다. /usr/bin/python3를 사용하며 이미 설치된 사용자 패키지를 사용하는 기존 방식도 유지합니다. 공용 환경 버전을 시스템 Python에 억지로 설치하지 마세요. Ubuntu 20.04의 시스템 Python 3.8에 위 Python 3.11 조합을 그대로 넣는 방식이 아닙니다. 기존 시스템 환경 검사는 아래처럼 합니다. 실패하면 시스템에 설치를 밀어넣지 말고 공용 가상환경을 선택하는 게 편합니다.

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
/usr/bin/python3 scripts/runtime/check_traffic_light_env.py --device 0 --check-ros
```

이미 본인 신호등 Conda 환경이 있으면 YAML의 traffic_light_python에 그 환경의 bin/python 경로를 적으면 됩니다. 기존 환경을 수정하는 설치 명령을 실행할 필요는 없습니다. 가상환경 선택에서는 사용자 site-packages가 섞이지 않도록 격리합니다. 지도 매칭은 기존처럼 별도 시스템 Python 프로세스에 남아 있고, 신호등 모델 추론만 선택한 Python으로 돌아갑니다. 판단 threshold·스무딩·신호 매칭 기준은 실행 환경 선택으로 바뀌지 않습니다.

YAML의 stack 아래에 다음 값을 넣습니다. Python 환경·경로·장치는 GUI 저장 대상이 아니며 VSCode에서 수정합니다. 신호등 on/off만 기본 탭에서 수정·저장할 수 있습니다. 아래 경로의 ~는 각자 홈 폴더로 해석되니 /home/luv로 바꿀 필요 없습니다.

```yaml
stack:
  # 나머지 기존 stack 설정은 유지
  traffic_light: true
  traffic_light_runtime: virtualenv  # system이면 /usr/bin/python3 사용
  traffic_light_python: ~/.local/share/ajounice/traffic_light_gpu/bin/python
  traffic_light_device: '0'  # CPU 시험만 cpu
```

신호등 on 상태로 실행하면 차량을 움직이기 전에 실제 환경 검사를 먼저 합니다. 실패하면 stack_logs/traffic_light_preflight.log와 GUI 로그를 확인하세요. 설정 검사 버튼은 경로·YAML 검사이고, 실제 GPU/모델 검사는 실행 버튼에서 합니다. off인 예제는 이 환경이 없어도 실행할 수 있습니다. 기본 예제는 계속 off로 유지합니다.

## 8. 이제 실제 실행 — 터미널은 어디까지 켜나

기존 수동 풀스택 터미널은 꺼주세요. udp, localization, bevfusion, planning, traj_fol_a, planviz를 따로 켜놓고 검증기도 실행하면 중복 발행으로 막힙니다. 검증기가 필요한 스택을 알아서 켭니다. roscore도 없으면 자체 준비하므로 따로 rc부터 입력할 필요 없습니다. 기존 roscore만 켜져 있는 것은 사용할 수 있습니다.

MORAI는 켜둔 채로, 새 터미널 하나에서:

```bash
cd ~/AjouNice2026
./planning_validator_gui
```

alias를 쓰고 싶으면 대신 아래로 실행해도 똑같습니다.

```bash
cd ~/AjouNice2026
source docs/common_alias.txt
pvgui
```

### 처음엔 내 미션의 두 장면만

**기본 탭에서 일상 옵션 수정 → 상단 왼쪽 YAML 저장 → 오른쪽 보조 기능의 설정 검사 → 새 실험 시작입니다.** 왼쪽 세 버튼은 저장·불러오기/새로고침·시작이며 기록·결과 폴더도 보조 기능에 있습니다. 목적지·시간·네트워크·Python 환경 같은 회색 값은 확인용입니다. 새 주석 예제는 위쪽에 수정할 값만 모으고 ‘여기부터 수정 X’ 아래에 고정 값을 둡니다.

고주로 소수 장면 반복은 highway/base/random_example.yaml을 복사합니다. JSON 2개 × repeat 10 = 20회이며 NPC 목표 50–60km/h·path offset −10–10%를 추출합니다. 공식 범위가 아니라 첫 시험용 값입니다. repeat/seed/범위는 YAML 위쪽에서 바꿉니다. 실제 로드 입력과 추출값은 회차마다 별도 저장됩니다.

**example.yaml을 골랐다고 바로 실행하는 게 아닙니다. 공유 원본이니까 반드시 베이스 → 내 작업 폴더 복사부터 눌러 주세요.** 복사하고 나서 열린 YAML 경로가 missions/내미션/workspace/ 안인지 확인합니다. 이후 설정·실행·배치 추가는 그 개인 복사본으로 합니다.

1. 예제 목록에서 missions/내미션/base/example.yaml을 고릅니다.
   **실행 전에 YAML 옆 README.md를 한 번 읽어 주세요.** 시작점·목적지·왜 이렇게 배치했는지·더 만들어볼 조건이 있습니다. 끝의 미션별 AI 프롬프트를 본인 AI에게 전달해도 됩니다. 처음에는 좌표를 바꿀 필요 없습니다.
2. **베이스 → 내 작업 폴더 복사**를 누릅니다. 공유 base는 그대로 두고 내 workspace에 시나리오·YAML이 생깁니다. YAML은 highway_starter.yaml처럼 날짜 없는 이름이고, 다시 복사하면 기존 개인 YAML을 열어 수정값을 유지합니다.
3. 기본 탭의 파일 패턴을 *_baseline_*.json으로 바꾸고 JSON당 반복 횟수를 1로 둡니다. **장면 선택**에서 검색·일괄 체크로 일부 JSON만 골라도 됩니다. 패턴을 직접 바꾸면 기존 개별 선택은 해제됩니다. 처음부터 24개 밤새 돌리지 말고 연결·주행·배우 동작부터 봅니다.
4. 실험 이름은 본인과 미션이 구분되게 적어 주세요. 예: yj_highway_baseline_01. 날짜는 프로그램이 결과 폴더에 붙입니다. 팀원끼리 공유할 때는 이름에 본인 이니셜을 넣는 게 좋습니다.
5. 기본 탭에서 측위 localization / 탐지기 bevfusion / 신호등 off를 확인합니다. 필요하면 이 선택들은 GUI에서 변경 가능합니다. 목적지·시간·네트워크 등 회색 고정값은 기본을 유지합니다. JSON 1–2개 빠른 대조는 planning 빠른 테스트 탭에서도 됩니다.
6. basic 토픽 / LiDAR·카메라 녹화 제외 / 영상 켜기 / split 배치로 둡니다. Drive 업로드는 우선 꺼둡니다.
7. **YAML 저장 → 설정 검사 → 새 실험 시작**입니다. 같은 개인 파일에 저장하고 고정값·주석은 유지합니다. 검사·새 실행 전에도 GUI 변경분은 자동 저장합니다. 회색/고급 설정은 VSCode 수정·저장 → YAML 경로 복사 → GUI 상단 Ctrl+V → Enter/불러오기입니다. 환경 검사는 planning 빠른 테스트 탭에 있습니다. GPU 검사는 설치 마지막 검사 또는 doctor CLI에서 할 수 있습니다. FAIL과 WARN 내용을 확인하세요.

Git에 안 올라가는 건 workspace 안의 개인 파일입니다. example.yaml만 추적되는 게 아니라 base 안의 공유 GT/BEVFusion 비교 예제들도 추적됩니다. 개인 YAML·시나리오를 base에 새로 만들지 말고 workspace에 넣어 주세요.

개인 YAML을 지우거나 이름을 바꾸려면 **내 작업 폴더 열기**를 누르세요. 내 YAML 열기는 파일 고르는 창이라 삭제용이 아닙니다. 폴더 버튼으로 파일 탐색기를 열고 휴지통으로 옮기면 됩니다. 실행·검사 중에는 못 열고 중단·정리 후 가능합니다. GUI에서 열어놓은 파일을 지웠다면 다른 YAML을 열어 주세요. GUI는 삭제한 YAML을 다시 만들지 않습니다. 녹화 결과는 지워지지 않지만, 예전 사본 없는 기록이나 배치 목록에서 쓸 YAML은 남겨 주세요.

예제 목적지는 이미 들어있어서 처음에는 지도 좌표를 직접 입력할 필요 없습니다. **YAML 하나에 정한 목적지·시간·스택이 선택한 모든 시나리오에 그대로 적용됩니다.** 다른 목적지나 제한시간으로 돌리고 싶으면 새 이름의 YAML을 만드세요. 장면별 보조 YAML로 덮어쓰는 방식은 없앴습니다. 실행 설정 파일은 schema_version: 2이고 GUI가 보조 파일·깡통 YAML을 받지 않도록 검사합니다.

실행하면 RViz는 왼쪽, MORAI는 오른쪽, GUI 모니터 바는 왼쪽 아래로 배치됩니다. 자동 배치를 창 관리자가 거부하면 직접 정리해도 됩니다. 전체 화면 녹화라 다른 앱을 위에 띄우면 그것도 찍힙니다. 전체 설정 보기를 누르면 GUI를 다시 펼칠 수 있습니다.

검사 통과는 설정 검사일 뿐, 주행 성공 보장은 아닙니다. 특히 새 미션 120개는 **오프라인 검사까지 완료했고 전체 live 주행은 아직 미검증**입니다. 동적 배우가 실제 움직이는지 첫 실행에서 확인해 주세요. 배우가 멈춰 있었는데 PASS한 것은 동적 대응 성공으로 보면 안 됩니다.

두 장면 확인 후 같은 YAML의 패턴을 *.json으로 바꿔 저장하고 새 실험을 시작하면 해당 미션 24개를 돌립니다. 이전 두 장면 기록은 남습니다. 실패한 baseline을 성공시키려고 장애물을 치우거나 판단 파라미터를 억지로 바꾸는 게 목적은 아닙니다. 시스템 오류와 실제 주행 실패만 구분해 주세요.

### YAML 여러 개 걸어두고 자려면

조건별로 개인 YAML을 저장해 두고 **실행 목록 / 배치** 탭을 엽니다. **YAML 여러 개 추가**로 파일을 넣고, 목록 관리 메뉴의 ↑/↓로 순서를 정합니다. 현재 저장된 YAML을 넣으려면 **현재 YAML 추가**를 쓰면 됩니다. **전체 검사 → 새 배치 시작** 순서로 누르면 끝입니다. 중단했던 배치를 이어갈 때만 **배치 재개**를 누릅니다.

각 YAML의 목적지나 탐지기는 달라도 됩니다. 다만 MORAI에 연결한 Sensor/Ego Network와 ScenarioLoad 대상은 같아야 합니다. 목록 중간에 다른 Network를 연결하는 배치는 지원하지 않습니다. 공유 base 파일 말고 개인 workspace YAML을 넣어 주세요.

모니터 바에 배치 2/5, 캠페인 이름, 현재 장면 진행이 같이 나옵니다. 파일·목록 검사는 실제 주행 성공 검사가 아닙니다. 환경 검사의 WARN도 내용을 봐 주세요. 기본 환경 검사는 GPU를 띄우거나 MORAI를 자동 연결하지 않고, 실제 GPU 검사는 설치기/doctor CLI에서 명시적으로 실행할 때만 합니다.

배치 시간은 제한시간+준비·마감 기준 추정이며 24장면짜리 다섯 미션은 대략 4시간 예상입니다. 밤새 돌리기 전에 두 장면의 실제 시간과 저장 용량을 확인하는 게 좋습니다. 설정 후 자주 쓰는 순서는 [짧은 사용법](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)에 따로 줄여뒀습니다. GUI 배치 탭에서도 문서를 열 수 있습니다.

## 9. 담당 미션과 내 작업 폴더

공통 위치는 base_ws/src/planning_addons/planning_scenario_validator/config/missions입니다.

| 미션 | 폴더 | 기본 주행 제한/장면 |
| --- | --- | --- |
| 회전교차로 | roundabout | 75초 |
| 정적 장애물 | static_obstacle | 65초 |
| 사거리 동적 장애물 | dynamic_obstacle | 70초 |
| GPS 음영 정적 장애물 | gps_shadow | 90초 |
| 고주로 | highway | 90초 |

각 미션에 base와 workspace가 있습니다. base는 공유 예제라 Git 추적하고, workspace는 개인 작업이라 안쪽 .gitignore만 추적합니다. 본인 YAML·시나리오가 자동으로 Git에서 제외됩니다. 녹화 결과는 여기가 아니라 planner_logs로 갑니다.

```text
missions/highway/
  base/
    example.yaml
    README.md                  # 기본 좌표·설계 의도·확장 예시·AI 프롬프트
    manifest.json
    scenarios/                 # MORAI 시나리오 JSON만
  workspace/
    .gitignore
    yj_highway_baseline_01.yaml
    yj_highway_dense_01.yaml    # 필요하면 계속 추가
    scenarios/                 # 복사된 시나리오, 여기서 수정
```

개인 YAML은 같은 이름으로 계속 수정·저장할 수 있습니다. 새 실험마다 당시 설정·시나리오·Network·토픽 사본을 결과 폴더에 보관하므로 과거 기록과 섞이지 않습니다. 단, 입력 사본이 없던 이전 고정 이름 실험의 YAML은 기존 보호 규칙을 유지합니다.

시나리오를 직접 만들 때는 JSON만 추가합니다. 그 장면들이 도달할 공통 목적지·시간은 workspace의 실행용 YAML에 적습니다. 목적지는 MORAI 배치 좌표가 아니라 planning의 map 좌표입니다. 서로 다른 목적지가 필요하면 YAML을 각각 복사하고 캠페인 이름을 다르게 주세요. 자세한 제작 순서는 [시나리오·YAML 제작 가이드](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md)에 따로 정리했습니다.

workspace 아래 하위 폴더를 새로 만들었으면 YAML의 scenarios.directory도 그 경로로 바꿔야 합니다. 하위 폴더를 자동으로 전부 재귀 실행하지 않습니다. 원본 JSON·YAML을 수정해도 이전 실험은 보관된 사본으로 재개하지만 결과 폴더의 사본을 직접 수정하면 안 됩니다.

몇 개만 시험하려면 GUI 실행 탭에서 파일 패턴을 바꾸거나 장면 선택으로 JSON을 고른 뒤 YAML 저장을 누릅니다. 전체는 패턴을 *.json으로 바꾸면 됩니다. YAML에서는 scenarios.include가 파일 목록이고 glob이 이름 패턴입니다. 원본을 바꾸지 않는 1–2개 대조는 planning 빠른 테스트 탭을 쓰세요. 반복·랜덤·업로드 없이 별도로 실행합니다.

시나리오 변경을 AI에게 부탁할 때는 이 문서, 해당 미션 base/README.md, base/manifest.json을 같이 읽혀주세요. 미션 안내 끝에 복붙할 프롬프트도 넣었습니다. 목적은 데이터셋 생성이 아니라 일반 상황과 엣지 케이스 검증입니다. 샘플 spawn 위치·종료 경로를 유지하면서 조건을 바꾸고, 시나리오 배우 오작동을 planning 실패로 집계하지 않도록 합니다.

## 10. 중단 / 다음 날 이어 돌리기

이전 보조 YAML 방식의 결과는 그대로 남겨두고 현재 주석 있는 v2 예제로 새 실험을 시작합니다. GUI에서 v1은 확인만 가능합니다. 과거 결과와 새 실행 계약을 섞어 resume하지 않습니다. 새 방식으로 실행한 캠페인은 아래처럼 정상 resume합니다.

중단은 GUI 중단 버튼을 누르고 bag·영상 마감과 스택 종료가 끝날 때까지 기다립니다. 강제 종료보다 이 방법을 써 주세요.

다음에 GUI를 켜면 마지막으로 열거나 저장한 **실존하는 YAML**을 기억해서 엽니다. 기억하는 것은 파일 경로이며, 저장하지 않은 설정까지 기억하는 건 아닙니다. 다른 설정을 열려면 `내 YAML 열기`를, 이전 실행을 찾으려면 `실험 기록 / 재개` 탭을 사용합니다.

오늘 highway_bev.yaml을 실행하고 내일 다시 같은 YAML로 **새 실험 시작**을 눌러도 됩니다. YAML은 그대로이고 결과 폴더만 highway_bev_날짜_시간으로 분리됩니다. 중단한 실험을 이어갈 때는 **실험 기록 / 재개 → 기록 선택 → 선택 실험 재개**를 누릅니다. 재개는 완료한 PASS/FAIL을 건너뛰고 중단·환경 오류 장면을 재시도합니다. **재개 시 FAIL도 재실행**을 체크하면 실패 장면도 다시 실행합니다.

원본 YAML을 수정해도 기존 실험은 **당시 입력 사본**으로 재개합니다. 결과 폴더의 config_snapshot.yaml·scenarios·network·topic_profile.json은 수정하거나 지우지 마세요. 과거 실험에 코드·지도·가중치까지 복제하는 것은 아니므로 업데이트 전후 비교는 새 실험을 시작합니다. 실패한 영상도 남겨두세요.

배치도 같은 방식입니다. 시작할 때 모든 YAML의 입력 사본을 보관합니다. 중단하면 다음 YAML은 시작하지 않고, 다음에 **배치 재개**하면 완료된 YAML·장면을 건너뜁니다. 주행 FAIL은 계속 진행하지만 시스템 오류는 목록을 멈춥니다. 원본 수정은 허용하지만 해당 배치의 목록과 입력 사본은 고정됩니다. MORAI 연결은 재개할 배치의 Network와 맞춰야 합니다.

**목록 초기화**는 목록만 비우고 결과·YAML은 남깁니다. 예전 배치를 다시 열려면 **이전 목록 열기**에서 planner_logs/validation_batches/배치ID/queue.json을 고릅니다. 같은 YAML 목록으로 처음부터 다시 시험하려면 **새 배치 시작**을 누릅니다. YAML을 추가 복사하지 않습니다. FAIL 재실행 체크는 단일 실험 재개에만 적용합니다.

목록은 ~/.config/ajounice/planning_scenario_validator/batch.json에 저장해서 GUI를 껐다 켜도 남습니다. Git 추적 대상이 아니고, 각 YAML의 설정을 바꾸는 별도 설정 파일도 아닙니다.

## 11. 결과 어디서 보나 / 공유는 어떻게 하나

GUI에서 단계·시간·거리·속도·충돌 수와 완료 장면 결과가 실시간으로 보입니다. 실패 장면을 더블클릭하면 관측 근거와 의심 단계가 나옵니다. 분석은 녹화 후 replay하는 게 아니라 주행 중 구독값과 로그로 합니다. 의심 단계는 조사 우선순위이지 근본 원인을 확정한 것은 아닙니다.

결과 폴더 버튼을 누르면 되고, 실제 경로는 아래입니다.

```text
~/AjouNice2026/planner_logs/validation/<캠페인 이름>/
  summary.csv
  campaign_state.json
  stack_logs/
  success/<장면>/             # PASS
  failure/<장면>/             # 주행 실패
  error/<장면>/               # 환경·실행 오류
```

한 장면의 bag·mp4·결과 json은 planbag_시나리오_P또는F_날짜시간처럼 같은 파일명을 씁니다. P는 통과, F는 주행 실패이고, 환경 오류·중단은 E로 남습니다. summary.csv는 전체 표이고, 장면별 JSON·이벤트·로그에는 더 자세한 근거가 있습니다. CSV는 Google 스프레드시트에서 파일 → 가져오기로 열면 됩니다. Drive에 올라갔다고 자동으로 스프레드시트로 바뀌지는 않습니다.

Drive 자동 업로드를 켜면 경로 기본값은 ajou_drive:planning_validator입니다. 실제 공유 위치는 **AjouNice2026/planning_validator/<캠페인 이름>**입니다. 경로에 AjouNice2026을 한 번 더 넣지 마세요.

공유 모드는:

- 기본 공유: CSV·JSON·로그 등, bag/영상 제외
- bag 공유: 기본 공유 + bag
- 영상 공유: 기본 공유 + mp4
- 원본 공유: 기본 공유 + bag + mp4

용량 아끼려면 기본 공유나 영상 공유부터 추천합니다. 이 선택은 **업로드 대상**만 바꾸고 로컬에는 bag과 영상이 그대로 남습니다. 장면이 끝날 때마다 분류 폴더를 유지해 올리고 전체 CSV도 갱신합니다. 공유 폴더 권한이 있는 팀원이 볼 수 있습니다. 업로드 오류는 주행 PASS/FAIL과 별도이며 로컬 결과는 보존됩니다.

미션 24개는 대략 40–55분, 센서 제외 bag+영상은 이전 측정 비율로 약 2.3–3.2 GiB 예상입니다. 새 묶음 실측은 아니니 첫 두 장면의 실제 용량을 봐 주세요. 기본 설정은 시작과 장면 사이에 여유 공간 10 GiB 미만이면 멈추도록 했습니다. 녹화 도중 디스크가 안 찬다는 보장은 아니니 다른 다운로드도 같이 하는 경우 주의해 주세요.

여러 YAML 배치의 총괄 표는 planner_logs/validation_batches/배치ID/summary.csv입니다. GUI의 **배치 CSV**로 바로 열 수 있습니다. 캠페인별 CSV와 Drive 업로드는 그대로이고, **배치 총괄 CSV 자체는 자동 업로드하지 않으니** 필요한 경우 직접 스프레드시트로 가져와 공유해 주세요.

## 12. 막히면 이 순서로 확인

- **ScenarioLoad가 안 됨:** 일반 competition Network가 아니라 validator Network를 Load했는지, Connect했는지 확인.
- **시나리오 설치 폴더를 못 찾음:** MORAI 경로/맵 폴더를 YAML의 morai.scenario_install_dir에 지정. MR과 MRP 설치가 같이 있으면 특히 확인.
- **checkpoint not found:** 위 고정 경로에 E25가 있는지, .env.bevfusion.local이 옛 경로를 덮어쓰는지 확인.
- **Docker 권한/GPU 오류:** sudo 없이 docker ps, docker compose version이 되는지와 verify_team_image.sh 결과 확인.
- **중복 publisher 오류:** 수동 udp/localization/bevfusion/planning/control 터미널을 껐는지 확인. 여러 캠페인 동시 실행 금지.
- **화면 녹화/창 배치 안 됨:** X11 로그인인지, YAML의 recording.video.display가 MORAI 화면의 DISPLAY와 맞는지 확인. 다른 PC도 무조건 :0.0이라고 단정하지 말고 터미널의 echo "$DISPLAY"를 참고.
- **주행 TIMEOUT/충돌:** 일단 실패 기록을 봅니다. 연결 오류가 아니라 실제 planner가 못 간 거면 그걸 찾으려고 만든 검증기라 정상적인 FAIL입니다.
- **동적 배우가 안 움직임:** 시나리오 동작 검증부터 합니다. PASS 숫자만으로 미션 성공이라고 판단하지 않습니다.
- **배치 재개가 거절됨:** 배치 목록이나 결과 폴더의 입력 사본이 변경됐는지 확인하세요. 원본 YAML 수정은 새 방식의 재개를 막지 않습니다. 이전 고정 이름 배치라면 원래 입력이 필요합니다. 기존 결과는 보존합니다.

오류를 보내줄 때는 브랜치/미션/캠페인 이름, 마지막 터미널 오류, summary.csv와 해당 장면 JSON을 같이 주면 원인 찾기 편합니다. 영상도 있으면 좋고, bag은 필요할 때 추가로 공유하면 됩니다.

일단 처음엔 두 baseline만 해보고, 연결과 배우 동작 확인되면 전체 묶음 돌려두고 다른 작업하시면 됩니다. 안 되는 장면이 나오면 없애지 말고 그대로 남겨주세요. 그게 이번 작업의 결과입니다.
