# 신호등 v5 후보 설치·팀 시험

현재 상태: **명시적으로 선택하는 시험 후보**, 운영 기본 승격 전이다.
[CURRENT](CURRENT.md)와 [황색·상실 정책 제안](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md)을 함께 읽는다.
새 모델은 YOLO26n960 + 지도 housing 연결 + ResNet18 CNN v5다.
기존 YOLO640/HSV와 planning 황색·fail-open 값은 자동 교체하지 않는다.

## 팀장님 4090 — 설치

이미 준비된 AjouNice2026 ROS1 workspace·NVIDIA 드라이버가 필요하다.
Ubuntu20.04/Noetic x86_64 기준이며 시스템 Python/pip·기존 Conda 환경은 업그레이드하지 않는다.
전용 환경이 이미 있으면 재사용하고, 없을 때만 설치한다. 전용 환경은
`~/.local/share/ajounice/traffic_light_gpu`, 모델은
`~/.local/share/ajounice/traffic_light_models/v5_3d29ee7f`에 별도로 둔다.

```bash
cd ~/AjouNice2026
git pull --ff-only origin planning/autoware-universe-to-ros1
bash scripts/runtime/install_traffic_light_candidate.sh
source scripts/workspace/source_aw149.sh
```

설치기는 모델 다운로드·해시 검사·전용 Python/CUDA 환경·실제 YOLO/CNN 추론 및 ROS import 검사·
관련 패키지 증분 설치를 한다. **노드/주행/환경 활성화/GUI/운영 전환은 시작하지 않는다.**
실행 시에도 없는 모델을 조용히 다른 모델로 바꾸지 않는다.

### 모델 다운로드·공유 범위

2026-10-25 사용자 승인으로 **모델 압축본 한 파일만 링크 공개 읽기**를 허용했다.
기본 설치는 Drive 로그인·rclone 인증 없이 고정 file ID로 내려받아 SHA를 검사한다.
부모 폴더·재현 데이터 백업은 계속 비공개이며, 설치기는 공유 권한을 바꾸지 않는다.
Git/설치기에 OAuth secret·refresh token을 넣거나 다른 PC에 인증파일을 복사하지 않는다.
공개 다운로드가 차단/할당량 초과되면 모델 압축본을 직접 전달해 아래 오프라인 경로를 사용한다.
`public_file_id`가 없는 별도 사내 배포 계약에서만 기존 인증 rclone `--remote NAME` 경로를 쓴다.

압축본 `final_weights_bundle.tar.gz`를 Downloads에 받았다면 위 기본 명령이 먼저 재사용한다.
다른 위치라면 다음과 같이 지정한다. 이 경로는 모델 파일만 설치하며 전체 데이터셋은 필요 없다.

```bash
bash scripts/runtime/install_traffic_light_candidate.sh --archive /path/to/final_weights_bundle.tar.gz
```

고정 Drive 파일 ID: `1OOT_cVJxBn_Es4sEZYUmf3QskcBT2Gzg`.
압축본 SHA256: `8e9e184812ff87e0bba4215483893b6453207df5d4ed163f46239bab4fc5e78e`.
배포 계약은 [release JSON](../../../configs/perception/traffic_light_v5_candidate_release.json) 한 곳에서 관리한다.
모델 단독 준비 `--models-only`는 GPU/ROS 검증 완료가 아니다.
이미 별도 환경이 있다면 `--existing-python /absolute/bin/python --skip-build`로 **변경 없이 검사**할 수 있다.
`--check`는 다운로드·설치·빌드 없이 기존 모델/환경을 검사한다.

### 시스템 Python에 관하여

이번 고정 검증 조합은 Python3.11·Torch2.7.1/cu128·Pillow12.2다. Ubuntu20.04의 기본
Python3.8에는 이 조합을 그대로 설치할 수 없다. 팀장님 시스템 환경을 원격 확인한 것이 아니므로
기존 YOLO가 돌아간다는 이유만으로 새 YOLO26/CNN도 된다고 보장하지 않는다.
이번에는 검증된 전용 환경 재사용/설치를 기본으로 한다. `--existing-python`도 동일한 고정 버전과
CUDA/두 모델/ROS를 실제 검사하며, 임의의 시스템/user-site 환경을 인증하는 우회 옵션이 아니다.

## Validator에서 새 실행으로 선택

- 최신 GUI의 `설정 확인 (YAML 수정) → 신호등 모델 → v5 지도+CNN 후보`를 명시적으로 선택한다.
- 시스템 Python이던 YAML에서 이 후보를 선택하면 전용 환경 경로를 선택한다.
  기존 사용자 지정 가상환경은 유지한다. 특수 경로는 원본 YAML에서 수정한다.
- `신호등 사용`을 ON으로 하고 **새 실험**을 시작한다. 과거 실행 snapshot은 수정하지 않는다.
- 기본 모델은 계속 `기존 YOLO+HSV`다. 예전 YAML을 열고 저장해도 후보 설정을 덧붙이지 않는다.

직접 YAML을 편집할 때의 추가 키:

```yaml
stack:
  traffic_light: true
  traffic_light_profile: v5_candidate
  traffic_light_runtime: virtualenv
  traffic_light_python: ~/.local/share/ajounice/traffic_light_gpu/bin/python
  traffic_light_device: '0'
  traffic_light_models_root: ~/.local/share/ajounice/traffic_light_models/v5_3d29ee7f
```

수동 노드 시험은 실제 network/sensor/localization/planning이 이미 준비된 상태에서만:

```bash
roslaunch traffic_light_detection traffic_light_v5_candidate.launch device:=0
```

다른 신호등 노드와 canonical publisher를 중복 실행하지 않는다. LAN에서는 해당 실제
`sensor_config:=.../Sensor_competition_lan.json`도 전달한다. native 신호·Ego는 모델 입력이 아니다.

## 승격 전 확인

- 실제 route·localization/TF에서 A/B/C(두 head)/D가 맞게 선택되고 옛 D-1은 제외되는지.
- 검출·원시 CNN·보정·유지된 출력·planning 최종 동작을 같은 source stamp로 대조.
- 정차 중 Y/R+Y 새 출발, 선 전 카메라/신호 단절, 선 통과 후 정상 화각 이탈은 별도 정책 시험.
- 4090에서 BEVFusion 동시 실행 시 처리시간/입력 밀림/실제 device/VRAM 확인.
- 현재 full-route 주행 통과 및 LAN 실시간성 인증은 아직 아니다.

실패 시 기본 legacy 선택으로 새 실행을 만들면 된다. 과거 모델·환경을 삭제하거나
실패 실행을 PASS로 바꿔 재개하지 않는다.
