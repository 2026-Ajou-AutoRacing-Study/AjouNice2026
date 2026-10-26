# 신호등 v5 후보 설치·팀 시험

현재 상태: 2026-10-26 **v5와 신호 준수 정책이 traffic·Validator·planning의 새 실행 기본값**이다.
[CURRENT](CURRENT.md)와 [황색·상실 정책 제안](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md)을 함께 읽는다.
새 모델은 YOLO26n960 + 지도 housing 연결 + ResNet18 CNN v5다.
옛 YOLO640/HSV는 명시적 복귀로 보존한다. 준수 가드 ON이어도 기존 competition fail-open의 미수신·timeout GO는 유지한다.
**D 근접 누락은 [조건부 재검출·지도 보완](261025_NEAR_SIGNAL_RECOVERY.md)으로 수정했다.
로컬 Sunny13·Foggy13 무객체 폐루프 PASS이며 LAN·BEVFusion 동시 주행 인증은 아니다.**
파일명/profile의 `candidate`와 가중치의 연구 metadata는 호환·원본 보존용이다. 현재 실행 선택은 CURRENT를 따른다.

짧은 이동 중 저확신 완충과 원본 MORAI 판정선 계산은 [최신 개정](261026_SIGNAL_SMOOTHING_AND_ORIGINAL_JUDGING_LINE.md)을 따른다.
`low_confidence_hold_sec=0.3`(0이면 OFF), 황색 master/timed는 OFF 유지다. 새 C++·YAML·helper가 함께 필요하므로 아래 설치기를 다시 실행한다.

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
신호등 C++ plugin만 재빌드하고 관련 base 패키지를 설치한다. build/install ELF·launch·Python 일치를
receipt로 확인한다. **노드/주행/환경 활성화/GUI는 시작하지 않는다.** 기동 중인 스택은 먼저 정상 종료한다.
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
이미 별도 환경이 있다면 `--existing-python /absolute/bin/python`으로 재사용한다.
`--skip-build`는 모델/환경 검사만 하며 runtime READY가 아니다. `--check`는 다운로드·설치·빌드 없이
기존 모델/환경과 source/build/install receipt를 검사한다. 검사는 기동 중 차를 멈추는 runtime gate가 아니다.

### 시스템 Python에 관하여

이번 고정 검증 조합은 Python3.11·Torch2.7.1/cu128·Pillow12.2다. Ubuntu20.04의 기본
Python3.8에는 이 조합을 그대로 설치할 수 없다. 팀장님 시스템 환경을 원격 확인한 것이 아니므로
기존 YOLO가 돌아간다는 이유만으로 새 YOLO26/CNN도 된다고 보장하지 않는다.
이번에는 검증된 전용 환경 재사용/설치를 기본으로 한다. `--existing-python`도 동일한 고정 버전과
CUDA/두 모델/ROS를 실제 검사하며, 임의의 시스템/user-site 환경을 인증하는 우회 옵션이 아니다.

## Validator 새 실행의 기본값

- 최신 GUI의 `신호등 모델 → v5 지도+CNN 후보`가 기본이다. '후보'는 기존 profile 호환 명칭이다.
- profile을 생략한 원본 YAML도 새 실행은 v5이며, 옛 system 설정은 전용 환경으로 해석한다.
  기존 사용자 지정 가상환경은 유지한다. 특수 경로는 원본 YAML에서 수정한다.
- `신호등 사용`을 ON으로 하고 **새 실험**을 시작한다. 과거 실행 snapshot은 수정하지 않는다.
- 기존 실행 snapshot을 수정하지 않는다. profile이 없던 과거 snapshot은 계속 legacy로 재개한다.
- 새 snapshot에는 유효 profile·Python·모델 경로를 명시하여 이후 기본값 변경과 분리한다.

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

평소처럼 `traffic` 또는 아래 기본 launch를 쓴다. 실제 network/sensor/localization/planning은 먼저 준비한다:

```bash
roslaunch traffic_light_detection traffic_light_node.launch device:=0
```

다른 신호등 노드와 canonical publisher를 중복 실행하지 않는다. LAN에서는 해당 실제
`sensor_config:=.../Sensor_competition_lan.json`도 전달한다. native 신호·Ego는 모델 입력이 아니다.

## 기본 전환 뒤에도 남은 검증

- 실제 route·localization/TF에서 A/B/C(두 head)/D가 맞게 선택되고 옛 D-1은 제외되는지.
- 검출·원시 CNN·보정·유지된 출력·planning 최종 동작을 같은 source stamp로 대조.
- 정차 중 Y/R+Y 새 출발·선 전/후 상실은 [추가 개발 시험](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md#production-261026)과 실제 LAN 단절을 구분한다.
- 4090에서 BEVFusion 동시 실행 시 처리시간/입력 밀림/실제 device/VRAM 확인.
- 로컬 무객체 Sunny13·Foggy13은 주행 PASS. LAN·팀4090 동시 부하·공식 신호 준수는 별도다.

복귀하려면 `traffic_legacy` 또는 GUI의 `기존 YOLO+HSV`를 명시적으로 선택해 새 실행한다. 과거 모델·환경을 삭제하거나
실패 실행을 PASS로 바꿔 재개하지 않는다.

### 정책 기본값과 명시적 복귀

`traffic_light_node.launch`/`traffic_light_v5_candidate.launch`는
`planner_compliance_observations:=true route_progress_guard:=true`가 기본이다.
`planning.launch`는 `enable_signal_compliance_guard:=true traffic_light_state_timeout_sec:=1.0`다.
`morai_planning_stack.launch`도 같은 인자를 전달한다. 정차 중 황색/R+Y는 출발하지 않고,
`enable_yellow_passthrough:=false`가 현재 기본이며, ON일 때만 이동 중 최근 허가→황색의 제동 경계 예외를 검토한다. 관측 3개는 3초가 아니다.
신선한 적색·UNKNOWN에는 정지 판단을 유지한다. 기존 `competition_fail_open=true`에서는 미수신·timeout이면
가드보다 먼저 GO로 처리한다. 신호등 없는 planning 시험은 기존 명령 그대로 가능하지만, 노드 다운도 같은
GO가 될 수 있다. 완전 fail-closed는 기존 설정을false로 바꾼 별도 운영 정책이며 이번 기본값은 아니다.
선 통과 후에는 해당 교차로 통과를 이어가지만 다른 장애물 제동을 해제하지 않는다.

모델 복귀와 정책 복귀는 별개다. **개발 대조용** 옛 정책은 planning에
`enable_signal_compliance_guard:=false traffic_light_state_timeout_sec:=1.0`, traffic에
`planner_compliance_observations:=false route_progress_guard:=false`를 명시한다.
그 경우 기존 황색 GO·상실 fail-open이 복원되므로 권장 운영 복귀가 아니다.
`traffic_light_legacy.launch`는 모델/상류 두 옵션만 되돌리고 planning 가드를 임의 해제하지 않는다.
새 git 코드로 과거 실행 snapshot의 resume 계약을 우회하지 말고 **새 실험**을 시작한다.

운영 기본값 그대로 로컬 Sunny13·Foggy13 무객체 전체 경로를 각각 완료했고 A/B/C/D의 허가 신호
통과와 마지막 정지/P를 대조했다. 중단된 첫 Foggy 주행은 ERROR로 별도 보존했다.
이는 실제 planner/controller·localization 시험이며 팀 4090/LAN·BEVFusion 동시 부하 인증은 아니다.

## Validator 황색 스위치·전방 디버그 녹화

GUI 행은 **신호등 인지 → 황색 통과 허용 → 신호등 디버그 화면 → 화면 영상 녹화**다.
새 두 옵션은 새 실험에서 기본 OFF이며, 기존 창은 정상 종료 후 다시 열어야 보인다.
로컬 설치가 끝났으면 `source scripts/workspace/source_aw149.sh` 후 `./planning_validator_gui`다.
전체 코스의 개인 YAML을 불러와 `탐지=od_gt`, `측위=localization`으로 두면 BEVFusion을 제외한
실제 판단·제어와 신호등을 시험할 수 있다. 신호등·디버그 화면·화면 녹화를 켜고, 황색은 우선 OFF로 둔다.
기본 bag을 켜면 디버그 compressed 토픽도 요청한다. bag OFF는 존중하며 화면 영상 체크와 독립이다.
원본 JPEG까지 필요하면 토픽 선택에서 `/sensor/camera/front/compressed`도 선택한다.
화면 녹화는 전체 DISPLAY를 녹화하므로 디버그 창을 다른 창으로 가리거나 최소화하지 않는다.

```yaml
stack:
  traffic_light: true
  traffic_light_yellow_passthrough: false
  traffic_light_debug_view: true
recording:
  video:
    enabled: true
```

디버그 창은 읽기 전용이며 주행·인지 노드를 추가로 시작하지 않는다. 수동 스택이 이미 신호등 debug
이미지를 발행할 때만 `roslaunch traffic_light_detection traffic_light_debug_view.launch`로 단독 열 수 있다.
직접 신호등을 실행하면 `publish_debug_image:=true`가 필요하고, 판단 쪽 스위치는 별개다.
원시 CNN·profile 보정·최종 램프·미션/head·held/missing·관측 age·황색 **설정**을 표시하며 실제 GO 판정으로 부르지 않는다.
수신/원본 시각이 오래되면 STALE로 표시한다. 과거 frozen snapshot 미기재 키는 당시 yellow ON·image 발행 계약을
읽기 전용으로 유지하고 원문/hash 보호를 바꾸지 않는다. 새 기본을 쓰려면 새 실험을 만든다.
