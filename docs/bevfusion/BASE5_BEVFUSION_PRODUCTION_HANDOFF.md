# Base5 E24 BEVFusion 배포와 실행

문서 상태: CURRENT. 2026-10-13 사용자 요청으로 E24를 production 기본 모델로 승격했다.
이는 실제 대회용 4090 24GB의 ROS 입력·LAN·풀스택 안전 검증 완료를 뜻하지 않는다.
팀 공유 브랜치는 `planning/autoware-universe-to-ros1`이다. 이번 전달은 최신 planning 변경을
보존한 통합 코드와 E24 설치기를 함께 사용한다. 가중치만 바꾸고 E25 입력 설정을 유지하면 안 된다.

## 고정 실행 계약

| 항목 | 값 |
|---|---|
| 기본 진입점 | `bevfusion` 또는 `bash scripts/runtime/run_bevfusion.sh` |
| 가중치 | `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth` |
| SHA256 | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` |
| 클래스와 threshold | Vehicle/Pedestrian/Obstacle, 0.40/0.10/0.10 |
| 모델 | 512×1056, 5cm/20cm, depth 0.25m, single-frame 7D |
| 입력 | 원본 JPEG, 원본 XYZI 0–255를 서버에서 정확히 한 번 ÷255 |
| 추론 | AMP·camera 병렬화 ON, compile OFF |
| 센서 기본 | `Sensor_competition_lan.json`, 실행 인자로 local/다른 실제 센서 JSON 지정 가능 |
| 출력과 후처리 | canonical detection topic, class-aware rotated BEV NMS 0.50 |
| 후단 | 기존 tracking → UNKNOWN stationary policy → Map-CV. 이번 승격만으로 tracker/planner 제약을 완화하지 않음 |

`bevfusion_production_profile.py`가 checkpoint·모델 config·bridge config·launch를 함께 선택한다.
서버 health는 weights·모델 config·센서 SHA와 Base5·AMP·병렬화·compile·클래스·입력 계약을 검사한다.
기존 서비스나 canonical detector publisher가 있으면 실행을 거부한다. launch만으로 GPU 서버가 준비되는 것은 아니다.
인자 없는 역사적 Compose 명령은 E25 호환 기본값을 유지한다. E24는 `bevfusion` 실행기로
weights·서버 옵션·센서·bridge를 함께 선택하며 Compose만 따로 켜서 두 계약을 섞지 않는다.

## 팀원 최초 준비

기존 풀스택·팀 Docker 이미지·NVIDIA Container Toolkit이 준비된 팀원은 주행과 기존 detector를
종료한 뒤 다음 명령을 실행한다. 기존 이미지·E25·개인 `.env`는 삭제하거나 덮어쓰지 않는다.

```bash
cd "$HOME/AjouNice2026"
git fetch origin
git checkout planning/autoware-universe-to-ros1
git pull --ff-only origin planning/autoware-universe-to-ros1
bash scripts/runtime/install_base5_production.sh
```

`ajou_drive` remote와 팀 공유 폴더 접근 권한이 필요하다. 다른 remote 이름이면
`--remote 이름`을 추가한다. 설치기는 다음 작업을 수행하며 MORAI·ROS 노드·주행을 시작하지 않는다.

1. 필요한 tracker와 MMDetection3D submodule을 현재 Git revision에 맞춘다. 개인 수정이 있으면 중단한다.
2. Drive ZIP을 다운로드하고 크기·SHA256·압축 내부 파일·현재 runtime 소스를 검사한다.
3. 가중치만 `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth`에 원자적으로 설치한다.
4. 기존 Docker의 모듈 import와 작은 CUDA 연산을 확인한다.
5. `autoware_launch`, `perception_object_pipeline`, `ekf_multi_object_tracking`, `bevfusion_ros`,
   `planning_scenario_validator`를 증분 빌드한다. 전체 clean이나 드라이버 재설치는 하지 않는다.

기존 가중치의 SHA가 맞으면 다운로드하지 않는다. 다른 파일은 자동 덮어쓰지 않는다.
정상 ZIP은 `~/bevfusion_artifacts/downloads`에 재사용하며 토큰은 각 PC의 rclone 설정에만 둔다.

전달 ZIP은 `model/bevfusion/production/base5_e24_261013/ajounice_base5_e24_production_261013_runtime.zip`이며
449,655,082 bytes, 약 429MiB다. SHA256은
`446fb95d9cf5a518e3f28ea02555f79ece6f5ecd5c60a92e2be1f1f7c4dadac7`이다.
[공유 Drive에서 ZIP 받기](https://drive.google.com/file/d/1_PSBmQlEHbk_c6c16mgwnL4-0V_382ho/view).
가중치·검증 manifest·README·현재 runtime과 상속 config의 참고 사본만 포함한다.
`reference/`는 비교용이며 checkout에 덮어쓰지 않는다. 데이터셋·optimizer·인증 정보·Docker 이미지는 없다.
원본 `E24.pth`도 같은 Drive 폴더에 보존했다.

ZIP을 브라우저로 받았다면 같은 설치기에 경로만 넘긴다.

```bash
bash scripts/runtime/install_base5_production.sh --archive /전달받은/ajounice_base5_e24_production_261013_runtime.zip
```

모델 설치만 이미 확인했으면 `--check-only`로 재검사한다. 이 옵션은 빌드를 생략하고 작은 Docker GPU
검사는 수행한다. CUDA 검사도 생략할 때는 `--skip-docker-check`를 추가한다.
직접 증분 빌드를 마쳤을 때만 `--skip-build`를 사용한다. Git pull만으로 최신 tracker 바이너리라 가정하지 않는다.

```bash
bash scripts/runtime/install_base5_production.sh --check-only
```

초기 ROS workspace나 Docker가 없는 PC는 기존 설치 절차로 의존성을 준비한 뒤 위 E24 준비를 수행한다.
기존 `setup_bevfusion_inference.sh`는 E25 archive도 준비하는 호환 설치기이며 E24 설치 완료로 해석하지 않는다.

## 실행과 복귀

기존 sensor/TF/localization 터미널을 준비하고 실행한다. 동일 canonical topic에 detector를 두 개 띄우지 않는다.

```bash
ajou
bevfusion
```

개인 PC의 loopback 센서 설정을 사용할 때는 다음과 같다.

```bash
bevfusion "$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json"
```

sensor JSON은 서버와 bridge에 같은 파일로 전달된다. 기본값이 LAN이라도 시뮬레이터 UI의 IP를
자동으로 바꾸지는 않는다. `tracking`·planning은 기존 순서로 실행하며 `start_detector:=false`를 유지한다.

E25로 복귀하려면 현재 detector 터미널을 먼저 종료한 뒤 다음을 실행한다.

```bash
bevfusion --model e25
```

E25 가중치는 기존 경로에 보존한다. 없으면 [E25 복귀 계약](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)으로 준비한다.
옛 `.env.bevfusion.local`의 `BEVFUSION_INFERENCE_CHECKPOINT`는 E25 복귀에서만 사용한다.
명시적인 `BEVFUSION_CHECKPOINT` override는 두 모델에서 사용하되 SHA가 다르면 거부한다.
기존 .env는 수정하지 않으며 단순 config override로 E24를 E25 입력에 연결하지 않는다.
Validator GUI의 기본 모델은 `Base5 E24 production`이다. 새 YAML에 `bevfusion_model`이 없으면 E24를
선택하고 새 execution snapshot에는 선택 모델을 명시적으로 저장한다. E24는 위 고정 가중치 경로를
사용하며 Git 제외 평가 폴더를 요구하지 않는다. E9는 0.30/0.20/0.12의 별도 비교 프리셋이다.

명시적으로 `e25`를 선택한 YAML은 E25이며 실행기에 `--model e25`를 전달한다.
구 실행의 checkpoint·코드·계약이 바뀌면 기존 결과를 보존하고 새 실험을 시작한다. 강제 resume하지 않는다.
Validator가 소유한 Base5 서버는 별도 포트 29515와 기존 `/perception/base5_trial/status`·`latency_ms`를
유지하지만 detection 출력은 canonical topic이다. 수동 `bevfusion`과 GUI 자동 실행을 중복하지 않는다.

## 통합 코드 검증

2026-10-13 planning 기준 `0ab05110`의 최신 네 커밋을 합쳤다. 로터리 stopline·early pass 설정과
원거리 CollisionData 오염 처리, 정적 장애물 17장 캠페인은 보존했다. 새 인지 배포 때문에 해당
planning 정책을 별도로 변경하지 않았다.

설치·production 검사 12개, validator 221개, ROS bridge 5개가 통과했다. Docker CPU 검사 28개와
센서 보정 검사 1개도 통과했으며 CPU 검사에서는 CUDA 전용 4개를 생략했다.
공용 이미지에서 E24 전체 state를 현재 config에 strict load했고 실제 작은 CUDA 연산도 확인했다.
ZIP으로 빈 HOME에 설치한 뒤 같은 명령을 반복해도 기존 가중치를 유지했다.
최종 Drive ZIP을 빈 HOME에서 실제 내려받아 SHA256·내부 파일·가중치 설치를 확인했고 재실행에서도
가중치 mtime을 유지했다. 이 전달 검사는 실제 폐루프 주행을 대신하지 않는다.
팀장님 브랜치의 개인 17장 YAML은 `/home/ajou/Desktop`의 MORAI 설치 경로를 사용한다.
개인 경로를 이 PC의 `/home/luv`로 바꾸지 않았으며 해당 PC에서 GUI 환경 검사로 확인한다.

## 팀장님 PC에서 확인할 항목

- 실제 체크포인트·옵션·센서가 일치하고 canonical publisher가 하나인지 확인한다.
- LiDAR 주기 대비 latency p50/p95, 입력 age·drop·queue, 실제 24GB VRAM·CPU 부담을 기록한다.
- 자차 근처 회신점과 자차 위치 Vehicle 출력이 있는지 확인한다. 구 Base3 입력에서는 E24가 자차를 검출한 사례가 있어 무조건 안전한 자차 mask를 적용하지 않는다.
- 근접 장애물의 ID·경계·누락·중복과 고주로 전후방 차량의 클래스·속도·연속 전달을 확인한다.
- 유효 trajectory·제어·CollisionData까지 확인한다. 오프라인 detector/tracker 비교는 폐루프 인증이 아니다.

E24 선택 근거와 알려진 한계는 [모델 CURRENT](CURRENT.md)와
[43절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-e9-e24-selection-261013),
[44절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-on-base3-val-261013)을 따른다.
