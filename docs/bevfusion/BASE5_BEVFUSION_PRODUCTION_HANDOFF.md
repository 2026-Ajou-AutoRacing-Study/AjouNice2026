# Base5 E24 BEVFusion 배포와 실행

문서 상태: CURRENT. 2026-10-13 사용자 요청으로 E24를 production 기본 모델로 승격했다.
이는 실제 대회용 4090 24GB의 ROS 입력·LAN·풀스택 안전 검증 완료를 뜻하지 않는다.
팀 공유 브랜치는 `planning/autoware-universe-to-ros1`이다. 이번 전달은 최신 planning 변경을
보존한 통합 코드와 E24 설치기를 함께 사용한다. 가중치만 바꾸고 E25 입력 설정을 유지하면 안 된다.
2026-10-14 전체 통합은 nearest·중심 보완·보행자 연결 순위·출력 동일 CPU 최적화와
validator 분석 녹화/공유를 planning 브랜치용 코드에 함께 반영한다. 팀 PC는 아래 pull·증분 설치 후
기존 tracker·planner를 종료하고 fresh stack으로 실행한다. 코드 통합은 팀 PC의 폐루프 검증 완료가 아니다.
2026-10-15 PR #38·#39·#40에서 LiDAR 50을 포함한 전체 코드를 planning·develop·main에 병합했다.
현재 전달본은 `261015_lidar50`이며, 이전 ZIP의 센서 launch 해시 불일치에 따른 설치 차단을 해소한다.
기존 가중치·모델 설정은 변경하지 않았으며 팀 PC의 실제 설치·LAN 주행 확인은 별도다.

## 고정 실행 계약

| 항목 | 값 |
|---|---|
| 기본 진입점 | `bevfusion` 또는 `bash scripts/runtime/run_bevfusion.sh` |
| 가중치 | `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth` |
| SHA256 | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` |
| 클래스와 threshold | Vehicle/Pedestrian/Obstacle, 0.40/0.10/0.10 |
| 모델 | 512×1056, 5cm/20cm, depth 0.25m, single-frame 7D |
| 입력 | 원본 JPEG, 원본 XYZI 0–255를 서버에서 정확히 한 번 ÷255 |
| 추론 | AMP·camera 병렬화·nearest depth hint ON, compile OFF |
| 센서 기본 | `Sensor_competition_lan.json`, 실행 인자로 local/다른 실제 센서 JSON 지정 가능 |
| LiDAR 묶음 | 2026-10-14 전환: 50 packets, 900 RPM, 첫 패킷 stamp, cut_angle −0.01. 적용 브랜치는 아래 범위 참조 |
| 출력과 후처리 | tagged bottom→gravity center 변환 한 번, canonical detection topic, class-aware rotated BEV NMS 0.50 |
| 후단 | pedestrian Gaussian 순위 ON → UNKNOWN stationary policy → Map-CV. gate·수명·planning 정책 유지 |

`bevfusion_production_profile.py`가 checkpoint·모델 config·bridge config·launch를 함께 선택한다.
서버 health는 weights·모델 config·센서 SHA와 Base5·AMP·병렬화·compile·클래스·입력 계약,
명시적인 바닥 원점 태그와 nearest policy를 검사한다.
다른 canonical detector publisher가 있으면 실행을 거부한다. 이미 켜진 eager 서버는 준비를 기다리고
선택 weights/config·센서·입력 계약까지 같을 때만 재사용하며 해당 컨테이너의 종료 소유권을 가져오지 않는다.
다른 계약의 서버를 자동 종료하거나 실행 중 모델을 교체하지 않는다. launch만으로 GPU 서버가 준비되는 것은 아니다.
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
공용 OAuth 앱의 403 요청 제한이 있으면 본인이 구성한 전용 클라이언트 remote를 사용한다.
이 PC의 예는 `--remote ajou_drive_private`이며 다른 사람의 token을 공유하는 방법이 아니다.

1. 필요한 tracker와 MMDetection3D submodule을 현재 Git revision에 맞춘다. 개인 수정이 있으면 중단한다.
2. Drive ZIP의 크기·SHA256·내부 파일·당시 참고 소스의 무결성을 검사한다. 현재 checkout을 옛 참고 소스에 맞춰 되돌리지 않는다.
3. 가중치만 `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth`에 원자적으로 설치한다.
4. 기존 Docker의 모듈 import와 작은 CUDA 연산을 확인한다.
5. `autoware_launch`, `sensor_interface`, `perception_object_pipeline`, `ekf_multi_object_tracking`, `bevfusion_ros`,
   `planning_scenario_validator`를 증분 빌드한다. 전체 clean이나 드라이버 재설치는 하지 않는다.

기존 가중치의 SHA가 맞으면 다운로드하지 않는다. 다른 파일은 자동 덮어쓰지 않는다.
정상 ZIP은 `~/bevfusion_artifacts/downloads`에 재사용하며 토큰은 각 PC의 rclone 설정에만 둔다.

현재 전달 ZIP은 `model/bevfusion/production/base5_e24_261015_lidar50/ajounice_base5_e24_production_261015_lidar50.zip`이며
449,672,211 bytes, 약 429MiB다. SHA256은
`199f7328fdee518b4d5fcb2c9583f7d366d4a5272023ececabda8857030d37e4`이다.
[공유 Drive ZIP](https://drive.google.com/file/d/1FC90CTXcOYHuxiGS-tFG0_JWk57JiK6f/view)은 새 경로에 업로드하고 원격 크기·MD5를 로컬과 대조했다.
위 설치기가 현재 release의 Drive 경로를 자동 선택한다. 10월 13일 및 14일 runtime/fullmerge ZIP은
이전 코드 재현용으로 보존하며 새 checkout 설치에 혼용하지 않는다.
가중치·검증 manifest·README·현재 runtime과 상속 config의 참고 사본만 포함한다.
`reference/`는 비교용이며 checkout에 덮어쓰지 않는다. 데이터셋·optimizer·인증 정보·Docker 이미지는 없다.
가중치 자체는 변경하지 않았다. 구 폴더의 원본 `E24.pth`도 보존했다.

ZIP을 브라우저로 받았다면 같은 설치기에 경로만 넘긴다.

```bash
bash scripts/runtime/install_base5_production.sh --archive /전달받은/ajounice_base5_e24_production_261015_lidar50.zip
```

모델 설치만 이미 확인했으면 `--check-only`로 재검사한다. 이 옵션은 빌드를 생략하고 작은 Docker GPU
검사는 수행한다. CUDA 검사도 생략할 때는 `--skip-docker-check`를 추가한다.
직접 증분 빌드를 마쳤을 때만 `--skip-build`를 사용한다. Git pull만으로 최신 tracker 바이너리라 가정하지 않는다.

```bash
bash scripts/runtime/install_base5_production.sh --check-only
```

2026-10-23에는 가중치 설치와 과거 코드 snapshot 감사를 분리했다. 현재 checkout에 새 진단·
가속기 코드가 추가됐다는 이유로 정상 E24 가중치의 설치를 거부하지 않는다. 동결 ZIP·가중치
SHA, 안전한 압축 경로, 기존 다른 weights 보존은 유지한다. 기존 ZIP·가중치의 재업로드는 필요 없다.
이 설치 결과는 최신 코드의 실행 인증이 아니며 아래 preflight와 팀 PC 기동 시험으로 구분한다.
과거 코드의 정확한 byte 대조가 목적일 때만 `base5_production_bundle.py audit-runtime-snapshot`을 사용한다.

초기 ROS workspace나 Docker가 없는 PC는 기존 설치 절차로 의존성을 준비한 뒤 위 E24 준비를 수행한다.
기존 `setup_bevfusion_inference.sh`는 E25 archive도 준비하는 호환 설치기이며 E24 설치 완료로 해석하지 않는다.

## 실행과 복귀

<a id="competition-startup"></a>

### 대회 전 오프라인 기동 확인과 안전 실행

**10/23 로컬 구현·검증이며 팀 PC 배포·실제 4090 LAN 풀스택 검증 완료라는 뜻은 아니다.**
정상 준비된 이미지·가중치로 추론할 때 W&B·Drive·Docker Hub 로그인이나 인터넷은 필요 없다.
다음 첫 명령은 읽기 전용 검사다. 두 번째는 자체 임시 컨테이너에서 실제 모델 warmup과
실제 서버 health PONG을 확인하며 TCP listen·ROS publisher·주행은 시작하지 않는다.

```bash
cd "$HOME/AjouNice2026"
python3 scripts/runtime/preflight_bevfusion_competition.py --safe
python3 scripts/runtime/preflight_bevfusion_competition.py --safe --offline-smoke
```

그 뒤 센서/TF/localization과 기존 풀스택 순서에 맞춰 실행한다. alias가 없으면 shell 경로를 사용한다.

```bash
bevfusion --safe --model e24
# 또는
bash scripts/runtime/run_bevfusion.sh --safe --model e24
```

`--safe`는 같은 E24 checkpoint/config·threshold·AMP·camera 병렬화·nearest·센서 계약을
유지하고, 선택 TensorRT/시험용 pooling을 끈다. E25로 바꾸는 옵션이 아니다. 선택 E25 복귀는
`bevfusion --safe --model e25`이며 그 가중치도 미리 준비해야 한다.
안전 모드가 GPU driver 고장, 누락 weights, 잘못된 센서, 중복 detector publisher까지 무시하지는 않는다.

기본은 선택 engine이 없으면 eager이며, engine을 명시하고 정책을 생략하면 `prefer`다.
이때 engine 파일·GPU/TRT·가중치 호환 불일치는 **첫 engine enqueue 이전의 시작 단계에서만**
같은 선택 모델의 eager로 복귀한다. 실제 backend·복귀 이유를 서버 status에 기록한다.
`BEVFUSION_CAMERA_TENSORRT_POLICY=require`는 엄격한 가속기 자격 시험용이며 미준비 팀 PC의
대회 기본값으로 쓰지 않는다. raw 수치 동일성/AP/실험 receipt gate를 매번 production 시작 전에
재평가하거나 최신 W&B/Drive에 접속해야 하는 구조는 아니다.
Validator의 새 E24 실행 계약도 현재 소스로 생성·검증했으며 engine을 전달하지 않는다.
코드를 바꾼 뒤 **옛 실행 결과에 resume**하는 보호 검사는 유지하므로 새 실험으로 시작한다.
이 shell의 `--safe`가 validator GUI에도 새로 연결됐다는 뜻은 아니다.

첫 시작에는 CUDA extension 컴파일·모델 warmup 때문에 수십 초가 걸릴 수 있다. 로컬 5060Ti의
네트워크 차단 cold smoke는 약 82초에 실제 ready PONG까지 통과했다. persistent JIT cache를
추가했지만 GPU/소스별 재컴파일이 필요할 수 있으므로 출발 직전에 처음 켜지 말고 사전 warmup한다.
cache volume이 아직 없으면 smoke는 임시 cache를 사용하므로 실제 서버의 첫 시작은 따로 준비한다.
실행기는 이미 켜진 서버가 warmup 중인 경우에도 제한된 준비 대기를 적용한다.

센서가 안 들어오거나 카메라와 LiDAR 차이가 50ms를 넘는 프레임은 기존 동기화 정책상 사용하지
않는다. single-frame production은 deskew·temporal sweep·pose coverage를 요구하지 않는다.
준비 완료와 검출 토픽의 실제 연속 발행은 다르므로 팀 PC에서 다음 확인까지 수행한다.

```bash
rostopic echo -n 1 /perception/bevfusion/status
rostopic hz /perception/object_recognition/detection/objects
```

illegal memory access/device-side assert 같은 CUDA 손상은 같은 프로세스에서 무리하게 원형으로
전환하지 않는다. 주행을 안전하게 중단한 뒤 담당 세션에서 새 프로세스의 `--safe`로 복구한다.
다른 노드/컨테이너를 자동 kill하지 않으며 물리 비상정지나 무중단 주행을 보장하는 기능이 아니다.
구현·GPU 시험·남은 검증은 [61절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#continuous-trt-competition-startup-261023)을 따른다.

### LiDAR 50 적용과 51 복귀

2026-10-14 사용자 승인으로 `lidar.launch`와 `morai_udp_interface.launch`의 기본 묶음을 50으로
변경했다. 당시 `dataset/base5`에만 반영돼 planning에는 후속 통합이 필요했으나,
2026-10-15 PR #38·#39·#40에서 planning·develop·main까지 전체 병합했다.
과거 planning `82321449` 조회에서 기본값이 비어 있던 기록은 병합 전 상태이며 현재 기본값은 50이다.
팀 브랜치의 코드 반영과 팀 PC 설치 완료는 별개다. 팀 PC는 최신 pull과 증분 설치 후
설치된 launch의 실제 50 기본값을 확인한다.
코드 반영 뒤 `sensor_interface`를 증분 설치하고 기존 UDP/LiDAR 수신기를 종료·재시작해야 한다.
실행 중 `rosparam set`만으로 이미 초기화된 드라이버의 묶음을 바꾸지 않는다.
MORAI의 LiDAR 15Hz 설정·카메라·timestamp 정책·E24 가중치는 변경하지 않는다.
기존 데이터 재구축이나 재학습은 필요하지 않다. LAN·팀 PC의 연속 검출/추적 지연은 후속 확인한다.

새 기본값은 기존 UDP 실행 명령을 그대로 사용한다. 51로 복귀할 때만 게이트웨이 명령에
`lidar_npackets:=51`을 추가한다. LiDAR 단독 launch는 `npackets:=51`이다.
빈 CLI 인자 `lidar_npackets:=`는 Noetic에서 무시되어 새 기본값 50이 적용되므로 복귀 인자가 아니다.
가중치 자체는 그대로이며 재학습은 필요하지 않다. 다만 전달 ZIP은 센서 launch의 해시도
고정하므로 50 전환 뒤에는 최신 코드 기준 재동결이 필요하다. `261014_fullmerge`를 현재 코드에
설치하면 보호 검사가 중단되며, 이를 우회하지 않고 `261015_lidar50` 전달본을 사용한다.

2026-10-14 현재 박스 원점이 수정됐다. 기존 detector·tracker·planner를 모두 정지하고
새 stack으로 시작한다. 실행 중인 track/planner memory에 새 중심 기준을 hot-switch하지 않는다.
이 변경은 실제 MORAI 에셋 높이·수집 GT·가중치 변경이 아니다.
[Live 검토](261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md)에 반복성·실제 ROS·future-Z의 범위를 구분했다.

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

보행자 순위 후속은 tracker submodule `1cdf0a9`와 새 object pipeline launch를 함께 사용한다.
정상 SHA의 E24 가중치가 이미 설치돼 있으면 다시 내려받지 않는다. 설치기의 최신 배포 명세와
Git/submodule 증분 빌드는 필요하며, 구 ZIP을 현재 checkout의 `--archive` 인자로 혼용하지 않는다.
시작 로그의 `pedestrian_gaussian_rank=1`을 확인한다. 독립 비교·복귀는
`tracking pedestrian_gaussian_association_ranking:=false`이며 기존 노드를 종료한 뒤 실행한다.
Tracker 옵션은 detector 모델 선택과 독립적이므로 E25 가중치로 복귀하는 것만으로 이 순위가 OFF되지는 않는다.
전체 통합 이후에도 팀장님 PC의 checkout·submodule·설치본 갱신 전에는 적용됐다고 가정하지 않는다.

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

### 2026-10-15 전달본 재동결 검증

`261015_lidar50`는 50패킷 센서 launch를 포함한 현재 runtime 62개 파일의 해시를 고정한다.
기존 `261014_fullmerge`와 달라진 runtime 참고 파일은 `lidar.launch` 하나이며 E24 가중치 SHA는 동일하다.
보호 검사를 완화하지 않고 배포 명세·ZIP·Drive 경로를 함께 갱신했다. 이전 전달본은 삭제하지 않았다.

배포 bundle 검사 8개와 production profile/health 검사 7개가 통과했다. 기존 정상 가중치가 있어도
낡은 소스 해시는 거부하는 회귀 검사를 포함한다. Drive 업로드 뒤 원격 크기·MD5를 대조하고,
별도의 빈 임시 HOME에서 설치기가 실제 ZIP을 다시 다운로드해 archive SHA256·runtime 사본·
가중치 SHA를 검증한 뒤 설치했다. 이어 `--check-only` 재실행에서 다운로드하지 않고 가중치 mtime을
유지함을 확인했다. 이 전달 검증에서는 Docker GPU 검사·ROS 빌드·노드 실행·실주행을 생략했다.
팀 PC는 최신 코드를 pull하고 위 기본 설치 명령으로 필요한 증분 빌드까지 수행해야 한다.

### 2026-10-13 최초 통합 검증 이력

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
