# MORAI OD dataset workspace

MORAI 기반 3D 객체 인식 데이터셋의 시나리오 생성 코드, 설정, 메타데이터와
검증 결과를 한곳에서 관리한다.

## 저장 원칙

Git으로 추적한다.

- 시나리오 생성·검증 스크립트
- 생성 설정과 템플릿 설명
- MORAI Internal Scenario JSON
- manifest와 train/val/test split
- 생성 메타데이터와 검증 보고서
- 작고 재현 가능한 예제

Git으로 추적하지 않는다.

- 카메라 이미지와 LiDAR PCD
- rosbag
- 변환된 학습 데이터
- 모델 체크포인트
- 실행 로그, 캐시와 임시 파일

대용량 데이터의 기본 위치는 저장소 밖의 다음 경로다.

```text
$AJOUNICE_DATA_ROOT/morai_od/
├── raw/
├── captures/
├── rosbags/
├── processed/
└── checkpoints/
```

팀원은 셸 환경에 데이터 루트만 지정한다.

```bash
# 팀 공통 대용량 데이터 루트 설정
export AJOUNICE_DATA_ROOT="$HOME/ajounice_data"
```

## 디렉터리

```text
dataset/morai_od/
├── scripts/       # 생성·검증·변환 코드
├── configs/       # 생성 규칙과 실행 설정
├── templates/     # MORAI 템플릿 설명·승인 템플릿
├── docs/          # 지도·자산·시나리오 생성 및 자동화 설계
├── schemas/       # manifest 등 파일 형식
├── scenarios/     # Git으로 관리하는 MORAI 시나리오 JSON
├── manifests/     # 샘플 목록과 split
├── reports/       # 생성·검증 보고서
├── metadata/      # seed, 환경 조건, 입력 버전
└── local_reference/ # 벤더 원본 예제 복사본; Git 제외
```

## 현재 프로토타입 생성

현재 생성기는 공식 경로와 MGeo 링크를 이용해 Ego 주변 20 m에 정적 NPC,
보행자와 장애물을 배치한다. 날씨와 시간은 MORAI Internal Scenario JSON에
검증된 필드가 없어 metadata에만 기록한다.

```bash
# 기본 seed와 anchor로 프로토타입 시나리오 생성
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_morai_od_prototype.py
```

기본 출력:

```text
dataset/morai_od/scenarios/prototypes/*.json
dataset/morai_od/metadata/prototypes/*.metadata.json
```

주의: 현재 보행자는 도로 가장자리 근사 규칙으로 배치한다. 대량 생성 전에는
GIS/Lanelet 횡단보도 제약, MORAI asset catalog와 시뮬레이터 실제 로드 검증을
추가해야 한다.

## Scenario Runner 검토 결과와 참조 리소스

개인용 MORAI Scenario Runner v1.7.0에서 확보한 K-City 2025 MGeo와 XOSC
예제의 활용 방법은 다음 문서에 정리했다.

- `docs/scenario_runner_adoption_review.md`
- `docs/scenario_runner_resource_integration.md`
- `configs/scenario_runner_asset_catalog.yaml`

원본 리소스는 MORAI 설치본에서 다시 얻을 수 있고 버전·라이선스에 종속되므로
Git에는 올리지 않는다.

Scenario Runner의 외부 Python API는 현재 환경에서 재현 가능한 공식
의존성을 확보하지 못해 런타임 자동화 경로에서 제외했다. 대량 실행은
MORAI Internal Scenario JSON, `/ScenarioLoad`, `/SaveSensorData` 기반으로
구현하고 Runner에서 확보한 지도와 예제는 생성기 참조 자료로만 사용한다.

## ROS 자동 로드·캡처 PoC

정교한 랜덤 시나리오를 대량 생성하기 전에 3개 장면으로
`ROS /ScenarioLoad → EgoNetwork sidecar 기반 rosbridge 복구
→ /SaveSensorData → 제한형 Ego 이동`을 검증한다.

- 실행 문서: `docs/ros_scenario_autocapture_poc.md`
- PoC manifest: `manifests/ros_capture_poc.json`
- 생성기: `scripts/generate_ros_capture_poc_set.py`
- 자동화 노드: `scripts/ros_scenario_autocapture_poc.py`
- 이동 모드는 각 장면에서 AV-ExternalCtrl과 `D` 기어를 명시한다.

첫 실행은 반드시 `p01`, stationary, 1장으로 저장 경로까지 검증한 뒤
저속 이동과 3개 장면 batch로 확장한다.

시나리오별 `EgoNetwork/<scenario_stem>_MN.json`은 대상 MORAI 버전에서
직접 저장한 ROS bridge network profile을 복제한다. 제어 batch runner의
대회용 UDP `configs/network/ego_network_1.json`과 혼용하지 않는다.

개인용 MORAI의 custom `file_dir` 캡처에서 3D LiDAR가 누락되므로,
`SaveFile/SensorData` 기본 경로에 전체 센서를 저장한 뒤 검증된 파일 묶음을
외부 `--capture-dir`로 재배치한다.

ROS Synchronous Mode는 개인용 런처에서 `frame=0`,
`can_send_tick=false` 교착과 `/SyncModeCmd` 무응답이 재현되어 현재 수집
경로에서 제외한다. 이 시험에는 Scenario Runner를 사용하지 않았다.
기본 대량 수집은 일반 `/ScenarioLoad` → 제한형 저속 주행 →
`rolling-grouped` 연속 캡처 방식을 사용한다. 상세 재현 결과는
`docs/ros_scenario_autocapture_poc.md`의 “Synchronous Mode PoC 결과와
제외 결정”에 기록한다.

처리량 탐색용으로 `rolling-safe`와 `rolling-fast`, 실제 수집용으로
`rolling-grouped` 모드를 제공한다.
전자는 정차 없이 캡처별 파일 완료를 기다리고, 후자는 1 m/s 주행 중
0.6 m 간격 trigger를 겹쳐 보낸 뒤 마지막에 파일을 수거한다.
`rolling-fast`는 파일을 trigger별 학습 프레임으로 검증하지 않으므로
속도 실험에만 사용한다. `rolling-grouped`는 같은 연속 trigger 방식을
쓰되 마지막에 한 번만 파일 쓰기 완료를 기다리고, MORAI 공통 timestamp로
Camera 4대·LiDAR·GT를 프레임화한다. 누락 묶음은 학습에 섞지 않고 해당
장면을 실패 처리한다.

현재 초기 수집 정책은 3.6 km/h, 장면당 6프레임이다. 대기시간
`load-settle=0.5초`, `capture-stable=0.3초` 조합은 p01~p03 총
18프레임에서 필수 센서 파일 누락 없이 통과했고, route-follow 대비 전체
실행시간이 68.839초에서 34.666초로 줄었다. 다만 연속 재실행에서
MORAI·rosbridge의 제어 service와 `/Ego_topic`이 응답하지 않는 현상이
한 차례 발생했으므로, 자동화 노드는 service timeout과 Ego heartbeat로
무한 대기를 차단한다. 장면당 3~4프레임으로 줄이고 시나리오 수를 늘리는
전략은 초기 파인튜닝과 수집 안정성 검증 이후 비교한다.

수천 장 무인 수집에서는 시나리오 `_MN.json`에 Ego Network
`UNIQUEID=0`만 설치한다. 현재 route-follow/rolling에 필요한
ScenarioLoad·SaveSensorData·CtrlCmd·`/Service_MoraiEventCmd`가 모두 여기에
포함된다. Simulator Network `UNIQUEID=-100`은 `/ego_setting`·Sync Mode
시험용이며 sidecar에는 포함하지 않는다. 이 분리 구조로 p01~p03 각
6프레임 route-follow를 과거 세 차례, 복원 후 한 차례 성공했다.

`speed01`은 p01과 같은 경로 anchor에서 모든 actor를 제거한 7.2 km/h
전용 benchmark 장면이다. 기본 p01~p03 batch에서는 제외되며
`--scenario speed01`을 명시할 때만 실행된다.

Sunny 13시 고정 180-scenario 파인튜닝 smoke dataset은
`docs/sunny13_180_smoke_capture.md`를 따른다. split은 scenario 단위
126/27/27이며, 개별 실패는 기록하고 다음 장면으로 진행한다.

연속 로드 안정성만 빠르게 확인할 때는 생성기에 `--stress-count 20`을
주어 `manifests/ros_capture_stress.json`과 `stress001~stress020`을 만든다.
이 묶음은 동일한 검증 anchor에서 seed만 바꾸고, stationary 모드로 장면당
1장만 저장한다. 객체 배치나 학습 품질을 평가하는 데이터가 아니라
ScenarioLoad·network 복구·센서 저장의 짧은 batch 시험용이다.

최초 stress20 실행은 16장면·452파일을 누락 없이 저장한 뒤 17번째
시나리오의 장애물 수 불일치로 중단됐다. rosbridge network는 17번째에도
복구됐다. 대량 실행에서는 `--continue-on-scene-error`로 개별 장면 실패를
report에 남기고 다음 장면을 계속 처리한다.

180-run PointPainting/향후 BEV 공용 파일럿의 sampling·split·환경·센서
설계는 `docs/od_180_scenario_generation_plan.md`, 기계 판독 설정 초안은
`configs/od_180_pilot.yaml`을 사용한다. 임시 수집 속도는 3.6 km/h다.

MORAI Scenario Edit에서 확인하고 사람 결합형 obstacle을 제외한 109개
고유 자산의 DataID·표시명과 1차
ontology는 `configs/morai_asset_catalog_260807.json`에 정리했다. 특히
시나리오의 `pedestrianList`·`objectList`를 학습 class와 동일시하지 않는다.
스쿠터·킥보드 탑승자와 유모차를 미는 사람 같은 결합형 obstacle은 자동
배치에서 제외하고, 사람이 없는 자전거·스쿠터는 일반 obstacle로 배치한다.
`pedestrianList`의 자전거 탑승자와 사람+동물 복합 자산은 실제
Semantic/Instance/3D box를 확인하기 전까지 격리한다. 근거와 audit 절차는
`docs/morai_asset_catalog_260807.md`를 따른다.

30개 base scenario를 여섯 날씨·시간 조건에서 반복하는 OD180 파일럿은
`scripts/generate_od180_pilot_set.py`로 생성한다. 환경별 캡처 폴더,
base 단위 split, 1,080 sample handoff index 생성 방법은
`docs/od180_capture_and_handoff.md`를 따른다.
