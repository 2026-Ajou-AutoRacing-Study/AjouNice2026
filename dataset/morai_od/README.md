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
기본 대량 수집은 검증된 일반 `/ScenarioLoad` → 제한형 `route-follow`
→ 정차 → `/SaveSensorData` 방식을 사용한다. 상세 재현 결과는
`docs/ros_scenario_autocapture_poc.md`의 “Synchronous Mode PoC 결과와
제외 결정”에 기록한다.
