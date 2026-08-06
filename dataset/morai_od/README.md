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
├── schemas/       # manifest 등 파일 형식
├── scenarios/     # Git으로 관리하는 MORAI 시나리오 JSON
├── manifests/     # 샘플 목록과 split
├── reports/       # 생성·검증 보고서
└── metadata/      # seed, 환경 조건, 입력 버전
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
