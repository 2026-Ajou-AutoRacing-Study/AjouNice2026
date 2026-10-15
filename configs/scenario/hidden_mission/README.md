# GPS 음영 구간 히든 미션 시나리오

`hid_start_pos.json`의 출발 자세를 유지하고, K-City 2025 MGeo 중심선
`A2256W000446 → A2256W000448 → A2256W000126`을 따라 만든 MORAI SaveFile
모음이다. 기존 `hid_start_pos.json`과 `hid1.json`은 수정하지 않는다.

## 구성

| 대분류 | 폴더 | 개수 | 변화 조건 |
|---|---|---:|---|
| 정적 장애물 2개 | `static_2/` | 14 | 차량·보행자·자전거·스쿠터·일반 장애물, 간격 6–20 m, 횡방향 stagger |
| 정적 장애물 3개 | `static_3/` | 14 | 동종/혼합 조합, 간격 5.5–22 m, 조밀/광폭/stagger |
| 동적 장애물 1개 | `dynamic_1/` | 14 | 좌→우/우→좌, 보행자·아동·작업자·경찰·자전거·스쿠터류, 속도/작동거리/경로 길이 변화 |

총 42개이며, 상세 DataID·좌표·간격·속도 설정은 `manifest.json`에 기록된다.
정적 장애물은 출발점 전방 38 m부터 GPS 음영영역 안에 놓았고, 원래 차로를
막아 반대 차선 회피 후 복귀하는 궤적을 확인하도록 구성했다.

## 권장 실행 순서

1. `static_2/s2_01_sedan_suv_gap08.json`으로 차량 회피 기본 동작 확인
2. `static_2` 전체 실행
3. `static_3`에서 조밀(`dense`, `tight`) → 일반 → 광폭(`wide`) 순으로 실행
4. `dynamic_1`에서 보행자 기본 2개를 먼저 실행한 뒤 이륜차/스쿠터류 실행

MORAI에서 SaveFile을 불러온 뒤 매번 AV ExternalCtrl 연결과 Ego 제어 반응을
다시 확인해야 한다. 동적 객체의 `speed`/`moveSpeed` 단위와 결합형 이륜차의
모델·애니메이션은 MORAI 버전에 따라 다를 수 있으므로 첫 import에서 화면상
속도와 진행 방향을 확인한다. `activeDistance`는 Ego 접근 시 작동거리다.

## 재생성 및 무결성 검사

```bash
python3 configs/scenario/hidden_mission/generate_hidden_mission_scenarios.py
python3 configs/scenario/hidden_mission/generate_hidden_mission_scenarios.py --check
```

테스트 결과는 최소한 인지 → 추적 → 플래닝 반영 → 제어 → 최소 이격거리 →
충돌 여부 순서로 남긴다. 시나리오 로드만으로 실제 회피 성공이 보장되지는 않는다.
