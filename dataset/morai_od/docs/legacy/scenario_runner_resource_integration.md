# Scenario Runner 리소스를 OD 시나리오 생성기에 활용하는 방법

## 1. 리소스의 역할과 출처

개인용 MORAI 설치본의 Scenario Runner v1.7.0에서 다음 자료를 로컬로
복사했다.

| 자료 | 저장 위치 | Git | 용도 |
|---|---|---:|---|
| K-City 2025 MGeo | `kcity_map/mgeo_map_scenario_runner_2025/` | 제외 | 링크·차선·횡단보도·주차면 기반 배치 |
| 선별 XOSC 예제 6개 | `dataset/morai_od/local_reference/scenario_runner_v1.7.0/xosc_examples/` | 제외 | XOSC 출력기와 Runner 연동 참고 |
| 모델명 카탈로그 | `dataset/morai_od/configs/scenario_runner_asset_catalog.yaml` | 추적 | 생성 가능한 객체 후보 |

원본 경로는 다음과 같다.

```text
~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/ScenarioRunner/
  scenario_runner_v1.7.0_linux/data/openscenario/
├── R_KR_PR_K-city_2025/MGeo/
└── R_KR_PG_KATRI/*.xosc
```

MORAI 원본 파일은 설치 버전과 라이선스에 종속되므로 저장소에는 올리지
않는다. 팀원은 같은 설치본에서 복사하거나 팀 공유 스토리지에서 받는다.

XOSC는 `2016_Hyundai_Ioniq`, `Man1`, `WoodBox` 같은 문자열 모델명을
사용하지만, 현재 Internal Scenario JSON 생성기는 `DataID` 숫자를 사용한다.
두 식별자는 그대로 치환할 수 없다. 카탈로그에는 양쪽에서 관찰한 값을
분리해 기록했으며, XOSC 출력기를 만들 때는 문자열 모델명을, Internal JSON
출력기는 검증된 `DataID` 또는 원본 actor clone을 사용한다.

## 2. 기존 MGeo와 풍부한 MGeo의 차이

두 지도 모두 UTM zone 52, WGS84이고 `local_origin_in_global`은
`[302595, 4124145, 0]`으로 같다. 하지만 저장 시점과 링크 구성은 다르다.

| 항목 | `kcity_map/mgeo_map` | Scenario Runner K-City 2025 |
|---|---:|---:|
| 링크 | 634 | 842 |
| 기본 파일 | 3개 | 19개 |
| 차선 경계 | 없음 | 1,245 |
| 단일 횡단보도 | 없음 | 100 |
| 주차면 | 없음 | 31 |
| 도로 레코드 | 없음 | 514 |
| 고정 객체 레코드 | 없음 | 385 |
| lane-change link | 비포함 | 포함 |

Scenario Runner 지도는 배치 제약을 풍부하게 만들기 위한 **참조 지도**다.
대회 런처와의 최종 호환성 기준은 대회 제공 지도와 실제 시뮬레이터 로드
결과다. 두 지도의 링크 ID나 형상이 다르면 참조 지도만 믿고 시나리오 링크를
기록하지 않는다.

또한 두 `global_info.json`의 `workspace_origin`은 다르다. 좌표 변환 시
`workspace_origin`을 서로 바꾸어 쓰지 말고, 시나리오를 실제 로드해 Ego와
객체 위치를 확인한다.

## 3. LLM-made Python 생성기에서 사용할 정보

### 차량

- `link_set.json`: 링크 중심선, 진행 방향, 제한 속도, 도로 ID
- `lane_boundary_set.json`: 차량 footprint가 차선을 벗어나지 않는지 검사
- `road_set.json`, `junction_set.json`: 교차로 밀집 장면의 비율 제한
- `parking_space_set.json`: 정차·주차 차량 후보 위치

차량 yaw는 선택한 링크 접선 방향으로 맞춘다. 역방향 차량이 필요한 별도
hard-negative 시나리오가 아니라면 180도 반전을 금지한다.

### 보행자

- `singlecrosswalk_set.json`: 횡단 보행자의 1순위 위치
- `crosswalk_set.json`: 연관된 횡단보도와 신호 정보
- `lane_boundary_set.json`: 보도 쪽 대기 보행자 위치의 근사 경계

현재 생성기의 “링크 중심에서 횡방향 offset” 방식은 프로토타입일 뿐이다.
다음 구현에서는 횡단보도 polygon 내부 또는 보도 측 경계에만 보행자를
생성하고, 차량 차로 한가운데 정지한 보행자는 별도 위험 장면으로 분류한다.

### 장애물과 충돌 검사

- `object_set.json`, `surface_marking_set.json`: 고정 시설물과 금지 영역 참고
- 모든 동적 객체에 2D footprint를 만들고 일정 margin을 더한 뒤
  polygon 교차를 검사
- Ego, NPC, 보행자, 장애물 사이 최소 거리를 클래스별로 적용
- 겹침·도로 이탈·센서 반경 이탈 후보는 rejection sampling으로 폐기

LLM은 제약과 코드 구조를 만드는 데 사용하되, 최종 좌표의 유효성 판단은
수치 기하 검사와 MORAI 로드 검사로 수행한다.

## 4. 현재 생성기를 풍부한 MGeo로 실행

날씨와 시간은 MORAI GUI에서 수동 설정하므로 아래 인자는 manifest용
레이블로만 기록한다.

```bash
# 풍부한 Scenario Runner K-City 2025 MGeo를 사용해 프로토타입 생성
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_morai_od_prototype.py \
  --mgeo-dir kcity_map/mgeo_map_scenario_runner_2025 \
  --weather sunny \
  --time-hour 13
```

현재 코드는 이 중 `link_set.json`만 직접 읽는다. 파일을 바꿔 실행할 수는
있지만, 차선 경계·횡단보도·주차면 기반 제약은 후속 구현 대상이다.

## 5. 선별한 XOSC 예제에서 얻는 정보

| 예제 | 얻는 정보 |
|---|---|
| `Scenario_Double_Lane_Changer_2.xosc` | parameter declaration, 차량 모델 치환, `LinkPosition` |
| `...DeterministicParameterSet.xosc` | 정해진 모델·속도·링크 조합 반복 |
| `...StochasticParameterSet.xosc` | seed와 run 수, uniform/normal/Poisson/histogram 분포 |
| `Scenario_Pedestrian_1.xosc` | `Pedestrian`, `WorldPosition`, 보행 동작 |
| `Scenario_Obstacle_1.xosc` | `MiscObject`, WoodBox/CargoBox 배치 |
| `Scenario_Spawn_Multiple_Vehicle_1.xosc` | `TrafficSourceAction` 기반 다중 차량 spawn |

중요한 한계도 있다.

- Parameter Distribution은 기존 XOSC의 값은 바꾸지만 XML 구조와 객체 수를
  자유롭게 바꾸는 제약 해결기가 아니다.
- 예제의 `TrafficSourceAction`은 교통 흐름 생성에는 유용하지만, Ego 반경
  20 m 안의 정확한 객체 수·거리·비중을 보장하지 않는다.
- Runner 문서에 완전한 겹침 방지나 클래스별 도로 의미 제약 해결 기능은
  확인되지 않았다.
- 따라서 객체 수, 위치, 방향, 겹침은 우리 Python 생성기가 확정하고,
  Runner에는 명시적인 한 장면을 전달하는 편이 재현성이 높다.

## 6. Script only와 Scenario Runner 병행 비교

날씨와 시간은 사람이 MORAI에서 바꾸므로 이 기능은 비교 기준에서 제외한다.

| 기준 | Script only | Python 생성기 + Scenario Runner |
|---|---|---|
| 객체 수·거리·방향 제약 | 직접 완전 제어 | 생성기가 완전 제어 |
| 겹침 rejection | 직접 구현 | 생성기가 직접 구현 |
| 현재 Internal JSON 재사용 | 가장 빠름 | XOSC 출력기 추가 필요 |
| 표준 시나리오 형식 | MORAI 종속 | OpenSCENARIO 사용 |
| 대량 순차 실행·초기화 | 별도 ROS orchestration 필요 | Runner batch/sync 활용 가능 |
| 실행 결과·실패 수집 | 직접 구현 | Runner callback/API 활용 가능 |
| `/SaveSensorData` 캡처 | 별도 publisher 필요 | 여전히 별도 publisher 필요 |
| 초기 구현 난이도 | 낮음 | 중간 |
| 3천 장면 이상 확장성 | orchestration 부담 | 더 유리 |

## 7. 최종 결정

초기 검토에서는 **우리 Python 생성기 + Scenario Runner 실행기**의 혼합
구조를 권장했다. 그러나 외부 Python API의 재현 가능한 확보와 26.R1
호환성 검증에 실패하여, 2026-08-06 기준 생산 파이프라인에서는 Scenario
Runner 실행기를 제외한다.

```text
공식 경로 + 풍부한 MGeo
        ↓
Python 제약 sampler / 겹침 검사
        ↓
MORAI Internal Scenario JSON + seed/manifest
        ↓
/ScenarioLoad 기반 순차 실행·검증·retry
        ↓
/SaveSensorData 캡처 orchestrator
```

현재 결정의 이유는 다음과 같다.

1. 데이터 품질을 좌우하는 map-aware 제약과 split 재현성은 우리 코드가
   가져야 한다.
2. 설치된 Runner v1.7.0에는 외부에서 import 가능한 SDK가 없었고, 공개된
   22.R3 API는 현재 공식 경로에서 Python 3.7용 SourceDefender 런타임을
   확보할 수 없었다.
3. 구형 API를 실행하더라도 26.R1 SIM과 K-City 2025의 완전한 호환성이
   보장되지 않는다.
4. 카메라·LiDAR Ground Truth 캡처는 Runner를 사용해도
   `/SaveSensorData` publisher를 별도로 연결해야 한다.
5. 현재의 정적·단순 이동 OD 데이터 생성은 Native Scenario JSON으로
   표현할 수 있어 추가 의존성의 이점이 작다.

Runner에서 확보한 MGeo, 모델명과 XOSC 예제는 계속 참조 자료로 사용한다.
도입 과정과 중단 근거, 재검토 조건은
`scenario_runner_adoption_review.md`에 상세히 기록했다.
