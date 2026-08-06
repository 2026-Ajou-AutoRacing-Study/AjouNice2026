# MORAI Scenario Runner 도입 검토와 중단 결정

## 1. 문서 목적

이 문서는 MORAI OD 데이터셋 생산 파이프라인에 Scenario Runner를 도입하려
했던 이유, 실제로 확인한 기능, 자동화 검증이 막힌 원인과 최종 전환 결정을
팀에 공유하기 위한 기록이다.

- 결정일: 2026-08-06
- 결론: **Scenario Runner를 런타임 자동화 경로에서 제외한다.**
- 유지 사항: Runner에서 확보한 K-City 2025 MGeo, XOSC 예제와 모델명
  카탈로그는 생성기용 참조 자료로 계속 사용한다.

여기서 “중단”은 Scenario Runner GUI 자체가 동작하지 않는다는 뜻이 아니다.
GUI와 수동 Batch Simulation은 동작하지만, 우리에게 필요한 대량 무인 실행을
현재 배포 조건에서 안정적으로 구현할 수 없다고 판단한 것이다.

## 2. 원래 해결하려던 문제

목표는 대회 공식 경로의 여러 Ego 위치에서 NPC, 보행자와 장애물을 제약에
맞게 무작위 배치하고, 카메라·LiDAR·Ground Truth를 수만 프레임 규모로
수집하는 것이다.

대략적인 생산 단위는 다음과 같다.

```text
Ego anchor 약 100개
× 날씨 2종
× 시간 3종
× anchor/조건별 여러 객체 배치
× 장면별 여러 프레임
```

우리 Python 생성기는 객체 수, 거리, 진행 방향, 도로 영역과 객체 간 겹침을
직접 통제한다. 생성된 장면을 수백~수천 개 순서대로 로드하고 캡처하려면
별도의 실행 orchestration이 필요하다.

## 3. ROS 기반 방식과 비교해 Runner를 검토한 이유

### ROS 기반 기본안

```text
MGeo + 공식 경로
  → Python 제약 sampler
  → MORAI Internal Scenario JSON
  → /ScenarioLoad
  → /SaveSensorData 반복
```

이 방식은 현재 사용하는 MORAI JSON과 ROS 메시지를 그대로 재사용하므로
구현이 단순하다. 반면 다음 기능은 우리가 직접 구현해야 한다.

- 시나리오 로드 완료 판정
- timeout, retry와 실패 격리
- 장면별 안정화 대기
- 캡처 횟수 및 센서별 파일 수 검증
- 중단 후 재개 checkpoint
- 시나리오 초기화와 다음 장면 전환
- 실행 결과와 manifest 연결

### Scenario Runner에서 기대한 차별점

Scenario Runner는 OpenSCENARIO(XOSC)를 읽고 MORAI SIM과 gRPC로 통신한다.
다음 기능을 이미 제공하므로 실행 계층을 재사용할 수 있을 것으로 기대했다.

- 표준화된 XOSC 시나리오 표현
- 여러 시나리오 Batch Simulation
- 반복 횟수 설정
- start/stop/skip과 callback
- synchronous mode
- Story/Act/Event/Trigger 기반 동적 객체 행동
- 성공·실패 조건과 상태 조회
- 날씨와 시간 설정 API

따라서 최초 검토안은 다음과 같았다.

```text
우리 Python 생성기
  → map-aware 배치 및 충돌 검사
  → 명시적 actor가 포함된 XOSC
  → Scenario Runner Python API로 batch/reset
  → ROS /SaveSensorData로 센서 캡처
```

Runner의 랜덤 배치 기능에 데이터 품질을 맡기려던 것은 아니다. 배치 제약은
우리 생성기가 담당하고, Runner는 대량 실행과 상태 관리를 담당하는
혼합 구조였다.

## 4. 실제 확인한 사항

### 최신 Runner

개인용 MORAI Launcher에서 다음 구성을 확인했다.

- MORAI SIM: Drive 26.R1
- Scenario Runner v1.7.0
- Linux 실행 가능
- GUI에서 XOSC 로드와 Batch Simulation 기능 제공
- K-City 2025 MGeo 포함
- 최신 문서에 `OpenScenarioClientAPI` 명세 존재

Runner v1.7.0은 2026년 릴리스이므로 Scenario Runner가 2022년에 개발
중단된 것은 아니다.

### 외부 Python API

최신 공식 문서에는 다음 API가 설명돼 있다.

- `is_connected()`
- `get_simulator_version()`
- `set_weather()`
- `set_time_of_day()`
- `set_scenario_config()`
- `start_batch_scenario()`
- `stop_batch_scenario()`
- 객체·보행자·차량 상태 조회

하지만 설치된 Runner는 PyInstaller로 패키징된 GUI 실행파일이었고, 외부
Python 프로그램에서 import할 수 있는 SDK 파일은 찾지 못했다.

### 공개된 구형 API 검토

공식 구버전 문서에서 `OpenSCENARIO_API_22.R3.zip`을 확보하여 확인했다.

- Python 3.7.3 고정
- `localhost:7789` gRPC 통신
- SourceDefender로 암호화된 `.pye` 라이브러리
- 2022년 K-City MGeo와 XOSC 예제 포함

Python 3.7.3 Conda 환경은 정상 생성했다. 그러나 현재 PyPI에서 설치 가능한
SourceDefender는 Python 3.10 이상을 요구하며, 실제 최신 배포물에도
CPython 3.7 Linux 로더가 없다. 과거 Python 3.7용 배포본은 공식 설치
목록에서 제거돼 22.R3 암호화 라이브러리를 정상적인 공식 경로로 import할
수 없었다.

또한 22.R3 API가 26.R1 SIM의 최신 gRPC 규격, K-City 2025 MGeo와 완전히
호환된다는 보장도 없다.

## 5. 실패 원인

실패 지점은 Scenario Runner GUI가 아니라 **외부 headless Python API의
재현 가능한 확보와 호환성 검증**이다.

1. Runner v1.7.0 설치본에 외부용 Python SDK가 노출되지 않았다.
2. 공개 다운로드 가능한 API는 22.R3용 구형 패키지였다.
3. 22.R3 패키지는 정확한 Python 3.7.3과 당시 SourceDefender 런타임을
   요구한다.
4. Python 3.7용 SourceDefender 런타임을 현재 공식 배포 경로에서 구할 수
   없었다.
5. 비공식 보관 파일을 실행하는 것은 공급망 및 라이선스 위험이 있다.
6. 런타임을 확보해도 22.R3 API와 26.R1 SIM 사이의 gRPC·지도·모델
   호환성을 별도로 검증해야 한다.
7. Runner를 사용하더라도 카메라·LiDAR Ground Truth 저장은 결국
   `/SaveSensorData`와 파일 검증 계층을 따로 구현해야 한다.

즉, 추가 비용과 불확실성에 비해 현재 정적 OD 데이터 생성에서 얻는 이점이
충분하지 않았다.

## 6. Runner에서 확보하여 유지하는 자산

Runner 실행 자동화는 중단하지만 다음 정적 자료는 유용하므로 유지한다.

| 자료 | 위치 | 활용 |
|---|---|---|
| K-City 2025 MGeo | `kcity_map/mgeo_map_scenario_runner_2025/` | 링크·차선·횡단보도·주차면 기반 배치 |
| 선별 XOSC 예제 | `local_reference/scenario_runner_v1.7.0/xosc_examples/` | 객체/분포/동작 구조 참고 |
| 모델명 카탈로그 | `configs/scenario_runner_asset_catalog.yaml` | 차량·보행자·장애물 후보 이름 |
| 공식 Runner 문서 | `docs/jsw/morai_docs/` | 향후 동적 시나리오 설계 참고 |

Runner의 53개 XOSC에서 확인한 주요 정보는 다음과 같다.

- 차량 모델명 7종
- 보행자 모델명 3종
- 장애물 모델명 3종
- deterministic/stochastic parameter distribution
- 차량 spawn, 보행, trajectory, 차선 변경, 객체 삭제
- 교통신호, TTC trigger, EnvironmentAction

반면 전체 MORAI asset 목록, XOSC 모델명과 Internal JSON `DataID`의 전체
매핑, 객체 실제 크기와 Native Scenario JSON 스키마는 얻을 수 없었다.

## 7. 최종 채택 구조

대량 생산 파이프라인은 ROS 기반으로 구현한다.

```text
공식 waypoint + K-City 2025 MGeo
        ↓
Python map-aware sampler
        ↓
footprint 충돌 검사 및 rejection
        ↓
MORAI Internal Scenario JSON + manifest
        ↓
/ScenarioLoad
        ↓
/Ego_topic + /Object_topic 기반 로드 완료 검증
        ↓
/SaveSensorData 반복
        ↓
센서별 파일 수·GT 유효성 검사
        ↓
성공 기록 또는 retry
```

자동화기는 최소한 다음 상태를 저장해야 한다.

- scenario ID와 seed
- 예상/관측 객체 수
- 로드 및 안정화 timeout
- 캡처 요청 횟수
- 센서별 생성 파일 수
- 성공/실패 사유와 재시도 횟수
- 마지막 완료 scenario ID

이 구조는 MORAI 종속성이 있지만 현재 환경에서 검증 가능한 인터페이스만
사용하며, 1,000개 이상의 시나리오도 중단 후 재개 가능한 상태 기계로
처리할 수 있다.

## 8. 향후 재검토 조건

다음 중 하나가 충족될 때만 Scenario Runner 자동화를 다시 검토한다.

1. MORAI가 Scenario Runner 1.7.0/26.R1용 외부 Python SDK를 제공한다.
2. 현재 SIM과 동일한 버전에서 동작하는 공식 API 예제와 의존성 lock이
   제공된다.
3. TTC, 상호작용, 복합 Trigger 등 Native JSON으로 표현하기 어려운 동적
   시나리오가 데이터 품질에 필수적인 단계가 된다.

그 전까지 Runner 관련 자료는 참조 자산으로만 사용하며 생산 파이프라인의
필수 의존성으로 만들지 않는다.
