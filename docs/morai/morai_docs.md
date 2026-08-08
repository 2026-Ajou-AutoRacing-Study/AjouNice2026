# MORAI local documentation index

2026-08-06에 로컬로 보존한 MORAI 문서를 세 종류로 분리해 단일 Markdown
문서로 변환한 인덱스다. 기존
`docs/external_docs/MORAI_26R1_Documentation_combined.md`는 그대로 유지하며,
여기의 26.R1 문서는 별도 변환 결과다.

## 중요: 대회 런처와 문서의 버전 차이

- 이 폴더의 주 공식 문서는 **2026년 MORAI SIM: Drive 26.R1** 기준이다.
- 실제 대회 런처는 **2025년 빌드**다.
- 대회 런처의 기능은 2024년 문서와 2026년 문서 사이에 해당할 수 있다.
- Scenario Runner 문서는 2026-08-06에 수집했으며, 개인용 런처에는
  Scenario Runner v1.7.0이 설치되어 있다.
- SIM API Guide는 정확한 적용 버전이 불명확하며 구버전일 가능성이 있다.

따라서 26.R1 문서는 기능을 찾고 설계 가능성을 판단하는 **참고 자료**이지,
대회 런처의 지원 기능을 보장하는 명세가 아니다. 토픽 존재 여부, 메시지 필드,
UI 항목, Capture/Replay/Scenario Runner 지원 여부는 반드시 2025 대회
런처에서 다시 확인한다.

## 문서

1. [MORAI SIM: Drive 26.R1](MORAI_SIM_Drive_26R1_single.md)
   - 일반 MORAI 기능, 센서, 네트워크, ROS/UDP, Scenario Edit, Capture 등을
     확인할 때 가장 먼저 사용한다.
2. [MORAI Scenario Runner (KR)](MORAI_Scenario_Runner_KR_single.md)
   - OpenSCENARIO `.xosc`, Scenario Runner 설치·실행, 지원 범위, Python API,
     예제와 변환 방법을 확인할 때 사용한다.
3. [MORAI SIM API Guide](MORAI_SIM_API_Guide_legacy_single.md)
   - Simulator Control API, OpenSCENARIO API와 gRPC API 보조 자료다.
   - 현재 SIM/Runner와 버전이 다를 가능성이 있으므로 26.R1 문서 및 실제
     설치 파일과 교차 검증한다.

## 답변 시 확인 순서와 최종 기준

기능을 조사할 때는 다음 순서로 문서를 찾는다.

1. 로컬 26.R1 공식 문서
2. 로컬 Scenario Runner 공식 문서
3. 구버전 가능성이 있는 SIM API Guide
4. 필요한 경우 온라인 공식 MORAI 문서

최종 판단의 우선순위는 다음과 같다.

1. 2025 대회 런처의 실제 실행·관측 결과
2. 대회 프로젝트에 설치된 `morai_msgs`, UDP/ROS 코드와 설정
3. 문서에 명시된 버전이 대회 런처와 일치하는 자료
4. 2026년 26.R1 공식 문서
5. 버전이 불명확한 SIM API Guide

문서와 실제 대회용 SIM 동작이 다르면 대회 런처의 관측 결과를 우선하고,
어느 버전의 근거인지 답변에 명시한다.

## 현재 설치된 Scenario Runner

개인용 런처에서 다음 설치를 확인했다.

```text
/home/luv/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/ScenarioRunner/
└── scenario_runner_v1.7.0_linux/
    ├── runner
    ├── program/Scenario Runner
    └── data/openscenario/
        └── R_KR_PR_K-city_2025/
```

따라서 개인용 런처에는 Scenario Runner v1.7.0과 K-City 2025용
OpenSCENARIO 데이터 경로가 존재한다. 설치 여부와 실제 사용 가능 여부는
구분해야 한다. 다음 단계에서 GUI 실행, SIM 연결, 제공 예제 `.xosc` 실행,
현재 K-City 맵·차량 asset 호환성을 차례로 검증해야 한다.

## 재생성

로컬 HTML 또는 HTTrack archive를 갱신한 뒤 다음 명령으로 세 문서를 다시
만든다.

```bash
# 세 MORAI HTML 문서군을 각각 단일 Markdown으로 재생성
cd ~/AjouNice2026

python3 docs/jsw/morai_docs/build_single_docs.py
```
