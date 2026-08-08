# MORAI Scenario Runner (KR) — single document

> 로컬에 저장된 MORAI 공식 HTML을 검색하기 쉬운 단일 Markdown으로 기계 변환한 문서다. 표·코드·목록은 최대한 보존했지만, 정확한 UI 배치와 이미지는 원본 HTML 또는 온라인 공식 문서를 함께 확인한다.

> **프로젝트 버전 주의:** 이 자료의 주 문서는 2026년 26.R1 기준이지만 대회 런처는 2025년 빌드다. 대회 런처는 2024~2026 문서 사이의 기능 상태일 수 있으므로, 여기의 기능·토픽·필드를 그대로 지원한다고 가정하지 않는다. 실제 대회 런처에서 관측한 UI, 토픽, 메시지 정의와 실행 결과를 최종 기준으로 삼는다.

- 범위: Scenario Runner, OpenSCENARIO, Python API와 예제 문서
- 기준 URL: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/
- 포함 페이지: 21개
- 로컬 변환일: 2026-08-06

## 포함 페이지

- Release Notes
- 공식 버전
- 베타 버전
- 개요
- 설치 및 실행
- Quick Start Guide
- NPC 차량 Cut-in 시나리오 실행
- 시뮬레이션 수행
- 시나리오 열기/저장
- .xosc 시나리오 파일 구조 (OpenSCENARIO 기반 포맷)
- Example Scenarios
- 시나리오 생성
- 시나리오 편집
- OpenSCENARIO Support
- MORAI에서 정의한 OpenSCENARIO 요소
- MORAI Custom → OSC v1.2 Converting
- 시뮬레이션 결과 확인
- Scenario Runner Clients
- Introduction
- API Documentation
- Python API

---

## Release Notes

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/release-notes

**MORAI**[**Scenario Runner**](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/index.html)**제품의 공식 및 베타 버전에 대한 릴리스 노트입니다.**

---

© 2023 MORAI Inc.

×


---

## 공식 버전

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-7

[data-colorid=d0h8kd00wr]{color:#ff5630} html[data-color-mode=dark] [data-colorid=d0h8kd00wr]{color:#cf2600}[data-colorid=sr73vmpz2e]{color:#ff5630} html[data-color-mode=dark] [data-colorid=sr73vmpz2e]{color:#cf2600}

시나리오 러너의 공식 버전에 반영된 변경 사항을 최신 배포 일자 순서로 설명합니다.

| **배포 일자** | **Version** | **운영체제** | **변경 사항** | **MORAI SIM: Drive 호환** **버전** |
| --- | --- | --- | --- | --- |
| 2026-04-08 | 1.7.0 | WindowsUbuntu 20.04 / 22.04 | 이번 릴리즈는 OpenSCENARIO 지원 확장과 Agent 주행 안정성 개선 및 제품의 성능 개선이 이루어졌습니다.**New Features****Trajectory**shape 옵션 확장 (Clothoid, Nurbs)**Vehicle**Kinematics / Dynamics Model 선택 기능 추가**Position**Lane Position 지원 확장Link Position offset 속성 추가 (MORAI Custom)**Controller**Controller Property 기반 주행 성향 제어 가능**TrafficSignalController**RoadNetwork 에 정의된 TrafficSignals 를 이용해 TrafficSignalControllerAction 을 사용 가능**Scenario** MORAI Custom → OpenSCENARIO v1.2 변환 기능 추가![(info)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/info_32.png) MORAI 에서만 사용하는 Custom 표준을 v1.2 표준으로 변환 가능하다.매뉴얼 내 “**시나리오 편집/MORAI Custom → OSC v1.2 Converting**” 페이지 참조**Improvements****Action**AssignRouteAction첫 waypoint를 기준으로 route planning 되도록 개선**Position**WorldPositionLink 에 맞춰 World Position 지정시 도로의 진행 방향에 맞춰 Heading 값 자동 반영**Trajectory**Trajectory Action 실행 시 지정된 Waypoint 를 추종하지 못하는 오류 수정**Bug Fixes****Action**Lane Offset Action 완료 처리 오류 수정Speed Action (rate) 값 할당 방식 수정 (Min/Max 적용)Lane Change Action 이 간혈적으로 동작하지 않는 오류 수정Routing Action 의 After Completion 이 hide 옵션이 동작하지 않는 오류 수정**Vehicle Agent Behavior**정지선 앞 과도하게 멀리 정지하는 현상 개선정지 시 Brake 떨림 현상 개선**장애물 인식**경로를 벗어난 객체 인식 문제 해결Pedestrian 오인식 문제 해결**Scenario**Obstacle 배치가 되지 않던 오류 수정 | 26.R1 |
| 2025-09-26 | 1.6.4 | WindowsUbuntu 20.04 / 22.04 / 24.04 | **지원 OS 추가**Ubuntu 22.04 와 Ubuntu 24.04 사용 가능하도록 개선**Improvements**좌측 패널 Private Data View에 entity 이름이 보이게 개선![image-20250926-013038.png](https://help-morai-sim.scrollhelp.site/__attachments/293109761/image-20250926-013038.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678) | 24.R2.H2 |
| 2025-08-25 | 1.6.3 | WindowsUbuntu 20.04Ubuntu 22.04 필요 시 별도 문의 | **New Features****시나리오 파일을 Drag & Drop 방식으로 Import 가능****ObjectController, AssignControllerAction, ActivateControllerAction 기능 일부 지원**Cotnroller 의 Properties에서 아래 4가지 항목을 설정하여 운전 성향 조절 가능`max_acceleration``min_distance``headway_time``emergency_braking_distance`**Improvements****apply_acc 를 기능을 ActivateControllerAction 을 사용하도록 대체**기존 방식 사용 가능하지만 경고 메시지 출력향후 ActivateControllerAction 방식으로 완전히 전환 예정**Bug Fixes****Follow Trajectory Action 동작 오류 수정**Point 간격이 1 m 미만일 경우 동작하지 않던 문제 해결Route Action 주행 완료 시 간헐적으로 Scenario가 종료되던 문제 해결 | 24.R2.H2 |
| 2024-07-31 | 1.6.2 | WindowsUbuntu 20.04 | 배치 시뮬레이션 UX 개선배치 시뮬레이션 실행 시, 전체 실행 결과에 대한 로그 파일 생성전체 Test Suite 에서 성공/실패 시나리오 개수 등을 표시배치 시뮬레이션 창에서, ‘Delete’ 클릭 시 선택된 모든 시나리오 삭제Test Suite 실행 시, 팝업 창에서 ‘이어하기’ 여부를 묻지 않고 ‘새로 시작’ 여부를 확인함Event 활성화 시 ‘priority’ 옵션에 따라 기존 수행 중인 다른 Event 의 상태를 제어함Follow Trajectory Action 오류 수정‘Polyline’ 옵션 선택 시 정상적으로 직선 경로를 생성할 수 있음Waypoint 도달 처리 안정화시작 위치에서의 자세 값이 정상적으로 반영 됨Robot Entity 도 Ego Vehicle 로 활용할 수 있음프로그램 안정성 개선Bug FixSave 와 Save As 의 동작이 뒤바뀌었던 버그 수정외부 제어기를 활용한 배치 시뮬레이션에서 ‘Ego 초기화 무시’ 기능이 동작하지 않던 버그 수정Lane Change Action 안정화 | 24.R2.0 |
| 2023-12-01 | 1.6.1 | WindowsUbuntu 20.04 | NPC 차량의 ACC(Adaptive Cruise Control) Mode 에 종방향 운전자 모델인 Intelligent Driver Model 을 적용하여 고도화프로그램 안정성 개선 및 보안 이슈 해결Bug Fix서버로부터 맵, 차량 등 사용자 보유 옵션을 받아오지 못할 경우, 기본 옵션 리스트를 표출Lane Change Action 이 간헐적으로 반대로 동작하는 버그 수정State Log 에 동일한 데이터가 반복적으로 기록되지 않도록 수정Synchronize Action 의 속도 단위를 m/s로 입력 받도록 수정 | 23.R1.124.R1.0 |
| 2023-09-27 | 1.6.0 | WindowsLinux | OpenSCENARIO의 ParameterDistrubution Element를 활용한 OpenSCENARIO 파일 생성 기능 추가EuroNCAP AEB Scenario에 해당하는 OpenSCENARIO 파일 생성 기능 추가GlobalAction 수행 기능 추가Environment Action 수행Season, RoadFriction 지원TrafficAction > TrafficSwarmAction을 사용하여 주변에 NPC 차량을 생성 기능 지원PrIvateAction 기능 추가RoutingActionFollowTrajectoryAction보행자에 대해 선언 기능 지원UserDefinedAction 기능 추가Map Object ActionActivateMapObjectActionFaultInjectionActionAction 내에 대상 entity 정보 지원SensorFaultActionSensor 위치, Sensor 의 고장(data publish) 여부를 설정 기능 추가statelog에 Object, Pedestrian 상태 저장 지원grpc command를 사용한 제어 기능 지원특정 국가의 도로교통규정에 맞춰 주행한국우측 주행, 신호시스템, 비보호 좌회전/우회전일본좌측 주행, 신호시스템, 비보호 좌회전/우회전/유턴상태 정보 Publish 기능 추가Simulation Start/End, Batch Simulation Start/End 시나리오 정보TCP/IP 통신 지원GUI 기능 추가MGeo 파일을 로드 지원MGeo 파일 가시화 기능 추가카메라의 시점 변경 지원MGeo 요소 선택 지원드래그를 통한 요소 다중 선택Element 선택 알고리즘 개선 (mouse picking)File Dialog 주소 창의 내용을 편집하는 기능 지원Vehicle 모델명 입력시 대소문자 변경 지원시나리오가 실패할 경우 다음 시나리오를 이어서 진행하도록 변경 | 23.R1.0 |
| 2023-04-14 | 1.5.3 | WindowsLinux | 각도 단위 degree에서 radian으로 수정Random 경로 생성 시 교차로에서 무작위적으로 경로 선택이 되도록 수정OpenSCENARIO 지원 기능 확장 ActionFollowTrajectoryActionBatch Simulation 반복 기능 추가Sync mode 안정화NPC 제어 로직 안정화 | 22.R4.022.R4.122.R4.2 |
| 2023-02-10 | 1.5.2 | WindowsLinux | 차선 변경 로직 오류 수정차선 경로 생성 로직 오류 수정 | 22.R4.022.R4.1 |
| 2022-12-22 | 1.5.1 | WindowsLinux | 서버로부터 차량 리스트를 가져오지 못하는 오류 수정spawn in link position 옵션 수행 시 NPC가 link position 위에 spawn 되지 않는 오류 수정 | 22.R4.0 |
| 2022-12-21 | 1.5.0 | WindowsLinux | 시뮬레이션 시작 옵션 추가시뮬레이션 시 Synchronous 모드 추가Spawn in link position 옵션 추가WorldPosition의 Z축 투영 기능 추가OpenSCENARIO 지원 기능 확장ActionEnvironmentActionLightStateActionAddEntityAction TrafficStopAction FollowTrajectoryActionConditionCollisionConditionTraveledDistanceConditionTimeHeadwayConditionTrafficSignalControllerCondition MORAI SIM Controller 모드에서는 Ego 차량이 OpenSCENARIO 파일에 기술된대로 동작하지 않도록 수정 | 22.R4.0 |
| 2022-09-06 | 1.1.0 | WindowsLinux | Ego 차량 제어 모드 추가변경 전cruise mode: on (==MORAI SIM: Drive controller)cruise mode: off (==None)변경 후MORAI SIM: Drive controllerScenario Runner controllerExternal controller (== None)Lateral Offset 기능 구현특정 필드 값 수정 시 dropdown control 지원OpenSCENARIO v1.2 지원Tree를 통한 OpenSCENARIO Item 추가 기능 구현OpenSCENARIO 지원 기능 확장ActionEnvironmentActionConditionAccelerationConditionDistanceConditionOffroadConditionReachPositionConditionRelativeSpeedConditionSpeedConditionStandStillConditionTimeToCollisionConditionTimeOfDayConditionTrafficSignalConditionPositionOrientationRelativeWorldPositionRelativeObjectPosition배치 시뮬레이션 기능 구현시뮬레이션 결과 화면 구현NPC 차량이 도로 끝에 도달한 경우 멈추도록 수정 | 22.R3.0 |
| 2022-06-29 | [PM022059](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/windows/scenario_runner_220628_PM022059_win.zip) | WindowsLinux | Ego 차량 고장 시나리오 기능 구현Ego 차량 화재 시나리오 기능 구현신호등 제어 시나리오 기능 구현주차 시나리오 기능 구현Stop 표지판 시나리오 기능 구현Synchronize Action 기능 구현시뮬레이터의 Scenario Object 이름을 지정한 이름으로 변경하는 gRPC API 추가챠량 제원 정보를 조회하는 gRPC API 추가Ego 차량 설정 시 pause를 false로 주는 경우 차가 주행하지 않는 오류 수정OpenSCENARIO 파일 생성 기능 구현OpenSCENARIO 파일 편집 기능 일부 구현OpenSCENARIO 트리를 통한 값 수정 및 삭제Scenario Object 생성Event 생성OpenSCENARIO 파일 저장 기능 구현시나리오 성공 여부 판단 기능 구현NPC 차량이 신호등을 보고 주행하는 기능 구현NPC 차량 ACC 기능 구현 | 22.R2.022.R2.1 |
| [PM013958](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/linux/scenario_runner_220725_PM013958_linux.zip) |  |  |  |  |
| 2022-04-14 | [PM035446](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/windows/scenario_runner_220411_PM035446_win.zip) | WindowsLinux | **Map Editor에 Scenario Runner 병합**보행자 생성 기능 추가Ego Cruise 모드 시 경로 주행하는 기능 추가경로 주행 시 차량과 도착지 사이의 거리를 차량 정보에 포함시켜서 전달해주도록 수정GT 센서 기능 추가Scenario Runner의 MGeo 맵에 Scenario Object 위치 출력하는 기능 추가여러개의 Act, ManeuverGroup, Maneuver, Event 지원StoryboardElementStateCondition 지원장애물 생성 기능 추가차선 변경 시 LaneChangeActionDynamics에 따라 차선 변경 경로를 다양한 모양으로 생성할 수 있도록 하는 기능 추가속도 변경 시 SpeedActionDynamics에 따라 속도를 다양한 모양으로 변경할 수 있도록 하는 기능 추가차선 변경 시 Absolute Target Lane 지원Spawn Point 생성 기능 구현 | 22.R1.222.R1.3 |
| [PM032549](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/linux/scenario_runner_220511_PM032549_linux.zip) | **Map Editor에 Scenario Runner 병합**보행자 생성 기능 추가Ego Cruise 모드 시 경로 주행하는 기능 추가경로 주행 시 차량과 도착지 사이의 거리를 차량 정보에 포함시켜서 전달해주도록 수정GT 센서 기능 추가Scenario Runner의 MGeo 맵에 Scenario Object 위치 출력하는 기능 추가여러개의 Act, ManeuverGroup, Maneuver, Event 지원StoryboardElementStateCondition 지원장애물 생성 기능 추가차선 변경 시 LaneChangeActionDynamics에 따라 차선 변경 경로를 다양한 모양으로 생성할 수 있도록 하는 기능 추가속도 변경 시 SpeedActionDynamics에 따라 속도를 다양한 모양으로 변경할 수 있도록 하는 기능 추가차선 변경 시 Absolute Target Lane 지원Spawn Point 생성 기능 구현 |  |  |  |
| 2021-12-16 | [PM011031](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/linux/scenario_runner_211216_PM011031_linux.zip) | Linux | 시뮬레이션 시간 기반 Event Condition 기능 추가 | v4.7.211109.H3 |
| 2021-11-17 | [PM005516](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/windows/scenario_runner_211116_PM005516_win.zip) | Windows | v4.7.211109.1 v4.7.211109.H1 v4.7.211109.H2 v4.7.211109.H3 |  |
| 2021-11-09 | [PM035725](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/windows/scenario_runner_211108_PM035725_win.zip) | Windows | ID 중복 처리 기능 추가NPC ID 설정 시 string 지원 기능 추가Map 및 Simulator 시나리오 로드 기능 추가NPC 차량 경로 주행 기능 추가KCity Example 추가Euro NCAP Example 추가Action 기능 추가longitudinal acceleration 추가 | v4.7.211109.1 v4.7.211109.H1 v4.7.211109.H2 v4.7.211109.H3 |
| 2021-09-29 | [PM010900](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/release/windows/scenario_runner_210929_PM010900_win.zip) | Windows | NPC 차량 정차 시나리오 제공Ego 차량 앞 NPC 차량 긴급 제동 시나리오 제공Ego 차량 앞 NPC Cut-in 시나리오 제공 | v4.6.210907.H5 |

×


---

## 베타 버전

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-8

[data-colorid=bxooqd5vyh]{color:#ff5630} html[data-color-mode=dark] [data-colorid=bxooqd5vyh]{color:#cf2600}[data-colorid=oujm2gvwad]{color:#ff5630} html[data-color-mode=dark] [data-colorid=oujm2gvwad]{color:#cf2600}

시나리오 러너의 베타 버전에 반영된 변경 사항을 최신 배포 일자 순서로 설명합니다.

| **배포 일자** | **Version** | **운영체제** | **변경 사항** | **MORAI SIM: Driven 호환** **버전** |
| --- | --- | --- | --- | --- |
| 2022-10-25 | 1.3.0 | [Windows](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_1.3.0_win.ziphttps://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_1.3.0_win.zip) | Simulation 시 External Contoller를 선택한 경우 이전에 선택한 Ego 차량의 모드가 유지되도록 수정 | 220928.S3.Suresoft6 |
| [Linux](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/linux/scenario_runner_1.3.0_linux.zip) |  |  |  |  |
| 2022-04-29 | [PM052431](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_220429_PM052431_win.zip) | Windows | 수직 주차(전면, 후면) 기능 구현Ego 차량 화재 시나리오 기능 구현철길 시나리오 기능 구현Stop 표지판 시나리오 기능 구현시뮬레이터의 Scenario Object 이름을 지정한 이름으로 변경하는 기능 구현챠량 제원 정보를 조회하는 gRPC API 추가Ego 차량 설정 시 pause를 false로 주는 경우 차가 주행하지 않는 오류 수정 | 220429.S1.SureSoft1 |
| 2022-03-24 | [PM040317](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_220324_PM040317_win.zip) | Windows | 속도 변경 시 SpeedActionDynamics에 따라 속도를 다양한 모양으로 변경할 수 있도록 하는 기능 추가차선 변경 시 Absolute Target Lane 지원신호등 제어 기능 구현Spawn Point 생성 기능 구현Ego 차량 고장 시나리오 기능 구현차량 상태 정보에 주행 중인 Link와 연결된 신호등 정보를 포함하도록 수정주차 기능 일부 구현 (평행 주차) | 220323.S1.1 |
| 2022-03-16 | [PM050319](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/linux/scenario_runner_220316_PM050319_linux.zip) | Linux | 장애물 생성 기능 추가차선 변경 시 LaneChangeActionDynamics에 따라 차선 변경 경로를 다양한 모양으로 생성할 수 있도록 하는 기능 추가 | 220316.S1.AutowareFoundation |
| 2022-02-09 | [PM034656](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_220209_PM034656_win.zip) | Windows | 장애물 생성 기능 추가차선 변경 시 LaneChangeActionDynamics에 따라 차선 변경 경로를 다양한 모양으로 생성할 수 있도록 하는 기능 추가 | v.D1.220209.1 |
| 2022-01-14 | [PM111218](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/linux/scenario_runner_220114_AM111218_linux.zip) | Linux | **Map Editor에 Scenario Runner 병합**보행자 생성 기능 추가Ego Cruise 모드 시 경로 주행하는 기능 추가경로 주행 시 차량과 도착지 사이의 거리를 차량 정보에 포함시켜서 전달해주도록 수정GT 센서 기능 추가Scenario Runner의 MGeo 맵에 Scenario Object 위치 출력하는 기능 추가여러개의 Act, ManeuverGroup, Maneuver, Event 지원StoryboardElementStateCondition 지원 | v4.7.220112AutoEver1 |
| 2022-01-13 | [PM035731](https://morai-core.s3.ap-northeast-2.amazonaws.com/scenario_runner/beta/windows/scenario_runner_220107_PM035731_win.zip) | Windows | **Map Editor에 Scenario Runner 병합**보행자 생성 기능 추가Ego Cruise 모드 시 경로 주행하는 기능 추가경로 주행 시 차량과 도착지 사이의 거리를 차량 정보에 포함시켜서 전달해주도록 수정GT 센서 기능 추가Scenario Runner의 MGeo 맵에 Scenario Object 위치 출력하는 기능 추가여러개의 Act, ManeuverGroup, Maneuver, Event 지원StoryboardElementStateCondition 지원 | v4.7.220112AutoEver1 |

×


---

## 개요

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-2

**Scenario Runner**는 ASAM 그룹에서 배포된 OpenSCENARIO 양식을 기반으로 정의된 시나리오 데이터(.xosc)를 로드하고, MORAI SIM 과 연동하여 시나리오를 수행하는 유틸리티이다. Scenario Runner와 MORAI SIM 사이의 통신은 gRPC(Google Remote Procedure Call) 프레임워크를 기반으로 이루어지며, 이를 통해 MORAI SIM 내의 차량 및 물체를 제어한다. 시나리오는 MGeo 양식의 지도를 기반으로 정의되며, 따라서 MGeo 데이터를 다루는 Map Editor 의 사용법을 먼저 숙지하는 것이 추천된다.

- Map Editor Manual
  - KR : [https://help-morai-sim.scrollhelp.site/morai-map-editor-Add-on-kr/](https://help-morai-sim.scrollhelp.site/morai-map-editor-Add-on-kr/)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150536423/image-20220331-083026.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

시나리오는 시나리오에 등장하는 Scenario object, 각 object의 초기 상태, object들이 수행하는 Story로 구성되며, Story는 간단하게 특정 조건이 만족될 때 활성화되는 Trigger와, 해당 Trigger가 활성화 될 때 수행되어야하는 Action으로 정의된다.

Scenario Runner는 우선 시나리오 파일과 이에 상응하는 MGeo 파일을 읽어들여, Simulator에서 구현되어야 할 시나리오의 내용을 메모리에 로드한다. 시나리오를 실행하면 Scenario Runner는 gRPC Interface 통신을 통해 Ego 및 NPC 차량의 상태를 주기적으로 확인하고, Trigger 활성화 여부에 따라 수행해야하는 Action에 대한 제어 명령을 Simulator로 송신한다.

---

© 2022 MORAI Inc.

×


---

## 설치 및 실행

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/--

이 페이지에서는 **Scenario Runner**의 설치 및 실행 방법을 안내한다.

**목차**

---

### 설치 및 실행

MORAI SIM: Drive 런처를 실행하고, 지급된 계정 정보를 입력하여 로그인 한다. (MORAI SIM: Drive 메뉴얼 : [https://help-morai-sim.scrollhelp.site/morai-sim-standard-kr/](https://help-morai-sim.scrollhelp.site/morai-sim-standard-kr/) )

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/image-20220607-011556.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

MORAI SIM: Drive 로그인 화면

계정에 Scenario Runner의 실행 권한이 있을 경우, 원하는 시뮬레이터 버전을 선택하면 우측 상단의 ‘Install Scenario Runner' 버튼이 생성되고, 이를 눌러 설치한다.

![Image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/BJMI_qVvV07u4ylkLSgmqTt2GnNRW0LpLuu18OYGPaUEHAJJFK6st_80MZaYR8msEae_4P1pLbKZxYAy4Xr1R59UnVcd82DTcZdbrv6UYPvJa_SyWl_Y8d2Q9B8mc_McSSG88kSrDEfcbkbrAeVr?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

설치가 완료되면 아래와 같이 'Start Scenario Runner' 버튼을 눌러 실행한다.

![Image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/t_AXoZuXnzBwQQJRc9DkAUBHj6TkwzmPDI9LFMMx63XhnfS_dZD2iEjbMHT3Kr4TiLUG5f6tdsqSTmIb0K3ahRqLGEU9vG9i62dD39n4kGYzMHV06651_hIJ1WC66NEDzKfPJ5uL2rAVXi6Ptl0z?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![Image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/-XhKOqE1Msa8BKJ1OO6gb_OU0_XiLf8kn-Zn6GcxI6Yx3aafRKN9iztCwjlX38ekfpHiSEUAmw-rexSStL0n-1vHgOSvA293s0tWY7nkNWRQdJtDkvlvy4id24zsDFsPDs--tc35AlIw8RfEteDr?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### '22. 6. 이전 버전

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

22년 6월 이전에 출시된 버전의 경우, 기존 사용법과 같이 별도 로그인을 진행한다.

런처의 'Start Scenario Runner' 버튼을 눌러 실행하면, Scenario Runner 로그인 창이 팝업된다.

지급된 계정 정보를 입력하여 로그인한 후, Launch를 클릭하여 프로그램을 실행한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/image-20220308-034617.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372378/image-20220308-034841.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

기존 로그인 화면을 통한 실행 과정

---

© 2022 MORAI Inc.

×


---

## Quick Start Guide

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/quick-start-guide

이 페이지에서는 **Scenario Runner** 와 MORAI SIM: Drive 를 활용하여 예제 시나리오를 실행시키는 방법을 안내하고, OpenSCENARIO 포맷의 시나리오 파일(.xosc) 구조에 대해 간단히 설명한다.

NPC 차량 Cut-in 시나리오 실행

.xosc 시나리오 파일 구조 (OpenSCENARIO 기반 포맷)

Example Scenarios

---

© 2022 MORAI Inc.

×


---

## NPC 차량 Cut-in 시나리오 실행

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/npc-cut-in

이 페이지에서는 MORAI 에서 제공하는 OpenSCENARIO 포맷 시나리오 예제 파일을 로드하고, MORAI SIM: Drive와 연계하여 시뮬레이션 하는 방법을 안내한다. 사용된 시나리오는 NPC 차량이 EGO 차량이 주행하고 있는 차선 앞으로 Cut-in 하는 시나리오이다. 단, 이에 앞서 이전의 [설치 및 실행](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/--.html) 페이지에 안내에 따라 MORAI SIM: Drive 및 Scenario Runner를 설치하여야 한다.

Quick Start Guide 를 통해 아래 내용을 확인할 수 있다.

- MORAI Launcher를 통해 MORAI SIM: Drive와 Scenario Runner를 실행할 수 있다.
- Scenario Runner에서 제공된 예제파일을 불러올 수 있다.
- MORAI SIM: Drive와 연계하여 Scenario Runner 에서 로드한 시나리오를 시뮬레이션할 수 있다.

**목차**

---

### Cut-in 시나리오 설명

Cut-in 이란 차선을 변경하려는 차량이 해당 차선에 주행 중인 차량의 앞쪽으로 충분한 거리를 확보하지 않고 차선을 변경하는 행위를 말한다. 직접적으로 사고를 발생시키지는 않지만 잠재적으로 위험하고 충돌사고를 유발할 수 있다.

시나리오의 내용은 아래 그림과 같다. 초기 EGO 차량 및 NPC_1 차량은 50 km/h 의 속도로 주행하고 있으며, 뒤에서 주행하는 NPC_2 차량이 60 km/h의 속도로 EGO 차량을 추월한 뒤 차선을 변경한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220613-050656.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 시뮬레이션 실행 절차

##### 1. MORAI Launcher 로그인

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220607-011556.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 2. MORAI SIM: Drive 버전 선택 후 실행

(참고: [MORAI SIM: Drive 실행 방법](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-1.html) )

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/174981142/0MngsXolvuWsdBEjBDXNOYl6Czji-2tNl9D-vedAj_P04eviBZCofw1wehchP0SDWsbOhtE8dpc0VyyKcBh5k1PdHyofJuMoKWJikyMngBdo7_59_L--cEXnyqkjdaXHf-0xTRab2sbSt8ju-3X1?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220308-080204.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

MORAI SIM: Drive

##### 3. Scenario Runner 실행

(참고: [Scenario Runner 실행 방법](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/--.html) )

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/174981142/mgfctnLldRLDpH-kPLbiMpd8JOqh-4nGcvNvAxt_dafYtW5GV4GbHrNa2eMHZgraO_hG_zUhp8xjIourToFQvi5LQXFUFNaIwQ2ELypV3DMN2r-BB_laxlRTw_aEXxZ428ejPozBeuWBVL5pjsNQ?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/174981142/ryZ6-gipKfLQgWs1v1-qmj1xtTSkk1-VpXJiiHdHYQuT8xEdr8BnvDUC7jBpO0Tv3y5esNVunGuolZazs7iItaKJJpmpLVTmj4-QFtQFhF6PxUj9aq1A1898wGV5Ys9vtNMQ0tmr5Q3RDsC0YT3r?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 4. (Scenario Runner) File > Load OpenSCENARIO

(파일 위치: `MoraiLauncher_Win_Data\ScenarioRunner\${ScenarioRunner version}\data\openscenario\V_RHT_Suburb_03\Scenario_Cut_In_1.xosc`)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220621-054428.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

MGeo 데이터 변경 경고의 경우 시뮬레이션에 영향을 주지 않으므로 무시하여도 된다

![Image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/ltWf3cOZd3gXOpHejIWiRxDFw8ITp4L6i5ehht-fOoi2NPEeyC5COdJveY4Ohq1d8_YZsT8ZJMnayvq8camHwo2vPrErsdpY1qSl2-dr-OYMppTaTDV3iaTK2AZ6-Zo_bhN0wx_xUtmLBM5na4P3?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 5. (Scenario Runner) Run > Start Simulation

![텍스트, 스크린샷, 도표, 평행이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/174981142/8QXpR-s3YOUBNDCNGvRTZw5VDyqnlBQGexJs44us9k97UfGts0FvTF-jat2YYnJx6VTI3C_w__oZIwbS43x3m1ot6uPSDwwfPdgk2K4s7SdFVFtGiRJHQekar4lMxh6PtOOzyVROObPpHHijsgA5?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220308-080620.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220613-044431.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220613-044547.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

MORAI SIM: Drive 에서 시뮬레이션을 수행하는 모습

##### 6. (Scenario Runner) Run > Stop Simulation

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/174981142/dtY3lis2y1K4Q6ANbpl_Q0jIFNjhV2p38xMSxWfjsQ78dgowfHgdXCNbNyoxcfWt0MgsmnMVnUlqXfXOk5IyehN9n1Iegsk6KBRxfa5D6Z_YBlWS1ala0iKc0vRe9-id47_-IBWm5iRmId1PDFY3?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/174981142/image-20220621-061400.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

시나리오를 정지할 경우, NPC 차량은 사라지고 EGO 차량은 정지한다

---

© 2022 MORAI Inc.

×


---

## 시뮬레이션 수행

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-1

이 페이지는 **Scenario Runner**에서 시나리오 파일을 기반으로 시뮬레이션을 수행하는 방법을 안내한다.

---

### MORAI SIM 설치 및 실행

Scenario Runner 와 호환되는 MORAI SIM: Drive Launcher를 설치하고 실행한다.

- 버전 호환성 체크 : [Release Notes](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/release-notes.html)
- MORAI SIM: Drive 공식 메뉴얼
  - (KR) [https://help-morai-sim.scrollhelp.site/morai-sim-standard-kr/%EC%84%A4%EC%B9%98-%EB%B0%8F-%EC%8B%A4%ED%96%89.4259841.html](https://help-morai-sim.scrollhelp.site/morai-sim-standard-kr/설치-및-실행.4259841.html)
  - (EN) [https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/Installation-and-Setup.32999.html](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/Installation-and-Setup.32999.html)

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/9IY_fvTbkYiCrTTGGxguv8jTuLCw20ptSSxbpBKjWKDzqofmAFIYacrGyx2ED_Bn3F9ZefzqVM3vss5aCxboZdOtEOHWQBAIEr_dpry54BWyPfiwgg8fVU9_CU0xcb_H8erJTZ-pou_UoyG7VEQk?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

왼쪽의 릴리스 목록에서 버전을 선택한 후, 오른쪽 하단에 있는 '**Install**'를 클릭한다.

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/5J8AnX2zbC9gx1pcw0c1mRx_6KdwTZL8s1bTyCRVB6MUHDnZBPVUfDZ3rO3hKV1hzTrkshnOGu_7RZYWXQdHYLexg9EweXhi3yINF3wgHii06wGsxqx93mmU2kpLegkvEARHJR-OVbDOvb_3Cnms?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

설치가 완료되면, 오른쪽 하단에 나타나는 '설치' 버튼 대신에 나타나는 **'Start'** 버튼을 클릭한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372511/pl2yyKp-sj-6PK_Bx2X3KBuMd9cQgq6sYEAFvTlOB99yep11Dc1yKITPeMTlcPaaxc5YYW9hvK9N9ilxbeevCrkxjYzAXJ-QrFvcV0hJmr4kizh7dfDZC7arA5Wg2bNAr4SYxMZonfRRwGpfOexy?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372511/rxH75nQosCwHShterCk7uMYYgAre04wR1WwO-gFwiTVu2rA9yRN23gW2onkLzSKFYtUyjxpMhHwg99vPKJrum3JAiiTT0uM8UMSlMIUOmLrcZ-P9EDCela-WTzTGAR0YCNaIsZsSXNj5Ki9tEaJb?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 시뮬레이션 시작

#### 로드한 시나리오에 대한 시뮬레이션

1. ‘Run’ > ‘Start Simulation’ 을 클릭하여 (참고: [시나리오 열기/저장](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-3.html)) 불러온 시나리오를 기반으로 시뮬레이션을 실행한다.
2. Ego 차량 제어 방식을 아래 항목 중에서 선택한다.
  1. Built-in
    - Scenario Runner 에서 ego 차량을 제어한다. npc 차량과 동일한 로직을 활용하며, ego 차량이 정의된 시나리오를 따라 제어된다.
  2. MORAI SIM: Drive
    - Scenario Runner 에서는 ego 차량의 초기상태만을 정의하며, 이후의 주행은 MORAI SIM: Drive 의 로직에 따른다. 차량이 정의된 시나리오대로 주행하지 않는다.
  3. External
    - Scenario Ruuner 에서는 ego 차량의 초기상태만을 정의하며, 사용자 알고리즘의 검증을 위해 ego 차량을 별도로 제어하지 않는다.
3. (선택사항) 아래와 같은 추가 시작 옵션을 선택할 수 있다.
  1. Spawn vehicles in LinkPosition
    - 차량의 위치 또는 자세가 정확히 도로 중앙에 위치할 수 있도록 차량을 주어진 위치에서 가장 가까운 도로 위에 배치되도록 한다. 차량의 정확한 위치 또는 자세를 모를 때 활용할 수 있다.
  2. Synchronous mode
    - 시뮬레이터를 실시간으로 구동하지 않고 한 프레임씩 진행하여, 모든 차량이 매 루프마다 한 번씩 제어명령 업데이트가 이뤄지도록 한다. 다수의 차량을 배치하여 실시간 제어 성능이 떨어거나, 보다 일관적인 시뮬레이션 결과가 필요할 때 활용할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

**Ego 차량 제어 기능의 목적과 활용법**

본 기능은 시나리오를 작성하는 과정에서, EGO 차량의 상태에 따라 주행해야 하는 NPC 차량이 존재할 경우 해당 차량이 시나리오를 정상적으로 수행하는지 확인하기 위한 기능이다. (예시: EGO 차량과 동시에 교차로에 도착해야 하는 Synchronize Action이 잘 수행되는지 확인)

**시나리오를 검증한 이후, 사용자의 자율주행 로직을 검증하는 단계에서는 External Mode를 활성화하여 EGO 차량을 직접 제어할 수 있도록 한다.**

![image-20240116-005601.png](https://help-morai-sim.scrollhelp.site/__attachments/150372511/image-20240116-005601.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372511/image-20220308-080620.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

Scenario Runner가 시나리오를 시작하면 MORAI SIM: Drive는 자동으로 대상 지도를 로드한 다음 즉시 시나리오를 실행한다. (NPC Cut-in 예시)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/150372511/image-20220613-044547.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

예시 시나리오 ‘Cut In’ 실행 모습

#### 배치 시뮬레이션

Scenario Runner는 여러 개의 시나리오 파일에 대한 시뮬레이션을 순차적으로 수행하는 배치 시뮬레이션 기능을 제공한다.

1. ‘Add’ 또는 ‘Load Test Suite’ 버튼으로 시뮬레이션 하고자 하는 시나리오의 목록을 불러온다.
2. ‘Start Simulation’으로 배치 시뮬레이션을 수행한다.

![텍스트, 소프트웨어, 컴퓨터 아이콘, 스크린샷이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/hCe0WG-gI7d8z-hrEkH1KN37X6umXZhNSsvMOUMgDozeDNvCPFcfjWSp0Ev5Py6oKsmCngyvINw69XGjs---hXpmoFl_R52ggoG0kYMGgk21JZiFBiSWQx10bJWNEpscx-KEdg1Msw9hLT6Pseew?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 버튼 설명
  - (1) Load Test Suite: 저장한 Test Suite 파일(.json)을 불러온다.
  - (2) Save Test Suite: 현재 시나리오 목록을 Test Suite 파일(.json)으로 저장한다.
  - (3) Add: 개별 시나리오 파일(.xosc)을 목록에 추가한다.
  - (4) Delete: 클릭하여 high-light 된 시나리오를 목록에서 삭제한다.
  - (5) ↑ : high-light 된 시나리오를 목록의 위로 이동시킨다.
  - (6) ↓ : high-light 된 시나리오를 목록의 아래로 이동시킨다.
  - (7) Select All : 모든 시나리오를 선택 또는 선택해제 한다.
  - (8) Start Simulation: 선택된 시나리오에 대한 시뮬레이션을 순차적으로 수행한다.
  - (9) Stop Simulation: 현재 실행 중인 시뮬레이션을 ‘모두’ 종료한다.
  - (10) →| : 현재 실행 중인 시뮬레이션을 종료하고 다음 시나리오에 대한 시뮬레이션을 실행한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/note-macro-icon--625ed763b7218abfc3ad.svg)

배치 시뮬레이션은 하나의 시나리오에 대한 시뮬레이션이 종료될 경우 자동으로 다음 시뮬레이션을 실행한다. 시나리오 파일(.xosc) 안에 Storyboard > StopTrigger 또는 Evaluation 항목을 통해 시뮬레이션 종료 조건이 작성되어 있지 않을 경우 아래와 같은 경고가 발생한다. 이를 무시하고 실행할 경우 해당 시뮬레이션은 수동으로 종료 시켜주어야 한다.

![텍스트, 전자제품, 스크린샷, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/i1Mk0L4hodRRstXXPcgvdT42CjCcBoaZ2zqExJfu9CXpIz_4vCNeFIfpoPHlGeGfwApfXtOmTyfezwm9VyC4I0Ad4eGrTFUwYeTcbA5KA01RxrPT2uvv6RO0anmrFENgc32MZwKyKG7PUWVL5TwK?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

#### 배치 시뮬레이션 창을 통한 개별 시나리오에 대한 시뮬레이션

배치 시뮬레이션 창의 시나리오 목록에서 개별 시나리오를 우클릭하여 시뮬레이션을 수행할 수 있다.

![텍스트, 소프트웨어, 컴퓨터 아이콘, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/n_ITW_KBKZD7yrwhm6uuEDr2TWD-WaCwYZr2QUsbGtu3lV03EBIcqhlNDAUR_oMLKIkeeGMAHoMFGM1rzIXhQYP-AWcEYpFerDLpD8ubg9KyiQaomEx9fJ4JYYz71aecSv4_gFGp3NrIbU5lQ1ag?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 시뮬레이션 중지

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

시뮬레이션을 재시작하거나, 다른 시나리오의 시뮬레이션을 실행하기 위해서는 반드시 기존 시뮬레이션을 중지해야 한다.

시뮬레이션 시작 방법에 따라, 아래 두 가지 방법으로 진행 중인 시뮬레이션을 종료할 수 있다.

- ‘Run’ > ‘Stop Simulation’ 을 클릭하여 실행 중인 시뮬레이션을 중지한다.
- ‘Batch Simulation’ 탭의 ‘Stop Simulation’ 버튼을 클릭하여 실행 중인 배치 시뮬레이션을 중지한다.

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150372511/iiVCMVtPHt1BQAlSxw2ckmOMQOd-GiIhQFh8IRkqXV9UQoioNqbP8p7LoWGxc2diBhaT4HUFq4Ory8ATXHR4PDZpWtG4lztPjs-0SmkvnuTeaBZWRDRwB28VhkqTtHlzoRnCoPeycIuBs_Av1tLC?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

'Run’ > ‘Stop Simulation’ 메뉴를 통한 시뮬레이션 중지

---

© 2022 MORAI Inc.

×


---

## 시나리오 열기/저장

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-3

이 페이지에서는 **Scenario Runner**를 이용하여 시나리오 파일을 불러오는 방법 및 현재 로드된 시나리오를 파일로 저장하는 방법을 소개한다. 시나리오 파일을 불러올 경우, 해당 파일과 같은 폴더 내의 MGeo 지도가 함께 로드되어 GUI 상에 나타난다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

예제 시나리오 폴더에 포함된 MGeo 지도는 시나리오의 시인성을 높이기 위해 활용되는 것으로, MORAI SIM: Drive 에 사용되는 지도와 다르다. 따라서 Scenario Runner에서 지도 파일을 수정하여도 시뮬레이션에는 반영되지 않는다.

**목차**

---

### 파일 열기

#### 시나리오 파일 (.xosc)

- ‘File’ > ‘Load OpenSCENARIO’ 메뉴를 클릭하여 파일 브라우저를 연다.

![텍스트, 소프트웨어, 컴퓨터 아이콘, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150896750/G9dIvxpPFEth4N_Ot724I59QG2KhoDQrmhxrtL96pSbFzOEPNU3TixsSMvh1rjwyKVgchZW8jHSqsHtEnNT2Td1XzsH6ZOgD8-rMaq6k0QdlER3K-ytBxIG5vvAKuReXfWN1uUzgOBUllSQ2DXIf?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 시나리오 파일(.xosc)을 선택하여 불러온다.
  - 이 페이지에서 사용된 예제: ${설치경로}\data\openscenario\V_RHT_Suburb_03\Scenario_Cut_In_1.xosc

![텍스트, 스크린샷, 소프트웨어, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150896750/WU5hqg9LEi3_DCAXAgUiIgYOd44TPJjM-I95KBdvCvRCf2D9CKLetSDlGEgEOHmmo1blo8u4r1vZWwkhUKiD-FfW2x4F2LbbwQGwO1B2dXYBKeucJfvywCKehLsex_4ZNMjeYdUskUqREW0aZU9I?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 불러오기에 성공하면 아래와 같은 화면이 나타나며, 시점을 이동하여 시나리오에 포함된 개체들을 확인할 수 있다. (참고: [https://help-morai-sim.scrollhelp.site/morai-map-editor-Add-on-kr/%EA%B8%B0%EB%B3%B8-%EC%8B%9C%EC%A0%90-%EC%A1%B0%EC%9E%91%ED%95%98%EA%B8%B0.72876251.html](https://help-morai-sim.scrollhelp.site/morai-map-editor-Add-on-kr/기본-시점-조작하기.72876251.html))
  - 기본 설정
    - EGO vehicle: Red square
    - NPC vehicle: Blue square

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150896750/6hOooR4AIF4iDaqjCjq1w6hvMuViJr3xlsNwDeTJFZERfc7Z1TLsnGJrxgfw65pQPl62lAXgWvk92CJ8amS8YMCvF-2BCN9Qh9Ysw_GJqVAw0wOlWcjUwGAjAuvat1U3v5f6GsS-D8i9_kwf8IPy?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

#### Drag & Drop (.xosc)

**(MORIA Sim 26.R1 / Scenario Runner 1.7.0 - 이후 버전 지원)**

.xosc 형식의 파일을 Drag & Drop 으로 불러올 수 있다.

다른 형식의 파일은 불가능하며 잘못 정의된 Scenario File 의 경우 오류가 발생한다.

![image-20260331-072258.png](https://help-morai-sim.scrollhelp.site/__attachments/150896750/image-20260331-072258.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 파일 저장

#### 시나리오 파일 (.xosc)

- ‘File’ > ‘Load OpenSCENARIO’ 메뉴를 클릭하여 파일 브라우저를 연다.

![텍스트, 스크린샷, 소프트웨어, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150896750/Eis3oH1TlgN4cpdnhJUkHPQhmbfCRMy7TFG7jW4tFyYsnFjlg9UauZ66BmtHjmS16JcYu9ULpM3uuFY-b0_-3B3yZVgMaYsrnSn0D908nt2s0mzlHczL7KIcJ-A4_mA_MJhMZc8UyOzkawRdTcNH?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 저장하고자 하는 파일 이름을 입력한 후 ‘Save’를 클릭한다.

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/150896750/zZolxcWM4TxKDadV0HAOQijDerZ2yf5AfprEoFHou3KOETmgFAgIjH_1m10kOx05ol98v28SxSsjJXSzKE9kh3yiuH5Pok8o0_8Phu71XKZZKaaVgKZHzo0ClBcfhmp_tMM_PIFr1smr4Nb7Nt2f?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

---

© 2022 MORAI Inc.

×


---

## .xosc 시나리오 파일 구조 (OpenSCENARIO 기반 포맷)

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/xosc-openscenario

이 페이지에서는 앞서 실행해보았던 Cut-in 시나리오의 각 부분을 발췌하여 OpenSCENARIO 양식 기반의 시나리오 파일(.xosc) 구조에 대해 간단히 설명한다.

자세한 사항 및 추가 정보는 ASAM 에서 제공하는 [ASAM OpenSCENARIO: User Guide](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4908&token=ae9d9b44ab9257e817072a653b5d5e98ee0babf8) 문서에서 확인할 수 있으며, 그 중 '10. Tutorial: How to create a scenario' 파트에서는 OpenSCENARIO 양식을 기반으로 NPC 차량이 EGO 차량을 추월하는 Overtake 시나리오를 작성하는 방법을 다루고 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/note-macro-icon--625ed763b7218abfc3ad.svg)

OpenSCENARIO 양식에는 정의되어 있지 않지만 Scenario Runner 에서 추가로 기능을 제공하는 요소에는 (*)로 표시하였다.

**목차**

---

### 시나리오 파일

MORAI SIM: Drive 에서 제공하는 V_RHT_Suburb_03 지도 상에서 동작하는 Cut-in 시나리오이다.

- 파일 경로: `MoraiLauncher_Win_Data\ScenarioRunner\${ScenarioRunner_version}\data\openscenario\V_RHT_Suburb_03\Scenario_Cut_In_1.xosc`

### Scenario Definition

XML

```xml
<?xml version="1.0" ?>
<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
              xsi:noNamespaceSchemaLocation="OpenScenario.xsd">
  <FileHeader author="MORAI Inc." date="2022-06-10T10:00:00" description="Cut-in example scenario"
              revMajor="1" revMinor="1"/>
  <ParameterDeclarations/>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="MGeo"/>
  </RoadNetwork>
  <SimulatorInfo>
    <Map name="V_RHT_Suburb_03"/>
  </SimulatorInfo>
```

xosc 파일은 기본적으로 XML 양식을 기반으로 작성되어있다. 파일의 가장 위쪽에는 Scenario 에 대한 전반적인 정보가 서술된다.

##### FileHeader

시나리오 파일의 저자, 작성 일자, OpenSCENARIO 버전 등 시나리오에 대한 전반적인 정보를 포함한다. 현재 Scenario Runner 는 OpenSCEANRIO ver 1.1 과 호환된다.

##### ParameterDeclarations

파일 내에서 쓰일 매개변수를 지정할 수 있다. 해당 예제에는 사용되지 않았으나 아래와 같은 방법으로 활용할 수 있다. ( [ASAM OpenSCENARIO: User Guide, 9.1 Parameters](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4908&token=ae9d9b44ab9257e817072a653b5d5e98ee0babf8#_parameters) )

XML

```xml
<ParameterDeclarations>
  <ParameterDeclaration name="Ego_vehicle_model" parameterType="string"
                        value="2016_Hyundai_Genesis_DH" />
</ParameterDeclarations>
<Entities>
  <ScenarioObject name="Ego">
    <Vehicle name="$Ego_vehicle_model" vehicleCategory="car">
    <!-- Same with <Vehicle name="2016_Hyundai_Genesis_DH" vehicleCategory="car"> -->
    </Vehicle>
  </ScenarioObject>
</Entities>
```

##### CatalogLocations

Catalog 파일의 위치를 선언하여 시나리오 파일에서 해당 파일에 포함된 매개변수를 사용할 수 있도록 한다.

##### RoadNetwork

시뮬레이션이 수행될 도로에 대한 정보를 정의한다. 현재 Scenario Runner 는 MGeo 도로정보를 기반으로 동작하므로, 현재 xosc 파일의 위치를 기준으로 MGeo 파일이 포함되어있는 폴더의 이름을 기재한다. 해당 시나리오에 대응하는 MGeo 파일은 xosc 파일과 같은 위치 내에 있는 MGeo 폴더 아래에 있다.

##### SimulatorInfo(*)

시뮬레이션에서 로드해야하는 지도의 이름을 정의한다. Scenario Runner는 MORAI SIM: Drive 에 명시된 이름의 지도를 로드하도록 요청한다. 반드시 위의 RoadNetwork 에서 정의한 MGeo 정보와 대응되는 지도의 이름을 기재한다. 해당 시나리오는 `V_RHT_Suburb_03` 지도 환경에서 실행된다.

### Entities

XML

```xml
<Entities>
  <ScenarioObject name="Ego">
    <Vehicle name="2016_Hyundai_Ioniq" vehicleCategory="car"/>
  </ScenarioObject>
  <ScenarioObject name="NPC_1">
    <Vehicle name="2016_Hyundai_Ioniq" vehicleCategory="car">
      <Properties>
        <Property name="check_tl" value="false"/>
      </Properties>
    </Vehicle>
  </ScenarioObject>
  <ScenarioObject name="NPC_2">
    <Vehicle name="2016_Hyundai_Ioniq" vehicleCategory="car">
      <Properties>
        <Property name="check_tl" value="false"/>
      </Properties>
    </Vehicle>
  </ScenarioObject>
</Entities>
```

시나리오에 등장하는 개체를 정의한다. 각각의 개체는 ScenarioObject 요소로 정의되며, Vehicle, Pedestrian, MiscellaneousObject 세 가지 타입을 제공한다. 개체가 Vehicle일 경우, Vehicle name 에는 차량의 모델명을 기재한다. NPC 차량의 경우, check_tl (tl: traffic light) 속성을 정의할 수 있는데, 이 값이 false일 경우 신호등을 무시하고 현재 차선을 기반으로 주행한다. 이 시나리오에는 총 세 대의 현대 아이오닉 차량이 등장하며, NPC 차량들은 신호를 무시하고 주행하도록 정의되어 있다.

### Storyboard

XML

```xml
<Storyboard>
  <Init />
  <Story />
  <StopTrigger/>
</Storyboard>
```

Storyboard는 Init, Story, StopTrigger의 세 부분으로 구성되어 있다. Init 은 시뮬레이션 시작 시 초기 정보를 포함하며, 날씨, 시간, 각 차량의 위치 및 속도 등을 포함한다. Story 는 시뮬레이션 중 일어나는 사건에 대한 정보를 포함한다. StopTrigger 가 활성화될 경우 시뮬레이션을 중단하며, 그렇지 않을 경우 Story 내의 모든 사건이 종료되면 시뮬레이션이 종료된다.

#### Init

##### Global Action

XML

```xml
<Init>
  <Actions>
    <GlobalAction>
      <EnvironmentAction>
        <Environment name="Environment1">
          <TimeOfDay animation="false" dateTime="2021-07-19T12:00:00"/>
          <Weather cloudState="free">
            <Sun azimuth="0.0" elevation="1.571" intensity="1.0"/>
            <Fog visualRange="100000.0"/>
            <Precipitation precipitationIntensity="0.0" precipitationType="dry"/>
          </Weather>
          <RoadCondition frictionScaleFactor="1.0"/>
        </Environment>
      </EnvironmentAction>
    </GlobalAction>
```

시뮬레이션이 수행될 환경을 정의한다. OpenSCENARIO 에서는 다양한 Global action 을 정의하고 있으며, 기를 활용하여 날씨, 신호 정보, 교통 상태 등을 정의할 수 있다. 이 시나리오에서는 Environment Action을 통해 시뮬레이션 내의 시각과 날씨를 정의하고 있다. 현재 Scenario Runner에서는 맑음/흐림/비/눈 등 간단한 날씨 변화 및 아침/밤/낮 등의 간단한 조도 변화 시각효과를 제공하고 있다.

##### Private

XML

```xml
<Private entityRef="Ego">
  <PrivateAction>
    <LongitudinalAction>
      <SpeedAction>
        <SpeedActionTarget>
          <AbsoluteTargetSpeed value="13.88888889"/>
        </SpeedActionTarget>
        <SpeedActionDynamics dynamicsDimension="time" dynamicsShape="step" value="0"/>
      </SpeedAction>
    </LongitudinalAction>
  </PrivateAction>
  <PrivateAction>
    <TeleportAction>
      <Position>
        <WorldPosition h="104" p="0" r="0" x="550.57" y="42.166" z="0.0"/>
      </Position>
    </TeleportAction>
  </PrivateAction>
</Private>
<Private entityRef="NPC_1">
  <PrivateAction>
    <TeleportAction>
      <Position>
        <LinkPosition id="_18_0_L2_" index="60"/>
      </Position>
    </TeleportAction>
  </PrivateAction>
</Private>
```

Entities 에 정의된 각 요소의 초기 상태를 정의할 수 있다. Ego 차량은 초기 속도 `50 km/h (13.8889 m/s)`, 초기 위치 `( 550.57 m, 42.17 m, 104 deg )` 상태에서 출발한다. 위치의 경우 절대 위치 대신 RoadNetwork 에 정의된 차선의 정보로도 입력할 수 있다. NPC_1 차량의 경우 `_18_0_L2_` 차선의 `60` 번째 점에서 출발한다. (MGeo 정보 내의 각 차선은 등간격의 점으로 이루어져 있다)

#### Story

XML

```xml
<Story name="MyStory">
  <ParameterDeclarations/>
  <Act name="MyAct">
    <ManeuverGroup name="MyManeuverGroup" maximumExecutionCount="1" >
      <Actors selectTriggeringEntities="false">
        <EntityRef entityRef="NPC_2"/>
      </Actors>
      <Maneuver name="MyManeuver">
        <Event name="MyEvent" maximumExecutionCount="1" priority="overwrite">
          <Action name="MyAction">
            <PrivateAction>
              <LateralAction>
                <LaneChangeAction targetLaneOffset="0">
                  <LaneChangeActionDynamics dynamicsDimension="distance" dynamicsShape="cubic"
                                            value="30"/>
                  <LaneChangeTarget>
                    <RelativeTargetLane entityRef="Ego" value="0"/>
                  </LaneChangeTarget>
                </LaneChangeAction>
              </LateralAction>
            </PrivateAction>
          </Action>
          <StartTrigger />
        </Event>
      </Maneuver>
    </ManeuverGroup>
    <StartTrigger />
    <StopTrigger/>
  </Act>
</Story>
```

시뮬레이션 중에 발생하는 사건 및 각 사건이 활성화 되었을 때 각 개체의 동작을 정의한다. 사건 및 동작의 정의는 Story, Act, ManeuverGroup, Maneuver, Event, Action 총 여섯 가지 요소로 정의되며, 각 요소는 여러 개의 하위 요소를 포함할 수 있고, 자신보다 상위 요소가 활성화 되었을 때만 활성화 될 수 있다. 이에 대한 자세한 설명은 OpenSCENARIO의 설명을 참고한다. ([ASAM OpenSCENARIO: User Guide, 8.4 Storyboard element execution](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4908&token=ae9d9b44ab9257e817072a653b5d5e98ee0babf8#_storyboard_element_execution) ) 최종적으로 Event 요소가 자신이 갖고 있는 StartTrigger 요소에 의해 활성화 되었을 때, 그 아래에 있는 Action 요소가 활성화되어 각 개체의 동작을 제어하게 된다.

이 시나리오에는 Story, Act, ManeuverGroup, Maneuver, Event, Action 요소가 각각 하나씩 정의되어 있다. ManeuverGroup 아래에 정의된 바에 같이 `NPC_2` 차량이 그 하부의 Maneuver를 수행한다. (selectTriggeringEntities 옵션이 true 일 경우, 정의된 조건에 따라 해당 Maneuver를 수행할 개체를 자동으로 선택한다.) Action 은 PrivateAction 중 LaneChangeAction 으로 정의되어 있으며, 활성화 될 경우, Actors 요소에서 정의된 대로 NPC_2 차량이 차선 변경을 시도한다. 이 때 `Ego` 차량을 기준으로 `0` 번 위치의 차선, 즉 동일한 차선으로 변경하며(RelativeLane 은 부호 있는 정수로 정의되며, 중앙선 방향을 양의 방향으로 한다), 전진방향 `distance` 기준 `30` m 동안 차선을 변경하고 이 때의 차선 변경 경로는 3차함수(`cubic`) 형태를 따른다.

#### Trigger

XML

```xml
<Story name="MyStory">
  <ParameterDeclarations/>
  <Act name="MyAct">
    <ManeuverGroup name="MyManeuverGroup" >
      <Actors />
      <Maneuver name="MyManeuver">
        <Event name="MyEvent" >
          <Action name="MyAction" />
          <StartTrigger>
            <ConditionGroup>
              <Condition conditionEdge="rising" delay="0" name="StartCondition1">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any">
                    <EntityRef entityRef="NPC_2"/>
                  </TriggeringEntities>
                  <EntityCondition>
                    <DistanceCondition freespace="false" rule="greaterThan" value="6">
                      <Position>
                        <RelativeObjectPosition dx="0" dy="0" entityRef="Ego"/>
                      </Position>
                    </DistanceCondition>
                  </EntityCondition>
                </ByEntityCondition>
              </Condition>
            </ConditionGroup>
          </StartTrigger>
        </Event>
      </Maneuver>
    </ManeuverGroup>
    <StartTrigger>
      <ConditionGroup>
        <Condition conditionEdge="none" delay="0" name="Act_Condition">
          <ByValueCondition>
            <SimulationTimeCondition rule="greaterThan" value="0"/>
          </ByValueCondition>
        </Condition>
      </ConditionGroup>
    </StartTrigger>
    <StopTrigger/>
  </Act>
</Story>
```

특정 요소가 활성화되는 조건을 정의한다. ConditionGroup 및 그 하부의 Condition으로 정의되며, ConditionGroup은 모든 Condition이 True 일 때 활성화되며, Trigger는 ConditionGroup 중 하나가 활성화되면 활성화된다. Trigger 는 Storyboard, Act, Event 아래에 정의될 수 있다.

이 시나리오에서는 Act 와 Event 아래에 각각 하나의 ConditionGroup과 Condition 으로 이루어진 Trigger 가 정의되어 있다. Act StartTrigger 는 SimulationTimeCondition 으로, Simulation 이 시작된 후 경과된 시간이 `0` 초보다 클 때(`greaterThan`) 활성화되도록, 즉 무조건 활성화되도록 정의되어 있다. 이 때 ConditionEdge는 `none`, delay 는 `0` 초로 정의되어, 조건이 만족하기만 하면 바로 활성화된다. ([ASAM OpenSCENARIO: User Guide, 7.6.2. Condition Edges](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4908&token=ae9d9b44ab9257e817072a653b5d5e98ee0babf8#_conditions_and_triggers) )

Event StartTrigger는 `NPC_2`에 대한 DistanceCondition 으로 정의되어 있다. triggeringEntitiesRule이 `any` 일 경우 그 하부에 정의된 개체 중 하나라도 조건을 만족하면 활성화된다. 결과적으로 Trigger는 `Ego` 차량 기준 NPC_2 차량과의 중심간 거리(`freespace="false"`)가 `6` m 보다 커질 때(`rule="greaterThan", conditionEdge="rising"`) 활성화 된다. 즉, Ego 차량을 추월하여 6 m 이상 앞지르는 순간 활성화된다.

### Evaluation(*)

XML

```xml
<Evaluation>
  <!-- <SuccessConditionGroup /> -->
  <FailureConditionGroup>
    <Condition name="Collision_Condition" delay="0" conditionEdge="once">
      <ByEntityCondition>
        <TriggeringEntities triggeringEntitiesRule="any">
          <EntityRef entityRef="Ego" />
        </TriggeringEntities>
        <EntityCondition>
          <CollisionCondition>
            <ByType type="vehicle" />
          </CollisionCondition>
        </EntityCondition>
      </ByEntityCondition>
    </Condition>
  </FailureConditionGroup>
</Evaluation>
```

시뮬레이션 종료 후 시나리오의 성공 조건을 정의한다. OpenSCENARIO 에는 정의되어 있지 않으나 Scenario Runner 에서 추가로 지원하는 요소이다. 여러 개의 SuccessConditionGroup과 FailureConditionGroup을 하위 요소로 가지며, ConditionGroup이 하나도 없을 수도 있다. 각 ConditionGroup은 Trigger 아래에서와 동일하게 하위의 모든 Condition 이 만족하면 활성화된다. 시나리오는 시뮬레이션 종료 시점에 모든 FailureConditionGroup이 비활성화 상태이면서, SuccessConditionGroup 중 하나 이상이 활성화 상태이면 성공으로 판단되며, FailureConditoinGroup 중 하나 이상이 활성화 상태이면 실패로 판단된다. 따라서 각 ConditionGroup의 상태에 따라 성공도 실패도 아닌 경우가 존재한다.

이 시나리오는 시나리오 성공 조건을 가지고 있지 않으며, CollisionCondition을 활용하여 실패 조건을 정의하고 있다. `Ego` 차량이 임의의 차량 (`type="vehicle"`)과 충돌할 경우 시나리오는 실패로 판단된다.

---

© 2022 MORAI Inc.

×


---

## Example Scenarios

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/example-scenarios

**Scenario Runner**는 OpenSCENARIO 구조에 대한 사용자의 이해를 돕기 위해 다양한 예제 시나리오를 제공한다. 이 페이지에서는 예제 시나리오 파일의 위치 및 각 시나리오 파일의 내용에 대해 설명한다.

**목차**

---

### 시나리오 파일 위치

시나리오 파일은 아래의 위치에 제공된다. 시나리오 파일은 시나리오가 실행될 지도를 기준으로 폴더에 분류되며, 각 폴더는 MGeo 양식의 지도 데이터를 포함하고 있다.

`${Launcher_folder}\MoraiLauncher_Win_Data\ScenarioRunner\${ScenarioRunner_version}\data\openscenario\${Map_name}\${Scenario_name}.xosc`

(예: `morai_launcher\MoraiLauncher_Win_Data\ScenarioRunner\scenario_runner_220411_PM035446_win\data\openscenario\V_RHT_Suburb_03\Scenario_Cut_In_1.xosc` )

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/130876307/image-20220621-064518.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 시나리오 파일 내용

시나리오 파일 제목은 해당 시나리오를 표현하는 용어로 정의되어 있으며, 그 뜻은 아래와 같다. (Scenario Runner의 버전 및 사용자 권한에 따라 다른 예제 시나리오가 제공된다)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/note-macro-icon--625ed763b7218abfc3ad.svg)

OpenSCENARIO 양식에는 없지만 Scenario Runner에서 기능을 제공하는 예제에는 (*)로 표시하였다.

| **Scenario Title** | **Description** |
| --- | --- |
| Animal | 지정된 위치에 동물을 배치한다. |
| CBNAO (Car-to-Bicyclist Nearside Adult Obstructed) | EGO 차량이 접근할 때 자전거가 가려진 상태에 있다가 갑자기 도로에 진입한다. |
| CCRB (Car-to-Car Rear Braking) | 전방 주행 중이던 차량이 갑자기 감속하여 충돌 위험을 유발한다. |
| CPNC (Car-to-Pedestrian Nearside Child) | EGO 차량이 접근할 때 보행자가 가려진 상태에 있다가 갑자기 도로에 진입한다. |
| Create map object (*) | 지정된 위치에 지형 및 물체를 생성한다. |
| Cut in | 옆 차선에서 주행 중이던 차량이 충분한 안전거리를 확보하지 않고 차선을 변경하여 들어온다. |
| Double lane change | 전 후방에서 주행 중이던 차량이 동시에 차선 변경을 시도한다. |
| Double lane changer | 후방에서 주행하던 차량이 추월 후 연속하여 차선 변경을 시도한다. |
| Fault (*) | 대상 차량에 특정 기능 고장을 일으킨다. |
| NPC ACC (Auto-cruise control) | NPC 차량이 주변 환경에 따라 주행할지 결정한다. 비활성화 할 경우 초기에 지정된 경로와 속도로 주행한다. |
| NPC Check TL (Traffic light) (*) | NPC 차량이 신호에 따라 주행할지 결정한다. |
| Obstacle | 지정된 위치에 장애물을 배치한다. |
| Overtake | 후방에서 주행하던 차량이 EGO 차량을 추월한다. |
| Parking (*) | 차량을 전/후방 주차 시키거나 주차 칸에서 도로로 진입한다. |
| Pedestrian | 지정된 위치에 보행자를 배치한다. |
| Route | EGO 또는 NPC 차량에 경로를 설정하여 지정된 경로대로 주행한다. |
| Spawn | 차량 또는 보행자를 지정된 속도로 생성하는 가상의 Spawn 지점을 생성한다. |
| Stop sign | 차량이 stop sign에서 정차했다가 다시 주행한다. |
| Synchronize intersection | OpenSCENARIO의 Synchronize action을 활용하여 EGO 차량에 맞추어 NPC 차량을 교차로에 동시에 도착시킨다. |
| Traffic Light | 지도 내의 신호등을 제어한다. |
| UTurn | 지도(MGeo)에 정의된 유턴 경로를 활용하여 유턴을 수행한다. |

---

© 2022 MORAI Inc.

×


---

## 시나리오 생성

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-5

이 페이지에서는 **Scenario Runner**를 이용하여 새 시나리오를 생성하는 방법을 소개한다.

---

### 시나리오 생성

- 다양한 Scene 의 구성 Component & Parameter 에 따라 시나리오를 자동으로 생성한다.
  - 이를 기반으로 해서 여러 다른 Scene 에 해당되는 다양한 시나리오를 생성 할 수 있다.

#### **EuroNCAP Scenario Generation_(**[**Protocol 링크**](https://cdn.euroncap.com/media/75436/euro-ncap-aeb-lss-vru-test-protocol-v43.pdf)**)**

- EuroNCAP은 The European New Car Assessment Programme의 약자로, 간소화 된 방법으로 자동차 사고로 사망하거나 사망 할 수 있는 실제 사고 시나리오를 나타낸다.
  - 안전 등급은 EuroNCAP이 디자인하고 수행한 차량 테스트 시리즈를 기반으로 결정되며, 이 테스트는 실제 사고 시나리오를 간소화하여 차량 안전성을 평가한다.
  - Scenario Runner에서 EuroNCAP Scenario 를 Generation 을 할 때, AEB(Autonomous Emergency Braking) Car-to-Car test procedure 을 따른다.
  - 이 AEB Car-toCar Testing 같은 경우 CCRs, CCRm, CCRb 의 시나리오를 가지고 테스트 하고, 테스트의 중점은 차량이 충돌 가능성을 감지해서, 차량의 충돌을 피하기 위해서 속도를 감소시키는 시나리오 기반으로, 시나리오를 생성 할 수 있다.

| **종류** | **설명** |
| --- | --- |
| CCRs (Car-to-Car Rear Stationary) | 차량이 전진하여 다른 정지된 차량에 전면 구조물이 다른 차량의 후면 구조물에 충돌하는 사고 |
| CCRm (Car-to-Car Rear Moving) | 차량이 전진하여 일정한 속도로 이동 중인 다른 차량에 전면 구조물이 다른 차량의 후면 구조물에 충돌하는 사고 |
| CCRb (Car-to-Car Rear Braking) | 차량이 전진하여 일정한 속도로 이동 중인 다른 차량에 전면 구조물이 다른 차량의 후면 구조물에 충돌하는데, 그 후 감속하는 사고[https://drive.google.com/file/d/1GO92g5MTHPYXhqJeADoZTrldOz83kz7B/view?usp=sharing](https://drive.google.com/file/d/1GO92g5MTHPYXhqJeADoZTrldOz83kz7B/view?usp=sharing) |

#### VRU (Vulnerable road user)

- VRU는 자동차, 버스 또는 트럭에 타지 않은 도로 사용자
  - 일반적으로 보행자, 오토바이 탑승자, 자전거 탑승자, 7세 이하 어린이, 노인 및 이동 수단 사용자를 포함한다.
- Morai runner에서는 CBNAO, CPNC, Pedestrian, Animal 등의 상황을 제공한다.

| **종류** | **설명** |
| --- | --- |
| CBNAO (Car-to-Bicyclist Nearside Adult Obstructed) | EGO 차량이 접근할 때 자전거가 가려진 상태에 있다가 갑자기 도로에 진입 [https://drive.google.com/file/d/1iiTugZVLhwD7lIvTufTctstVX3NdilmO/view?usp=sharing](https://drive.google.com/file/d/1iiTugZVLhwD7lIvTufTctstVX3NdilmO/view?usp=sharing) |
| CPNC (Car-to-Pedestrian Nearside Child) | EGO 차량이 접근할 때 보행자가 가려진 상태에 있다가 갑자기 도로에 진입 |
| Pedestrian | 지정된 위치에 보행자를 배치 |
| Animal | 지정된 위치에 동물을 배치 [https://drive.google.com/file/d/1bvoccbCiborB1nrSdr7BMe8NHcXV2kUl/view?usp=sharing](https://drive.google.com/file/d/1bvoccbCiborB1nrSdr7BMe8NHcXV2kUl/view?usp=sharing) |

### 새 시나리오 생성 (Manual)

**1. File > New Scenario 클릭**

![텍스트, 스크린샷, 소프트웨어, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/177176713/-8eTzCUQcWYfhvpNBnUaSJJK16Vmxq_WMY4Hu4h4NtkAb3aA39cBCgBaDtAWMR1FXDelXQtoxYYEiijmc6vKbFrlTPne7h6OwcXDdfwgJmKrlChQJ_1wfeRCOiMzUxouF-Ngu324fNE69DKSkeWo?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**2. 데이터 입력**

RoadNetwork Path 에 MGeo 폴더를 선택한 후, Map Name 에 지도 이름을 입력한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/note-macro-icon--625ed763b7218abfc3ad.svg)

MORAI SIM: Drive과 연계하여 시뮬레이션하기 위해서는 해당 Map Name 이 MORAI SIM: Drive 내의 지도 명칭과 동일해야 한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/177176713/image-20220621-082456.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**3. 생성 완료**

주어진 경로의 MGeo 지도를 로드하고, OpenSCENARIO 구조에서 반드시 필요한 요소들을 자동 생성한다. 생성된 시나리오에서 차량 및 차량의 동작을 정의하기 위해서는 [시나리오 편집](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-4.html) 의 내용을 활용한다.

![텍스트, 스크린샷, 번호, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/177176713/mRWsv6px_G2jFMLyfEyiIDaDR5zdNvyRrbHQfXe0xkdX7JcxCagWAxWPGXZCW1utds9IVZ7Z39rAjm54kTvmpPzkBsmROxH8a1hla9O9fbg_7UE5wj2_VdcAznP8yst9I1eFPQY1SzV8xHFY8ODC?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### OpenSCENARIO의 Parameter Distribution을 이용한 시나리오 생성

OpenSCENARIO에는 특정 시나리오를 기준으로 전역 매개변수를 변경하여 다수의 시나리오를 파생시킬 수 있는 Parameter Distribution 기능이 정의되어 있다. 매개변수는 결정 분포(deterministic distribution) 또는 확률 분포(stochastic distribution)에 따라 정의된다.

**1. Tools > Generate Scenarios (ParameterDistribution) 클릭**

![텍스트, 스크린샷, 소프트웨어, 컴퓨터 아이콘이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/177176713/CyER9Rx0Paff5hwtmmeN5cARgndAUTovzZgwIOpTBFB1lN34pF_Hp78S7DXD3CIc5TuWn3MY_ZoPU8SHnd4aIfj85ukHQilEpiHby3eBxzdhjEJknfAILIeCwb83Wdddtj8vlk8YK4huO45reOHZ?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**2. 데이터 입력**

OpenSCENARIO 양식으로 정의된 Parameter Distribution 파일을 선택하고, 생성될 시나리오가 저장될 폴더와 파일 이름을 입력한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

입력된 파일 이름 뒤에 숫자가 붙은 OpenSCENARIO 형식의 파일로 생성된다. ({File Name}_00X.xosc)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/177176713/pFUvF3oRhHgu5vW1TTjIJXWcESg3IaIumnNcRrmAddWM-iGAxucmT30CPEvDNsfGKWF1SuL7UOPRwFHwW6lvA2sFAo0KRZe57IL1rpozPeUJEeufJpSmj8P5gZfgFrH_BCc6SFbhs18?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**3. 생성 완료**

정상적으로 생성될 경우 로그로 총 몇 개의 시나리오 파일이 생성되었는지 확인할 수 있다.

![텍스트, 소프트웨어, 스크린샷이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/177176713/KUsSCheVihvORnjhGPSkTHHjBlL7YQnEABsgdbV5JkHbbmQztOWpSV9kb07qLg_0Kf9IIX6YCDOLm97aMy-U5ZyjQVEpiQDcZyuxTb17pkow5IiKK8Rc23-wlshmMDOpfkHUP1-XajU9WIRtDFlj?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

생성된 파일 경로로 따라가면 아래와 같이 파일이 생성된 것을 확인할 수 있다. (12개의 파일 중 example_01)

CODE

```
<?xml version="1.0" ?>
<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="../Schema/OpenSCENARIO.xsd">
  <FileHeader author="MORAI:scenario-module-team" date="2022-07-21T10:00:00" description="Scenario: Double Lane Change (parameterized)" revMajor="1" revMinor="2"/>
  <ParameterDeclarations>
    <ParameterDeclaration name="npc1_vehicle" parameterType="string" value="2016_Hyundai_Ioniq"/>
    <ParameterDeclaration name="npc2_vehicle" parameterType="string" value="2021_Hyundai_G80"/>
    <ParameterDeclaration name="weather_type" parameterType="string" value="dry"/>
    <ParameterDeclaration name="ego_init_speed" parameterType="double" value="9.8"/>
    <ParameterDeclaration name="ego_init_link_index" parameterType="int" value="40.0"/>
    <ParameterDeclaration name="npc1_init_speed" parameterType="double" value="16.5"/>
    <ParameterDeclaration name="npc1_init_link_index" parameterType="int" value="78"/>
    <ParameterDeclaration name="npc2_init_speed" parameterType="double" value="16"/>
    <ParameterDeclaration name="npc2_init_link_index" parameterType="int" value="3"/>
  </ParameterDeclarations>
  <VariableDeclarations/>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="."/>
  </RoadNetwork>
  <SimulatorInfo>
    <Map name="R_KR_PG_KATRI"/>
  </SimulatorInfo>
  <Entities>
    <ScenarioObject name="Ego">
      <Vehicle name="2016_Hyundai_Ioniq" vehicleCategory="car">
        <ParameterDeclarations/>
        <Properties/>
      </Vehicle>
    </ScenarioObject>
    <ScenarioObject name="NPC_1">
      <Vehicle name="$npc1_vehicle" vehicleCategory="car">
        <ParameterDeclarations/>
        <Properties/>
      </Vehicle>
    </ScenarioObject>
    <ScenarioObject name="NPC_2">
      <Vehicle name="$npc2_vehicle" vehicleCategory="car">
        <ParameterDeclarations/>
        <Properties/>
      </Vehicle>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <GlobalAction>
          <EnvironmentAction>
            <Environment name="Environment1">
              <TimeOfDay animation="false" dateTime="2022-07-21T10:00:00"/>
              <Weather atmosphericPressure="80000" fractionalCloudCover="zeroOktas" temperature="170">
                <Sun azimuth="0.0" elevation="1.571" illuminance="1.0"/>
                <Fog visualRange="100000.0"/>
                <Precipitation precipitationIntensity="0.0" precipitationType="$weather_type"/>
                <Wind direction="0" speed="0"/>
              </Weather>
              <RoadCondition frictionScaleFactor="1.0" wetness="dry"/>
            </Environment>
          </EnvironmentAction>
        </GlobalAction>
        <Private entityRef="Ego">
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$ego_init_speed"/>
                </SpeedActionTarget>
                <SpeedActionDynamics dynamicsDimension="time" dynamicsShape="step" value="0"/>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
          <PrivateAction>
            <TeleportAction>
              <Position>
                <LinkPosition id="A219BS010310" index="$ego_init_link_index"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
        <Private entityRef="NPC_1">
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$npc1_init_speed"/>
                </SpeedActionTarget>
                <SpeedActionDynamics dynamicsDimension="time" dynamicsShape="step" value="0"/>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
          <PrivateAction>
            <TeleportAction>
              <Position>
                <LinkPosition id="A219BS010310" index="$npc1_init_link_index"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
        <Private entityRef="NPC_2">
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$npc2_init_speed"/>
                </SpeedActionTarget>
                <SpeedActionDynamics dynamicsDimension="time" dynamicsShape="step" value="0"/>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
          <PrivateAction>
            <TeleportAction>
              <Position>
                <LinkPosition id="A219BS010311" index="$npc2_init_link_index"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="Story1">
      <Act name="Act1">
        <ManeuverGroup maximumExecutionCount="1" name="Sequence1">
          <Actors selectTriggeringEntities="false">
            <EntityRef entityRef="NPC_2"/>
          </Actors>
          <Maneuver name="Maneuver1">
            <Event maximumExecutionCount="1" name="Event1" priority="overwrite">
              <Action name="Action1">
                <PrivateAction>
                  <LateralAction>
                    <LaneChangeAction targetLaneOffset="0">
                      <LaneChangeActionDynamics dynamicsDimension="distance" dynamicsShape="cubic" value="54.8254917969"/>
                      <LaneChangeTarget>
                        <RelativeTargetLane entityRef="Ego" value="0"/>
                      </LaneChangeTarget>
                    </LaneChangeAction>
                  </LateralAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition conditionEdge="rising" delay="0" name="StartCondition1">
                    <ByEntityCondition>
                      <TriggeringEntities triggeringEntitiesRule="any">
                        <EntityRef entityRef="NPC_2"/>
                      </TriggeringEntities>
                      <EntityCondition>
                        <DistanceCondition freespace="false" rule="greaterThan" value="5.0">
                          <Position>
                            <RelativeObjectPosition dx="0" dy="0" entityRef="Ego"/>
                          </Position>
                        </DistanceCondition>
                      </EntityCondition>
                    </ByEntityCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
            <Event maximumExecutionCount="1" name="Event2" priority="overwrite">
              <Action name="Action2">
                <PrivateAction>
                  <LateralAction>
                    <LaneChangeAction targetLaneOffset="0">
                      <LaneChangeActionDynamics dynamicsDimension="distance" dynamicsShape="cubic" value="54.8254917969"/>
                      <LaneChangeTarget>
                        <RelativeTargetLane entityRef="Ego" value="1"/>
                      </LaneChangeTarget>
                    </LaneChangeAction>
                  </LateralAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition conditionEdge="rising" delay="0" name="MyStartCondition2">
                    <ByValueCondition>
                      <StoryboardElementStateCondition state="completeState" storyboardElementRef="Event1" storyboardElementType="event"/>
                    </ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition conditionEdge="none" delay="0" name="Act1_StartTrigger_SimulationTime">
              <ByValueCondition>
                <SimulationTimeCondition rule="greaterThan" value="0"/>
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
        <StopTrigger/>
      </Act>
    </Story>
    <StopTrigger>
      <ConditionGroup>
        <Condition conditionEdge="none" delay="0" name="Time_out">
          <ByValueCondition>
            <SimulationTimeCondition rule="greaterThan" value="30"/>
          </ByValueCondition>
        </Condition>
      </ConditionGroup>
    </StopTrigger>
  </Storyboard>
  <Evaluation/>
</OpenSCENARIO>
```

---

© 2022 MORAI Inc.

×


---

## 시나리오 편집

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-4

(이 페이지에서는 **Scenario Runner**를 통해 생성하거나 로드한 시나리오를 편집하는 방법에 대하여 다룬다. 편집 방법에는 크게 UI를 통한 OpenSCENARIO 요소 생성과 위젯을 통한 속성 변경 및 요소 삭제가 있다.

**목차**

---

### OpenScenario Element 생성

#### Scenario Element 생성

[OpenSCENARIO](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/) 에서 정의된 Elements 들을 Data View 에서 생성하고, 정의할 수 있다. 대표적으로 Entities, Storyboard, Story, 그리고 Act 해당되는 하위 항목에 Elements 들을 추가하고 세팅할 수 있다.

##### Data View 를 통해서 Scenario Element 생성

- 좌측 Data View 에서 추가할 항목 상위의 element 를 마우스 우 클릭으로 통해서 생성할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

Data View 에서 OpenScenario 에서 정의된 element 를 클릭했을때, `add` 와 `set` 을 확인할수 있는데, add 같은 경우에는 scenario element 의 Cardinality 가 0 이상일 경우에 그 add 할 element 를 추가를 시켜주며, set 같은 경우 Cardinality 가 xor 이나 값이 하나로 지정하고 싶을때 사용한다.

#### Scenario Object 생성

Scenario Object 에는 세 가지 종류(Vehicle, Pedestrian, Miscellaneous object)가 있으며, 두 가지 방법으로 생성할 수 있다.

##### 1. 메인 메뉴를 통한 생성

- Edit > Add Scenario Objects > Vehicle 을 선택하여 차량 생성 UI를 불러온다.

![Image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/du9YB6wh9Xy5obfglxJmtZPFwWtV9GAQ9aGWZO_D1d32UPvqNV88zeqXkn2asNtxBA-ADKTR8AwGvbZP9ZC2B6_4REShHILM_diPmkPbYbjE9lfHMfMIGEsFUrEldDOnQ_1UnXip02Yt0dSPUTIi?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

차량의 이름, 차종, 초기 위치 및 속도를 입력한 후 `OK` 를 클릭하여 생성한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

위치 정보는 World Position, Link Position, Relative World Position(orientation=absolute), Relative Object Position(orientation=absolute) 의 4 가지 포맷으로 입력할 수 있으며, Link Position으로 입력할 경우 차량의 heading 은 자동으로 설정된다.

또한 MORAI에서 정의된 Vehicle Property 에 Check TL(Traffic Light) 과 Apply ACC(Adaptive Cruise Control) 을 옵션으로 주어, 차량이 신호등을 볼수 있거나, 앞차의 간격을 유지시킬 수 있게 할수 있다.

![텍스트, 스크린샷, 번호, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/5o-eyAHIFUBQJ-BmxlVgdPKIxBHm2ut7HmSzJXtLdm1-YV05yOrDqLZmoJBDiLtrL5YnNsc22mRdcTleNXwiKQkoKqOD1-BLCq45G0DhN122VfH4W0Li6Wn7N2vHn0dNFktVrkzeGkbZo4ftSmq_?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 도표, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/AtPU3tlCwlu8mrPwHnaIEKSatASj1Uqb7hEOvFkHohGZqMblL4iRpA8qGOeEPruJkULRmNpfcn4KsMAS8T7GW7OHLHf0D0O9BfzNvCb8t8HwVHW_VxIPcYR20xE04iwmZcLZvuJvG_XRXdx88_ZP?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 같은 방법으로 Pedestrian 및 Miscellaneous object 도 생성할 수 있다.

![Image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/8qhRtJd7GPj8irn0K5JRCebWVHUY3woXKOZi6L4hFSGwszPrJCr9xrKsJX1lHXDD8R-gdRh_k6mJ0n5zrnkZZlXEAFBkNSu-C-PuXKGm5sPeKcbMwFz-A6_a7d0CLUYs-ehNZ0J5B5JPeroN25NY?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![Image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/KtyJkTzfHPjukDLWqZdeS7eJPjVBUPydxsBHhnfpntcfPR2r_5Ok9jk9uQDmtrFsx5nGecoweDbaXATOd-7LWX2_464F46QVhU-yMZWARF7F3NH3cN__4lj78A0ILxl7hi4fUUd4kuq44U5_doPa?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 도표, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/Hs4XW_wt_R1XUSk43hrt4Lemc0JkIJxYH2CBfCwmmiubX3SeaPOugOHn4hJaikY5UsgFl5MZJa-ptD82wTuKoW5_qAFYbj1doG1aVgfvss9P5ZFKOkJgMzSMzEsFr7dvEk8pByAqi7_7itPhN_PW?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 2. 선택된 Link point 에 생성

- 좌측 `Data View` 에서 `LINK` 를 클릭한 뒤, 생성하기 원하는 링크를 클릭하고, 키보드 위/아래 방향키를 이용하여 원하는 지점을 선택한다.

![텍스트, 스크린샷, 평행, 번호이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/3KsGQppEytoNTVSFqOY8S24l_PbmFrgVIF4uHiiP1gPAlz6ERhgDUu-kuMrr4NaemCpUoQzfstKv72HUMNu3fDA-ne6DuUWyOVdP_KnEo-uPA33xvFzuE8BhLs2BS-HXUQkbF25xh-j6WPTkPGmA?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 마우스 우클릭 > `Add Scenario Objects` 를 활용하여 같은 방법으로 Scenario object를 생성할 수 있으며, 이 경우 위치 항목이 미리 채워져 있는 것을 확인할 수 있다.

![텍스트, 스크린샷, 평행, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/Qi8ZaDdY3l_eA1oX8m9TdVP3pYFL0kiFfgjveFMdHb1tZT9V5hb9d7LagU_kWoPP4owiFZfn8wBVwN1SOXnExbxgGFq6hsEGdZoTjlWmcW82C6yUwpdL-AY-hjtSX0DMVCiFH4DyVelCAdB5EK8e?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 번호, 폰트이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/H2p3yR1VCitcXr_cN_62C5IIQhGpDgN5eG1M1xjBNVDF-nyBFJ1fE2kdygsl0Qi-0olOzfq_4In5W_gP_4Fykd0KKv-vmUa_WS0ecjxfJRSIdWWpYerhiYoHuvjGfWpRNdwS7QVeGuPEuZV6DKjM?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/BbqLWdQVlv-VUVmQ8g_1W7Bhk7o0f_EGfxa9UsXbOg-TCJ6KWeOFy9ca8rtq4TLRlapzYQkxT18BqdZf_6oJRgLsCzlv6Utnbb2i1-OAjWWOXhy4DPe9t3_Bl1o73LP4L19QJnX9TPunmsZ5rZMg?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 3. Data View 에서 `Entities` 를 마우스 우 클릭을 통한 Scenario Objects 생성

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

이 방법을 통해서 생성했을 시, 차량의 속력과 위치를 따로 추가해야 한다. (아래의 예시를 참고)

Data View 에서 Entities 선택한 이후, 오른쪽 클릭을 했을 시, `Add Entity Wizard` 를 통해서, Scenario Object 들을 아래와 같이 생성할 수 있다.

![텍스트, 스크린샷이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/bKHXZ6i-4srWdQDv2MqLjMKvuvDVyzyVZZ0a0Yx19cbSeYlx3lYBfp8cn9FAeHDNjqEdQov5grjeHkyuhn4yo4ZuoyL-T4jqNNtyj1C8iwdg_FaKMV5ny5hHz8NtImOnVHJPIJm154sYEbHBZHtF?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/PTsQFyk7s4zrkUqu_ptARqj8J5SvWoqWzH8G6reDyxoo_DPDwvJFs0x7h5q6ifsvrewECsevRw_zz5DUrL-kC3Mj7HU1NN2pOgQ0g5GR4cy6gh53BYkKW_qOz-e9NK830u9j5sS9XE5KMGBYJFEs?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

위 `Add_Scenario_Objects` UI 와 같아 보일수 있지만, 해당되는 차량의 속력 값과 위치 값을 설정할 수 없으며, `Entity Type` 에는 Scenario Object(Vehicle / Pedestrian / MiscObject)의 종류를 지정할 수 있다.

![텍스트, 스크린샷이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/2-egIQ17z_PoKrpFLKBi-ihdv4_f8f19F67BSjdrCMpZAZKAKh8CoNOzc7k-ZaDWCWYtPOf6pJj0Fk60YXev6WbCPc-b5tlwRAMFf45ZZEQ6QcoDU5HGLjh1GtOxyyQbUS6rMhZWtNEtRqtaodIe?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

이후 Vehicle / Pedestrian / MiscObject 의 위치 값이나 속력(MiscObject 제외)의 정보를 넣으려면, `Data View`****에서, Storyboard > Init > InitActions 마우스 우 클릭을 해서, 해당 entity 의 private action 을 만들어주어야 한다.

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/N7IwaZFRj3jm-gAjDK9qWeNHPLfwUvSjXL0z7UQTh6wBmQZY5z86YgiKahFqq1xAlGZ8CB21R5VykdynQXgUU5BgO7bes_sIPYp38OjgDkiN2pcaH_UHjaIL4SXoYDXTa3bJXRs2_qY9wqvNA8m1?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

아래와 같이, `Entity Ref` 를 눌렀을 때, 현재 Scenario Object 리스트들을 볼수 있고, 위의 예제에 만든 NPC_4 를 클릭해서, NPC_4의 속력과 위치를 정할 수 있다

![텍스트, 스크린샷, 소프트웨어, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/njt1EoPftKRLzzj0aZapGKrRkXxUXfbl1GbdiFwxl9-12OyCT7P0NAnk8UQRsxvX7OIEkgLR6nw2EUxgXRYFegR9tx4sQbO-PiYVAk_z4QLzxYyLUjoPn4FLvggmqPpJDhhJNztHmgcl6FIL8Uyg?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

**SpeedActionTarget**아래의 Type 은 두가지로 **Link** 와 **Custom** 으로 나누어져 있으며. **Link** 같은 경우, 그 링크의 제한 속력을 기준으로 scaling 한 값을 range 로 입력하면 되고, **Custom** 일 경우에는 Link 속력과 관계없이 속력을 정해줄 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/manual_7.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

Entity Wizard 안에서 NPC_4의 초기 위치를 설정

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/manual_8.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

NPC_4의 초기 속력 설정 및 위치 결과 화면

#### Event 생성

##### 1. Event UI 를 통한 생성

- Edit > Add Event 를 클릭하여 이벤트 생성 UI를 불러온다.

![텍스트, 스크린샷, 소프트웨어, 멀티미디어 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/4CkItNZsutalY2MaMLpNUQrdxkrfeezObnoWNqxqy4JZIEkuv8YAxg7mxBwe7PKqzxU6wLDw6binFwYmnvmXFtSQM43ngSOXkXzMV11ESmT6BthtkrHc7D2bhPzzbsBDVhO6YxiF4xbRGfcgktg2?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- Event 생성에 필요한 정보를 입력한 뒤, ‘OK’ 를 클릭한다. Event 생성에 필요한 정보는 일반적으로 [ASAM OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)의 정의에 따르며, 추가 정보는 [OpenSCENARIO Support](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/openscenario-support.html) 페이지를 통해 확인할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/y7RUvixnCQ3UTuVJqftUydUZ9BdBUxJb2NDP2vmdcv4ROKAUxCvFEcRPJxZQtLvNCP_IsRiOVJjdqOY4b0MbdocUkM9vLoyAqmCTjj8By0Y8RrQCfUQ6NmA7Rd68Hr3iHiudVrIBYYau4CUK_tIE?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/ub0oFKlBe4FZLcuC1vDXcwZcHLE8p332AuljaqYlBrqzzpRe0yx-ga8xzZkXkVnsLOQMwzrMEAydPCg7lPiTzoVSy6kKeYz7YV-LAelpHH-L3-_4gzyDz5agat0a6RGHN6EZkK3RSeZYLP6KrW5e?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 2. Data View 에서 `Maneuver` 를 마우스 우 클릭을 통한 Event 생성

원하는 Event 를 생성하려면, 해당되는 `Maneuver` 에 마우스 오른쪽 우 클릭을 통해서, `Add Event`를 클릭한다.

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/z8NQuMTtJQ9ooWsxnQbFxx8eJHE79qWU9Q9kWBo6xW2tBW8azITICiAKIkOaX2JHfhciyTE3g3OCJc5FzGxeASVoWXiI0NiT82D7Dmru6oggIgdFXXIJhef44KkEDOktLx34LZuCxCPGPninyvGY?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

아래와 같이 `Event` element 가 생성이 된 걸 확인할 수 있다. 이 방법으로 Event element 를 생성시, Event 안에 attributes 의 값들이 기본값(unspecified) 로 저장된 걸 오른쪽 Property View 에서 확인할 수 있으며, 보여지는 attributes 를 수정 가능하다.

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/71THbyeZmKSR82OR_sactXgxdF48p2fxA6YKljqPhuROn2uz-vlzrZB7CRpbYEUuc_cn6L25CygXmZBMSydF-QwPES6SsYOLrvF5fpDG-rmuSEzpGC9IxSVSDX5xyjLaITNKd4IhH1szbmEW4npL?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

만약 Event 안에 있는 elements(Action, Trigger) 를 추가하려면, Event 하위 항목에 있는 element 이기 때문에 추가하려는 Event 마우스 우 클릭을 통해서 Action 과 StartTrigger 를 추가할 수 있다. 또한 아래와 같이 `MyManeuverGroup` 안에 있는 Actor(`NPC_2`)에 새롭게 생성된 Event Element 를 확인할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/manual_10.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

아래와 같이 `Action Wizard` 를 통해서 해당하는 Event 의 `longitudinal action` > `SpeedAction` element 를 생성할 수 있다.

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/_TZYh9jQngVuBqApoAZ9HHB5OP6EVzY7W5uL3zLnoZEMKsyT0s3ChaErrJxE4CzDKJXSMLw6Tgzch-ZiO5a3Apwtx2vmaN1FEtP53fFvRy_9LNEA3aJBkRFlGOJzbTKE5vD0vpnSGcuKgbza_HFN?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

위에서 Action 을 추가하는 것처럼, 아래 `Set Trigger` 를 통해서 `ConditionGroup` element 를 생성해, `Condition` element 를 만들 수 있다. 이후 `ConditionGroup` 을 만들려면, `Trigger` element 에 마우스 우클릭으로 `Add ConditionGroup` 을 생성하면 된다.

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/4MAO_w-wh30d96wAvA4-OvHaD6WuwvXd0A_FJYqjgXndmbTnaeAuahpM_rPRDWDqi5CSL19LNyr7hyB6h16ZAaVfNkanSe82PE_oG49FouwKoFYO91oA-PcSjLjsgwVdfGMtCZoEnTKeiSxVfQTU?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/SEvmTkgFKp1LHujLr75qHFyxbJtUCM3zSN--Z_ri2QAXzcgoIUMJnJmOr0PTBCUfeFXOSLyBOugELfb4Pjp7hQCuexVtCMwy6jweuRrp1olLMDaEIInX0IlXgleE8jxgouowXvVX999mmIpCTr8s?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

이후 `ConditionGroup` 을 마우스 우 클릭을 하고, Submenu 에서 `Add Condition Wizard` 클릭하면 아래와 같이 `Condition` element 세팅할 수 있는 화면을 볼 수 있다.

![텍스트, 스크린샷, 소프트웨어, 디스플레이이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/rQaLetrVLk4ZOZmcg9tQwyAEb9gjYBcZVb3gbd_tiBAHinphR07HIiFjxhMfjo_oYyLgr17FV3ltyJOgRZWe2i-MAhL-G-dLIXP1inUBnglhi1B0cOfiUZUw8-kqh8Tkf-6eEfyoKc1FUzR0jU6Y?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어, 번호이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/TbWGkrSC1afJoxGdvg6Axc9a7FTw6TdiOAQx-gB3WJTxi5IAf4d2iAUoom5YOBbx68QG33PnpvsE9EHbshD1I8uAs6s5jxQZBsdTKNFIjSDSiDTeodtgrjExpaJ8sxSsPOIqdXU4-JTw7w2nb7VB?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### Element attribute 수정

1. 좌측의 `Data View` 에서, 수정하고자 하는 속성(attribute)를 포함하는 요소(element) 또는 그 상위 요소를 클릭한다.
  - 오른쪽의 `Properties View` 에 클릭한 요소 및 하위 요소의 속성 값이 표시된다.
2. `Properties View` 에서, 수정하고자 하는 속성을 더블클릭한다.
3. 팝업된 `Property Editor` 에 수정된 값을 입력하고 'OK' 버튼을 클릭한다.

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/d-WdwyLMZHIrRb2orPhy9GWVQlt3tp2EefCiGmy0YlspTNZf7uH9Yi6UBLSs-exy61pyOuW2iGzvmKyHJI8k2KFZh_eWA7O98hZv1SDtl3srx8dB2L-EduYQkQTaussrzhe2b6-6akkFJ86llShA?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/EfANYvTp_JkJVZ2292-kQLZ8xaDojkhz994Ao4Nr-AMETuEThlOANmwvCjPwv-6H54j8FaKoC43yuvwQFrbPTlrKNrE9gqe9VnB6u9c6jqMmlEvs73S58CLQVas4Lm-P8JWPDmBniP0R2Ua0dyKr?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- OpenScenario 에서 정의된 Enumerations Type 들은 ComboBox 의 UI 형태로 되어있다.

![스크린샷, 텍스트, 소프트웨어, 그래픽 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/GSBEDAwxUUEaBOIfDRuTblFg1UNzDBcEXDvJxv_7BCpqEaX2e5TiDyPUGA67RN_PT2rjryT1isgb8i6LuZ2SVeJeS1duaP_wzi_a63y-ReHz_GBVWjojWbmyQnkHbtxMrvl8-7tEf5C8D43a3ZWT?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### Element 삭제

1. 좌측의 Data View 에서, 삭제하고자 하는 요소(element)를 우 클릭하고 `Delete` 버튼을 클릭한다.
2. 삭제 확인 팝업 창의 `OK` 를 눌러 요소를 삭제한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/151060620/IMgr5gE5O5tNe4LFfYBsNQyxaGZysQjoi6E1LdhpF-PVBIKaot5Qfy1-Vq4b38zqL-OyKiWspNC9TRZ4cLGgrTCB73JVIKf4zBNAIe5ZwJWtx65f2b8N3IO3mQHC8DS8NkW7WxLcdA29TE_UwIO2?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/L7xYxqQdQnZiPPxU5UAwgAEBjXbRjRrxX_2ZcG6vXAKse0vwfdqf1sQOYdgirlJ05fASrWVVVZJ02rHWDe-rOMxe7AmL4fP0QkOGkfN2vvR_JZMVfx-tMB1PdTihGVmWL4RgHncxVqbGwbf0F78L?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 삭제가 불가능한 요소의 삭제를 시도할 경우, 명령이 무시되고 하단에 로그가 출력된다. (상위 요소가 해당 하위 요소를 반드시 필요로 하는 경우 삭제할 수 없다)

![텍스트, 스크린샷, 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/_XpOt_lh-mXCWbsUTrNOQpb9lO5g25AUJbnyW00ezy4z3SQXhpo5rcJIEBsSrJpMcPO3bapJ546IZIFqed6h3bgF59UV9Rh0B3vp583o39PsscrJlpRIHr1i1PppYUJF7LRyQrGpCiXuDYj1mhdv?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 차량 경로 편집

#### **Trajectory** 편집

- Trajectory는 차량 또는 보행자의 이동 경로를 나타내며 다양한 수학적 모양을 가질 수 있다.
  - 수학적 모양은 Polyline, Clothoid, Nurbs의 세가지 모양으로 나뉜다.
  - 차량이나 보행자가 어떤 Trajectory를 따라갈지 정의할 때 이러한 모양들을 활용할 수 있다. (현재 Polyline만 지원되며, Clothoid와 Nurbs는 지원 안됨)
- Follow Trajectory 를 사용하기 위해서는, Trajectory 도 편집도 필요
  - Trajectory의 동작은 시간(TimeReference) 정보와 함께 정의된다.
  - TimeReference는 Timing으로 지정이 되어있을 때, 어떠한 지점(Vertex)를 어느 시점에 도착할건지를 Absolute와 Relative로 저장할 수 있다.
  - Absolute로 지정되어 있을 때 각 지점마다 도착 해야되는 시간을 의미하며, Relative일 경우, 각 지점 간의 상대시간을 표현한다.
- Morai Defined & Custom Element로서 Spline이 있다.
  - Spline 같은 경우, 값을 각도와 위치값(Vertex)로 받는다.
- [https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/content/FollowTrajectoryAction.html](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/content/FollowTrajectoryAction.html)

#### 경로 생성

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

차량이 이미 출발위치에 배치되어 있을 경우, `Set Start Location` 과정을 생략할 수 있다.

- 경로를 설정하기 원하는 차량의 `Private` element 를 클릭한다.
- 원하는 링크를 클릭한 뒤, 키보드 위/아래 방향키를 사용하여 차량의 출발지점을 선택한다.
- 선택한 지점을 우 클릭하여 `Set Start Location` 을 클릭한다.

![텍스트, 스크린샷, 도표, 평행이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/RmD9PBHUsQkVjdm-Ma4rWL7f_Pvz7RHNVfsPIkflz-hrFm4DQrqF3OUZQYX-O45txfTC6uW2X_HdekrDQ0NNfthtYh1iIz2UEjZoQmMq__XyP59fYe3DCM9YnAlnQg7EZ69PDDNACQXT-eM_gTxE?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/e10BMV7rK4e_8fQD_c78hZQAgRNtct6BXonQlbvNLPsw47fmzQnxyZjnw0WjK6zzJTnhmPehGBWSI6MhsnKGabe525PQZemhPWc2vNnhwPifLCurn_PLAySmHogej8UGZVmEOADT6kx5Y2cpJL0R?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 같은 방식으로 원하는 목적지를 선택한 후, 우 클릭하여 `Add Waypoints` 클릭한다.

![텍스트, 스크린샷, 소프트웨어, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/kT4Xe2s3QAUda2JldWr2mmLf337eeanBa0CQY5SWPzebYstw6lPJAieaTjUYCJiiJoHr0bxG9Xm0z-PpkpR1zYk2ACuX-9feDjDXNXGc9cCsWEpsCFGafMcN6QOHic1s1T49gva3rPdtlwVZYPCS?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![텍스트, 스크린샷, 소프트웨어, 번호이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/tkws0IPZqmzYPIFV77lT3HwlT5zgi7Gso7S4gkwxffx9Gueg63rYBoz55E3XoiM-16kSpbEuMlwZ3vdPR3rzB5cxQ3ClKcYsvos-1wFMAl42X4rrvvIN4kL-ruGSCRzbReK7Vna_5fpTrNhDtWl8?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

- 같은 방법으로 Waypoint 를 추가하면, 이전에 추가한 Waypoint 는 경유지점이 되고, 마지막 Waypoint 가 목적지가 된다. 이 때, Waypoint 사이의 경로는 임의의 최단경로로 설정된다.

#### 경로 삭제

- 경로를 삭제하고자 하는 차량의 `Private` element 를 클릭한다.
- 마우스 우 클릭하여 `Delete Waypoint` 를 클릭한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

해당 차량의 마지막 Waypoint 부터 순차적으로 삭제되며, 남은 Waypoint 가 2개 미만일 경우 Routing Action 이 삭제된다.

![텍스트, 스크린샷, 소프트웨어, 그래픽 소프트웨어이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/AZKkMzbuXEqUATNiLz3OgsRto9vzcOisT9jPtONLVeSQ7C7zT_dnGXhZqgF6SLlE0CA6R3wuHY20TpnKB_AaJikUXa222BF9sM6WGfl8OXubfLu-7fo9MOijVVdASaKkcwrX0jvM6uLo20lGZNPq?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 부가 기능

#### WorldPosition의 Z축 투영

배치된 물체의 고도(Z축)값을 모를 경우, WorldPosition으로 X, Y 축 값을 입력한 뒤, Scenario Runner 의 Z축 투영 기능을 이용하여 고도값을 생성할 수 있다.

- Project all WorldPosition onto roads
  - 물체의 위치를 현재 위치와 가장 가까운 도로 위로 이동시킨다.
- Project all WorldPosition onto roads (ignore z-coord)
  - 물체의 위치를 현재 위치와 가장 가까운 도로 위로 이동시킨다. 가까운 도로를 찾기 위해 거리 계산 시 고도 차이는 고려하지 않는다.

![텍스트, 스크린샷, 도표이(가) 표시된 사진 자동 생성된 설명](https://help-morai-sim.scrollhelp.site/__attachments/151060620/P64547824AGTSBcKvGhpMdz0wbmHUJnTDgUbiE3CbapNVovqmAJ1jvGfdMR4m2cot1_-j_U1RXbkV-bb-iaSnq0OMxdePDCm_vbz_xK366xuMuoOwA6OOPlFvY16ePjpcj38q4KokfRs_ij0-oT7?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

---

© 2022 MORAI Inc.

×


---

## OpenSCENARIO Support

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/openscenario-support

Scenario Runner는 ASAM (Association for Standardization of Automation and Measuring Systems) 에서 정의한 [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) v1.2 기반으로 정의된 시나리오를 기반으로 동작하며, 다른 version (1.0 & 1.1) 도 지원중이다. 현재 OpenSCENARIO(.xosc) 파일의 모든 요소에 대한 읽기/편집/저장 기능을 지원하고 있으며, 이 중 MORAI SIM: Drive 와 연동하여 시나리오 테스트를 수행할 수 있는 요소를 점차 확충해나가고 있다. 본 문서는 각 OpenSCENARIO 요소에 대한 지원 여부 및 MORAI 에서 추가로 규정한 요소에 대해 다루고 있다.

### Level of support

( ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) : Support, ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) : Not support yet, ✔ : Limited support )

#### Scenario Definition

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `FileHeader` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `ParameterDeclarations` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `VariableDeclarations` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `CatalogLocations` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `RoadNetwork` | ✔ | MORAI SIM: Drive 와 연동하려면 반드시 아래 요소에 시나리오가 수행 될 맵에 대한 MGeo 폴더 경로를 지정해줘야 한다. `<LogicFile filepath="MGeo"/>`**(MORIA Sim 26.R1 / Scenario Runner 1.7.0)**1.7.0 기준으로 VariableDeclarations 에 Map 정의 가능하여 이전 버전에만 필요한 값이다. XML <VariableDeclarations> <VariableDeclaration name="moraiMap" variableType="string" value="R_KR_PG_KATRI"/> <VariableDeclaration name="evaluation" variableType="string" value="none"/> </VariableDeclarations> |

#### Storyboard

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `Storyboard` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `Story` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `Act` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `ManeuverGroup` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `Maneuver` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `Event` | ✔ | Priority 기능을 지원하지 않아 활성 중인 Event가 종료된 후 다른 Event 가 활성화 될 수 있다. |
| `Action` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |

#### Actions

##### Global Action

| **Category** | **Element** | **Support** | **Note** |
| --- | --- | --- | --- |
| EnvironmentAction | `EnvironmentAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | 일부 구현 완료. 동시에 Sun / Fog / Percipitation 을 Control 할수 없음 |
| EntityAction | `AddEntityAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | 일부 구현 완료. AddEntityAction 을했을때 경로와 속도가 random 과 0으로 지정된다. |
| `DeleteEntityAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |  |
| ParameterAction | `ParameterSetAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) | Deprecated in OSC v1.2 → VariableSetAction |
| `ParameterModifyAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) | Deprecated in OSC v1.2 → VariableModifyAction |  |
| VariableAction | `VariableSetAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `VariableModifyAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |  |
| InfrastructureAction | `TrafficSignalStateAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TrafficSignalControllerAction` | ✔ | 미리 선언된 Traffic signal controller 를 참조하지 않고, 특정 교차로의 신호를 직접 수정한다. |  |
| TrafficAction | `TrafficSourceAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TrafficSinkAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| `TrafficSwarmAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| `TrafficStopAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |  |

##### Private Action

| **Category** | **Element** | **Support** | **Note** |
| --- | --- | --- | --- |
| LongitudinalAction | `SpeedAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `LongitudinalDistanceAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| `SpeedProfileAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| LateralAction | `LaneChangeAction` | ✔ | AbsoluteLaneChangeAction 미지원 |
| `LaneOffsetAction` | ✔ | RelativeTargetLaneOffset 미지원 및 continuous / DynamicShape 와 미적용 |  |
| `LateralDistanceAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| VisibilityAction | `VisibilityAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| SynchronizeAction | `SynchronizeAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| ControllerAction | `ActivateControllerAction` | ✔ | 종방향 지원 |
| `AssignControllerAction` | ✔ | 종방향 지원Properties에서 아래 4가지 항목을 설정하여 운전 성향 조절 가능max_accelerationmin_distanceheadway_timeemergency_braking_distance |  |
| `OverrideControllerValueAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |
| TeleportAction | `TeleportAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| RoutingAction | `AcquirePositionAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `AssignRouteAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | `<Init>` 아래에만 선언할 수 있다. |  |
| `FollowTrajectoryAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | **(MORIA Sim 26.R1 / Scenario Runner 1.7.0)**1.7.0 이후 버전에는 Clothoid, Nurbs 지원 추가 |  |
| AppearanceAction | `LightStateAction` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | TailLight(Left / Right), Stop Signal Light, |
| `AnimationAction` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |  |

#### Conditions

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `Trigger` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `ConditionGroup` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `Condition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |

##### Entity Condition

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `AccelerationCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `CollisionCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | Map 의 기본 생성된 Object 해됨 |
| `DistanceCondition` | ✔ | Entity/Lane 좌표계를 사용 가능하며, `freespace` 값은 false로 고정된다. |
| `EndOfRoadCondition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `OffroadCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `ReachPositionCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `RelativeDistanceCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | Entity/Lane 좌표계를 사용 가능하며, `freespace` 값은 false로 고정된다. |
| `RelativeSpeedCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `SpeedCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `StandStillCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TimeHeadwayCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TimeToCollisionCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TraveledDistanceCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `RelativeClearanceCondition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |

##### Value Condition

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `ParameterCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `SimulationTimeCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `StoryboardElementStateCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TimeOfDayCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TrafficSignalCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `TrafficSignalControllerCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `UserDefinedValueCondition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `VariableCondition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |

#### Positions

| **Element** | **Support** | **Note** |
| --- | --- | --- |
| `GeoPosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `LanePosition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `RelativeLanePosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `RelativeObjectPosition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) | **(MORIA Sim 26.R1 / Scenario Runner 1.7.0)** |
| `RoadPosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `RelativeRoadPosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `RoutePosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `TrajectoryPosition` | ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) |  |
| `WorldPosition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |
| `RelativeWorldPosition` | ![(tick)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/check_mark_32.png) |  |

### MORAI-Defined & Custom Elements

각 MORAI-Defined & Custom Element 같은 경우 하위 항목에 OpenScenario Format 의 예제로 볼수 있다.

MORAI에서 정의한 OpenSCENARIO 요소

#### Scenario Definition

| **Element** | **Note** |
| --- | --- |
| `SimulatorInfo` | MORAI SIM: Drive 와 연동하려면 반드시 아래 요소가 선언되어야 한다. `<Map name = ${Map Name} />`**(MORIA Sim 26.R1 / Scenario Runner 1.7.0)**아래와 같이 VariableDeclarations 에 Map 정의가 가능하다. XML <VariableDeclarations> <VariableDeclaration name="moraiMap" variableType="string" value="R_KR_PG_KATRI"/> <VariableDeclaration name="evaluation" variableType="string" value="none"/> </VariableDeclarations> |

#### Actions

##### Global Action

| **Category** | **Element** | **Note** |
| --- | --- | --- |
| TrafficAction | `PedestrianSourceAction` | 보행자를 생성한다. |

##### Private Action

##### User-defined Action

| **Category** | **Element** | **Note** |
| --- | --- | --- |
| FaultInjectionAction | `InoperabilityAction` | 특정 기능을 비활성화한다. (accel, brake, steering-wheel 등) |
| `SensorFaultAction` | 차량에 부착된 센서의 위치를 변경 및 센서 데이터의 출력 여부를 지정 할수 있다. |  |
| `SteeringDisturbanceAction` | 입력된 조향 신호에 노이즈를 추가한다. |  |
| `TireFaultAction` | 지정된 타이어에 펑크가 발생한다. |  |
| CreateMapObjectAction | `CreatePuddleAction` | 지정된 위치에 물웅덩이(Puddle)를 생성한다. |
| ActivateMapObjectAction | `ActivateMapObjectAction` | MapObject 를 Activate 시킨다 (ex: MapObject: `Puddle_1`, 물웅덩이를 채워준다) |

#### Positions

| **Category** | **Element** | **Note** |
| --- | --- | --- |
| Position | `LinkPosition` | MGeo 상에 정의된 Link 중, 차량이 위치한 Link 및 해당 Link 위에서의 point index |

#### Spline

| **Category** | **Element** | **Note** |
| --- | --- | --- |
| Shape | `Spline` | 각 위치들(Vertices)의 정보들을 몇 차(degree)식으로 Trajectory Shape 을 결정한다. |

#### Evaluation

OpenSCENARIO 에서는 각 시나리오의 성공 및 실패 조건을 다루지 않는다. 따라서 별도의 Trigger 요소를 규정하여 시나리오의 성공 여부를 판단한다.

| **Element** | **Note** |
| --- | --- |
| `SuccessConditionGroup` | `ConditionGroup` 과 동일하게 동작하며, 활성화 될 경우 해당 시나리오가 성공한다. |
| `FailureConditionGroup` | `ConditionGroup` 과 동일하게 동작하며, 활성화 될 경우 해당 시나리오가 실패한다. |

---

© 2022 MORAI Inc.

×


---

## MORAI에서 정의한 OpenSCENARIO 요소

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/morai-openscenario

[data-colorid=ip8v2yb0fi]{color:#ff5630} html[data-color-mode=dark] [data-colorid=ip8v2yb0fi]{color:#cf2600}[data-colorid=ob0pxjo9qu]{color:#ff5630} html[data-color-mode=dark] [data-colorid=ob0pxjo9qu]{color:#cf2600}[data-colorid=mgvuy1osfp]{color:#ff5630} html[data-color-mode=dark] [data-colorid=mgvuy1osfp]{color:#cf2600}[data-colorid=jhvzn06h4s]{color:#ff5630} html[data-color-mode=dark] [data-colorid=jhvzn06h4s]{color:#cf2600}이 페이지에서는 다양한 시나리오의 구현을 위해 **Scenario Runner**에서 OpenSCENARIO에 추가하거나 수정한 요소를 정리한다. 각 항목은 해당 요소에 대한 정의 및 사용예시로 구성되어 있다.

**목차**

---

### 추가된 요소

#### Vehicle Property

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| Check TL | string | 0..1 | Traffic Light 정보 고려 여부 |
| ![(error)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/cross_mark_32.png) Apply ACC | string | 0..1 | 전방 장애물 고려 여부(Deprecated)**(MORIA Sim 26.R1 / Scenario Runner 1.7.0)****1.7.0 이후 버전에서는 본 기능을 공식 지원하지 않습니다. ActivateControllerAction 사용을 권장합니다.** |
| Is Kinematics | string | 0..1 | Kinematics Model 선택 여부 |

#### Pedestrian Property

Pedestrian 이라는 Scenario Object 를 생성 할때, Property 의 값으로 Pedestrian 의 Animation 이 동작 조건을 정의한다. 동작 조건은 Ego 와의 거리와 Animation 이 동작 했을 때이며, 동작 이후의 거리를 정의한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| activeDistance | string | 1..1 | Ego 와 의 일정 거리에서, Pedestrian Animation 조정 |
| moveDistance | string | 1..1 | Pedestrian Animation 의 동작 거리 |

CODE

```
<Properties>
    <Property name="activeDistance" value="20"/>
    <Property name="moveDistance" value="30"/>
</Properties>
```

#### MiscObject Property

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| moving_obstacle | string | 1…1 | TCR On / Off |

CODE

```
<Properties>
    <Property name="moving_obstacle" value="true"/>
</Properties>
```

#### ObjectController Property

Scenario Object 에 Controller 를 지정할 때, Property의 값으로 Scenario Object 의 동작 방식을 제어한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| max_acceleration | float | 0..1 | 최대 가속도 |
| comfort_deceleration | float | 0..1 | 편안한 감속도 |
| min_distance | float | 0..1 | 정지시 최소 유지 거리 |
| headway_time | float | 0..1 | 선행 차량과의 시간 간격 |
| acceleration_exponent | - | 0..1 | 가속도 지수 (속도 변화에 따른 민감도 값) |
| deceleration_limit | float | 0..1 | 일반 주행(Free Flow, Car-Following)에서의 최대 감속 제한 |
| emergency_braking_distance | float | 0..1 | 긴급 제동 발동 거리 (앞차와의 거리 기준) |
| max_deceleration | float | 0..1 | 긴급 제동 시 적용되는 최대 감속도 |

#### Evaluation

Triggers to stop a simulation and Returns a result of the simulation. (None, Success, Failure)

When there isn’t any condition group, it wouldn’t be triggered and returns None.

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| SuccessConditionGroup | 0..* | At least one success condition group should be triggered to succeed the simulation. |
| FailureConditionGroup | 0..* | All failure condition groups should not be triggered to succeed the simulation. |

CODE

```
  <Evaluation>
    <SuccessConditionGroup>
      <Condition name="Ego_position" delay="0" conditionEdge="once">
        <ByEntityCondition>
          <TriggeringEntities triggeringEntitiesRule="any">
            <EntityRef entityRef="Ego" />
          </TriggeringEntities>
          <EntityCondition>
            <DistanceCondition value="5.0" freespace="false" rule="lessThan">
              <Position>
                <RelativeObjectPosition entityRef="Stop_Point" dx="0" dy="0" />
              </Position>
            </DistanceCondition>
          </EntityCondition>
        </ByEntityCondition>
      </Condition>
      <Condition name="Ego_position_limit" delay="0" conditionEdge="none">
        <ByEntityCondition>
          <TriggeringEntities triggeringEntitiesRule="any">
            <EntityRef entityRef="Ego" />
          </TriggeringEntities>
          <EntityCondition>
            <DistanceCondition value="500.0" freespace="false" rule="lessThan">
              <Position>
                <RelativeObjectPosition entityRef="Stop_Point" dx="0" dy="0" />
              </Position>
            </DistanceCondition>
          </EntityCondition>
        </ByEntityCondition>
      </Condition>
    </SuccessConditionGroup>
    <FailureConditionGroup>
      <Condition name="Collision_occured" delay="0" conditionEdge="once">
        <ByValueCondition>
          <UserDefinedValueCondition name="collision" rule="equalTo" value="true" />
        </ByValueCondition>
      </Condition>
    </FailureConditionGroup>
  </Evaluation>
```

#### LinkPosition

MORAI에서 사용하는 MGeo 데이터의 Link를 활용한 Position을 정의한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| id | string | 1..1 | link id |
| index | int | 1..1 | link 내 point index |
| offset | double | 0..1 | 지정된 index 지점에서 Link 중심선으로부터의 측면 오프셋. 중심선 진행 방향의 법선 방향으로 적용된다. 할당되지 않으면 **0**으로 해석 단위: [m]. **(Left +, Right- )** |

CODE

```
<Route name="NPC_1_Route" closed="false" afterCompletion="hide">
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010381" index="2"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010402"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010422"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010073" index="10"/>
        </Position>
    </Waypoint>
</Route>
```

#### PositionInLinkCoordinates

RoutePosition에서 Link Position을 활용할 수 있도록 추가된 요소이다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| id | string | 1..1 | link id |
| index | unsigned int | 1..1 | link 내 point index |

CODE

```
<RoutePosition>
  <RouteRef />
  <Orienation />
  <InRoutePosition>
    <FromLinkCoordinates id=_3_0_L1_ index="2" />
  </InRoutePosition>
</RoutePosition>
```

#### Spline

Trajectory 의 Shape 은 다항식으로 표현되며, 다항식의 차수(degree) 결정해야 한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| degree | unsigned short | 1..1 | 다항식의 차수(계수) [1차 ~5 차] |
| verticies | list | 1..1 | 각 Vertex 의 List 정의 |

XML

```xml
<Trajectory closed="false" name="trajectory_name">
  <Shape>
    <Spline degree="2">
      <Vertex time="5">
        <Position>
          <LinkPosition id="48638" index="100"/>
        </Position>
      </Vertex>
      <Vertex time="5">
        <Position>
          <LinkPosition id="48650" index="66"/>
        </Position>
      </Vertex>
    </Spline>
  </Shape>
</Trajectory>
```

#### SimulationInfo

Simulator 관련 정보를 정의하기 위해 SimulatorInfo 요소를 정의한다.

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| ScenarioFile | 1..1 | 로드할 MORAI SIM built-in 시나리오 파일 이름 정의 |
| Map | 1..1 | 로드할 MORAI SIM 맵 이름 정의 |

CODE

```
<SimulatorInfo>
  <ScenarioFile name="1.json" />
  <Map name="V_RHT_HighwayJunction_1" />
</SimulatorInfo>
```

#### GlobalAction

##### PedestrianSourceAction

Spawns pedestrian objects.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| radius | double | 1..1 | Radius of spawn area (not implemented) |
| count | integer | 0..1 | Maximum spawn pedestrians (default: 1e6) |
| period | double | 0..1 | Period on which pedestrians appear at the source location (sec). (default: 1 sec) |
| mode | enum | 1..1 | Defines behavior of a spawned pedestrian. (once/ closedLoop/ repeat/ loop/ newPath/ stand) |
| pedestrianType | string | 0..1 | Defines type(model) of a spawned pedestrian. (random (default) / man(1-8)_ai / woman(4-8)_ai) |

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| Position | 1..1 | Position of the spawn area |
| Waypoint | 0..1 | Target location of spawned pedestrians |

CODE

```
<GlobalAction>
  <TrafficAction>
    <PedestrianSourceAction radius="10" count="5" period="10"
                            mode="closedLoop" pedestrianType="random" >
      <Position>
        <WorldPosition x="307.426506536839" y=" 118.975891092156" z="0.15"/>
      </Position>
      <Waypoint routeStrategy="random">
        <Position>
          <WorldPosition x="314.494013321081" y="191.865" z="0.15" h="0.0" p="0.0" r="0.0"/>
        </Position>
      </Waypoint>
    </PedestrianSourceAction>
  </TrafficAction>
</GlobalAction>
```

#### UserDefinedAction

##### CreateMapObjectAction

###### CreatePuddleAction

Creates a puddle at the given position. The puddle area has scaled friction.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name | string | 1..1 | ID of a puddle |
| positionX | double | 1..1 | The x coordinate value. (unit: m) |
| positionY | double | 1..1 | The y coordinate value. (unit: m) |
| positionZ | double | 1..1 | The z coordinate value. (unit: m) |
| rotationX | double | 1..1 | The roll angle of the object, defining a mathematically positive rotation about the x-axis (unit: deg) |
| rotationY | double | 1..1 | The pitch angle of the object, defining a mathematically positive rotation about the y-axis (unit: deg) |
| rotationZ | double | 1..1 | The yaw angle of the object, defining a mathematically positive rotation about the z-axis (unit: deg) |
| radiusH | double | 1..1 | Horizontal radius of the puddle (unit: m) |
| radiusV | double | 1..1 | Vertical radius of the puddle (unit: m) |
| height | double | 1..1 | Height of the puddle (unit: m) |
| friction | double | 1..1 | Multiplier of the friction on the puddle (range: [0..1]) |

CODE

```
<Init>
  <UserDefinedAction>
    <CreateMapObjectAction>
      <CreatePuddleAction name="Puddle_1" positionX="20.2" positionY="17.6" positionZ="0.19"
                                          rotationX="0.0" rotationY="0.0" rotationZ="90.0"
                                          radiusH="1.5" radiusV="2.0" height="0.01" friction="0.1" />
    </CreateMapObjectAction>
  </UserDefinedAction>
</Init>
```

##### ActivateMapObjectAction

Activate or deactivate Map Object in simulation.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name | string | 1..1 | ID of the map object to be activated or deactivated. |
| activation | boolean | 1..1 | true: Activate map object. false: Deactivate map object. |

CODE

```
<Init>
  <UserDefinedAction>
    <ActivateMapObjectAction name="Puddle_1" activation="false" />
  </UserDefinedAction>
</Init>
```

##### FaultInjectionAction

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| entityRef | string | 1..1 | ID of entity to inject fault |

###### DamageAction

Injects a type-wise damage to the actors. The corresponding state would be maintained as it was when the action was activated.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| type | enum | 1..1 | Damage type (currently, ‘fire’ is the only option) |

CODE

```
<Action name="Fire_Action">
  <UserDefinedAction>
    <FaultInjectionAction entityRef="Ego">
      <DamageAction type="fire" />
    </FaultInjectionAction>
  </UserDefinedAction>
</Action>
```

###### InoperabilityAction

Injects a type-wise Inoperability to the actors. The corresponding state would be maintained as it was when the action was activated.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| type | enum | 1..1 | Fault type (accel, brake, steer) |

CODE

```
<Action name="Steering_Fault_Action">
  <UserDefinedAction>
    <FaultInjectionAction entityRef="Ego">
      <InoperabilityAction type="steer" />
    </FaultInjectionAction>
  </UserDefinedAction>
</Action>
```

###### SensorFaultAction

Injects an sensor fault to a sensor. The sensor would be shifted by the given offsets.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name | string | 1..1 | ID of a sensor in MGeo |
| active | boolean | 1..0 | true: publish sensor datafalse: stop publishing sensor data |
| offsetX | double | 1..0 | Artificial translation offset of the sensor in X coordinate |
| offsetY | double | 1..0 | Artificial translation offset of the sensor in Y coordinate |
| offsetZ | double | 1..0 | Artificial translation offset of the sensor in Z coordinate |
| offsetP | double | 1..0 | Artificial rotation offset of the sensor about the X axis |
| offsetQ | double | 1..0 | Artificial rotation offset of the sensor about the Y axis |
| offsetR | double | 1..0 | Artificial rotation offset of the sensor about the Z axis |

CODE

```
<Action name="Sensor_Fault_Action">
  <UserDefinedAction>
    <FaultInjectionAction entityRef="Ego">
      <SensorFaultAction name="3" active="true" offsetX="1" offsetY="1" offsetZ="1" 
                                  offsetP="1" offsetQ="1" offsetR="1" />
    </FaultInjectionAction>
  </UserDefinedAction>
</Action>
```

###### SteeringDisturbanceAction

Injects a steering disturbance to actors.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| duration | double | 1..1 | Duration of the fault. (unit: sec , range: [0..inf]) |
| magnitude | double | 1..1 | Magnitude of the fault. (range: [-1..1]) |

CODE

```
<Action name="Steering_Disturbance_Action">
  <UserDefinedAction>
    <FaultInjectionAction entityRef="Ego">
      <SteeringDisturbanceAction duration="12.5" magnitude="0.8" />
    </FaultInjectionAction>
  </UserDefinedAction>
</Action>
```

###### TireFaultAction

Injects a tire fault to actors.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| tireIndex | enum | 1..1 | Index of a tire of the actors ( 0, 1, 2, 3, … ) |

CODE

```
<Action name="Tire_Fault_Action">
  <UserDefinedAction>
    <FaultInjectionAction entityRef="Ego">
      <TireFaultAction tireIndex="3" />
    </FaultInjectionAction>
  </UserDefinedAction>
</Action>
```

### 수정된 요소

#### Route

afterCompletion 속성을 추가로 정의하여 경로 주행 후 동작을 정의한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name | string | 1..1 | Name of the route. |
| closed | boolean | 1..1 | In a closed route, the last waypoint is followed by the first waypoint to create a closed route. |
| afterCompletion | enum | 1..1 | Behavior after arriving at the target waypoint ( ‘hide’: remove the vehicle (default), ‘stop’: stop, ‘random’: drive along a random route) |

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| ParameterDeclarations | 0..* | Definition of additional parameters. |
| Waypoints | 2..* | At least two waypoints are needed to define a route. |

CODE

```
<Route name="NPC_1_Route" closed="false" afterCompletion="hide">
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010381" index="2"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010402"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010422"/>
        </Position>
    </Waypoint>
    <Waypoint routeStrategy="shortest">
        <Position>
            <LinkPosition id="A219BS010073" index="10"/>
        </Position>
    </Waypoint>
</Route>
```

#### SpeedActionTarget

##### AbsoluteTargetSpeed

**MGeo** 상에 정의된 Link Speed 및 속도의 오차범위를 추가로 정의한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| value | double | 1..1 | Value of the absolute speed. ( link_type: [0..1], custom_type: [0..inf] (m/s)) |
| type (added) | enum | 0..1 | Type of the defined speed. ( ‘link’, ‘custom’ ) |
| range (added) | double | 0..1 | Range of the speed ( link_type: [0..1], custom_type: [0..inf] (m/s)) |

CODE

```
<AbsoluteTargetSpeed type="link" value="0.05" />
<AbsoluteTargetSpeed type="custom" value="-15" range="5" />
<AbsoluteTargetSpeed type="link" value="0.9" range="0.05" />
```

##### RelativeTargetSpeed

**MGeo** 상에 정의된 Link Speed 및 상대속도의 오차범위를 추가로 정의한다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| continuous | boolean | 1..1 | True: a controller tries to maintain a continuous relative speed (not implemented) |
| entityRef | string | 1..1 | Reference entity |
| speedTargetValueType | enum | 1..1 | The value is either a delta or a factor. |
| value | double | 1..1 | Value of the relative speed. This value is either given as a delta or as a factor. ( link_type: [0..1], custom_type: [0..inf] (m/s)) |
| type (added) | enum | 0..1 | Type of the defined speed. ( ‘link’, ‘custom’ ) |
| range (added) | double | 0..1 | Range of the speed ( link_type: [0..1], custom_type: [0..inf] (m/s)) |

CODE

```
<RelativeTargetSpeed continuous="false" entityRef="Ego" speedTaretValueType="delta"
                     type="link" value="0.05" />
<RelativeTargetSpeed continuous="false" entityRef="Ego" speedTaretValueType="delta"
                     type="custom" value="-15" range="5" />
<RelativeTargetSpeed continuous="false" entityRef="Ego" speedTaretValueType="factor"
                     type="custom" value="1.1" range="0.05" />
<RelativeTargetSpeed continuous="true" entityRef="Ego" speedTaretValueType="factor"
                     type="link" value="0.9" range="0.05" />
```

#### TrafficDefinition

OpenScenario 에서와는 달리 각 Vehicle Distribution 마다 Controller를 지정하지 않아도 된다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name | string | 1..1 | Name of the traffic definition. |

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| VehicleCategoryDistribution | 1..1 | Distribution of vehicle categories within the traffic. |
| ControllerDistribution | 0..1 | Distribution of controllers within this traffic. |

CODE

```
<TrafficDefinition name="TrafficDefinition_1">
  <VehicleCategoryDistribution>
    <VehicleCategoryDistributionEntry model="2016_Hyundai_Ioniq" weight="1" />
  </VehicleCategoryDistribution>
</TrafficDefinition>
```

##### VehicleCategoryDistributionEntry

차량의 카테고리 또는 특정 모델을 지정하여 정의할 수 있다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| model (added) | string | 0..1 | Model name of the vehicle which is implemented in MORAI SIM: Drive. If it’s specified, ‘category’ would be ignored. Either ‘model’ or ‘category’ is required. |
| category (modified) | enum | 0..1 | The category of the vehicles that appear in traffic. (bicycle/ bus/ car/ motorbike/ semitrailer/ trailer/ train/ tram/ truck/ van/ **mini**/ **compact**/ **middle**/ **large**) |
| weight | double | 1..1 | The weight of a vehicle category within a traffic distribution. [0..inf] |

CODE

```
<TrafficDefinition name="TrafficDefinition_1">
  <VehicleCategoryDistribution>
    <VehicleCategoryDistributionEntry model="2016_Hyundai_Ioniq" weight="1" />
  </VehicleCategoryDistribution>
</TrafficDefinition>

<TrafficDefinition name="TrafficDefinition_2">
  <VehicleCategoryDistribution>
    <VehicleCategoryDistributionEntry category="compact" weight="1" />
    <VehicleCategoryDistributionEntry category="middle" weight="1" />
    <VehicleCategoryDistributionEntry category="large" weight="1" />
  </VehicleCategoryDistribution>
</TrafficDefinition>

<TrafficDefinition name="TrafficDefinition_3">
  <VehicleCategoryDistribution>
    <VehicleCategoryDistributionEntry model="2016_Hyundai_Ioniq" weight="1" />
    <VehicleCategoryDistributionEntry category="mini" weight="1" />  <!-- would be ignored -->
  </VehicleCategoryDistribution>
</TrafficDefinition>
```

#### GlobalAction

##### TrafficSignalControllerAction

교차로 신호 Phase를 변경한다. Phase 요소를 하위에 정의하여 교차로 신호 Phase 정의 자체를 변경할 수도 있고, Phase 요소 없이 phase 속성만 정의하여 현재 신호만 변경할 수도 있다.

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| trafficSignalControllerRef (modified) | string | 1..1 | ID of the intersection controller in MGeo. |
| phase (modified) | unsigned int | 1..1 | The phase of the signal controller would be changed to the given value (0, 1, 2, …) |
| PhaseRef (deleted) | string | 0..0 | reference to pre-defined Phase |

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| Phase (moved) | 0..1 | (According to OpenSCENARIO, It should be defined under RoadNetwork. Now it is directly defined under the action.) |

CODE

```
<GlobalAction>
  <InfrastructureAction>
    <TrafficSignalAction>
      <TrafficSignalControllerAction trafficSignalControllerRef="IntTL1" phase="1" />
      <!-- The phase of "IntTL1" would be changed to phase "1" -->
    </TrafficSignalAction>
  </InfrastructureAction>
</GlobalAction>
```

CODE

```
<GlobalAction>
  <InfrastructureAction>
    <TrafficSignalAction>
      <TrafficSignalControllerAction trafficSignalControllerRef="IntTL1" phase="1">
        <Phase name="phase1" duration="5">
          <TrafficSignalState trafficSignalId="C119BS010034" state="G_with_GLeft"/>
          <TrafficSignalState trafficSignalId="C119BS010072" state="R"/>
          <TrafficSignalState trafficSignalId="C119BS010033" state="R"/>
        </Phase>
        <Phase name="phase2" duration="5">
          <TrafficSignalState trafficSignalId="C119BS010034" state="R"/>
          <TrafficSignalState trafficSignalId="C119BS010072" state="G_with_GLeft"/>
          <TrafficSignalState trafficSignalId="C119BS010033" state="R"/>
        </Phase>
        <Phase name="phase3" duration="5">
          <TrafficSignalState trafficSignalId="C119BS010034" state="R"/>
          <TrafficSignalState trafficSignalId="C119BS010072" state="R"/>
          <TrafficSignalState trafficSignalId="C119BS010033" state="G_with_GLeft"/>
        </Phase>
      </TrafficSignalControllerAction>
    </TrafficSignalAction>
  </InfrastructureAction>
</GlobalAction>
```

##### TrafficSignalStateAction

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| name (modified) | string | 1..1 | ID of the signal in MGeo. |
| state (modified) | enum | 1..1 | Targeted state of the signal. The available states are listed in TrafficSignal list of `_TRAFFICLIGHT_LIGHTCOLOR` |
| impulse (added) | boolean | 1..1 | true: return to the phase cycle, false: keep the given signal state |
| setSibling (added) | boolean | 1..1 | true: change all synced signals, false: change this signal only |

CODE

```
<GlobalAction>
  <InfrastructureAction>
    <TrafficSignalAction>
      <TrafficSignalStateAction name="C119BS010034" state="SG" impulse="false" setSibling="false"/>
    </TrafficSignalAction>
  </InfrastructureAction>
</GlobalAction>
```

##### TrafficSourceAction

Spawns vehicle objects. Several attributes are added for the compatibility with MORAI SIM: Drive

| **Attribute** | **Type** | **Cardinality** | **Description** |
| --- | --- | --- | --- |
| radius | double | 1..1 | Radius of spawn area (not implemented) |
| velocity | double | 0..1 | Initial velocity of spawned vehicles (km/h). It would be ignored when ‘InitSpeed’ is defiend. |
| count | integer | 0..1 | Maximum spawn vehicles (default: 1e6) |
| rate | double | 0..1 * | Rate on which vehicles appear at the source location (vehicles/s). It would be ignored when ‘Period’ is defined |
| period | double | 0..1 * | Period on which vehicles appear at the source location (sec). Either ‘rate’ or ‘period’ is required. |
| periodRange | double | 0..1 | Range of the period (sec) |
| offset | double | 0..1 | Signed number the vehicle should respect as an offset from the center of the current lane. [-1..1] |
| offsetRange | double | 0..1 | Range of the offset [0..1] |
| closedLoop | bool | 0..1 | True: spawned vehicles would come back to the spawn location. False: vehicles would be disappear at their target location. |
| discretionaryLaneChange | bool | 0..1 | True: a spawned vehicle would change its lane randomly. False: the vehicle would keep its initial lane. |

| **Element** | **Cardinality** | **Description** |
| --- | --- | --- |
| Position | 1..1 | Position of the spawn area |
| InitSpeed | 1..1 | Initial speed of spawned vehicles |
| DesiredSpeed | 1..1 | Desired speed of spawned vehicles |
| Waypoint | 0..1 | Target location of spawned vehicles |
| TrafficDefinition | 1..1 | Defines the vehicle and controller distribution for the source. |

CODE

```
<GlobalAction>
  <TrafficAction>
    <TrafficSourceAction radius="10" count="5" period="10" periodRange="5" 
                         offset="0" offsetRange="0.1"
                         closedLoop="false" discretionaryLaneChange="true" >
      <Position>
        <LinkPosition id="A219BS010313" index="0"/>
      </Position>
      <InitSpeed>
        <AbsoluteTargetSpeed type="link" value="0.35" range="0.05"/>
      </InitSpeed>
      <DesiredSpeed>
        <AbsoluteTargetSpeed type="custom" value="55" range="15"/>
      </DesiredSpeed>
      <Waypoint routeStrategy="random">
        <Position>
          <LinkPosition id="A219BS010319" index="280"/>
        </Position>
      </Waypoint>
      <TrafficDefinition name="TrafficDefinition_1">
        <VehicleCategoryDistribution>
          <VehicleCategoryDistributionEntry model="2014_Kia_K7" weight="1" />
        </VehicleCategoryDistribution>
      </TrafficDefinition>
    </TrafficSourceAction>
  </TrafficAction>
</GlobalAction>
```

---

© 2022 MORAI Inc.

×


---

## MORAI Custom → OSC v1.2 Converting

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/morai-custom-osc-v1-2-converting

**목차**

---

### 1. Overview

해당 기능은 MORAI Custom 형식으로 작성된 Scenario 파일을 OpenSCENARIO v1.2 표준 형식으로 변환하는 기능을 제공합니다.

### 2. How to Use

#### 2.1 Convert OpenSCENARIO (MORAI Custom to v1.2)

이미 만들어진 Scenario File 에서 MORAI Custom 표준을 OpenSCENARIO 표준으로 변경합니다.

**Step 1. Converter 실행**

- 상단 메뉴에서 아래 기능 선택
  - **File → Convert OpenSCENARIO (MORAI Custom to v1.2)**

![image-20260331-074048.png](https://help-morai-sim.scrollhelp.site/__attachments/884834308/image-20260331-074048.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**Step 2. Scenario 파일 선택**

- 변환할 MORAI Custom Scenario 파일 선택

![image-20260331-074127.png](https://help-morai-sim.scrollhelp.site/__attachments/884834308/image-20260331-074127.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**Step 3. 변환 실행**

- 선택한 Scenario가 자동으로 OpenSCENARIO v1.2 형식으로 변환됨

**Step 4. 결과 확인**

- **File Name 변경**
  - Before: **Scenario_SpeedAction-1.xosc**
  - After: **Scenario_SpeedAction-1_converted.xosc**

#### 2.2 Save As OpenSCENARIO → OpenSCENARIO (v1.2)

생성 / 편집 중인 Scenario 에서 사용중인 MORAI Custom 표준을 OpenSCENARIO 표준으로 변경합니다.

- 상단 메뉴에서 아래 기능 선택
  - File → Save As OpenSCENARIO → OpenSCENARIO (v1.2)

![image-20260331-074726.png](https://help-morai-sim.scrollhelp.site/__attachments/884834308/image-20260331-074726.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 3. Conversion Result

#### 3.1 Position 구조 변경

Link 기반 위치 → Lane 기반 표준 Position으로 변환

##### Before (MORAI Custom)

CODE

```
<Position>
    <LinkPosition id="_69_0_R1_" index="40" offset="-1"/>
</Position>
```

##### After (OpenSCENARIO v1.2)

CODE

```
<Position>
    <LanePosition roadId="4000324" laneId="_69_0_R1_" s="20.0" offset="-1.0"/>
</Position>
```

#### 3.2 Map 정보 구조 변경

MORAI Custom Map 정의 → OpenSCENARIO Variable 구조로 변환

##### Before

CODE

```
<SimulatorInfo>
    <Map name="V_Extra_Scene"/>
</SimulatorInfo>
```

##### After

CODE

```
<VariableDeclarations>
    <VariableDeclaration name="moraiMap" variableType="string" value="V_Extra_Scene"/>
</VariableDeclarations>
```

×


---

## 시뮬레이션 결과 확인

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-6

이 페이지는 **Scenario Runner**에서 수행한 시뮬레이션의 결과를 확인하는 방법을 안내한다.

---

### 실시간 시뮬레이션 상태 확인

'Simulation Status' 창을 통해 실시간 시뮬레이션 상태를 확인할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/214269953/image-20221017-065207.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

‘Simulation Status’ 창 기본 화면

#### (1) 개요

현재 시뮬레이션 중인 시나리오 파일의 이름과, 시뮬레이션 시간이 표시된다.

#### (2) Status Monitor

시나리오 파일에 정의된 Storyboard 요소에 대한 이름과 상태 정보가 표시된다.

- Storyboard 요소의 상태 목록
  - init : storyboard가 시작 되기 이전 상태
  - standby : 활성화 대기 상태, 특정 조건을 만족할 경우 running 상태로 천이됨
  - running : 요소가 활성화되어 정의된 동작을 수행하고 있는 상태, 동작을 완료한 후 standby 상태로 회귀하거나, complete 상태로 천이됨
  - complete : 요소에 정의된 동작을 마치고 종료된 상태
  - end : storyboard가 종료된 후의 상태

#### (3) Evaluation Index

Ego 차량의 주행안전성 평가 지표가 표시된다.

##### 안전성 평가 지표

1. Safe Distance (m) : 전방 차량과의 거리
2. Time To Collision (sec) : 전방 차량과의 충돌예상시간
3. Way-off Distance (m) : 차량이 차선 중앙으로부터 벗어난 거리
4. Lateral Acceleration (m/s2) : 차량의 횡방향 가속도
5. Longitudinal Acceleration (m/s2) : 차량의 종방향 가속도
6. Speed Excess (km/h) : 주행 중인 도로의 최대 규정 속도 초과치
7. Speed Deficit (km/h) : 주행 중인 도로의 최소 규정 속도 미달치

### 시뮬레이션 결과 확인

'Simulation Result' 창을 통해 시뮬레이션 결과를 확인할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/214269953/image-20221017-091729.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

‘Simulation Result’ 창 기본 화면

#### (1) 개요

- Total : 해당 배치 시뮬레이션에 포함된 총 시나리오 개수
- ✔️ : 시나리오의 Evaluation 항목에 정의된 조건에 따라 성공한 시나리오의 개수
- ❌ : 시나리오의 Evaluation 항목에 정의된 조건에 따라 실패한 시나리오의 개수
- ![(question)](https://pf-emoji-service--cdn.us-east-1.prod.public.atl-paas.net/atlassian/question_mark_32.png) : Storyboard StopTrigger에 의해 종료되었거나, 사용자에 의해 종료된 시나리오의 개수
- Clear : 데이터 초기화 버튼

#### (2) 시나리오 목록

각 시나리오의 성공여부 및 총 시뮬레이션 시간을 확인할 수 있다.

#### (3) 시뮬레이션 결과 요약

시뮬레이션 결과는 아래 두 가지 항목으로 이루어져 있다.

- Ego 차량에 대한 주행안전성 평가지표 (각 지표에 대해 가장 위험한 상황일 때의 수치)
  - 예, 전방 차량과의 최소 거리, 차선 중앙으로부터의 최대 이격거리, 최대 횡가속도 등
- 시간에 따른 Storyboard 요소의 상태 및 오류 로그

#### (4) 진행 바 및 로그 폴더

- 진행 바: 전체 시나리오 수 대비 시뮬레이션 수행완료된 시나리오 수 표시
- Open Log Folder : 로그가 저장된 폴더를 탐색기에서 연다.
  - 로그 저장 경로
    - logs_scenario_runner/simulation_{배치 시뮬레이션 수행 시각}/{배치 시뮬레이션 수행 시각}_{지도 이름}_{시나리오 이름}_eventlog.csv
    - logs_scenario_runner/simulation_{배치 시뮬레이션 수행 시각}/{배치 시뮬레이션 수행 시각}_{지도 이름}_{시나리오 이름}_statelog.csv

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/note-macro-icon--625ed763b7218abfc3ad.svg)

시뮬레이션을 수행함에 따라 해당 폴더에 시뮬레이션 로그가 누적되므로, 파일 용량 증가에 주의할 필요가 있다.

---

© 2022 MORAI Inc.

×


---

## Scenario Runner Clients

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/scenario-runner-clients

Introduction

×


---

## Introduction

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/introduction

×


---

## API Documentation

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/api-documentation

Python API

×


---

## Python API

> 원문: https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/python-api

### [data-colorid=czgux07ukh]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=czgux07ukh]{color:#004eb3}[data-colorid=y352eoz3wf]{color:#36b37e} html[data-color-mode=dark] [data-colorid=y352eoz3wf]{color:#4cc994}[data-colorid=gx7wz2h9be]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=gx7wz2h9be]{color:#004eb3}[data-colorid=iy31ony192]{color:#36b37e} html[data-color-mode=dark] [data-colorid=iy31ony192]{color:#4cc994}[data-colorid=sybngzgl5l]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=sybngzgl5l]{color:#004eb3}[data-colorid=jgipokdphg]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=jgipokdphg]{color:#004eb3}[data-colorid=eyw07oq2fc]{color:#36b37e} html[data-color-mode=dark] [data-colorid=eyw07oq2fc]{color:#4cc994}[data-colorid=y25jwlr8zu]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=y25jwlr8zu]{color:#004eb3}[data-colorid=sw0htun0wy]{color:#36b37e} html[data-color-mode=dark] [data-colorid=sw0htun0wy]{color:#4cc994}[data-colorid=bdzevy8tsy]{color:#36b37e} html[data-color-mode=dark] [data-colorid=bdzevy8tsy]{color:#4cc994}[data-colorid=emyemwi3a4]{color:#36b37e} html[data-color-mode=dark] [data-colorid=emyemwi3a4]{color:#4cc994}[data-colorid=ks5l10iwu9]{color:#36b37e} html[data-color-mode=dark] [data-colorid=ks5l10iwu9]{color:#4cc994}[data-colorid=bgcgvdcgli]{color:#36b37e} html[data-color-mode=dark] [data-colorid=bgcgvdcgli]{color:#4cc994}[data-colorid=ylw684tvb4]{color:#36b37e} html[data-color-mode=dark] [data-colorid=ylw684tvb4]{color:#4cc994}[data-colorid=vxg5km5xuo]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=vxg5km5xuo]{color:#004eb3}[data-colorid=vtfbsedjd3]{color:#36b37e} html[data-color-mode=dark] [data-colorid=vtfbsedjd3]{color:#4cc994}[data-colorid=oi2o4ec7dk]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=oi2o4ec7dk]{color:#004eb3}[data-colorid=b91x32n8l9]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=b91x32n8l9]{color:#004eb3}[data-colorid=ehsqamy04w]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=ehsqamy04w]{color:#004eb3}[data-colorid=keti4i5h78]{color:#36b37e} html[data-color-mode=dark] [data-colorid=keti4i5h78]{color:#4cc994}[data-colorid=d1v7rxaqbo]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=d1v7rxaqbo]{color:#004eb3}[data-colorid=njkjktdlig]{color:#36b37e} html[data-color-mode=dark] [data-colorid=njkjktdlig]{color:#4cc994}[data-colorid=gwb5lfd2hp]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=gwb5lfd2hp]{color:#004eb3}[data-colorid=whtcag4m73]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=whtcag4m73]{color:#004eb3}[data-colorid=g8mih6g01r]{color:#36b37e} html[data-color-mode=dark] [data-colorid=g8mih6g01r]{color:#4cc994}[data-colorid=j5l6ornqsu]{color:#36b37e} html[data-color-mode=dark] [data-colorid=j5l6ornqsu]{color:#4cc994}[data-colorid=sez09ipzqq]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=sez09ipzqq]{color:#004eb3}[data-colorid=x0397qfkcb]{color:#36b37e} html[data-color-mode=dark] [data-colorid=x0397qfkcb]{color:#4cc994}[data-colorid=md72vvwenp]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=md72vvwenp]{color:#004eb3}[data-colorid=ndhcv5vncv]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=ndhcv5vncv]{color:#004eb3}[data-colorid=frhj7mmvu3]{color:#36b37e} html[data-color-mode=dark] [data-colorid=frhj7mmvu3]{color:#4cc994}[data-colorid=g8glz8z9gw]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=g8glz8z9gw]{color:#004eb3}[data-colorid=nef3v49sy3]{color:#36b37e} html[data-color-mode=dark] [data-colorid=nef3v49sy3]{color:#4cc994}[data-colorid=a4lfrole0s]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=a4lfrole0s]{color:#004eb3}[data-colorid=lbryqcxi48]{color:#36b37e} html[data-color-mode=dark] [data-colorid=lbryqcxi48]{color:#4cc994}[data-colorid=h5rmo4rc4e]{color:#4c9aff} html[data-color-mode=dark] [data-colorid=h5rmo4rc4e]{color:#004eb3}Introduction

이 참조 문서에는 Scenario Runner GUI 를 사용하지 않고, Scenario Runner Python API 를 사용해서, OpenScenario Data 에 기반으로 한 Scenario 파일을 **MORAI: SIM Drive** 와 연동해서 Simulation 를 진행할 수 있습니다.

이 Document 에서는 Scenario Runner Python API의 세부 내용이 포함이 되어있으며, Scenario Runner Release 버전에 따라서, API의 기능 명세가 다를 수 있습니다.

### Python API Reference

현재 Scenario Runner 에서 지원하는 Python API는 아래와 같습니다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

**Scenario Runner 의 API 기능을 사용하기 위해서는**[시나리오 설정 API](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/python-api.html#:~:text=set_scenario_config(self%2C%20%20scenario_file_list%2C%20controller_mode%3DControllerFrom.BuiltIn%2C%20sync_mode%3DFalse%2C%20num_iterations%3D1%2C%20is_ignore_init_ego%3DFalse)) **를 우선으로 호출 해야 한다.**

#### Instance: OpenScenario ClientAPI

Scenario Runner Client 와 연결하는 API 입니다. Scenario Runner Client 는 OpenScenario Data 를 **MORAI: SIM Drive** 와 연동해서 **Batch Simulation** ([시뮬레이션 수행 | 배치-시뮬레이션](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-1.html#%EB%B0%B0%EC%B9%98-%EC%8B%9C%EB%AE%AC%EB%A0%88%EC%9D%B4%EC%85%98)) 을 도와주는 Instance 입니다. **MORAI: SIM Drive**와 연결하려면 `Ip` 와 `Port` 를 설정합니다, 그리고 User 가 Scenario 의 시작과 끝을 명시하기위한 Custom 한 callback 함수를 사용해야 합니다.

##### Methods

- **__init__(self, host=”127.0.0.1”, port=”7789”, user_start_callback_func=None, user_stop_callback_func=None)** Client Constructor.
  - **Parameters:**
    - `host(IP)`*(str) -***MORAI: SIM** 에서 gRPC 로 연동 하기위한 IP 주소. **Default**: “127.0.0.1”
    - `port` *(str) -***MORAI: SIM** 에서 gRPC 로 연동 하기위한 TCP 포트. **Default:**“7789”
    - `user_start_callback_func` - Scenario 시작의 Event Callback. **Default**:****None
    - `user_stop_callback_func` - Scenario 끝의 Event Callback. **Default**:****None

- **is_connected(self)** 현재 **OpenScenarioClient API** 가 **MORAI: SIM** 과 연동의 여부를 확인할 수 있다.
  - **Return:**true (if the client is connected) / false (if the client is not connected)

- **delete_all_actors(self)** OpenScenario 에서 정의된 `Entity` 또는 다른 통신을 통해서 `Vehicle` 을 Spawn 했을 경우, 모든 Spawned Object 들을 삭제한다.
  - **Warning:**실패 시에는 `send_delete_all_actor()` Error 가 출력된다.

- **get_available_map(self)** **MORAI: SIM** 에서 어떤 Map 이 지원되는 지를 알 수 있다.
  - **Return:***List(string)*

- **get_current_scenario_name(self)** 현재 어떤 시나리오파일이 **MORAI: SIM** 에서 재생되고 있는지를 확인할 수 있다.
  - **Return:***string*

- **get_current_scenario_time_sec(self)** 현재 시나리오가 재생된 시점의 Simulation 의 시간을 확인할 수 있다.
  - **Return:***float*

- **get_is_client_running(self)** 현재 위의 OpenScenarioClient 가 동작하는지를 확인할 수 있다.
  - **Return:** *bool*

- **get_simulator_version(self)** 현재 사용하고 있는 **MORAI:SIM**Drive 버전을 확인할 수 있다.
  - **Return:***string*

- **get_states_object(self)** OpenScenario 에서 정의된 [https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/MiscObject.html](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/MiscObject.html) 들의 상태 정보들을 가지고 올 수 있다.
  - **Return**: *dict*

- **get_states_pedestrian(self)** OpenScenario 에서 정의된 [https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Pedestrian.html](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Pedestrian.html) 들의 상태 정보들을 가지고 올 수 있다.
  - **Returns:***dict*

- **get_states_vehicle(self)** OpenScenario 에서 정의된 [https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Vehicle.html](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Vehicle.html) 들의 상태 정보들을 가지고 올 수 있다.
  - **Returns:***dict*

- **get_storyboard_element(self)** OpenScenario 에서 정의된 [https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/StoryboardElementState.html](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/StoryboardElementState.html) 의 상태 정보 및 볼 수 있다. StoryboardElement 같은 경우에는 `Story`, `Act`, `ManeuverGroup`, `Maneuver`, `Event(s)`, `Action(s)` 의 종류로 나누어진다.
  - **Returns:**각 Element 의 *object(class)*

- **set_time_of_day(self, hour, animation=True)** MORAI:SIM 의 시간과 그 시간에 따른 Animation 의 변화를 줄지 안줄지를 결정할 수 있다.
  - **Parameters**
    - `hour`: *(int)* - 시간의 정보 (ex: 13 시)
    - `animation`: *(bool)* - 시간에 따른 Animation

- **set_weather(self, weather_type)** MORAI:SIM 에서의 날씨를 정의할 수 있다.
  - **Parameters**
    - `weather_type`: *(str) -*계절의 종류로서 아래와 같은 string 값을 사용한다. `Sunny`, `Cloudy`, `Foggy`, `Storm`, `Rainy`, `Snowy`

- **set_scenario_config(self, scenario_file_list, controller_mode=ControllerFrom.BuiltIn, sync_mode=False, num_iterations=1, is_ignore_init_ego=False)** **Remark**: OpenScenario Data 를 Play 하기 위해서, 필수 항목 들을 미리 세팅해 준다. 필수 항목의 상세내용은 아래와 같다.
  - **Parameters**
    - `scenario_file_list`: *(list)* - OpenScenario File 이 저장되어 있는 경로*(string)* 를 List 로 준다.
    - `controller_mode`: *(enum)* - Ego 의 차량 제어 방식. **Default**: ControllerFrom.BuiltIn
      - 제어 방식의 종류: `ControllerFrom.BuiltIn`, `ControllerFrom.External`, `ControllerFrom.MoraiSim_Drive`
      - Resource: [시뮬레이션 수행 | 로드한-시나리오에-대한-시뮬레이션](https://help-morai-sim.scrollhelp.site/morai-scenario-runner-kr/-1.html#%EB%A1%9C%EB%93%9C%ED%95%9C-%EC%8B%9C%EB%82%98%EB%A6%AC%EC%98%A4%EC%97%90-%EB%8C%80%ED%95%9C-%EC%8B%9C%EB%AE%AC%EB%A0%88%EC%9D%B4%EC%85%98)
    - `sync_mode`: *(bool)* - 시뮬레이터의 구동 방식. **Default:**false
      - sync_mode=true: 시뮬레이터를 한 프레임씩 구동
      - sync_mode=false: 시뮬레이터를 실시간으로 구동
    - `num_iterations`: *(int)* - 시나리오를 몇 번 반복할지에 대한 횟수. **Default**:**1**
    - `is_ignore_init_ego`: *(bool) -*전 시나리오에 Ego 의 위치와 속도 값을 무시

- **skip_scenario(self)** 재생중의 Scenario 들을 Skip 할수 있다.

- **start_batch_scenario(self)** Scenario File 에 따라서, Batch Simulation 을 진행할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/warning-macro-icon--c9b24710f892ce590a3e.svg)

이 API 를 사용하기 이전에, 위의 **set_scenario_config(…)**API 를 사용해서, Scenario File 를 List 형태로 설정 해줘야 한다.

- **stop_batch_scenario(self)** Batch Simulation 을 종료 할수 있다.

##### Example

PY

```py
def scenario_runner_api_worker(client_api:OpenScenarioClientAPI):
    # OpenScenario File Path (Absolute / Relative)
    scenario_list = [
        '..\..\..\data\openscenario\\v1.2\R_KR_PG_K-City\Scenario_CCRB_1.xosc',
        '..\..\..\data\openscenario\\v1.2\R_KR_PG_K-City\Scenario_CCRB_2.xosc',
    ]

    # Start Batch Simulation
    client_api.set_scenario_config(scenario_file_list =scenario_list,
                                   controller_mode =ControllerFrom.BuiltIn,
                                   sync_mode =False,
                                   num_iterations =1,
                                   is_ignore_init_ego =False)
    client_api.start_batch_scenario()
    
    element = None
    for i in range(20):
        if element is None:
            element = client_api.get_storyboard_element("Event1")
        if element:
            print(element.state)
        time.sleep(1)
    
    client_api.skip_scenario()
    time.sleep(10)
    client_api.stop_batch_scenario()

def custom_start_callback_func():
    pass

def custom_stop_callback_func():
    pass

if __name__ == '__main__':
    App = QApplication(sys.argv)

    # Set for Client for selected ip and port
    client_api = OpenScenarioClientAPI(ip="127.0.0.1", port="7789",
                                       user_start_callback_func=custom_start_callback_func,
                                       user_stop_callback_func=custom_stop_callback_func)
    t_worker = threading.Thread(target=scenario_runner_api_worker, args=(client_api,), daemon=True)
    t_worker.start()
    
    sys.exit(App.exec())
```

×


---
