# 지도 구간별 3초 / 6초 예측 후보

상태: **기존3/6초 시험 기록**. 기준: 2026-10-24.
후속 사용자·판단팀 승인으로 고주로 최대 8초·로터리 최대 3초이며 지도 종단에서는 가용 시간만 출력한다.
[현재 배포·검증](HIGHWAY_EIGHT_SECOND_HANDOFF.md)을 우선하며 아래의 당시 OFF 결론·6초 수치를 소급 변경하지 않는다.
운영 계약은 [CURRENT](TRACKING_PREDICTION_CURRENT.md)를 따른다.

## 의도와 전제

고주로에서 판단에 전달하는 미래 구간을 늘리되, 로터리의 기존 Top-K 분기와 첫 3초를 보존한다.
Ego의 미션 전환 스위치가 아니라 **각 NPC가 지도에서 어디에 있는지**로 선택한다.
대회 차량의 centerline 추종·출발 가속 이후 등속 조건을 전제로 한다. 출발 가속을 새로
모델링하거나 tracker 속도·coasting·detector threshold를 바꾸는 작업이 아니다.

로터리는 기존의 서로 다른 경로별 대표 후보를 유지한다. 정확한 후보가 Top-K에 포함되는지
minADE-K/minFDE-K와 분기 coverage를 함께 본다. 미래 GT로 고르는 min-K 지표를 runtime의
후보 선택 정책과 혼동하지 않는다. 실제 로터리 planner의 confidence 0.05 이상 경로 검사도 유지한다.

## 동작

1. 기존 predictor로 3초 / 0.1초 결과를 먼저 계산한다(현재 pose 포함 31개).
2. 현행 v85에서 `speed_limit=150`으로 표시한 고주로 도로 lanelet을 읽는다(현재 51개).
   이는 구간 표시이지 NPC가 150km/h로 달려야 한다는 뜻이 아니다.
3. 동일 UUID의 모든 선택 경로가 명시적 고주로에 속한 상태를 0.5초 확인한다.
   이는 track 출력의 분류 안정 시간이며, 30Hz 출력을 새 센서 관측 30회로 세지 않는다.
4. 기존 경로를 다시 선택하지 않고 3.1~6초만 붙인다(총 61개 pose).
   첫 3초 XYZ/yaw, 후보 수·순서·confidence와 현재 객체 필드는 유지한다.
5. 연장에 새 갈림길·경로 끝·고주로 밖 구간·길이/깊이 제한이 있으면 **그 객체 전체**를
   원래 3초로 둔다. 끝점을 반복해서 6초처럼 보이게 하지 않는다.

따라서 고주로 차량도 항상 6초인 것은 아니다. 같은 메시지에 31개/61개 경로가 공존할 수 있다.
보행자·정적 장애물·지도 매칭 실패 차량은 기존 3초다. 로터리/일반도로도 3초를 유지한다.

구간 이탈·지도 경로 소실·클래스 변경·객체 소실·시간 역행·비정상 위치 점프는 진입 이력을
초기화한다. 상태는 최대 512 UUID, 미갱신 2초로 제한한다. 없는 객체를 이 상태로 살리지 않는다.

## 시작 실패와 복귀

별도 지도 파일·네트워크 서비스·자격증명은 추가하지 않는다. 선택적 지도 속성이나 모듈이
없거나 연장이 실패하면 경고와 함께 기존 horizon으로 계속한다. 기존 필수 지도 로드 실패나
잘못된 입력 frame을 숨기는 fallback은 아니다. feature OFF이면 선택기를 생성하지 않는다.

운영 기본값은 여전히 OFF다. detector·localization과 중복되지 않는 기존 `tracking` 실행에서
시험할 때만 다음 인자를 추가한다. 변경은 fresh predictor 시작 시 반영된다.

```bash
tracking prediction_mixed_horizon_enabled:=true
```

기존값 복귀는 노드를 정상 종료한 뒤 같은 명령의 값을 `false`로 바꾸거나 인자를 생략한다.
가중치 재다운로드·재학습은 필요 없다. 새 인자를 처음 사용하는 PC는 평소 workspace 빌드·설치를
먼저 반영한다. 직접 launch는 기존 운영 alias의 다른 인자도 보존한다.

Validator는 개인 YAML의 아래 opt-in을 실제 launch에 전달하고 실행 snapshot에 보존한다.
GUI의 전용 체크박스는 추가하지 않았다. 기존 snapshot을 편집하거나 다른 조건으로 resume하지 않는다.

```yaml
stack:
  prediction_mixed_horizon_enabled: true
```

## 검증과 해석

`qualify_e12_mixed_horizon.py`는 동결 E12 14장면의 5,986프레임·11,773객체를 재생했다.
6초 suffix를 제거하면 모든 프레임의 ROS 직렬화가 3초 기준과 정확히 같았고 입력도 불변이었다.
1,810 객체 출력을 연장했다. 원래 코어 ID에서 UUID·누락 covariance 등을 재구성한 자료이므로
원본 ROS bag의 모든 필드를 복구했다는 뜻은 아니다. 실제 기록 TrackedObjects 대조는 별도로 한다.

같은 오프라인 실행의 비어 있지 않은 warm frame 4,665개에서 계산+메시지 생성 평균은
1.287→1.399ms, p95는 3.459→3.901ms였다. ROS 큐·LAN·실전 CPU 부하 지연은 아니다.
연장된 GT 대응 1,027개에서 6초 FDE 평균 6.24m·p95 24.32m였다. 길어진 예측의 정확도가
보장되는 것은 아니며, 기존 허용 출발 가속 구간도 포함한다. 고주로 폐루프의 대기·추월 변화는
첫 3초 보존과 별개로 확인해야 한다.

동결 근거는 `artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1/mixed_horizon_261024_v1/`에
source/install/map/input hash와 함께 둔다. 연구 실행기·단위 시험은 `scripts/evaluation/`에 있다.

실제 주행 시험은 로컬 Sunny13·od_gt·보정 Ego GT·실제 ROS1 planning/PP·PI 조건이다.
BEVFusion·실제 측위/LAN/4090 풀스택 성능 인증으로 확대하지 않는다. 최초 로터리 smoke의
`201095→201098` 지름길은 원형 회전부를 지나지 않았으므로 로터리 회전 검증에서 제외한다.
수정 시험에서는 출발 전에 실제 선택 route가 ring lanelet을 지나는지 확인한다. 이 확인은
`run_validator_route_probe.py`의 시험 전용 검사이며 production 노드의 새 시작 조건이 아니다.
개별 실행 bag·지표·판정과 한계는 `planner_logs/analysis/map_mixed_horizon_261024_v1/`에 보존한다.

후속 북측 출구 시험은 같은 Ego·K5 교통 조건에서 기존 3초와 혼합 후보 모두 실제 회전부를
통과해 도착했고 충돌 0·종료 정지/P를 확인했다. 로터리 차량 출력은 양쪽 모두 3초였으며,
실제 ROS 입력 2,761프레임·출력 2,610프레임 대조에서 첫 3초와 비경로 필드를 보존했다
(rospy publisher가 바꾸는 외부 `header.seq`만 비교에서 제외).
앞선 서측 출구의 기존 3초 실패는 NPC 생성점과 Ego 통과 경로가 겹쳐, 새 NPC가 나타난
약 32ms 후 접촉한 사례로 따로 보존했다. 이 실패를 혼합 정책의 악화나 일반적인 로터리
주행 불능으로 해석하지 않는다. 북측 시험은 NPC를 삭제하거나 planner를 완화한 결과가 아니다.
고주로 OFF/ON도 각각 도착했지만 native spawn 차종·개수가 같지 않아 도착시간 차이를
혼합 정책의 개선률로 제시하지 않는다. 운영 기본 OFF 결론은 유지한다.

## 로터리·고주로 모두 native 6초인 별도 시험

`prediction_horizon=6.0`과 mixed OFF는 위 혼합 연장과 다르다. 기존 predictor가 6초에 필요한
경로를 처음부터 다시 열거하므로 Top-K·순위·confidence와 첫 3초도 바뀔 수 있다.
지도 대응이 실패한 차량·비차량의 운동학 fallback도 6초가 된다. 모든 경로는 현재 pose를
포함해 61개이며, 단순히 3초 끝점을 반복하거나 로터리를 3초로 남긴 조건이 아니다.

Validator 개인 YAML은 `stack.prediction_horizon: 6.0`과
`stack.prediction_mixed_horizon_enabled: false`를 함께 사용한다. 생략 시 기존 3초이며,
값을 바꾼 비교는 새 실행으로 분리한다. `analyze_native_horizon_live.py`가 같은 기록 입력의
native 3/6초, 양방향 prefix 대응·실제 출력·주행 지표를 따로 대조한다.
실제 회전부를 지나는 북측 경로와 고주로의 닫힌 bag·소스/설정 hash는
`planner_logs/analysis/map_native6_261024_v1/`에 보존한다. 짧은 goal 정차 시험과
대회 경로의 톨게이트 이후 시험을 구분하며, native 6초를 운영 기본값으로 승격하지 않는다.

2026-10-24 로컬 기능 시험에서 북측 로터리 3초는 도착했고, native 6초는 양보 후 재출발 중
접촉해 안전 종료했다. 해당 NPC의 올바른 첫 3초 후보는 남아 있었고 confidence도 기준 이상이었다.
예측 후보 소실로 원인을 단정하지 않으며 판단의 정지 해제·실제 후보 사용 확인이 남는다.
고주로는 대회 preferred 경로를 현재 Ego부터 톨게이트 이후 `wp_200183`까지 적용한 새 3/6초
시험에서 둘 다 도착·종료 정지/P를 확인했다. 앞선 약 220m 중간 정차 시험의 PASS는
고주로 전체 검증에서 제외한다. 긴 경로의 기존 3초에서도 별도 급감속이 남아, 모든 감속을
짧은 goal이나 6초 예측 탓으로 설명하지 않는다. od_gt·보정 Ego GT·실제 planner/PP·PI의
로컬 시험이며 대회 센서/LAN·E12 검출을 포함한 안전 인증은 아니다. 운영값은 계속 3초다.
