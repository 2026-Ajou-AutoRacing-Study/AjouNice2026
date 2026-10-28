# 실행 도구 진입점

현재 명령을 찾는 인덱스다. 상세 옵션·측정 한계는 [OPERATIONS.md](OPERATIONS.md)를 읽는다.

| 목적 | 경로·안내 |
| --- | --- |
| Workspace 준비 | `workspace/`, `build_all.sh`, `source_aw149.sh` |
| 평소 실행 | [공통 runbook](../docs/runbooks/morai_full_stack_by_terminal.md), [aliases](../docs/common_alias.txt) |
| 인지 설치·실행 | `runtime/install_base5_production.sh`, `runtime/run_bevfusion.sh`, [현재 계약](../docs/bevfusion/CURRENT.md) |
| 신호등 후보 설치 | `runtime/install_traffic_light_candidate.sh`, [시험 배포 안내](../docs/perception/traffic_light/V5_CANDIDATE_HANDOFF.md) |
| 측위 평가 | [localization 도구](evaluation/localization/README.md) |
| 데이터 수집·학습 준비 | [dataset 현재 안내](../dataset/morai_od/docs/README.md), `portable_base5/`, `runtime/` |
| 지도 | [kcity_map](../kcity_map/README.md), `map/` |
| 문맥 검사 | `docs/check_perception_context.py` |

<a id="lan-두-pc를-함께-계측하기"></a>

## 개발용 기록

평소 스택을 켠 상태에서 Linux 알고리즘 PC는 아래 한 명령으로 기록한다.

```bash
bash ~/AjouNice2026/scripts/rosbag/record/start_lan_recording.sh
```

Windows MORAI PC는 전달받은 `start_morai_recording.ps1` 한 파일을 실행한다.
서로 같은 주행의 폴더 전체를 공유한다. 원시 입력·실제 메시지·단계 timing·부하 기록의 유무와
한계는 [상세 안내](OPERATIONS.md#lan-두-pc를-함께-계측하기)를 따른다.

개발 녹화는 시작/종료 시 실제 behavior-velocity planner, path optimizer,
velocity smoother와 종제어(`longi_pi_autoware`)의 ROS 파라미터도 읽어 보관한다.
같은 PC의 MPT·신호등 실행 파일/라이브러리 식별 정보도 선택적으로 보관한다.
조회 실패는 녹화를 막지 않으며, 이는 메모리·GPU 백엔드 검증이 아니다.
신호 관측 자료와 실제 `planner_decision_debug` 녹화 완전성은 따로 표시한다.
실제 vector map·route·운행 모드·차량 속도/조향 상태도 저용량 재생 문맥으로 기록하고
`planner_map_route_context`에서 유무를 구분한다. 이것만으로 전체 planner 재생 가능성을 인증하지 않는다.
이는 읽기 전용 기록이며 파라미터·주행 정책을 바꾸거나, 미발행 토픽 때문에 주행을 막지 않는다.
과거 녹화에 빠진 파라미터는 현재 소스 기본값으로 소급해 채우지 않는다.

`legacy/`·초기 FAST-LIO/구 detector 도구는 재현용이다. 현재 기본 실행 순서에 넣지 않는다.
수집기 동결 inventory와 resume 계약은 정리를 이유로 바꾸지 않는다.
