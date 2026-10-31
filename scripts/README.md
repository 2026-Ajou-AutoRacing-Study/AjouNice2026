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

`legacy/`·초기 FAST-LIO/구 detector 도구는 재현용이다. 현재 기본 실행 순서에 넣지 않는다.
수집기 동결 inventory와 resume 계약은 정리를 이유로 바꾸지 않는다.
