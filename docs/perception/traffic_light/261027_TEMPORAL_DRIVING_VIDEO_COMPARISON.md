# 신호등 시간 안정화 후보 — 실제 주행 영상 비교

문서 역할: EVIDENCE. 기록일: 2026-10-27. 운영 계약은 [CURRENT](CURRENT.md)다.
후속 상태: 3/2는 계속 OFF다. 이후 별도 순수 UNKNOWN 유예0.3초를 승격했다([배포 정리](261027_UNKNOWN_GRACE_RELEASE.md)). 아래 기본값 유지·미PR 기록은 당시 상태다.
**운영 기본을 바꾸거나 PR/병합하지 않았다.** 3/2 다수결은 기본 OFF인 시험 후보다.

## 결론

- 현재 방식의 B 판독 `G → UNKNOWN → G`는 재현됐다. 그러나 그 순간 실제 재정차·재출발은 없었다.
  출력/정지 목표의 변동만으로 팀 LAN bag의 큰 움찔을 재현·해결했다고 말하지 않는다.
- 허가 2회와 짧은 다수결은 단발 허가를 바로 열지 않는 효과가 있다. 반면 정차 후 출발이 늦어질 수 있다.
- 최초 다수결 후보는 같은 C 미션의 physical head가 바뀔 때 불필요한 UNKNOWN을 만들었다.
  투표 소유자를 regulatory group으로 고쳐 동일 입력에서 그 문제를 제거했고, 실제 재주행에서도 확인했다.
- 현재 YOLO/CNN/지도 선택을 옛 ROI/HSV로 통째로 원복하거나, A 때문에 즉시 전면 재학습해야 한다는 근거는 아니다.
  다수결의 제어 승격도 아직 하지 않는다. 작은 표본·자연 신호의 서로 다른 주행으로 전반적인 승차감 우열을 인증할 수 없다.

## 실제 실행 범위

Sunny13, Lanelet v88, 같은 전체 경로 A→B→C→D를 각각 처음부터 주행했다.
`planning_scenario_validator`의 실제 ROS1 planner, PP·PI, localization을 썼다.
객체 입력은 `od_gt`이며 BEVFusion 동시 부하·4090·LAN 성능 시험은 아니다.
황색 통과 OFF, 원래 미수신/timeout 계약과 첫 관측 전 처리 유지, 신호 강제 SET 없음.
native GET은 개발 평가만 하며 인지·판단 입력으로 전달하지 않았다.

| 실행 | 설정 | 주행 시간 | 충돌 / 종료 |
|---|---|---:|---|
| current / 160605 | 운영 직접 관측·공통 0.2초 완충·허가 1회 | 211.56초 | 0 / PASS·정지 P |
| confirm2 / 161122 | 위 인지 유지·판단 허가 2회 | 211.28초 | 0 / PASS·정지 P |
| vote3 / 161638 | housing별 3/2 최초 후보 | 210.07초 | 0 / PASS·정지 P |
| vote3_group / 163041 | group별 3/2 수정 후보 | 210.13초 | 0 / PASS·정지 P |

시간 차이를 성능 향상으로 해석하지 않는다. 신호·속도·영상 입력이 완전히 같은 paired 주행은 아니다.
네 번 × 네 미션의 원본 MGeo선 앞바퀴 통과는 모두 ±0.25초 native 조회 대조에서 방향 허가였다.
이는 개발용 기하·조회 proxy이며 공식 패널티/정지선 준수 100% 인증이 아니다.

## 바로 볼 영상

아래 짧은 파일은 실제 전방 녹화에서 잘랐다. 합성 재현 영상이 아니다.
화면에 남은 PTS는 **원본 전체 영상 시간**이고, 아래 표 구간의 시작점을 더해 찾을 필요가 없다.

| 영상 | 원본 PTS | 확인할 내용 |
|---|---|---|
| [현재 B](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/current_B_95-107.mp4) | 95–107초 | 98.6/98.7 UNKNOWN, 98.8 G 복구 |
| [허가 2회 B](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/confirm2_B_95-107.mp4) | 95–107초 | 97.9 새 Y 판독은 그대로 전달됨 |
| [수정 다수결 B](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/group_vote_B_95-107.mp4) | 95–107초 | 초기 단발 허가/공백 뒤 안정 G |
| [최초 후보 C 문제](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/rejected_housing_vote_C_120-127.mp4) | 120–127초 | 122.9–123.1 head 변경으로 생긴 UNKNOWN |
| [수정 후보 C](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/group_vote_C_120-127.mp4) | 120–127초 | 122.0 head 변경에도 G+좌화살표 유지 |
| [현재 A 초기 공백](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/review_clips/current_A_red_gaps_55-59.mp4) | 55–59초 | R/UNKNOWN 중에도 실제 속도 상승·브레이크 0 |

전체 전방 영상: [current](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/current/front_video/front_evidence.mp4),
[confirm2](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/confirm2/front_video/front_evidence.mp4),
[최초 vote3](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/vote3/front_video/front_evidence.mp4),
[수정 vote3_group](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/vote3_group/front_video/front_evidence.mp4).
외부 차량 시점: [수정 후보 MORAI](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/vote3_group/simulator.mp4).

## 판독 변동과 실제 제어를 구분한 결과

현재 B의 최종 공백은 0.255초였다. 원본 PTS 98.3/98.6/98.7/98.8초에서
실제 속도는 16.124/16.194/16.197/16.163m/s, 실제 브레이크는 모두 0이었다.
동일 B선 근방의 정지 목표가 잠깐 되돌아왔지만, 현재 위치 계획 속도는 약 16.2m/s였다.
이후 약한 제동은 있었으나 B 목표가 사라진 후에도 이어져 전부 신호 공백 탓으로 돌릴 수 없다.

confirm2의 B는 raw/final 모두 `G → Y → G`였다(새 Y 0.108초).
직전 단발 G로 판단의 2회 허가가 완성되지 않아 그 Y 때문에 새로 목표가 닫힌 것은 아니었다.
native B 조회는 두 실행 모두 해당 구간 녹색이었지만, 이것만으로 모든 렌더링 픽셀의 정답을 인증하지 않는다.

수정 후보 B는 raw `G → UNKNOWN → G`(97.2/97.4/97.5초), 초기 확인 후 출력은 안정 G였다.
**이 실행에는 raw Y가 없었다.** 따라서 새 주행의 Y 부재를 다수결의 Y 오인식 해결로 셀 수 없다.
97.4초 UNKNOWN은 이미 허가로 확정한 뒤의 유지 시험이 아니라 초기 확인 구간이었다.
G가 안정적으로 출력되고 B 정지 목표 재삽입이 0이어도 98.2초 무렵 약한 제동은 있었다.

A 초기 R/UNKNOWN 공백에서도 속도는 55.8→57.1초에 6.760→9.711m/s로 올라갔고 브레이크는 0이었다.
R과 미완충 UNKNOWN은 이미 관측한 미션에서 모두 정지 판단이므로, 램프 문자열 변동을 GO/STOP 반복으로 세지 않는다.
수정 다수결에도 A의 짧은 R/UNKNOWN은 남았다. 전체 누락·움찔 제거 또는 CNN 정확도 100%가 아니다.

## C 후보 결함 수정과 동일 입력 대조

최초 후보는 raw G+좌화살표가 같은데 head018→016→018로 바뀐 세 프레임을 초기화해
0.330831초의 `G → UNKNOWN → G`를 만들었다. 그때도 실제 추가 제동은 관찰되지 않았다.
이는 다수결 원리의 필연적 문제가 아니라 투표를 physical head에 묶은 후보 구현 결함이었다.

수정은 **현재 지도 연결이 검증한 head라는 조건은 그대로** 두고 같은 regulatory group의 투표를 잇는다.
미션/그룹 변경·현재 head 자격 상실·시계/원본 stamp 역전 시에는 여전히 초기화한다.
특정 C 위치·픽셀·속도에 대한 예외 규칙은 추가하지 않았다.

이전 후보의 원본 2,019개 입력을 동일 순서로 재대조하면 출력 5개만 바뀐다.
C 108.8/109.2초 UNKNOWN→R, 122.9/123.0/123.1초 UNKNOWN→G+좌화살표이며 A/B/D는 동일하다.
후보에 기록된 `identity_valid`와 원래 normalized tuple/신뢰도/원본 stamp를 사용했다.
이 대조는 TF/모델 재추론 또는 반사실적 차량 주행이 아니다.
float로 기록한 guard 시각의 ±512ns 대조에서도 출력·winner는 같았다.
실제 수정 후보 C에서도 head가 세 번 바뀔 때 출력이 유지됐고, raw G+좌화살표→최종 UNKNOWN은 0프레임이었다.

## 지연 비용과 승격 보류

아래는 첫 raw 허가 영상 stamp에서 실제 속도 0.3m/s 도달까지다. 내부 대기시간만 측정한 값이 아니다.

| 실행 | A | C | D |
|---|---:|---:|---:|
| current | 0.441초 | 0.445초 | 0.408초 |
| confirm2 | 0.499초 | 0.543초 | 0.521초 |
| 최초 vote3 | 0.546초 | 0.479초 | 0.456초 |
| 수정 vote3_group | 0.551초 | 0.530초 | 0.560초 |

서로 다른 자연 주행이므로 정확한 인과 지연의 추정치는 아니다.
동일 입력 재대조의 새 R/Y 입력→승리 관측 stamp 차이는 약 0.10–0.12초였다.
진짜 R/Y도 같은 규칙으로 늦어질 수 있다. 단발 Y를 없애는 것을 안전상 공짜 이점으로 주장하지 않는다.

후보는 옛 7프레임/4표 복구가 아니다. 원본이 다른 최근 3개 중 2개가 같아야 하며
startup도 2개를 요구하고 UNKNOWN도 빈 표로 센다. 오래된 승자의 stamp·신뢰도를 새 관측으로 갱신하지 않는다.
최대 원본 age 0.5초는 callback 검사이며, 카메라 자체가 끊겼을 때 별도 타이머로 UNKNOWN을 발행하는 기능은 아니다.
후보를 켜면 생산 경로의 공통 no_lamps/저확신/detector hold 대신 이 투표 분기를 사용하므로
완전히 한 요소만 추가한 비교로 표현하지 않는다. 단순 후보들의 실제 제어 우열은 추가 근거 없이 확정하지 않는다.

## 재현·기록 계약

- 후보 인자: `stamped_vote_window_size:=3`, 복귀/운영 기본 `:=0`.
  `planner_compliance_observations:=true`·지도 선택에서만 허용한다.
- [실행기](../../../scripts/evaluation/run_traffic_light_compliance_trial.py)의 `--record-video`는
  실제 annotated camera·속도·제동·계획속도와 Simulator 창을 함께 기록한다.
  프레임 재사용·영상 원본 stamp·최신 telemetry age를 sidecar에 남긴다.
- [최종 비교 JSON](../../../artifacts/evaluation/traffic_light_robustness_261025/temporal_video_compare_261027_v1/comparison_final.json)은
  bag·observer·launch override·실제 runtime·native crossing·정지/P를 분리한다.
  정지 목표 재삽입은 공간/시간 proxy이며 C++ traffic 모듈의 인과 trace가 아니다.
- 전방 영상은 각 267.1/266.6/265.1/264.6초, 모두 decode/callback 오류 0으로 마감됐다.
  영상의 `OUTPUT LAMPS`는 planner GO 판정이 아니다. 최신 runtime과 이미지 stamp가 다른 프레임은 명시한다.
- current의 외부 화면 녹화 시작 약 60초는 다른 창에 가려졌다. 전체 전방 영상은 유효하다.
  잘못 가린 자체 녹화·probe는 폐기하고 외부 영상은 원본 65초 이후만 별도 보관했다.
  이후 실행은 Simulator를 먼저 활성화했다. 외부 X11 영상이 다른 창에 덮일 가능성은 별도 시각 점검 대상이다.
- 공간 확보는 이번 시험 bag만 ROS 표준 무손실 압축했다. 각 메시지 payload/타입/주제/stamp의 집합과
  주제별 순서가 같음을 확인한 뒤 자체 압축 전 사본을 제거했다. 동일 stamp의 서로 다른 주제 간 순서는 바뀔 수 있다.
  분석은 압축 후 bag으로 다시 수행했다. 전방 전체 영상은 그대로이며, 앞 3회 외부 참고 영상만 낮은 bitrate로
  재인코딩해 해상도·프레임 수·길이가 같음을 확인했다. 외부 영상의 픽셀 동등성을 주장하지 않는다.
- 소스와 `install_aw149` 일치 및 관련 Python 테스트 **432개 PASS**. 이 수치를 실대회 인식 정확도/승차감 인증으로 쓰지 않는다.
- 종료 후 시험 노드·제어기는 내려갔고, 남은 시험 ROS 파라미터도 투표0·debug image OFF로 복구했다. launch의 운영 기본값은 처음부터 그대로다.
