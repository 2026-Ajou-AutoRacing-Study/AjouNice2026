# 신호등 7프레임·4표 — 실제 주행과 동일 입력 비교

문서 역할: EVIDENCE. 기록일: 2026-10-27. 운영 계약은 [CURRENT](CURRENT.md)다.
후속 상태: 7/4는 계속 OFF다. 이후 승격한 것은 별도 순수 UNKNOWN 유예0.3초다([배포 정리](261027_UNKNOWN_GRACE_RELEASE.md)). 아래 미PR/기본값은 당시 시험 상태다.
사용자가 허용한 7/4 시험이며 **운영 승격·PR/병합은 하지 않았다.**

## 결론

7/4만 켜면 움찔이 사라진다는 결론은 아니다.
안정된 동일 신호 이력이 충분하면 짧은 판독 변동을 줄이지만,
첫 관측 직후나 현재 신호등 identity 연결이 끊긴 공백은 남았다.
실제 R/Y 전환도 첫 새 판독보다 약0.3–0.4초 늦게 전달될 수 있다.
옛 코드가 robust했다는 관찰을 부정하는 결과가 아니라,
**옛 전체 처리와 7/4 투표 한 요소의 효과를 구별한 결과**다.

## 무엇을 비교했나

- Sunny13, Lanelet v88, 같은 전체 A→B→C→D 경로를 처음부터 각각 주행했다.
  실제 ROS1 planner·PP/PI·localization이며 객체 입력은 `od_gt`다.
  BEVFusion 동시 부하·팀4090·LAN 시험은 아니다.
- 운영 직접 관측+기존 완충+허가1회와, 현재 YOLO/CNN/지도 선택을 유지한 7/4 후보를 비교했다.
  후보에서는 기존 색 hold 대신 관측 투표 분기를 사용한다. 단순히 기존 hold 위에 창 하나를 더한 조건은 아니다.
- 황색 통과 OFF, 미수신/timeout·첫 관측 전 정책은 그대로다. 강제 신호 SET은 하지 않았다.
  native GET은 개발 평가만 하며 인지·판단 입력으로 사용하지 않았다.

| 실행 / campaign | 설정 | validator 주행 시간 | 결과 |
|---|---|---:|---|
| current / 170721 | 운영 직접 관측·기존 완충·허가1회 | 210.99초 | PASS·충돌0·종료 정지/P 확인 |
| vote7 / 171609 | 7/4·원본 age0.9초·첫6개 즉시 전달·허가1회 | 160.97초 | PASS·충돌0·종료 정지/P 확인 |

**50초 빨라진 결과가 아니다.** 전방 영상 시작→제어기 시작이 약38.11초와86.32초로 달랐다.
준비 단계의 약48.2초 차이로 서로 다른 자연 신호 시점에서 출발했고,
전체 전방 영상 길이는266.3초와263.9초였다. 주행 중 기다림·판독 입력이 달라
이 두 완주 시간으로 정책의 속도·승차감 우열을 계산하지 않는다.
후보의 A/C 첫 허가 때는 이미 각각6.70/15.68m/s로 이동해 정차 후 출발 지연 비교도 불가능하다.
두 실행의 네 MGeo 원본선 앞바퀴 통과는 모두 ±0.25초와±0.5초 exact native 조회 대조에서
방향 허가였다(A/B16, C48, D33). 이는 개발 proxy이지 공식 무패널티 인증은 아니다.

## 영상과 실제 제어

아래는 실제 전방 카메라와 속도·제동·계획속도·판독을 함께 녹화한 영상에서 자른 파일이다.
화면 PTS는 원본 전체 영상 기준이며 합성 재현 영상이 아니다.

| 영상 | 확인한 내용 |
|---|---|
| [현재 D 출발](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/review_clips/current_D_149.5-155.5.mp4) | 151.8초 허가→151.9초 identity 상실→152.1초 복구. 출발 중 작은 재제동 |
| [7/4 D 출발](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/review_clips/vote7_D_149.5-156.5.mp4) | 151.1초 첫 좌화살표→151.5초 최종 허가. 이 주행의 원시 입력에는 위 연결 상실이 없었음 |
| [7/4 C](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/review_clips/vote7_C_122-128.mp4) | 123.9초 초기 공백·124.2초 연결 상실 공백은 남음. 이후 G/G+좌화살표 변동은 감소 |
| [현재 C 참고](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/review_clips/current_C_119-126.mp4) | 적색 정차 후 허가 출발. 위 후보와 서로 다른 자연 신호 입력 |

전체 전방 영상: [현재](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/current/front_video/front_evidence.mp4),
[7/4](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/vote7/front_video/front_evidence.mp4).

- 현재 D는 약0.12초 허가 공백 뒤, 속도0.29→0.42→0.21m/s와 짧은 제동 명령 약0.16을 기록했다.
  작은 출발 움찔은 실제 존재했다. 다만 7/4에는 같은 원시 연결 상실이 없어 해결 인과를 입증하지 못했다.
- D 첫 raw 허가 원본 stamp→실제0.3m/s 도달은 현재0.317초, 7/4는0.697초였다.
  7/4의 새 허가→채택 원본 stamp 차이는0.385초이고, 채택 원본→움직임은0.312초다.
  기록상 추가 지연은 주로4표 확인 구간이며 이후 제어 반응이 더 느려진 것으로 보지는 않는다.
  서로 다른 자연 주행의 기술 통계이지 통제된 인과 추정치는 아니다.
- 7/4 C에는 `GO→UNKNOWN→GO` 약0.112초와0.240초가 남았다.
  첫 공백은 최초6개 즉시 전달 구간, 두 번째는 identity 상실로 창이 초기화된 구간이다.
  창이 가득 찬 뒤의 투표 동률 문제가 아니었다. 실제 속도는 약16m/s를 유지해 완전 재정차는 없었다.
- C의 원시 `G↔G+좌화살표` 반복은 최종 출력에서 한 번의 변경으로 줄었다.
  둘 다 직진 허가이므로 이것을 GO/STOP 움찔 해결로 세지 않는다.
- B는 새 두 주행 모두 내부 허가 공백이 없었다. 따라서 과거 B 문제가 이번 주행에서
  7/4 때문에 없어졌다고 주장하지 않고 아래 같은 입력 대조를 따로 사용했다.
- 정지 목표 재삽입은 근처 같은 선의 공간·시간 proxy이지 traffic C++ 모듈의 원인 trace는 아니다.

## 같은 입력을 넣었을 때

[재생 도구](../../../scripts/evaluation/replay_traffic_light_stamped_vote.py)는 기록된 원시 normalized tuple·confidence·
원본 stamp·현재 identity 자격으로만 투표한다. hold 결과를 다시 표로 넣지 않는다.
오래된 직접 관측 기록에서 현재 crop/선택 자격을 독립적으로 확인하지 못한 프레임은
성공·UNKNOWN으로 간주하지 않고 `UNAVAILABLE`로 남긴다. 모델/TF 재추론이나 반사실적 차량 주행은 아니다.

- [앞선 비교](261027_TEMPORAL_DRIVING_VIDEO_COMPARISON.md)의 B는 허가 G가 **한 장만** 나온 직후
  세 번의 판독 불가가 이어졌다. 첫6개 즉시 전달인 7/4는 그 약0.375초 공백을 그대로 전달한다.
  이후 창이7개가 되어도 같은 색4표가 없으면 추가 약0.123초 UNKNOWN이 생길 수 있었다.
  이전 G가 충분한 `G×7→UNKNOWN×3`이면 G를 유지하지만, 이 B 입력은 그 조건이 아니다.
- 완성된 창에서 새 Y 첫 판독→승자가 된 원본 영상 stamp는 같은 이전 D 입력에서
  3/2 약0.109–0.127초, 7/4 약0.324–0.359초였다. 10Hz의 네 번째 새 관측까지 기다리는 비용이다.
  물리 점등 시점의 직접 측정은 아니며 판독 시작 기준이다.
  약16m/s라면0.3–0.4초 동안4.8–6.4m를 이동한다. 이는 이동량 산술이지 제동거리·신호 안전 보증이 아니다.

## 옛 7/4와 같은 점·다른 점

옛 `SignalSmoother`의 첫6입력 즉시 전달과7개 중4표 결정은 재현했다.
현재 후보는 원본이 다른 입력만 세고, 같은 regulatory group의 검증된 head 사이에서 표를 공유한다.
UNKNOWN도 빈 표이며 group/identity/clock 상실 때 초기화한다.
오래된 표를 새 stamp로 바꾸거나 신호가 끊길 때 무기한 연장하지 않는다.
옛 HSV/profile 색 보정·RED fallback·D 관측 가드는 복구하지 않았다.

실제 시험 전 초기 구현의 age pruning 후 startup 재진입을 수정했다(v4).
첫6개 여부는 현재 queue 길이가 아닌 해당 identity 구간의 고유 accepted 입력 수로 정한다.
낮은 처리율이나 오래된 표 제거로 즉시 전달 모드에 계속 재진입하지 않는다.
다만 hard identity reset 후에는 의도대로 새 창을 시작하므로 초기 단발 G/UNKNOWN이 전달될 수 있다.

**시간제한 황색 ON에는 이번 후보를 인증하지 않았다.** 승리한 Y의 stamp는 네 번째 Y 등
실제 채택 프레임의 stamp이지 첫 Y 관측 시각이 아니다. 이를 점등 시작으로 사용하면
황색 잔여시간을 낙관할 수 있어 별도 onset 계약 검토가 필요하다. 이번 주행은 모두 황색 OFF다.

## 재현·기록·운영 상태

- [실행기](../../../scripts/evaluation/run_traffic_light_compliance_trial.py)의 시험 전용 인자:
  `--mode guarded --temporal-direct true --required-green-observations 1`
  `--stamped-vote-window-size 7 --stamped-vote-max-age-sec 0.9 --stamped-vote-startup-passthrough true`.
  `--record-video`로 전방·telemetry와 원본 stamp/frame sidecar를 저장했다.
- 실행 설정은 새 `seven_four_video_compare_261027_v1/full_regression.yaml`에 보존했다.
  소량 자체 녹화 예산에 한해 디스크 최소 여유4→2GiB를 명시했다. 기존 frozen 실행은 수정하지 않았다.
- [실제 비교 JSON](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/comparison.json),
  [이전 동일 입력 v4](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/replay_previous_inputs_v4.json),
  [새 두 입력 v4](../../../artifacts/evaluation/traffic_light_robustness_261025/seven_four_video_compare_261027_v1/replay_actual_pair_v4.json)에 근거를 남겼다.
- 실제 후보 runtime은 v4/window7/4표/age0.9/startup ON임을 확인했다.
  실제7/4의 고유 원본·group 입력2,028개를 같은 helper로 재생해 최종 색·confidence·채택 원본 stamp와
  출력 메시지 stamp가 모두 일치함을 확인했다. 판독 정답률2,028/2,028이라는 뜻은 아니다.
  소스·설치 helper/node/두 launch의 SHA가 같고 관련 테스트 **466개 PASS**다.
- 외부 X11 `simulator.mp4`는 중간에 VSCode 창이 덮인 것을 시각 확인해 주행 근거에서 제외했다.
  위 전방 영상은 실제 카메라 입력이며 관련 stamp·현재 이미지 일치/재사용 여부·제어 age를 sidecar로 확인했다.
- 시험 노드·제어기는 정상 종료했고 정지/P를 확인했다. MORAI는 그대로 두었다.
  잔여 ROS 시험 파라미터를 window0/age0.5/startup false/debug image false로 복구했다.
  launch 기본은 처음부터 window0이며 운영 직접 관측·허가1회·황색 OFF를 유지한다.

현재 판단은 **7/4를 만능 수정으로 바로 승격하지 않음**이다.
남은 핵심은 첫 관측/identity 공백이 실제 GO/STOP·재제동으로 이어지는 조건이다.
판독 문자열 흔들림만 줄이거나 전체 옛 파이프라인을 통째로 되돌리는 것으로 해결을 인증하지 않는다.
