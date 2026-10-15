# Base5 v38 — ROS 프로세스 공유 PASS / 네트워크 재사용 실패로 live 중단

상태: EVIDENCE + 재연결 대기. 2026-10-06. 본10K/학습은 미시작이다.
최신 실험 이후 MORAI client_count=0, Ego/Object/CollisionData publisher가 사라졌다.
추가 ScenarioLoad/제어/재수집을 하지 않는다. 재연결·정지/Park 확인이 먼저다.

## 동일6개 payload A/B/C

v36 pilot manifest의 indices0,3,5,9,12,19를 사용했다.
highway/obstacle/roundabout/background 정적4개와 walking/running2개다.
시간은 외부 time 명령의 real로, ROS 초기화·프로세스 종료까지 포함한다.

| 방식 | 실제 wall time | 결과 | 네트워크 처리 |
| --- | ---: | --- | --- |
| A: 기존 셀별 campaign/child | 50.484초 | 6/6 accepted, 모든 child 종료Park | 6회 재로드 |
| B: 기존 multi-row runner, ROS 프로세스 공유 | 43.491초 | 6/6 accepted, 최종Park | 6회 재로드 |
| C: ROS 공유 + 첫 장면만 네트워크 재로드 | 19.000초에 실패 종료 | 1장 accepted, 두 번째 load 실패, 최종Park 미확인 | 첫 reload 후 두 번째 flag=false |

A→B 전체13.85% 단축을 관측했다. A/B 선택 scenario SHA는 정확히 같다.
C는 완료하지 못했으므로19초를 처리량이나 성공적인 속도 개선으로 계산하지 않는다.
장면 수/통과수/원본까지 같은 성공 비교가 아니다.

주요 단계 합계 A→B:

- ROS setup:1.058→0.192초.
- preflight:1.967→0.338초.
- 계측 외 process/campaign 비용:4.942→0.723초.
- pose confirmation:8.030→7.978초.
- control readiness:12.685→17.483초, motion target/witness:13.862→9.059초.
  두 단계 합은 거의 같으며 preparation 시점이 달라지면 남은 객체 이동 대기가 달라진다.
  motion target만 줄었다고 객체 속도/자세 확인을 생략했거나 빨라졌다고 주장하지 않는다.
- capture write/stability:7.109→7.100초.

작은 순차 실험이다.14%가10K에서도 유지된다는 보장이나 하룻밤 완료 보장은 아니다.
실제 센서 원본/동작 자세까지 같은 프레임을 비교한 것은 아니다.

## C 실패 근거 / 처리

첫 b5v2_00000은 통과했고 boundary Park도 확인했다.
두 번째 b5v2_00003에서 load_network_connection_data=False를 사용하자,
rosbridge 로그의17:52:18에 모든 MORAI 클라이언트가 disconnect했다.
12초 위치확인 timeout, stale Ego/Object/CollisionData 때문에 boundary/최종Park가
모두 미확인이다. 직전 Park 성공은 새 장면의 종료Park 성공을 대신하지 않는다.
세 번째 장면은 로드하지 않았다. 실패 raw/ledger/실행 당시 source는 보존했다.

이 실행에서 flag=false가 연결 보존으로 동작하지 않았다는 근거다.
모든 MORAI 버전의 의도된 API 의미나 내부 버그 원인을 확정한 것은 아니다.
현재 --batch-reuse-network는 **ROS 초기화/파일 생성 전 명시적으로 거부**한다.
실패 폴더를 resume하거나 shutdown_safety/contract를 수동 수정하지 않는다.

## 현재 코드 / 남은 live 검증

- 네트워크는 항상 load_network_connection_data=True로 유지한다.
- 제한적 --shared-ros-batch 후보를 추가했다(최대16행, 아직 bulk에 연결하지 않음).
  한 ROS 프로세스에서 매 장면 네트워크 재로드, 새 장면 이후의 Ego/Object/CollisionData 및
  ordinary actor UID 수신, 장면마다 Park 확인, review/failure 시 다음 장면 전 중단을 요구한다.
- runner_options에 batch mode와 선택 scene ID 목록을 기록해 다른 옵션/plan으로 resume하지 않는다.
  batch 완료행의 boundary Park가 없거나 실패하면 resume을 거부한다.
- STOP을 dynamic arrival/file wait 중에도 확인하며 최종Park 경로로 빠진다.
- quality/GT/안전gate와 finite retry는 유지한다. production 모델·threshold·tracker·planning·loc 미변경.
- tests103개PASS: mode/plan provenance, post-load freshness/UID, stale/누락 거부,
  네트워크 재사용 금지·16행 초과 제한·unsafe boundary resume 거부와 기존 회귀.

**B의 성공은 기존 multi-row 실행의 결과다. 이후 추가한 --shared-ros-batch의
per-scene Park/post-load gate와 stop/resume 전체 live 검증까지 완료했다는 뜻이 아니다.**
통신이 끊겨 후속 pause/resume 시험은 실행하지 않았다.
현재 코드hash 변경으로 A/B/C 기존 세션은 읽기전용 근거로만 보존한다.

## 경로

- A: $HOME/morai_cap/base5_v2_261006/network_independent_v38
- B: $HOME/morai_cap/base5_v2_261006/network_ros_shared_v38
- C: $HOME/morai_cap/base5_v2_261006/network_reuse_v38
- 비교: $HOME/morai_cap/base5_v2_261006/network_comparison_v38_final.json
- C 재현 당시 코드: network_reuse_v38/repro_code/
- C 요약: network_reuse_v38/CONNECTION_FAILURE.txt

## 다음 — 사용자 재연결 후에만

MR Ego Network에서 아래 원래 profile을 다시 Load/Connect한다.

    /home/luv/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json

Ego가 정지/Park 상태인지 확인한다. roscore/rosbridge가 살아 있다면 터미널 전체 재시작은
먼저 요구하지 않는다. 에러가 있거나 profile 연결이 복구되지 않으면 그때 재실행한다.
Ego/Object/CollisionData 및 ScenarioLoad/SaveSensorData/ctrl/Service_MoraiEventCmd 연결과
사전Park 응답을 다시 확인한 뒤, 새output의 shared-ROS 소량 stop/resume을 시험한다.
통과 후에만 bounded quality campaign의 chunk/recovery/quota 경로로 통합할지 결정한다.
현재 single-cell campaign 기본 실행은 그대로이며10K 시작 명령으로 바꾸지 않았다.
