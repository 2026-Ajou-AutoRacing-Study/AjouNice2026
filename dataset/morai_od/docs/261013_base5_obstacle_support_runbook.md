# Base5 장애물 보강 후보 확인과 수집 준비

문서 상태: EVIDENCE — v150 당시 READY SUN13. 기준일: 2026-10-13.
현재 상태 주석(10/16): 아래는 v150 당시 검증·명령의 보존 기록이다. v151의 부족 1장은 [당시 차단 근거](261015_base5_front_recovery_release.md)로 남긴다. 이후 v152·v153·v154의 검증은 별도이며, 현재 실행은 [Base5 CURRENT](BASE5_V2_CURRENT.md)와 [v154 검토](261016_base5_precollection_final_review.md)를 따른다. 아래 과거 release로 우회하지 않는다.
기존 Sunny13 5K는 보존했다. 새 5K와 다음 25K는 배치 파일만 생성했으며 본수집을 시작하지 않았다.
장애물 보강은 [초기 실험 근거](261013_base5_obstacle_support_experiments.md), 최종 NPC 보완과 수집 준비는
[정지 NPC 검증](261013_base5_parked_npc_support_evidence.md)을 따른다.

## 현재 위치

| 항목 | 경로 또는 상태 |
|---|---|
| 실험 root | `~/morai_cap/base5_obstacle_improvement_261013` |
| 새 독립 Sunny13 5K 계획 | `candidate_v150c/sunny13/manifest.json` |
| 다음 다섯 환경 25K 계획 | `candidate_v150c/future_suite/suite.json` |
| Sunny13 동결 release | `release_v150_sunny13/release.json` |
| 정적 회귀와 유한 보충 | `qualification_v150_static64`, `qualification_v150_static64_completed`: 64/64 |
| 보호된 실제 이동 회귀 | `qualification_v150_animated8`: 8/8 |
| 긴 버스 4장 probe | `long_vehicle_probe4_capture_gt_restored_v150`: 4/4 |
| 본수집 gate | `bulk_ready=true`, 실제 본수집 미실행; Sunny13만 live qualification |

`generic-obstacle-rich-floor-v145`는 장애물 정책 family 이름이며 긴 NPC 보완과 현재 동결은 v150다.
초기 prototype과 불완전한 `candidate_v150`, `candidate_v150b`는 진단 이력이지 본수집 입력이 아니다.
구 v142/v149 release는 이전 source hash를 사용하므로 현재 코드로 강제 resume하지 않는다.

## 촬영 없는 사전 확인

```bash
cd ~/AjouNice2026
source base_ws/install_aw149/setup.bash
export BASE5_OBS="$HOME/morai_cap/base5_obstacle_improvement_261013"
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release "$BASE5_OBS/release_v150_sunny13/release.json" \
  --environment sunny13
```

기본은 CPU prepare-only다. 출력의 `prepared_waiting_for_resume`, `finished=0`은 촬영 완료가 아니다.
`--run`을 붙여야 실제 촬영한다. 사용자 승인 없이 실행하지 않으며 gate 파일을 수동으로 바꾸지 않는다.
launcher의 resume은 완료 raw·동결 source·metadata·코드 해시를 검사하며 기존 Sunny13 부모를 가져오지 않는다.

## 재연결 조건

MORAI가 종료되면 다시 실행하고 Sunny13·Capture Mode·기존 대회 센서를 유지한다.
카메라 1~4와 LiDAR 5의 Instance/GT 저장도 확인한다. Capture Mode만 켜져 있으면 RGB·LiDAR만 저장되고
annotation이 누락될 수 있다. 아래 Ego Network를 Load→Connect하며 master와 guarded bridge를 중복 실행하지 않는다.

```text
/home/luv/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json
```

구 `matched_baseline64`는 Park 불확실로 halted했고 첫 `long_vehicle_probe4_capture`는 annotation 누락으로
실패했다. 두 output을 강제 resume하지 않는다. 후속 새 output에서 버스 4장·64+8장·보충을 완료했다.
정지 25cm·CollisionData·가시성 조건을 완화하지 않았다. 동결 코드가 바뀌면 새 소량 검증과 release가 필요하다.

## 사용자 승인 후 본수집

아래 명령은 실제 5K를 시작한다. 이번 준비 작업에서는 실행하지 않았다.

```bash
cd ~/AjouNice2026
source base_ws/install_aw149/setup.bash
export BASE5_OBS="$HOME/morai_cap/base5_obstacle_improvement_261013"
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release "$BASE5_OBS/release_v150_sunny13/release.json" \
  --environment sunny13 --confirmed-environment sunny13 --run --complete
```

`--run`이 실제 5K 촬영 승인이고 `--complete`는 종료 후 유한 새 후보 보충까지 연결한다.
동일 명령이 안전한 일시정지 후 resume을 처리하지만 통신/Park 불확실·hash 변경은 우회하지 않는다.
유한 보충 후에도 부족분이 남을 수 있으며 exact 원래 자산·거리·개수의 변경은 mapping에 남긴다.
다른 날씨의 25K는 offline 계획만 있다. 실제 UI 환경 확인과 소량 smoke 후 해당 환경을 별도 qualification한다.
기존 v142 실측 8~11시간/35GiB를 이번 보강 release의 확정 예산으로 재사용하지 않는다.
이번 가중 예상은 약 8.14시간/31.25GiB, 운영 예산은 8~12시간/45GiB다. 다른 환경 25K를 한꺼번에
보관할 디스크 여유는 부족하므로 환경별 전송·보관 계획을 먼저 정하며 기존 데이터 자동 삭제는 하지 않는다.
