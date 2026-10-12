# Base5 장애물 보강 후보 확인과 수집 준비

문서 상태: CANDIDATE. 기준일: 2026-10-13.
기존 Sunny13 5K는 보존했다. 새 5K와 다음 25K는 배치 파일만 생성했으며 본수집을 시작하지 않았다.
구현·실측·남은 문제는 [실험 근거](261013_base5_obstacle_support_experiments.md)를 따른다.

## 현재 위치

| 항목 | 경로 또는 상태 |
|---|---|
| 실험 root | `~/morai_cap/base5_obstacle_improvement_261013` |
| 새 독립 Sunny13 5K 계획 | `final_v149/sunny13/manifest.json` |
| 다음 다섯 환경 25K 계획 | `final_v149/future_suite/suite.json` |
| Sunny13 후보 release | `release_v149_sunny13/release.json` |
| 정적 회귀와 유한 보충 | `qualification_v149_static64`, `qualification_v149_static64_completed` |
| 보호된 실제 이동 회귀 | `qualification_v149_animated8` |
| 남은 긴 버스 4장 probe | `long_vehicle_probe4/manifest.json` |
| 본수집 gate | `bulk_ready=false`; 긴 NPC live 검증과 MORAI 재연결 필요 |

`generic-obstacle-rich-floor-v145`는 정책 family 이름이며 이번 코드·manifest·검증 동결은 v149다.
초기 v1/v2/v3 prototype과 `candidate_v145`, `candidate_v148`은 진단 이력이지 본수집 입력이 아니다.
구 v142 release는 이전 source hash를 사용하므로 현재 코드로 강제 resume하지 않는다.

## 촬영 없는 사전 확인

```bash
cd ~/AjouNice2026
export BASE5_OBS="$HOME/morai_cap/base5_obstacle_improvement_261013"
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release "$BASE5_OBS/release_v149_sunny13/release.json" \
  --environment sunny13
```

기본은 CPU prepare-only다. 출력의 `prepared_waiting_for_resume`, `finished=0`은 촬영 완료가 아니다.
현재 release에 `--run`을 붙이면 `bulk_ready=false`로 거부된다. gate 파일을 수동으로 바꾸지 않는다.
launcher의 resume은 완료 raw·동결 source·metadata·코드 해시를 검사하며 기존 Sunny13 부모를 가져오지 않는다.

## 남은 live 검사

MORAI가 Vulkan SIGSEGV로 종료됐으므로 먼저 다시 실행한다. Sunny13·Capture Mode·기존 대회 센서를
유지하고 아래 Ego Network를 Load→Connect한다. ROS master와 guarded bridge가 살아 있으면 중복 실행하지 않는다.

```text
/home/luv/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json
```

직전 `matched_baseline64`는 Park를 확인할 수 없어 halted 상태다. 강제 resume하거나 안전 파일을 덮어쓰지 않는다.
신선한 Ego/Object/CollisionData와 제어 서비스·Park를 확인한 뒤 새 output으로 다음 4장을 실행한다.

```bash
cd ~/AjouNice2026
export BASE5_OBS="$HOME/morai_cap/base5_obstacle_improvement_261013"
python3 dataset/morai_od/scripts/run_base5_fresh_batch.py \
  --manifest "$BASE5_OBS/long_vehicle_probe4/manifest.json" \
  --output "$BASE5_OBS/long_vehicle_probe4_capture" \
  --limit 4 --chunk-size 4 --setup-profile static-ready
```

4장의 NPC before/after 위치·native ground·실제 정지와 접촉을 확인한다. 긴 차체의 방향/도로 지지 조건을
채택하면 primary 자산 수는 유지하고 바뀐 yaw 계약을 delta로 기록한 새 release를 다시 생성한다.
현재 조건에 맞추기 위해 정지 25cm·CollisionData·가시성 조건을 완화하지 않는다.

## 향후 본수집 진입점

live 문제가 해결된 새 release를 동결한 후에만 다음 형식을 사용한다. 현재 v149는 실행을 거부한다.

```bash
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release "$BASE5_OBS/release_v149_sunny13/release.json" \
  --environment sunny13 --confirmed-environment sunny13 --run --complete
```

`--run`이 실제 5K 촬영 승인이고 `--complete`는 종료 후 유한 새 후보 보충까지 연결한다.
동일 명령이 안전한 일시정지 후 resume을 처리하지만 통신/Park 불확실·hash 변경은 우회하지 않는다.
유한 보충 후에도 부족분이 남을 수 있으며 exact 원래 자산·거리·개수의 변경은 mapping에 남긴다.
다른 날씨의 25K는 offline 계획만 있다. 실제 UI 환경 확인과 소량 smoke 후 해당 환경을 별도 qualification한다.
기존 v142 실측 8~11시간/35GiB를 이번 보강 release의 확정 예산으로 재사용하지 않는다.
