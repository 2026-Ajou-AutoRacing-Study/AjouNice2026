# Base5 종료 후 새 환경 수집 — 단일 진입점

2026-10-10 / 새 환경 전용. 아래 명령은 `collector_release_v141/release.json`의 실제
qualification 결과와 code/manifest hash를 먼저 검사한다. 파일이 없거나 gate가 미달하면 실행하지 않는다.
Sunny13 최종 5K와 구 런북의 동결 수집에는 적용하지 않는다.

## 무엇을 보존하고 바꾸는가

Sunny13 `collection_5k_sunny13_flexible_v140`의 5,000장·native GT·selected ledger는 보존한다.
다른 sunny11/15, foggy11/13/15는 각 환경에서 5,000개의 새 배치로 생성한다.
패밀리/primary 할당/거리·방향/미션 단계/실제 이동/split을 유지하고 Ego·객체 geometry seed는 새로 만든다.
범용 허용 도로의 Ego 위치는 고정192 bank 대신 연속 arc를 쓴다. 실제 지면/가시성/충돌 판정은 캡처 때 유지한다.
source slot과 실제 geometry group을 구분하며 같은 slot의 환경 A/B라고 부르지 않는다.

최종 생성 입력 후보는 `~/morai_cap/base5_postcollection_261010/fresh_suite_final_v142`다.
release 증거가 완성되기 전에는 준비완료/본수집 승인으로 해석하지 않는다.
중간 `fresh_suite`, `fresh_suite_v141b`, `fresh_suite_v141c`, 실패 `fresh_suite_final_v141`은 진단용이며 사용하지 않는다.
Sunny13 기본 원본 `environment_suite`의 다른환경 복사manifest도 사용하지 않는다.

## 터미널 순서

이미 master/bridge가 떠 있으면 1·2를 중복 실행하지 않는다. 실제 UI sunny/foggy와 11/13/15시를
**이번 environment와 일치하게 설정**한다. config 생성이나 아래 실행기가 MR UI를 자동으로 바꾸지는 않는다.

터미널 1 — master:

```bash
ajou
rc
```

터미널 2 — SaveSensorData ROS bridge:

```bash
ajou
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

기존 guarded bridge를 사용 중이면 그대로 유지한다. plain rosbridge로 교체하지 않는다.
Ego ROS network는 Base5 SaveSensorData용
`configs/network/ego_network_mr_base5_savesensordata_collision_local.json`이다.
센서 ROS streaming/UDP 실주행 녹화 조합으로 교체하지 않는다. 필요한 계약은
`/Ego_topic`, `/Object_topic`, `/CollisionData`, `/ScenarioLoad`, `/SaveSensorData`,
`/Service_MoraiEventCmd`, `/ctrl_cmd`다. CollisionData 무수신/서비스·Park 실패는 전체 안전 중단이다.

터미널 3 — CPU 사전 확인(촬영 없음):

```bash
ajou
export BASE5_POST="$HOME/morai_cap/base5_postcollection_261010"
export BASE5_RELEASE="$BASE5_POST/collector_release_v141/release.json"
python3 dataset/morai_od/scripts/launch_base5_fresh.py \
  --release "$BASE5_RELEASE" --environment sunny11 --prepare-only
```

터미널 3 — 실제 sunny11 수집·resume:

```bash
ajou
export BASE5_POST="$HOME/morai_cap/base5_postcollection_261010"
export BASE5_RELEASE="$BASE5_POST/collector_release_v141/release.json"
python3 -u dataset/morai_od/scripts/launch_base5_fresh.py \
  --release "$BASE5_RELEASE" --environment sunny11 --confirmed-environment sunny11
```

같은 명령이 신규/paused resume/완료검증을 처리한다. 단, STOP/PAUSE 파일을 남겼으면 먼저 이유를
확인하고 사용자가 해당 파일을 제거한 뒤 재개한다. halted/hash mismatch에 강제resume하지 않는다.
다른 환경은 두 `sunny11`을 모두 해당 tag로 바꾼다. 같은 MR에 두 collector를 동시에 실행하지 않는다.
기본 chunk16, 정적 조건부 대기 조정, service 확인 순서는 유지한다. early service-probe overlap은 탈락했다.
실제 이동은 기존 대기·지면 준비를 유지하며 static 최적화로 우회하지 않는다.

`--complete`는 terminal 부족분의 신규 유한보충까지 연결하는 선택 옵션이다.
**새 보충의 live qualification이 release에서 확인되기 전에는 이 옵션을 거부한다.**
그 이후에도 셀당 누적3회/실행당30분, native QA 유지, 같은 실패payload 재사용 금지다.
완료된 보충을 다시 실행해도 상한이 초기화되거나 성공분이 재촬영되지 않는다.
가족/미션/semanticpool/split/실제primary gait와 moving 최소인원은 유지하고
변경 자산·거리·방향·개수는 mapping/adopted_manifest에 명시한다. 희귀primary floor를 보호한다.
상한 후 미달은 명시된 부족분이지 무조건5,000승인 보장이 아니다.

터미널 4 — 새 suite 대시보드(구 8075 유지, 예시 8082):

```bash
ajou
export BASE5_POST="$HOME/morai_cap/base5_postcollection_261010"
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$BASE5_POST/collection_5k_fresh_v141_sunny11" \
  --manifest "$BASE5_POST/fresh_suite_final_v142/sunny11/manifest.json" \
  --suite "$BASE5_POST/fresh_suite_final_v142/suite.json" \
  --port 8082
```

`http://localhost:8082/`. 실제 보충의 canonical이 생기면 capture-root는 `_completed` 출력으로 바꾼다.
그때 기존 parent 선택을 다시 합산하지 않는다. qualification pilot은 학습5K나 suite 승인수에 포함하지 않는다.

## 종료 후

status/quota/selected ledger·실제 asset별 positive coverage를 확인한다. 총30K 후 Sunny13 재수집 필요성을 판단한다.
다섯 환경의 입력을 생성했다는 것과 실제30K를 수집했다는 것은 다르다.
대략32–33GiB/5K raw 외에 실패·동적 QA·보충·infos·archive 공간이 더 필요하다.
다음25K의 예상 여유를 release/cost report에서 확인하며 기존 데이터 자동삭제는 하지 않는다.
OD single-frame 7D/velocity OFF, 4class는 유지한다. 최종 crop/해상도 정책 확정 후 visibility/infos를 재변환한다.
CPU qualification pilot을 본학습에 합치지 않는다.
