# Base5 종료 후 새 환경 수집 — 단일 진입점

2026-10-10 기준 새 환경 전용 런북이다. 아래 명령은 `collector_release_v142/release.json`에 기록된
실제 검증 결과와 코드·manifest 해시를 먼저 검사한다. 파일이 없거나 검증 기준에 미달하면 실행하지 않는다.
Sunny13 최종 5K와 구 런북의 동결 수집에는 적용하지 않는다.

## 무엇을 보존하고 바꾸는가

Sunny13 `collection_5k_sunny13_flexible_v140`의 5,000장·native GT·selected ledger는 보존한다.
다른 sunny11/15, foggy11/13/15는 각 환경에서 5,000개의 새 배치로 생성한다.
패밀리/primary 할당/거리·방향/미션 단계/실제 이동/split을 유지하고 Ego·객체 geometry seed는 새로 만든다.
범용 허용 도로의 Ego 위치는 고정 기준점 192개 대신 도로를 따라 연속적으로 샘플링한다. 캡처 때의 지면·가시성·충돌 판정은 유지한다.
원본의 구성표 slot과 실제 배치 group을 구분한다. 같은 slot이라도 배치가 달라지므로 동일 장면의 환경 A/B 비교라고 부르지 않는다.

최종 생성 입력은 `~/morai_cap/base5_postcollection_261010/fresh_suite_final_v142`다.
수집기 qualification/5 환경 CPU preflight는 완료했고 실제 25K 본 수집은 아직 시작하지 않았다.
중간 `fresh_suite`, `fresh_suite_v141b`, `fresh_suite_v141c`, 실패 `fresh_suite_final_v141`은 진단용이며 사용하지 않는다.
Sunny13 기본 원본 `environment_suite`의 다른 환경 복사 manifest도 사용하지 않는다.

## 터미널 순서

이미 master/bridge가 떠 있으면 1·2를 중복 실행하지 않는다. MR UI의 sunny/foggy와 11/13/15시를
**이번에 수집할 환경과 일치하게 설정**한다. config 생성이나 아래 실행기가 MR UI를 자동으로 바꾸지는 않는다.

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
export BASE5_RELEASE="$BASE5_POST/collector_release_v142/release.json"
python3 dataset/morai_od/scripts/launch_base5_fresh.py \
  --release "$BASE5_RELEASE" --environment sunny11 --prepare-only
```

터미널 3 — 실제 sunny11 수집·resume:

```bash
ajou
export BASE5_POST="$HOME/morai_cap/base5_postcollection_261010"
export BASE5_RELEASE="$BASE5_POST/collector_release_v142/release.json"
python3 -u dataset/morai_od/scripts/launch_base5_fresh.py \
  --release "$BASE5_RELEASE" --environment sunny11 --confirmed-environment sunny11 --complete
```

같은 명령이 신규 수집·일시정지 후 재개·완료 검증을 처리한다. 단, STOP/PAUSE 파일을 남겼으면 먼저 이유를
확인하고 사용자가 해당 파일을 제거한 뒤 재개한다. halted/hash mismatch에 강제 resume하지 않는다.
다른 환경은 두 `sunny11`을 모두 해당 tag로 바꾼다. 같은 MR에 두 collector를 동시에 실행하지 않는다.
기본 chunk 16, 정적 조건부 대기 조정, 서비스 확인 순서는 유지한다. 서비스 사전 확인을 다른 대기와 겹치는 후보는 검증에서 탈락했다.
실제 이동 장면은 기존 대기·지면 준비를 유지하며 정적 장면용 최적화로 우회하지 않는다.

`--complete`는 본 수집 종료 후 부족분을 새 후보로 보충하는 단계까지 연결한다(생략하면 본 수집만 실행한다).
새 정적 64장·이동 8장 검증에서 각각의 부족분 1장을 첫 새 후보로 복구했고, 부모 수집 보존·완료 후 resume 검증도 통과했다.
release의 실제 검증 근거가 사라지거나 코드·해시가 바뀌면 이 옵션을 거부한다.
그 이후에도 셀당 누적 3회/실행당 30분, native QA 유지, 같은 실패 payload 재사용 금지다.
완료된 보충을 다시 실행해도 상한이 초기화되거나 성공분이 재촬영되지 않는다.
패밀리·미션·semantic pool·split·실제 primary gait와 이동 객체의 최소 인원은 유지하고
변경 자산·거리·방향·개수는 mapping/adopted_manifest에 명시한다. 희귀 primary floor를 보호한다.
상한 후 미달은 명시된 부족분이지 무조건 5,000 승인 보장이 아니다.

터미널 4 — 새 suite 대시보드(구 8075 유지, 예시 8082):

```bash
ajou
export BASE5_POST="$HOME/morai_cap/base5_postcollection_261010"
python3 dataset/morai_od/scripts/base5_fresh_dashboard.py \
  --capture-root "$BASE5_POST/collection_5k_fresh_v141_sunny11" \
  --manifest "$BASE5_POST/fresh_suite_final_v142/sunny11/manifest.json" \
  --suite "$BASE5_POST/fresh_suite_final_v142/suite.json" \
  --port 8082
```

`http://localhost:8082/`에서 확인한다. 원본 해시와 부모 수집 계보가 일치하는 `_completed`가 생기면 자동으로 전환한다.
보충 ledger에 이미 들어 있는 부모 성공분은 다시 합산하지 않는다. 다른 환경의 보충도 suite 집계에 반영한다.
관측기만 추가했으며 원래 8075는 Sunny13 v140을 그대로 표시한다.
검증용 pilot은 학습 5K나 suite 승인 수에 포함하지 않는다.

## 종료 후

상태·quota·selected ledger와 실제 자산별 유효 GT 분포를 확인한다. 총 30K 후 Sunny13 재수집 필요성을 판단한다.
다섯 환경의 입력을 생성했다는 것과 실제 30K를 수집했다는 것은 다르다.
pilot 실측을 구성 비율로 가중한 5K 예상은 7.22시간/27.04GiB다. 운영 예산은 5K당 8–11시간/35GiB이며 보장값은 아니다.
이 수집기 release를 동결할 당시 디스크 여유는 약 187GiB였다. 다음 25K의 가중 예상은 약 135GiB, 보수 예산은 175GiB다. 후처리·대형 압축본 복사를 위한 여유는 별도로 필요하다.
다음 25K의 예상 여유를 release/cost report에서 확인하며 기존 데이터 자동 삭제는 하지 않는다.
수집 원본의 vehicle/pedestrian/obstacle/animal 의미와 component GT는 유지한다.
현재 학습은 Animal을 Pedestrian에 통합한 3클래스 single-frame 7D이며 velocity supervision은 OFF다.
이미지·crop·학습 입력은 [모델 CURRENT](../../../docs/bevfusion/CURRENT.md#model),
새 raw의 파생 GT/JPEG 변환은 [데이터 GT 계약](BASE5_V2_CURRENT.md#gt)을 따른다.
이 라벨 정책 정리는 동결 수집 release·시나리오·resume 해시를 변경하지 않는다.
CPU 검증용 pilot을 본학습에 합치지 않는다.
