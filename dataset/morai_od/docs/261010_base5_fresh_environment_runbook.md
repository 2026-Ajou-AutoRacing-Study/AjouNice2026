# Base5 종료 후 새 환경 수집 — 단일 진입점

2026-10-17 Sunny15 후속 작업은 [아래 신규 검증 기록](#sunny15-261017)을 먼저 본다.
기존 v142 실행 명령은 당시 이력이며 v158a 이후 원형 보강을 담은 새 환경의 승인이 아니다.

2026-10-13 이후 장애물 보강 작업은 [현재 후보 런북](261013_base5_obstacle_support_runbook.md)을 따른다.
아래 v142는 보존한 baseline이며 변경된 현재 코드로 강제 resume하지 않는다.

2026-10-10 기준 새 환경 전용 런북이다. 아래 명령은 `collector_release_v142/release.json`에 기록된
실제 검증 결과와 코드·manifest 해시를 먼저 검사한다. 파일이 없거나 검증 기준에 미달하면 실행하지 않는다.
Sunny13 최종 5K와 구 런북의 동결 수집에는 적용하지 않는다.

2026-10-13 후속 계획은 [장애물 생성기 배치 실험](261013_base5_obstacle_generator_experiment_plan.md)이다.
아래 명령은 기존 v142 baseline용이며 보강 후보의 실행 명령이 아니다. 새 정책은 실험·qualification 후
별도 release로 전달하며 기존 Sunny13·v142 manifest에 강제 resume하지 않는다.

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

<a id="sunny15-261017"></a>

## 2026-10-17: 로컬 Sunny15 RGB 비교와 독립 배치 재조립

사용자는 로컬 MORAI의 적극적인 조작·소량 실험을 허용했다. 빌린 PC에서 진행 중인
Sunny13 수집과는 별개다. 이 작업에서 빌린 PC에 접속하거나 프로세스·파일을 변경하지 않았다.
로컬 5K 본수집도 자동 시작하지 않는다. 아래 표본과 CPU 생성물은 완료된 학습 데이터가 아니다.

### 환경별 분포 원칙

- Sunny11/13/15, Foggy11/13/15는 같은 배치를 여섯 번 촬영하지 않고 각각 새 geometry를 만든다.
- family·주요 자산 coverage·거리/방향·미션 phase·원형 배율 규칙은 공통으로 유지한다.
  그 안의 Ego 연속 위치·객체 좌표·정지 yaw·허용 보조 조합을 새 seed로 샘플링한다.
- 시간마다 일부 지도 구역을 train에서 배제하거나, 15시에만 특정 클래스·거리 비율을 늘리지 않는다.
  모든 환경에서 알려진 지도 전역의 train coverage와 희귀 자산 하한을 보호한다.
- v150c의 frozen 전체 actor roster는 793/5,000장이다. 나머지 혼합 장면은 보조 종류·개수가
  자연스럽게 달라질 수 있으므로 v158a의 총 부모 수와 정확히 같다고 주장하지 않는다.
- 동일 geometry를 의도적으로 재촬영한 RGB 비교 표본은 독립 표본으로 합산하지 않는다.
  실제 반복 배치·연속 장면은 같은 split으로 묶고, 독립 환경의 같은 논리 slot만으로 묶지는 않는다.

### 실제 RGB 대조: 정적 16쌍

원본은 같은 로컬 PC의 `base5_unit_release_261017/capture_smoke_v4/selected_ledger.jsonl`이다.
`prepare_base5_environment_rgb_pilot.py`로 원본 payload·metadata·effective split/group을
고정한 16개 정적 배치를 등록하고, 같은 배치를 사용자 설정 Sunny15에서 새로 촬영했다.
움직이는 객체의 gait·위치 변화를 조명 변화와 섞지 않도록 static/standing만 선정했다.

원거리·음영의 동결 geometry 검사는 source 환경 태그까지 확인한다. 따라서 RGB 진단에서만
`config.environment_tag=sunny13`을 원본 배치의 identity로 보존하고, 실제 촬영 선언은
`config.collection_environment.hour=15` 및 `environment_rgb_pilot.physical_capture_environment=sunny15`에
따로 기록했다. 검사 제거·과거 source 수정·15시 본수집 승인 계승이 아니다. 실제 UI 시간은
사용자의 15시 설정 확인에 근거하며 프로그램이 UI 시간·그래픽 옵션을 읽어 인증한 것은 아니다.

| 항목 | 실제 결과 |
|---|---:|
| exact payload 정적 촬영 | 16/16 승인, 검토·실패 0 |
| 승인 장면 duration 합 | 75.29초. 재개 간격·분석·전체 5K 비용을 포함하지 않음 |
| native 파일 용량 | 83,749,021 bytes |
| 전후 센서 geometry 동일 | 16/16쌍 |
| 실제 Ego 위치 차이 최대 | XY 약 0.031mm, Z 약 0.357mm |
| 정적 source 전후 보존 | 보고서 입력 1,674개 파일 불변 |
| 장면 사이·종료 안전 | 충돌 검사 및 P 확인 통과 |

Q90 JPEG를 현재 512×1056 bottom crop으로 처리한 뒤 카메라별 평균 RGB 절대 차이는
0~255 기준 18.96 / 17.29 / 21.14 / 18.80이었다. 평균 밝기 변화는
−4.88 / −9.54 / −14.62 / +0.49다. foreground 주변 대비를 계산할 수 있는 72개
camera 기록 중 56개는 대비가 줄고 16개는 늘었다. 이는 소량 표본의 local 밝기 proxy이며
객체별 독립 표본 수·검출 AP·인지 난이도의 정답이 아니다. 복합 성분의 모호한 대응도 남겼다.

직접 RGB를 확인해 하늘·태양 방향·노면과 Ego 그림자·객체 명암 변화를 관측했다.
15시가 단순히 13시와 같은 영상인 것은 아니다. 다만 그래픽 옵션·렌더러 변화와 반복 잡음을
완전히 배제한 실험은 아니므로 시간대 하나의 인과 효과나 AP 향상으로 해석하지 않는다.
현재 근거로는 구성 비율·거리 규칙을 바꿀 이유보다 새 배치와 실제 조명 조건을 추가할 이유가 크다.

근거 루트는 `artifacts/evaluation/base5_sunny15_261017`이다.

- 사전 등록: `rgb_registration_v2/{manifest,registration}.json`.
- 실제 원본: `rgb_capture_v2/ledger.jsonl`, `shutdown_safety.json`, `captures/`.
- 최종 분석: `rgb_report_v4/report.json`, `contact_sheet_native*.png`, `contact_sheet_model512*.png`.
- `rgb_registration_v1`의 환경 identity 검사 실패는 MR 조작 전이다. `rgb_report_v1`의
  배경 BBox 누락 처리 실패·v2의 standing 제외 버그·v3 중간 결과는 당시 근거로 보존했다.
  최종 분석기는 정상 배경과 annotation 누락을 구분하고 standing을 정지로 처리하며,
  심볼릭 링크를 통한 raw 내부 출력과 원본 파일 추가·삭제도 차단·탐지한다.

### 새 환경 CPU 생성 경로

`prepare_base5_unit_environment.py`는 기존 동결 v158a를 바로 reseed하지 않는다.
pre-front v150c richfloor에서 새 geometry를 만든 뒤 front 보강 → primary 거리 제한 →
원거리 보조 차량 160개와 창 안 분산 → 정적 미션 전/안/후 300/120/180 → 원형 치수 QA →
새 eligible 초근접 관측 보강 → 최종 layout overlay → Sunny13와 실제 geometry 합본 감사를 수행한다.
초근접 변경 수를 이전 Sunny13의 23장에 강제로 맞추지 않는다. 배율은 모두 원형이며 확대 helper는 쓰지 않는다.

첫 실행에서 frozen 보조 차량에 필요한 우측 차선이 없는 새 Ego stratum을 찾았다.
새 wrapper의 opt-in 사전 선별에서 실제 MGeo의 차선 변경 허용과 목적 link 존재를 확인한다.
전체 객체·거리 계약을 줄여 통과시킨 것이 아니며 기존 수집 generator 파일도 바꾸지 않았다.
이어 known-map 임시 split이 geometry 변경 때 달라지는 단계 간 충돌을 발견했다.
새 생성물에만 기존 지원 필드 `spec.layout_split`을 명시적으로 동결하고, 실제 학습 split은
마지막 전수 overlay에서 다시 할당한다. 과거 source나 raw의 split을 바꾸는 기능이 아니다.

`environment_v1`·`environment_v2`·`environment_v3`의 실패는 삭제하거나 성공으로 소급 변경하지 않는다.
v3의 원거리 거리 분산에서는 가장 가까운 8개 후보가 기존 primary를 가리는 slot을 발견했다.
새 wrapper는 가까운 7개와 앞 단계에서 정상으로 생성한 위치 1개를 같은 8회 검사 안에 포함한다.
가림·도로 지지·거리 창 검사는 다시 실행한다. v4의 160개 중 3개에서만 마지막 후보를 썼으며,
목표 거리 오차는 중앙값 0.158m / p90 0.241m / p95 1.299m / 최대 16.628m다.
거리 창·Ego·primary는 보존하지만 실제 거리가 정확히 균등하다고 주장하지 않는다.
v4는 완료된 `environment_v3/04_far/manifest.json`을 명시적인 SHA로 읽기 전용 승계한다.
기존 다섯 단계를 이번 실행에서 다시 수행했다고 기록하지 않는다. v3가 실패한 실행이라는 이유로
전체 폴더를 지우면 v4의 실제 source 계보가 사라지므로 삭제하지 않는다.
새 환경의 source/runtime 해시·시나리오 전수·분포·분할·소량 native를 각각 확인해야 한다.
기존 음영 이동 두 장면의 지면 준비는 exact payload 계약이므로 새 경로에 그대로 재사용하지 않는다.
나머지 환경까지 생성한 뒤에는 Sunny13↔각 환경뿐 아니라 여섯 환경 전체를 누적 감사하고,
수집 후 실제 adopted/selected geometry로도 다시 검사해야 한다.

이 실행기의 `--run`은 **CPU 생성만** 뜻한다. native·5K 수집·본학습을 시작하지 않는다.

실제 완성된 CPU 후보는 `environment_v4/candidate_sunny15/manifest.json`이다. SHA는
`1df9c2948b507a8bc141de052202bac6e995e97253e37afb899df011efcd2e12`이며
`environment_v4/build_receipt.json`에 source·runtime·단계별 실제 계보가 있다.
시나리오와 고유 payload 각각 5,000개, train 4,250 / val 750, 독립 layout group 5,000개다.
Sunny13 계획 5K와 합쳐 검사한 10K에서 exact/near 반복 및 cross-split 중복은 0개였다.
이는 현재 두 계획의 감사이며 수집 후 회복 배치나 나머지 네 환경까지 검증한 값은 아니다.
같은 구성표 slot끼리 비교하면 Ego XY가 1mm 넘게 달라진 것은 4,997장, 5cm 넘게 달라진 것은
4,945장이다. 다만 새 위치에서 기존 전체 Ego 위치 중 가장 가까운 것까지의 거리 중앙값은
약 0.301m다. 같은 지도·차로를 다시 촬영하므로 배경이 완전히 새로운 5K라고 표현하지 않는다.
객체까지 포함한 실제 geometry 중복과 단순 Ego 근접은 별도 지표다.

primary 자산·카테고리·거리 구성·phase·이동 계약은 유지했고 전체 배치 부모는 11,423개다.
19종 물리 장애물의 계획상 부모는 0~20m 2,794 / 20~30m 839 / 30m 이상 347개다.
native 가시 GT 수가 아니다. 정지 4,822 / 이동 178의 구성과 음영 이동 24개를 유지한다.
초근접 관측 보강은 새 inventory에서도 23개가 나왔고 전부 적용됐다. 이전 23에 맞춘 것이 아니다.
현재 후보의 `bulk_ready=false`, `native_qualified=false`는 유지하며, 원형 v158a Sunny13의
release 및 manifest SHA는 각각 `0a3dd5c9…` / `4f7fc3b3…`로 기존 동결값과 동일하다.

```bash
python3 -B -u dataset/morai_od/scripts/prepare_base5_unit_environment.py \
  --source "$HOME/morai_cap/base5_obstacle_improvement_261013/candidate_v150c/sunny13/manifest.json" \
  --baseline artifacts/evaluation/base5_unit_release_261017/close_primary_fullsource_v1/candidate_v158a/manifest.json \
  --output <새로운_출력_경로> --environment sunny15 --seed 261017 --run
```

로컬 여유는 10/17 21:10 약 49GiB로 소량 시험은 가능하지만 새 5K 시작 기준 55GiB보다 적다.
본수집 전 별도 공간 확보가 필요하며, 이 작업에서 기존 원본·실패 근거·백업을 자동 삭제하지 않았다.

### 독립 배치의 실제 native 32장 시험

`prepare_base5_unit_environment_smoke.py`로 결과를 보기 전에 32장을 사전 등록했다.
물리 장애물 19종, 복합 자산 4종, 정적 미션 전/안/후와 회피 4단계, 초근접 관측 보강,
50~80/80~100m 전·후 차량, 두 횡단보도와 세 번째 독립 이동 장면, 음영 접근·내부,
train/val을 포함한다. 새 음영 이동 24장은 그대로 유지하되 이 시험에서는 제외했다.
이들의 새 경로에 과거 두 장면의 지면 준비 승인을 복사하지 않았다.

| 항목 | 실제 결과 |
|---|---|
| 사전 등록한 exact payload | 32/32 취득, 첫 4장 뒤 정상 종료·재개 |
| 기존 QA 판정 | 승인 후보 28장 / 검토 4장 / 통신 실패 0장 |
| native 관측 | 물리 장애물 19종 31부모의 8Points 대조, 복합 부모 5개, 전체 component 78개 |
| native 저장량·장면 duration 합 | 168,394,511 bytes / 168.78초. 분석·재개 간격은 별도 |
| 안전 확인 | 모든 장면의 충돌 근거와 장면 사이 P 확인, 두 chunk의 종료 P 확인 통과 |
| 본수집 자격 | `bulk_ready=false`, `native_qualified=false` 유지 |

32장은 계약을 폭넓게 확인하려고 고른 표본이다. 28/32를 무작위 전체 5K의 예상 승인율로
외삽하지 않는다. 4개 검토 원본과 audit는 수정하지 않았고, 같은 실패 payload를 반복 촬영하지 않았다.
검토 중단 후 원인과 안전 증거를 확인하고 이미 등록한 나머지 장면만 이어서 취득했다.

| ID 끝자리 | 기존 검토 사유 | 확인한 내용과 후속 처리 |
|---|---|---|
| 01156 | 심각한 지면 아래 배치로 판정 | 약 42m의 Bicycle_city 주변에서 상부 평면을 도로처럼 잡았다. Ego roll 약 2.58°를 생략한 지도 높이 대조도 영향을 줬다. 실제 8개 꼭짓점과 같은 capture의 IMU 자세를 쓰면 지도 높이 차이는 약 −3.7mm다. GT 오류나 실제 접촉 인증으로 단정하지 않는다 |
| 04512 | 심각한 지면 아래 배치로 판정 | 약 54m의 Woman4 주변에서도 상부 평면이 선택됐다. full RPY 대조 높이 차이는 약 −16.1mm다. 기존 좁은 예외는 일반 정적 보행자까지 허용하지 않는다 |
| 03754 | 보이는 부모 수 부족 | 약 18m의 Soul 뒤 약 44m의 Grandeur가 LiDAR 0점·카메라 근거 0이다. 실제 보조 객체 가림이며 지면 검사 후보로 우회하지 않는다 |
| 04027 | primary 센서 근거 없음 | 약 47m WoodBox가 LiDAR 0점·카메라 근거 0이다. 중앙 가드레일·기둥 뒤 영역이며 CPU actor OBB 검사만으로 고정 구조물 가림을 보장할 수 없다 |

두 지면 사례는 native GT를 고쳐야 한다는 증거가 아니라 **검사용 지면 연결과 좌표 변환의 한계**다.
`base5_static_surface_attitude_candidate.py`를 기존 수집기에 연결하지 않은 독립 분석기로 만들었다.
같은 SaveSensorData basename의 IMU quaternion·센서 장착 자세·실제 native 8Points·MGeo를 대조한다.
정지/P·낮은 각속도·IMU raw yaw와 Ego heading의 대응을 확인하고 −1.347°는 한 번만 적용한다.
근거리·복합체·실제 이동·확실한 심각한 부유/매몰은 예외로 통과시키지 않는 CPU 실패 대조도 넣었다.

RGB 16장과 독립 배치 32장, 합계 48장에 적용한 읽기 전용 후보에서 기존 정상 44장은 모두 불변이다.
물리 객체만 허용하면 01156 한 장, 일반 정적 보행자를 별도 opt-in하면 04512까지 두 장만
`surface association unverified` 후보로 분류된다. 도로 접촉을 확인한 정상 장면으로 승격한 것은 아니다.
실제 가시성 부족 두 장은 그대로 검토이며, 기존 수집기의 28/32 판정도 바꾸지 않았다.
39/48장의 native IMU `nSecs`는 ROS 범위 밖이므로 원문과 SHA를 보존했다. 정지·동일 파일명
대응을 썼을 뿐 정확한 센서 시간 동기화를 인증하지 않는다. 실제 심각한 매몰·부유 장면의
live 대조군은 이번 48장에 없으므로 해당 실패 대조는 CPU 검증과 구분한다.

근거는 같은 루트의 `environment_smoke_registration_v1/`, `native_audit_v1.json`,
`environment_native_v1/chunk_{0,1}/`, `static_attitude_candidate_v3/{physical_only,static_pedestrian_opt_in}/report.json`이다.
native 파일 1,106개가 읽기 전용 감사 전후 동일했고 첫 부분 취득 파일 134개도 재개 후 동일했다.
원본·native GT·이전 Sunny13 동결 수집기는 변경하지 않았다.

독립 리뷰에서 사전 등록 출력의 동시 생성 경합 가능성을 찾아 쓰기 직전 독점 `mkdir`로 차단했다.
기존 디렉터리뿐 아니라 검사 중 생긴 symlink도 덮어쓰지 않는 회귀 검사를 통과했다.
32장 사전 등록의 당시 helper SHA는 `fbbf7ad1b2d2aa967e6fcc55ed379de21bcd68fb1e3db4ba7365b201e7694b0b`이며,
원문은 `smoke_source_before_output_reservation.py`에 보존했다. `native_audit_v1.json`은 변경 전에
당시 helper/runtime/input으로 재생 검사를 완료한 결과다. 출력 안전 보강 뒤 현재 helper SHA가
달라진 것은 의도한 계약 변경이므로 구 등록을 현재 helper로 강제 승인·resume하지 않는다.
향후 native 시험은 새 등록 경로를 사용한다. 기존 collector와 Sunny13 release의 변경은 아니다.
격리 자세 분석기도 같은 출력 경합을 차단했다. v2는 당시 보고서로 보존하고, 출력 안전 보강 뒤
v3에서 같은 48장을 다시 읽어 판정 변화 목록과 원본 보존이 동일함을 확인했다.
최종 CPU 회귀는 `cpu_regression_v7/receipt.json`의 2,182개 통과다. 기존 Base3의 host torch
의존 테스트 1개만 정해진 제외 목록을 유지했으며 새 검토 사례를 통과시키려고 테스트를 제외하지 않았다.
runtime·테스트 SHA를 실행 전후 대조했고 GPU 추론·학습은 실행하지 않았다.
출력 경합 두 helper의 표적 검사 94개는 위 전체 검사에 포함되며 따로 더해 세지 않는다.

### 판단과 다음 순서

Sunny15는 Sunny13와 공통 quota·거리·객체 의미를 유지하면서 새 Ego·객체 배치로 수집하는 방향이다.
RGB 차이가 있으므로 15시를 추가할 근거는 있지만, 어둡게 보이는 객체만 골라 비율을 바꾸거나
밝은 장면으로 교체하지 않는다. 일반화 효과의 크기는 후속 학습·bag 평가로 판단한다.

1. 위 지면 QA 후보의 실제 도입 여부를 결정하고 도입한다면 새 runtime으로 별도 재검증한다.
   현재 분석 보고서를 기존 source의 승인 근거로 소급 사용하지 않는다.
2. 센서 근거가 없는 두 장면은 기존 기준을 유지한 채 새 geometry의 제한된 복구를 시험한다.
   성공분·실패 원본·기존 검토 사유는 보존하고, 최종 채택 배치로 공간·split 감사도 다시 한다.
   메모리에서 각각 32개를 생성했을 때 전체 spec·roster 일치 후보는 03754에서 3개,
   04027에서 20개였다. 첫 일치 후보도 다시 가릴 수 있으므로 기존 all-actor visibility 점수로
   정상 후보끼리 비교한 뒤 native로 확인한다. 아직 해당 새 후보를 촬영·최종 채택한 것은 아니다.
3. 새 음영 이동 24장의 개별 지면 준비, Q90/upright·실제 train/val 입력과 현재 runtime을 묶어
   최종 자격을 확인한다. 준비되지 않은 이동을 정적 촬영으로 바꾸지 않는다.
4. 로컬 공간을 별도 승인 범위에서 확보한 뒤 Sunny15 5K 본수집을 시작한다.
   Sunny11·Foggy로 확장할 때는 기존 환경 전체와 실제 배치 중복·split을 누적 감사한다.

이번에는 로컬 소량 촬영만 수행했고, 빌린 PC의 Sunny13 진행 상태는 조회하거나 변경하지 않았다.
종료 시점의 로컬 디스크 여유는 약 47GiB로 55GiB 시작 기준보다 적다. 공간 부족이나 위 네
검토 사례를 무시하고 본수집을 시작하지 않는다. 로컬 촬영 종료 시 P 확인은 완료했다.
