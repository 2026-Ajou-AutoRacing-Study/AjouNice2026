# Base5 전방 장애물·원거리 차량·복합 객체 소량 실험

문서 상태: EVIDENCE / CANDIDATE. 기준일: 2026-10-15.
완료 Sunny13 5K·기존 GT·E24·v150 동결 산출물은 보존한다. 이번 데이터는 별도 진단용이며
기존 5K의 quota나 train/val에 합산하지 않는다. 본수집과 학습은 실행하지 않았다.

아래는 최초 70장 진단 당시의 상태다. 이후 보충 통합·64+8장 회귀·새 release의 현재 결과는
[후속 보충 검증](261015_base5_front_recovery_release.md)을 따른다. 과거 미지원/미검증 기록을 소급 수정하지 않는다.

## 기존 실제 5K에서 확인한 부족분

배치 수, 가시 GT 수, 학습 클래스 수를 구분한다. 아래 첫 표는 기존 upright train 4,255장의
실제 Obstacle GT이며 거리 기준은 LiDAR XY다. 전후는 wedge가 아닌 X 부호다.

| 거리 | Obstacle GT | 관측 |
|---|---:|---|
| 0~10m | 780 | 전방 285 / 후방 495 |
| 10~20m | 837 | 기본 근거리 표본은 유지 |
| 20~30m | 610 | 종류에 따라 센서 지원 차이가 큼 |
| 30m 이상 | 388 | 보이는 GT를 일괄 삭제하지 않음 |

20~30m CargoBox 42개 중 LiDAR 0점 19개, 1~4점 14개였다. Bollard04 15개는 모두 1~4점이고,
Lane_Separation_Bar 20개는 0점 5개·1~4점 15개였다. 점 수만으로 잘못된 수집이라고 판정하거나
20점을 보편적인 승인 기준으로 요구하지 않는다. 카메라만 지원하는 정상 GT도 유지한다.

기존 5K의 Vehicle 80~100m 유효 GT는 219개다. 전방 107개 중 37개(34.6%)가 LiDAR 0점이며
후방 112개는 0점이 없었다. 전후 중앙값은 각각 2/4점이다. 카메라 gate는 213/219이지만,
512 입력 BBox 짧은 축 중앙값은 약 10px다. 가시성 통과가 높은 검출 AP를 보증하지 않는다.

특히 해당 거리 val은 6개이며 고주로 3개·음영 3개다. v150 계획도 고주로 far primary 207개 중
val 3개, val Ego 물리 link 1개다. 이 작은 분모의 AP를 안정적인 성능 추정으로 해석하지 않는다.
기존 split을 사후 이동시키지 않고 독립 bag/물리 구역 평가를 별도로 설계한다.

## v151: 객체 수를 늘리지 않는 전방 지원 후보

새 opt-in 정책은 `generic-obstacle-front-support-v151`이다. v145/v150의 추가 장애물 672개 중
자산별 약 절반인 338개만 전방으로 재배치한다. 작은 footprint 303개는 6~10m,
큰 footprint 35개는 8~14m다. 나머지 334개 추가 actor는 기존 정책을 유지한다.
자유 yaw, 19종 자산 할당, primary, 원래 actor 종류·수·seed·거리 계약, 미션과 split은 보존한다.

아래는 **계획된 parent actor의 Ego XY 거리**다. 위의 실제 LiDAR component GT 수와 분모가 다르다.

| 계획 항목 | v150 | v151 |
|---|---:|---:|
| 전체 parent actor | 11,425 | 11,425 |
| 물리 장애물 | 3,944 | 3,944 |
| 추가 장애물 | 672 | 672 |
| 전방 10m 미만 장애물 | 288 | 569 |
| 추가 actor 중 전방 10m 미만 | 67 | 350 |
| 장애물 0~10m | 1,202 | 1,401 |
| 장애물 10~20m | 1,583 | 1,383 |
| 장애물 20~30m | 862 | 863 |
| 장애물 30m 이상 | 297 | 297 |

338장만 재생성했고 4,662개 scenario payload는 byte-identical이다. 5,000장 보존 감사 오류는 0개다.
310장은 원래 generation attempt에서 생성됐고 28장만 추가 geometry retry가 필요했다.
추가 retry에서는 기존 허용 spatial stratum 안에서도 Ego가 크게 바뀔 수 있다(최대 392.8m).
따라서 동일 자산·계약 비교이지 고정 pose의 인과 A/B가 아니다. primary의 실제 위치 고정을 주장하지 않는다.

역할·거리 band·seed·자산 ID·stratum을 spec에 동결하고 구현 SHA도 검사한다.
실제 quality-loop strict replacement 2개에서 새 payload와 동일 spec/actor/split/group,
전방 거리 8.851m/10.197m를 확인했다. capture session과 campaign도 새 helper를 조건부 hash pin한다.
**Flexible donor completion은 아직 미지원**이다. 다른 seed에 옛 plan을 복사하면 안전 거부한다.
이를 지원하거나 명시적으로 제한하고 새 release를 동결하기 전에는 v151을 본수집 준비 완료로 부르지 않는다.
다른 환경은 **v150에서 환경별 새 geometry를 먼저 생성하고, 각 완성된 5K에 v151을 적용**한다.
v151 plan이 들어 있는 source를 fresh 생성기에 직접 넣어 seed를 바꾸는 경로는 초기 단계에서 거부한다.
기존 25K 계획에 v151을 소급 적용하거나 새 25K 완료를 선언하지 않았다.

원거리 quota는 이번 v151에서 변경하지 않았다. v150 고주로 범용 secondary 664개는 모두 50m 미만이다.
향후 기존 0~30m를 보호하면서 30~50m secondary 일부를 far로 보내는 낮은 비율 후보는 가능하지만,
단일 원거리 진단의 성공만으로 혼합 가림·학습 AP까지 검증됐다고 해석하지 않는다.

## 재시작 전 heading 이상과 복구

서로 다른 두 장면에서 요청 yaw와 Ego/CollisionData heading이 약 180° 달라 촬영 전에 중단했다.
Ego 실제 정지와 종료 Park는 확인했으며, 기준을 완화하거나 임의 180° 보정을 넣지 않았다.
기존 5K 전수와 10/13 정상 정적 68장의 heading은 약 +1.347°였고 180° 사례는 없었다.
동일 차량 DataID·지도·MolitComp03 버전명도 대조했다. 버전 업데이트를 원인으로 확정할 근거는 없다.

사용자의 ROS/MORAI 재시작 후 **동일 scenario SHA**의 첫 차량 장면이 정상 통과했다.
수신 heading −86.68512°, 요청 yaw −88.029°로 기존 보정 계약에 복귀했다.
PC 자체는 재부팅하지 않았다. 이 결과는 세션 상태 이상 가능성을 뒷받침하지만 근본 원인 확정은 아니다.
앞으로 같은 실패는 raw CollisionData·Ego trace·요청 pose·각도 오차를 failure JSON에 보존한다.
무수신/방향 불일치는 여전히 전체 안전 중단이며 일반 quota 부족분으로 우회하지 않는다.

## 복합 객체 진단 기준

새 `report_base5_compound_evidence.py`는 다음을 구분한다. 원본 라벨과 승인 gate는 바꾸지 않는다.

- parent UID / native raw class / synthetic component ordinal / 카메라 child ID를 각각 보존한다.
- 같은 parent·class의 여러 성분은 카메라 대응 **미확정**이지 자동 비가시가 아니다.
- full-RPY component OBB 내부 LiDAR 점, sibling 공유 점과 parent union을 따로 센다.
- 카메라 관측 있음·gate 미달·annotation 일부 없음·성분 대응 미확정을 분리한다.
- 동일 class·동일 geometry 중복 후보는 검토 단서일 뿐 자동 병합 규칙이 아니다.
- 유모차의 아기 등은 원래 지면 위에 있다. 모든 component 바닥을 지면에 붙이려 하지 않는다.

기존 5K의 복합 포함 장면 285장, parent 497개, native component 1,134개를 대조했다.
현재 512 upright GT에는 1,055개가 유지되며, LiDAR 0점·카메라 지원 84개는 모두 유지됐다.
같은 class·동일 geometry 중복 후보는 0개였다. 유모차 성인·아기 280개는 camera 대응 미확정이고,
그중 LiDAR 지원 223개·0점 57개다. 57개 중 높이 1m 미만 작은 성분이 53개다.
제외 79개는 미확정+0점 57 / camera gate 미달 13 / annotation 일부 없음 7 / 관측 근거 없음 2다.
빈 annotation 생략 가능성이 있으므로 7개를 손상·수집 실패로 단정하지 않는다.

## 소량 native 실측 결과

총 70장: 원거리 차량 16 + 작은/얇은 장애물 16 + 복합 22 + v150/v151 혼합 비교 각각 8장이다.
모두 기존 native QA에서 `accepted_candidate`이며 충돌·정지·GT 검사 기준을 완화하지 않았다.
이는 표적 소량 결과이며 5K 전체 성공률·모든 배치의 접지·AP를 인증하지 않는다.

| 원거리 조건 | 전방 / 후방 LiDAR 점 |
|---|---:|
| Sedan 85m | 5 / 5 |
| SUV 85m | 2 / 5 |
| Truck 85m | 13 / 12 |
| Sedan 95m | 0 / 4 |
| SUV 95m | 0 / 4 |
| Truck 95m | 4 / 8 |

60m Sedan은 동일/이웃 차선·전후 네 조건 모두 6~7점이었다. 원거리 16개 모두 현재 512 camera gate를
통과했지만 95m 전방 Sedan/SUV는 무가림 단독 장면에서도 0점이다. foreground 78/79px,
BBox 짧은 축 8.49/10.11px라 희소성이 가림만의 문제는 아니다. 트럭만 늘려 점 수를 높이지 않는다.
serialized pivot 대비 runtime 높이 오차는 −4.14~+3.86cm, 캡처 중 변화 최대 3.8µm였다.
16개 모두 독립 ground plane은 미측정이므로 이를 지면 접촉 인증으로 해석하지 않는다.

| 작은 장애물 22m, nominal yaw 0°/90° | LiDAR 점 | 512 foreground px |
|---|---:|---:|
| Lane_Separation_Bar | 1 / 1 | 92 / 43 |
| Bicycle_mountain | 3 / 6 | 122 / 383 |
| Bollard04 | 2 / 2 | 210 / 156 |
| CargoBox | 6 / 6 | 275 / 241 |

8m의 8장은 LiDAR 4~65점이며 ground plane 8/8에서 annotation gap −0.82~+1.52cm였다.
22m는 1~6점이며 plane 7개 미측정, CargoBox 90° 한 개만 측정(+0.36cm)됐다.
두 거리 모두 카메라 gate 16/16이며, yaw는 0~15°/90~105° 범위다. 두 anchor는 약 241m 떨어져 있다.
serialized pivot 대비 runtime 높이 오차 최대 2.99cm, 캡처 중 Z 변화는 0이었다.
측정 불가와 부유 실패를 구분하며, 센서 점 수를 높이려는 무한 재취득은 하지 않는다.

| 혼합 비교의 추가 장애물 8개 | v150 | v151 |
|---|---:|---:|
| 실제 전방 | 1 | 8 |
| 실제 전방 10m 미만 | 0 | 7 |
| LiDAR 20점 이상 | 3 | 5 |
| 점 수 평균 / 중앙값 | 20.25 / 12 | 71 / 42.5 |
| 512 카메라 gate | 8 | 8 |

이번 8쌍은 실제 Ego·기존 actor 위치도 동일했다. 원래 33개 actor·43개 native GT의 유지 GT 42개,
camera gate 33개, 20점 이상 8개는 동일했다. 기존 보행자 성분 하나는 3→2점이었다.
CargoBox 22→10점, Bollard04 26→15점의 하락도 있었고 두 경우 카메라 마스크는 커졌다.
가까워지면 모든 LiDAR 지원이 단조 증가한다거나, 이 결과가 AP 상승을 증명한다고 주장하지 않는다.

복합 22장은 parent 22개·component 62개이며 모두 bbox 내 LiDAR 1점 이상이다.
parent plane은 22/22 측정됐고 gap −5.28~+14.22cm로 심각한 부유는 관측되지 않았다.
다만 유모차 작은 Pedestrian leaf 18개는 전부 몸체 bbox와 점을 공유해 독립점이 0개다.
기존 5K의 작은 leaf 140개 중 LiDAR 양성 87개도 같은 구조였다. 성인·몸체·작은 leaf의
독립점 중앙값은 새 진단에서 각각 38.5/12.5/0점이다. 이를 독립적인 아기 관측으로 인증하지 않는다.
Ped_Pet1의 새 두 parent는 공유점 0이고 Bike3도 독립점이 있어 복합 전체 공통 오류로 일반화하지 않는다.
정상적인 중첩 GT를 삭제할 수 있으므로 component 삭제·병합·독립점 필수 gate는 적용하지 않는다.

## VS Code 종료 뒤 복구

마지막 v151 비교 8장 중 4장 저장 후 프로세스가 종료됐다. 원래 세션의 종료 Park 기록은 **없음**으로
남겼다. 새 시점의 정지·Park는 `interruption_park_recovery.json`에 별도 확인했으며 과거 종료를 인증하지 않는다.
저장된 4장은 보존하고 남은 4장만 새 output에서 촬영했다. 중단 세션의 ledger/status를 완료로 고치거나,
사후 Park 파일을 옛 `shutdown_safety.json`으로 복사하지 않았다. 이 자료는 진단용이지 학습 quota가 아니다.
강제 종료 때 남은 해당 session 소유 scenario/network 두 파일은 SHA 대조 뒤 실험 root로 옮겨 보존했다.
프로세스 부재·노드 응답 거부를 확인한 중단 수집기 한 개의 stale ROS 등록만 정리했다. 살아 있는 노드는 중단하지 않았다.
실험 당시 `run_base5_capture.py`와 safety helper도 session hash에 일치하는 사본을 별도 보존했다.
이후 실패 근거 파일명을 고유하게 만들어 같은 scene의 재실패가 과거 evidence와 ledger SHA를 덮어쓰지 않도록 했다.

## 검증과 남은 조건

Base5 전체 CPU 회귀 **702개 통과**, `git diff --check` 통과다. 대시보드 테스트의 localhost 임시 서버 외에
회귀 검사는 실제 MORAI를 제어하지 않는다. 실제 raw 보고서는 scenario SHA·source metadata 조건을 대조했고
그 보강 전후 원거리/근거리 수치도 동일했다. 기존 비교 산출물은 완료 후 수정하지 않는 immutable 입력으로 취급한다.

다음은 v151의 유연 보충 정책 지원 여부를 확정하고, 필요한 범위만 검증한 뒤 새 source로 release를 동결하는 것이다.
다른 환경에는 v150 새 geometry → 해당 환경 전체에 v151 적용 → UI 확인·소량 검증 순서를 사용한다.
원거리 차량 추가 재배분이나 아기 component GT 정책 변경은 이번 결과만으로 자동 적용하지 않는다.
새 모델을 학습하지 않았으므로 최종 AP·tracker bootstrap 향상은 독립 평가/학습 비교가 필요하다.

## 산출물과 재현

실험 root는 `artifacts/evaluation/base5_generator_261015`이며 Git 추적 대상이 아니다.

| 산출물 | 용도 |
|---|---|
| `candidate_v151/manifest.json`, `front_support_cpu_audit.json` | 새 5K offline 계획과 보존 감사 |
| `strict_replacement_v151_v1/report.json` | strict replacement CPU 경로 2개 |
| `heading_history_offline.json` | 기존 heading 전수 대조 |
| `far_support_v2.json` | 기존 5K 원거리 지원·val 분모 감사 |
| `far_deck`, `near_deck`, `compound_deck` | 16/16/22개 독립 진단 입력 |
| `compound_native5k_evidence.json` | 기존 복합 native 전수 근거 |
| `far_native16_report_v3.json`, `near_native16_report_v2.json` | 실제 scenario SHA·원본 metadata 대조를 추가한 실측 |
| `mixed_front_support_compare_v1.json` | 추가 actor와 기존 actor 지원을 분리한 8쌍 비교 |
| `compound_live22_evidence.json`, `compound_comparison_5k_vs_live22.json` | 복합 성분별 공유/독립 근거와 5K 대조 |

진단 준비기는 원본과 다른 새 출력만 허용하며 simulator를 조작하지 않는다.
실제 촬영은 master 단독 소유권·기존 capture runner·Park/정지/충돌/native GT 검사를 사용한다.
완료 원본·과거 release·모델 기본값·LiDAR 50·학습 compile OFF 설정은 이번 실험으로 변경하지 않는다.
