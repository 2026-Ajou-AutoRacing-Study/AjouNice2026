# Base5 정지 객체 yaw·랜덤화 재검토 — CURRENT / EVIDENCE

2026-10-08 17:21 KST 검증 스냅샷. Sunny13 수집을 사용자가 중단한 뒤의 재설계다.
현재 입력은 `~/morai_cap/base5_v2_261008/yaw_5k_v129_release/`다.
Base5-only 5K → 같은 설계의 6환경30K, single-frame/4class/7D/velocity off를 유지한다.
이 문서는 새로운 본5K 취득·학습 성능을 주장하지 않는다. 본수집은 다시 승인 후 시작한다.

## 기존 수집 보존

- v123의 기존 output은 `regions_5k_v123_release/collection_5k_regions_v123_sunny13`에 그대로 보존했다.
  상태는 사용자 중단 `halted`, finished780/승인763/review17, active5908.416초다.
  중단 기록은 없애거나 안전한 PAUSE로 위조하지 않는다. 새 runtime으로 이 output을 강제 resume하지 않는다.
- 별도 압축 백업:
  `~/morai_cap/base5_v2_261008/yaw_v127_backup/collection_v123_interrupted_reference.tar.zst` (약3.3GiB).
  SHA256 `b91ade4fca57307df3e344fd344074d9e1079d82287ff40f9d5dceb0b2e5d425`.
  압축 완료 후 `tar -df`로 원본과 전수 대조하여 차이0을 확인했다. 원본은 삭제하지 않았다.
- 기존5K manifest/시나리오/동결spec 및 v123 계획 노출 표는 덮어쓰지 않았다.
  기존 v125 재현 코드/지도/자산/reference 백업은 `prebulk_dashboard_v126/`에 유지한다.
  v127/v128 개발 raw·실패·대조 실험도 각 별도 release에 보존한다.
- 새 입력의 scripts/tests/센서·네트워크 config도 `yaw_5k_v129_release/runtime_v129_source_reference.tar.zst`에
  보존하고 원본과 `tar -df`로 전수 대조했다.
  SHA256 `eb9da6ddfd35eec29cd587047c897e31220e5f2d887b382890e4444dc92e1621`.

## 발견한 문제와 새 yaw 정책

실제 시나리오 rot를 읽어 검사했다. metadata에 placement_yaw가 없는 과거 미션도 0°로 간주하지 않았다.

- 범용 pedestrianList/aiPedstrianList 정지 배치 1,914parent는 도로 기준 ±90° 두 방향뿐이었다.
  주 객체760+보조1,154이며, 임의 객체 bearing/Ego yaw가 있다고 해서 객체 자체 yaw 다양성이 충분한 것은 아니다.
- 범용 objectList는 이미 360° 자유 각도이며 barricade/barrier에는 일부 도로 정렬 bias도 있었다.
  모든 비차량의 yaw가 빠진 문제는 아니었다.
- 정적 회피 미션의 사람·동물·복합 객체는 주로 0/±90°±8°, 음영 정지는 네 축±12°였다.
  걷기/달리기는 실제 경로 방향이므로 무작위 방향으로 돌려서는 안 된다.

새 opt-in `static-appearance-yaw-v128`:

| 적용 대상 | 새 정책 |
|---|---|
| 범용 정지 pedestrianList/aiPedstrianList | 70%: 도로0/90/180/−90°에 ±15°; 30%: −180~180° 자유 각도 |
| 정적 회피의 사람·더미·동물·반려동물 동반·유모차·탑승자 | 같은70/30정책. objectList인 복합 객체도 자산 pool로 포함 |
| 범용 objectList 실체 장애물/복합 | 기존 자유 각도·barrier bias 유지 |
| 일반 차량·고주로·로터리 NPC | 실제 link 접선/샘플 경로 유지. 임의 객체 XY/yaw jitter 없음 |
| 실제 걷기·달리기·음영 이동 | 경로 yaw 유지. 위 정책 적용 안 함 |
| 정지 횡단 관측·음영 정지 | 기존 횡단축/네 축±jitter 유지 |

- primary의70/30 mode는 scope/primary-pool별 deck에 먼저 배정한다. 실제 actor 전체의 비율은
  보조 객체 선택 및 배치 재시도로 정확히70/30과 같지 않을 수 있다.
- yaw는 별도 seeded RNG로 뽑아 XY/객체 선택 RNG를 추가로 소비하지 않는다.
  **재시도 attempt가 바뀌면 각도도 다시 뽑되 원래 cardinal/uniform mode는 유지한다.**
  같은 seed/attempt는 결정적이다. 완료 후 보충에서도 frozen spec의 정책이 유지된다.
- 회피 primary는 yaw를 먼저 선택한 뒤 회전된 BBox와 center offset으로 Ego 통과 간격을 푼다.
  마지막에 박스만 회전시켜 근접 간격을 망치지 않는다. 최종 XY의 지면 높이도 기존 방식으로 재계산한다.
- scale1, native sensor annotation, Ego Park/실제 정지/CollisionData/가시성/Z gate를 완화하지 않았다.

## 초근접 사람 배치 대조와 제한적 보완

- v127 대표31:29승인/2부족. 재시도까지 yaw를 고정한 버전이며 완료 후2회 보충에서도 두 셀이 남았다.
  v127 완료resume에서1,250sensor파일 내용/inode/mtime 불변·추가capture0을 확인했다.
- v128 대표31:30승인/1부족. Man1 셀9는 회전각도 재표본화가 있는 같은-quota 재배치로 통과했다.
  Man2 셀85의 계획 경계 간격 약0.30m는 반복해서 runtime actor/native annotation이 없었다.
- 이전 v123 수집에서도 셀9는 원안 실패 후 재배치 승인, 셀85는 부족분이었다.
  정확한 두 셀의 새 legacy 대조에서도1승인/1부족이었다. 따라서 새 yaw만의 결함이라고 단정하지 않는다.
  별도 추가 legacy 두 셀 대조는1승인/1음영 moving-ground 부족으로, 위 두 셀 대조와 혼동하지 않는다.
- Man2 간격 probe:0.30/0.45/0.60/0.80m를 사전 생성해 모두 캡처했다.
  0.30·0.45는 원안/보완 모두 미출현,0.60·0.80은 원안에서 정상 캡처다.
  0.60은 도로 지지 때문에 Ego/객체 공동 평행이동도 발생했다.
  0.80은 객체 XYZ/yaw가0.30·0.45와 같고 Ego 간격만 달랐다.
  **간격 보완이 유효한 근거이지 내부 collider 반경이나 정확한 spawn 임계값을 확인한 것은 아니다.**
- v129는 일반 사람 primary의 정적 미션 alongside 중33셀만 요청 간격0.6–0.8m로 바꾼다.
  원래 bin은 `character_spawn_gap_original_bounds_m`에 기록하고 새 spec을 동결한다.
  사람 근접 phase/좌우/종류/장면 수를 삭제하지 않는다. 차량·실체 장애물·동물·더미·복합의 좁은 통과는 유지한다.
  한 Man2 대조에서 출발한 **보수적 사람-only 생성 prior**이며 모든 사람의 물리 안전 인증은 아니다.
- offline audit는 원래 bin 총량 보존과 명시적인 사람-only override, 변경 후 frozen bin을 함께 검사한다.
  단순히 quota 검사를 끄거나 기존 bin으로 숫자를 위장하지 않는다.

## 범용·미션 전면 검토 결과

| 항목 | 판단 / 적용 |
|---|---|
| Ego 위치 | 도로 anchor/bin 및 region 제한 유지. 맵 XY 전체 균등 샘플로 바꾸지 않음 |
| Ego yaw/횡 offset | 기존±3/10/15° deck, 횡0/±0.15/±0.30m 및 road-support 유지. 회피/로터리는 실제 상황 접선·제한 사용 |
| 객체 개수/혼합 | 범용 positive2420셀의30/50/20 single/regular/dense·크기 budget·최대8 유지. 미션에는 억지 dense를 넣지 않음 |
| 거리/방향 | ±100×±30m LiDAR-frame ROI와 기존 패밀리별 거리/전후좌우 deck 유지. 먼 거리 quota를 yaw 보강에 쓰지 않음 |
| 근접 | 회전 footprint·경계 간격·좌우 통과 quota 유지. 사람33셀만 현실적인 spawn 여유로 보완 |
| 고주로/로터리 | 일반 차량만 배치하는 생성 prior 유지. 범용 고주로는 despawn 이후도 가능, 미션 NPC는 샘플 경로 범위 유지 |
| 음영 | 독립500셀, 밖에서 안을 보는 장면·중앙부 집중·오른쪽80/왼쪽20·좌우 엇갈림 유지 |
| 이동 외형 | 사거리154+음영24. 양방향/걷기·달리기/두 waypoint/저장 높이 lookup 유지. 정적 rigid를 움직이지 않음 |
| 높이/scale | Z 랜덤화를 추가하지 않음. MGeo+자산 pivot·최종 footprint 높이 및 심각한 높이 실패 gate 유지. scale1 |
| 객체 roster | 122종 primary 최소5 유지. 열린G80 공식제외/Fire 등 기존 GT 부적합 제외 유지. 희귀 객체를 일반 사람으로 대체하지 않음 |
| GT | native LiDAR Instance7D, lidar1 OR 실제 camera visibility, 복합 component 보존. 카탈로그 추정 박스로 raw GT를 덮어쓰지 않음 |
| 날씨/시간 | 첫Sunny13. 나머지sunny11/15·foggy11/13/15는 UI 수동 확인·별도output. 임의 환경 혼합 없음 |
| 평가/학습 | Base5-only 새 학습.6환경 같은group/split 유지; production E25/threshold/tracker/planning은 미변경 |

추가 무작위화를 최대화하지 않았다. 주행 NPC가 차선 밖을 향하거나, Z/scale·미션 맥락을 깨는 데이터보다
기존 분포를 유지하고 확인된 정지 외관 방향의 빈 곳을 메우는 편을 택했다.
원거리의 실제 센서 근거, 부분 가림, 옆/뒤 객체, 빈 배경과 희귀 primary는 계속 필요하다.
새 데이터가 일반화를 개선하는지는 이후 held-out mission rosbag으로 판단한다.

## v129 동결 입력 / 구성

- 기본 manifest SHA256: `539792e0189b03a4535e56a27d842ab946b83417b11e54cfaf465e6d6941cf56`.
- Sunny13 본수집 manifest SHA256: `33ec080b02ca92b047208a26314ccc39b822d59eacf49743075e0455553630fa`.
- 1,082셀 재생성/3,918payload 불변. 고주로1340/로터리350/음영500/배경230 및 이동178 payload 불변.
- 5K 구성: 고주로740범용+600미션, 로터리350미션, 기타차량80+50, 장애물770+600,
  사람·동물·복합830+250, 음영500, 빈배경230. primary 자산별 횟수는 v123과 완전히 같다.
- whole-link split 규칙 유지. 재생성 retry 때문에5개셀 split이 바뀔 수 있으나 새6환경에서는 모두 같은 split이다.
  구raw와 새raw를 자동 혼합하지 않는다. 인접 link 관측 누수 위험은 기존처럼 별도held-out bag으로 보완한다.
- 생성기 준비 단계에 `prepare_base5_batch_order`를 연결했다. **모든6환경은 정적4822→이동178 순서**다.
  순서 없는 개발 manifest는 본수집으로 안내하지 않는다.

### 계획 부모 객체 노출 — v129 스냅샷

2026-10-08 v129 및 위manifest에만 적용한다. v123의10079표는 과거 스냅샷으로 보존한다.
재배치로 소수 secondary가 달라졌으며 실제 visible GT/component 수는 완료 ledger로 별도 계산한다.

| 객체군 | 5K | 같은6환경30K |
|---|---:|---:|
| 일반 차량 |4112|24672|
| 실체 장애물 |2975|17850|
| 일반 사람 |2165|12990|
| 인체 더미 |148|888|
| 동물 |143|858|
| 반려동물 동반 사람 |142|852|
| 유모차 |131|786|
| 자전거·스쿠터 등 탑승자 |148|888|
| 차량형 Obstacle |117|702|
| 합계 |10081|60486|

## 검증 / 운영 경계

v129 대표35는 기본31strata+서로 다른 사람4명의 새 간격strata를 결과 보기 전에 등록했다.

- 전수5K `OFFLINE_PASS`, 오류0/고유payload5000. 기본 입력과 Sunny13 순서 입력을 각각 검사한다.
  `validation_report.json`은 보호된 payload·yaw 직렬화·6환경 동일group/split/hash·정적우선순서를 검증한다.
- 실제 MR 대표35/35 승인, 정적33/이동2, 부족0. 초기 누락1셀은 같은 quota 재배치로 승인됐다.
  초기 실패도 보존하며, 이35개는 전체5K의 성공률 인증이 아니다.
  승인 raw의 실제 정지/CollisionData 무접촉을 확인했고 심각한 Z/contact review는 없었다.
- 배치 parent78 중 가시 positive parent75/component91. CPU 변환은35프레임/91GT:
  pedestrian46/vehicle10/animal10/obstacle25. 가시성 정책에서 제외한 animal3component도 raw에 보존한다.
  train26/val9이며 이 작은 pilot으로 모델 성능을 평가하지 않는다.
- 실측 활성236.058초. 정적33셀 평균6.665초/이동2셀8.051초의 가중 외삽은5K9.326시간이다.
  이전 실제763승인/5908.416초의 장당 비용으로는5K10.75시간이다.
  **운영 예산10–13시간/5K, 같은6환경60–78시간**을 유지하고 첫100–200승인의 누적rate로 갱신한다.
  이동2개만으로 음영 이동24개의 전체 비용을 인증하지 않으며 yaw 변경만의 속도 A/B도 아니다.
- raw/QA/rejected 포함 저장 외삽5K약26.81GiB/30K약160.86GiB.
  hardlink는 한 번 집계하며 변환·압축 복제·예전 백업·다른 프로세스 저장은 별도다.
  dashboard의 실제 누적 추정을 우선한다.

- 동일35셀/chunk4 명령의 완료resume는 추가capture0이다. sensor raw886파일의 내용SHA256/
  inode/mtime/크기가 모두 불변이고 선택ledger SHA/승인35/누적236.058초도 같았다.
  검증 근거는 `pilot_raw_snapshot.json`/`resume_verification.json`이다.
- 기존 모든Base5 단위검사 포함 **403tests PASS (78.851초)**. retry yaw/mode, yaw-before-gap,
  대상scope, person-only gap override 거부, 추가4명 pilot·원본불변,6환경prefix,
  dashboard zero-approved/용량외삽/hardlink/status불변을 확인했다. `git diff --check`도 통과했다.
- 종료Park는9원안batch+1재배치session의10/10 `park_verified=true`다.
  collector는 종료했으며 master/guarded bridge는 그대로 유지했다.
- 8075를 새본수집output으로 전환했다. HTTP200/`not_started`/분모5000/정적4822·이동178/
  6환경분모30000·승인0을 확인했고 dashboard가 본수집폴더를 생성하지 않았다.
  디스크여유237.93GiB다. 기존763승인을 새수집진행률에 합산하지 않는다.

초기 실패·원본을 보존하고 실패한 장면을 빈 배경이나 성공 GT로 바꾸지 않는다.

런북은 [현재 실행 명령](261007_base5_5k_capture_runbook.md)이다.
새 본수집 경로는 `yaw_5k_v129_release/collection_5k_yaw_v129_sunny13`이며 본5K는 아직 실행하지 않았다.
일반 resume/완료후 replenish/누적 상한/안전 halt/모든 실패 근거 보존은 유지한다.
capture runtime 변경 이후 구v123/v127 output에 새 코드를 섞지 않는다.
