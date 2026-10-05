# Base5 자산별 정지/이동 외관 비교 — EVIDENCE

2026-10-06, MR / ROS / SaveSensorData 동기 캡처. 속도 회귀나 temporal 학습 실험이
아니라 **single-frame 학습에서 이동 자세를 별도로 수집할 필요가 있는지** 확인한 기록이다.
최신 진입점은 `BASE5_V2_CURRENT.md`다. Production E25·threshold·tracker·planning은 변경하지 않았다.

## 실험 및 최종 선택

- 대표13자산 × 정지/이동 × 2반복 = 최종52조건. 동일 자산·Ego·목표 XY·yaw로 비교했다.
  사람/동물/복합 객체는 기존 Woman2 진단의 목표 위치, NPC 차량은 샘플 고주로 legal route를 사용했다.
- 자연 속도를 유지하고 뒤쪽에서 출발했다. 사람/유모차/rigid object 1.1m/s,
  동물1.2m/s, Bike/Scooter2.5m/s, 실제 NPC 차량5m/s를 요청했다.
  실제 도착오차≤0.4m를 확인한 다음 캡처했다. 최대 관측 XY오차0.399m.
- 초기52시도에서50장 저장/2도착실패 차단. CargoBox 실패조건과 NPC 차량8조건을
  새 deck에서 재검증해9장 저장했다. 이후 object 계열6자산의 높이를 맞춰12조건을
  추가 촬영했다. 높이 재촬영 중1도착실패는 완료 resume으로 재시도했다.
- 합계 **74시도/71 raw 저장/3저장 전 도착실패 차단**이다. 최종52조건/80 component GT를
  선택했고,19개 이전 raw와3개 실패 이력은 삭제하지 않았다. 보고서의22개
  `superseded_attempts`에 근거를 보존했다. 최종52조건 모두 실제 정지/이동·도착·
  가시성 audit·CollisionData 비충돌을 통과했다. 본 학습 dataset release는 아니다.
- 완료 후 brake를 유지하며 Park 확인. 최종 EventCmd 첫timeout/두 번째 성공이었다.
  간헐 서비스 지연을 해결했다고 주장하지 않는다.

## 비교를 왜곡하던 높이 차이와 제거

처음에는 같은 scenario z를 요청해도 object 계열의 정지와 waypoint 이동에서
실제 높이가 달랐다. BabyStroller2/3, Bike3, Scooter1, CargoBox는 약0.65m,
OBJ_Hyundai_Grandeur는 약0.31m 차이가 있었다. **초기 RGB 배경/높이 차이를
동작 형상 변화로 해석하지 않았다.**

정지 표본의 실제 Object z와 이동 표본의 실제 z 차이를 측정해 진단 waypoint z만
이동시켰다. 다시 촬영한12장은 정지 표본과의 부모 z오차가 최대약0.000002m였다.
raw LiDAR Instance annotation·point cloud·RGB를 수정하거나 GT z를 사후 조작하지 않았다.
이 보정은 **동일 높이 외관 비교용**이며 일반 road-ground 보정표나 production 좌표 변환이 아니다.

별도로 초기 진단 NPC의 static spawn z에 helper의 +0.36m를 중복 적용한 부분을
preparer에서 수정하고 차량8조건을 새로 찍었다. 새 pair의 실제 부모 z 차이는
약0.3mm 이하였으며 원래 입력/촬영 이력을 유지했다.

## 관측 결과와 권장 수집 방식

아래 size 차이는 각 component의 정지/이동2표본 중앙값 간 최대 축 차이다.
**size만으로 mesh가 같거나 다름을 인증하지 않는다.** 원본 RGB crop을 함께 확인했다.

| 자산 | 관측 | Base5 외관 수집 권장 |
| --- | --- | --- |
| Dog1 / Dog2 | 다리/몸 자세 변화 관측, size 차이약0.012/0.022m | 정지 + 이동 phase 유지 |
| Deer1 | 다리 자세 변화, 종방향 size 약1.741→1.875m | 정지 + 이동 phase 유지 |
| Ped_Pet1 | 사람의 벌어진 다리 자세. 사람 종방향 약0.339→0.720m, 동물도 별도 component | 정지 + 이동, component별 GT 보존 |
| BabyStroller2 / BabyStroller3 | 높이 맞춘 RGB에서 뚜렷한 관절 자세 변화 미관측. 부모/유모차/아기 component size 동일 | 현재 template는 정지 우선 |
| Bike3 / Scooter1 | 실제 translation은 있으나 현재 template의 탑승 자세와 component size는 동일 | 현재 template는 정지 우선 |
| NCAP_EPTa | 고정 실루엣, size 동일 | 정지 우선 |
| CargoBox / OBJ_Hyundai_Grandeur | 높이 맞춘 뒤 rigid 형상 차이 미관측, size 동일 | 정지 우선 |
| 2016_Hyundai_Ioniq | 18km/h 직진 pair에서 큰 외관 차이 미관측, size 차이약0.000193m | 차량은 정지 배치 우선 |
| 2014_Hyundai_Xcient(Mixer) | 18km/h 직진 pair에서 큰 외관 차이 미관측, size 중앙값 동일 | 차량은 정지 배치 우선 |

이전 v26의 Woman2는 이동 다리/팔 자세와 큰 OBB 변화를 확인했으므로
ordinary pedestrian의 정지+이동 수집 근거도 유지한다.

### 해석의 제한

- 각 mode2표본, 한 방향·한 template이다. 모든 유모차/자전거/스쿠터 모델에
  그대로 일반화하지 않는다. 속도 설정이 explicit animation Walk/Run 선택을
  인증한 것도 아니다. 미검증 자산을 자동 정지 전용으로 바꾸지 않는다.
- 정지 우선은 **외관 학습용 캡처 방식**이다. 실주행에서 동적 객체로 취급하지 않는다거나
  tracker 속도를0으로 강제한다는 뜻이 아니다. 임의의 정지/이동 라벨 재지정도 하지 않는다.
- 실제 저장은 캡처 요청 뒤 진행되므로 이동 pair가 정확히 동일한 world XY에서
  저장된 것은 아니다. 실제 Object 전후 위치/속도, GT size/rotation을 보존했다.
  RGB 확대는 nearest 방식으로 원본 픽셀을 보여줄 뿐 새로운 센서 해상도가 아니다.
- LiDAR XZ 그림은 component OBB 내부의 희소 관측점을 box-local로 정렬한 것이다.
  animation mesh 복원, point-to-point 대응, 형상의 완전 동일성을 의미하지 않는다.
- 관측속도는 ROS 수신시간 위치차분이다. 공식 메시지 속도GT/모델 정확도 지표가 아니다.
- 일부 자산은 기존 geometry reference가 없어 배치 전 검사는 보수적인5×3×3m envelope를
  사용했다. 이는 안전 proxy일 뿐 학습GT가 아니다. 실제 component GT는 raw annotation이다.

## 재현 및 원본

공통 root: `/home/luv/morai_cap/base5_v2_261006/`.

- 초기 manifest/raw: `asset_motion_probe_v27d/`, `asset_motion_capture_v27/`.
- 차량 초기 높이 수정·준비시간8초 pair: `asset_motion_probe_v27f/`, `asset_motion_capture_v27_corrected/`.
- object 실제 높이 matched: `asset_motion_probe_v27_height/`, `asset_motion_capture_v27_height/`.
- **최종 비교:** `asset_motion_report_v27_canonical/report.json`.
  각 자산의 `*_comparison.png`와 `*_lidar_xz.png`를 함께 저장했다.
- `asset_motion_report_v27_selected`는 높이 차이를 발견한 중간 보고서다.
  이를 최종 외관 pair 근거로 사용하지 않는다. 이전 partial/failed preparation 폴더도 본수집용이 아니다.

코드: `prepare_base5_asset_motion_probe.py`, `report_base5_asset_motion_probe.py`.
후자는 입력 run 순서로 같은ID를 명시적으로 교체하고, 원래 시도·source root·scenario hash를
함께 보존한다. 최종 Base5 단위/안전 테스트59개·Python compile·diff whitespace 검사를 통과했다.
component 분리 집계·미관측 정지/이동 거부·높이 calibration 누락 거부 검증을 포함한다.

## 다음 작업

일반 생성기에 자산별 외관 정책을 반영하되 미검증 자산은 기존 요청을 유지하며 검토한다.
관절 자세가 바뀌는 자산은 자연 속도·뒤쪽 출발·실제 목표도착 gate를 적용하고,
rigid/fixed-pose 자산은 정지 배치로 목표 거리/각도/근접 분포를 직접 확보한다.
미션에서 이동 상대위치가 필요한 경우와 외관 sampling을 분리한다.
근접cell·다중객체 collision/visibility 검사와 quota audit를 다시 한 뒤 새10K를 생성한다.
**이번에는 일반10K generator default 변경·10K 생성·본 수집·GPU latency 측정·학습은 하지 않았다.**
