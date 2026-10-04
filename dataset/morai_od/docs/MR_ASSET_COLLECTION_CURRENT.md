# MR 전용 객체 카탈로그·수집 계약

상태: CURRENT. 2026-10-04. 향후 수집은 MR만 사용한다.

## 현재 기준

- 원본: `configs/morai_asset_catalog_260807.json` (대회 MR에서 저장한 목록).
- Eligibility: `configs/morai_asset_mr_collection_261004.json`.
- Template directory: `$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025`.
- `generate_dynamic_object_capture_set.py`의 catalog/compatibility/runtime directory 기본값을 모두 MR 기준으로 변경했다. MR의 vehicle_list.json, pedestrian_list.json, obstacle_list_without_vehicle_and_ped_like_objects.json을 읽는다.
- MRP 교집합·direct-load quarantine을 MR 수집에 적용하지 않는다. Grandeur, police_f_1, Boy2, Scooter_01 포함. 이전 MR minimal-hybrid에서 네 종 모두 두 위치 2/2 load 성공했다. 이번 변경은 offline pool 연결 검증이며 live 재수집은 아니다.
- WoodBox 포함. 정합 우려는 audit 대상으로 남기되 asset blacklist로 사용하지 않는다. Raw point를 임의 이동시키지 않는다.
- MR `aiPedstrianList`도 읽는다. NCAP dummy를 MRP 문제 때문에 제외하던 subtype gate는 MR에 적용하지 않는다. 신규 MR dummy의 실제 capture 품질은 이후 검증한다.

## 개수와 유지하는 필터

MR 원본은 Vehicle43/Pedestrian container33/Obstacle container33, 총109 entry이다. Speed_Bump와 Tree scope 제외 후 eligible ID는 43/33/31, 총107이다.

Runtime loader는 ontology/generation_policy도 적용한다. 실제 MR template에서 Vehicle43/Pedestrian28/Obstacle 계열31개를 읽었다. Obstacle 계열에는 조건부 배치·배경 역할도 있어 최종 positive-class 분포와 같지는 않다. Animal/Cyclist/복합 객체를 임의로 Pedestrian으로 재분류하지 않는다.

- Speed_Bump·Tree 및 기존 사용자 ontology/scope 제외 유지.
- scale1, 시간 정합·ego drift·가시성/GT point gate 유지. Asset 복구와 quality gate 완화는 별개이다.
- MRP catalog/compatibility는 과거 재현용으로 보존한다. 명시적으로 선택하면 기존 personal profile을 사용할 수 있다.

## 카탈로그 포함 ≠ GT 보정 완료

WoodBox, police_f_1, Boy2, Scooter_01은 MR 20-scene/40-capture 검증으로 measured reference를 확보했다. 별도 `configs/morai_asset_bbox_offsets_mr_restored_261004.json`에 출처와 잔차를 기록했고 MR catalog에도 reference를 연결했다. 기존75-asset production catalog는 변경하지 않았다. CargoBox control은 이번 MR과 역사적 reference의 size가 달라 두 버전을 분리했다. 상세 결과는 [보정 런북](261004_mr_restored_asset_gt_calibration_runbook.md)을 따른다.

**복구 네 종은 reference를 확보했지만 학습 데이터 편입은 아직 미실행이다.** 새 rosbag 수집·변환에서 명시적으로 reference를 적용하고 검증해야 한다. Converter unknown_assets를 확인하고 미식별 객체를 unlabeled background로 둔 채 학습하지 않는다. 신규 dummy/조건부 자산도 reference coverage를 확인한다.

이를 코드로 보장하기 위해 info converter의 `--unknown-asset-policy` 기본값은 `error`다. Missing calibration을 발견하면 PKL 확정 전에 중단한다. 과거의 누락 재현은 `--unknown-asset-policy drop`을 명시해야 하며 production 학습용으로 쓰지 않는다.

## Base5와 과거 결과

새 smoke generator는 WoodBox 포함27 scene, diversity56 scene이다. Pass smoke는 CargoBox/Barricade 두 종을 이름으로 선택한다. 기존 manifest overwrite를 거부하므로 새 출력 directory를 사용한다.

기존24-scene smoke,48-scene diversity,688/115 replay pilot은 변경 전 결과이며 새 객체가 자동 추가되지 않는다. 원본 bag/manifest/PKL와 production E25를 변경하지 않았다.

## 검증

- MR profile unit tests2개 PASS: MR-only fixture, AI pedestrian list, 복구 대상, scope 제외, 명시적 historical MRP profile.
- 기존 dynamic capture tests52개 PASS.
- Base5 tests6개 PASS: WoodBox 생성, 초기속도0, scale1, highway55m margin.
- 후속 MR live 보정20/20 scene, 40/40 capture PASS. 학습은 미실행이다. 동기 보정 이후 rosbag 수집에는 UDP sensor network 재연결이 필요하다.
