# Base5 MR 복구 객체 rosbag 보강

상태: EVIDENCE / 학습 PREPARED. 2026-10-04, dataset/base5. Production E25 변경 없음.

## 수집·변환

- MR minimal5 hybrid, ROS Ego/Object + UDP camera4/LiDAR. SaveSensorData는 앞선 geometric reference 측정에만 사용했다.
- CargoBox/WoodBox/police_f_1/Boy2/Scooter_01 각각 거리8/12m × yaw0/90도, Ego/객체 정지, scale1. 20/20 scene, 60 frame, 실패0. 원본20 bag 약780MiB.
- 원본 audit PASS: capture 내 camera nearest age 최대96.11ms, 100ms 초과0. capture 밖 Ego/Object stamp reversal2건씩은 warning이다. 선택60 frame의 camera age 최대31.093ms.
- MR heading -1.347도 한 번, pose interpolation, 측정 reference `configs/morai_asset_bbox_offsets_mr_restored_261004.json`, spatial7D, raw cloud/deskew OFF, ±30m, unknown asset0.
- 기본5-point 적용 시 CargoBox6/WoodBox0/police12/Boy2 12/Scooter12 GT 유지. WoodBox는 catalog 미식별이 아니라 sparse point gate에서 탈락했다.
- WoodBox front.jpg 12장을 직접 확인했다. 모두 보이지만 nearfield review 계약은10m 이내이므로8m scene b5rt05/06의6장만 예외 등록했다. `configs/base5_restored_sparse_camera_review_261004.json`은 sample+UID whitelist이며 asset 전체 예외가 아니다. 12m WoodBox는 원본을 보존하고 제외 상태를 유지했다.
- reviewed 변환 GT: CargoBox6/WoodBox6/police12/Boy2 12/Scooter12, 총48. sparse 제외12개. 전체60 frame validator PASS.

## 학습 결합

- Canonical: `artifacts/datasets/base5_nearfield_restored_reviewed_replay_pilot_261004`.
- Base3 train600/val100 + 이전 diversity·reviewed smoke + 이번 reviewed addon. 빈 addon frame은 학습 결합에서 제외한다.
- train727/val124/test0, 851 frame, 696 scene. GT obstacle1017/vehicle557/pedestrian452. 구조/파일/scene split/sparse whitelist validator PASS.
- GPU 없이 팀 Docker의 실제 train pipeline으로727/727 sample 로드 PASS. 무작위 augmentation 뒤 GT1706개, 빈 GT98 frame이었다. 이 수치는 detector 검출률이 아니며 augmentation에 따라 달라진다.
- `base5_nearfield_restored_replay_pilot_261004`는 이번 WoodBox review 이전845-frame 중간본이다. 실행 기준은 위 reviewed 경로다. 이전688/115 deck도 이력으로 보존한다.
- 이번 val은 인접한 정지 layout을 scene별로 분리한 learnability 확인용이지 일반화 test가 아니다. 동적 highway·초근접 회피·옆차로 통과 보강을 완료한 데이터라고 해석하지 않는다.
- CargoBox 이번 MR capture size는 역사적 reference와 다르다. 새 addon에는 이번 MR 측정값을, 과거 source에는 당시 reference를 보존하며 replay manifest에 출처를 기록한다. 크기 차이 원인/통일은 미해결이며 짧은 pilot으로 production 변경을 정당화하지 않는다.

## 실행

MR 종료 후 실행한다. GPU 학습은 아직 미실행이며 아래 recipe의 dry-run은 PASS.

```bash
ajou
export BASE5_REPLAY_ROOT="$AJOUNICE_ROOT/artifacts/datasets/base5_nearfield_restored_reviewed_replay_pilot_261004"
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base5_nearfield_replay_e2_pilot.yaml
```

E25-init E2, LR2e-5, 5060Ti에서 physical batch4/accum1, warmup25 update, 매 epoch validation/checkpoint, W&B online. Production weight를 덮어쓰지 않는다.

## 원본·재현

- generator: `scripts/generate_base5_restored_replay.py`; calibration deck의 검증된 정지 위치를 재사용하되 b5rt prefix와 독립 manifest를 사용한다.
- `artifacts/datasets/base5_mr_restored_replay_261004/scenarios/manifest.json`, `runtime.json`, `parsed_reviewed/`, `raw_audit.json`.
- `rosbags/datasets/base5/261004_mr_restored_replay/`.
- `/home/luv/morai_cap/261004_base5_mr_restored_replay/summary.json`.
- recorder는 마지막 snapshot이 닫힌 뒤 이번 실행 PID만 종료했고 사용자 MR/ROS 프로세스는 유지했다.
