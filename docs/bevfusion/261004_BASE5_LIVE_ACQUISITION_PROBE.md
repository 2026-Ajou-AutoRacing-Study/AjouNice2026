# Base5 live acquisition probe — 2026-10-04

상태: EVIDENCE. Branch `dataset/base5`. Production E25와 threshold는 변경하지 않았다.

## 결론과 한계

- MR 최소 hybrid 연결에서 자동 ScenarioLoad → gated rosbag → BEVFusion info 변환을 검증했다.
- 근접 장애물 3개, 정적 Ioniq 30/40/60/80 m 4개: 7 scene × 3 sample = 21 frame.
- 이것은 기존 anchor 619 주변의 **정지 visibility 진단**이다. 고주로 주행 분포, 동적 속도 학습, 모델 정확도 검증이 아니다. 도로 곡률/지형/가림 영향도 통제하지 않았다.
- 현재 결과만으로 SaveSensorData와 rosbag의 분포 차이가 없거나 한 방식이 우월하다고 결론 내리지 않는다.
- 다음 본 수집은 Base3 static을 유지하면서 근접/원거리 보강 deck으로 설계한다. 원거리에는 ROI·해상도 실험이 함께 필요하다. 기존 ±30 m 학습 계약에 60/80 m GT를 그대로 넣지 않는다.

## 연결·수집 결과

- 기존 5-interface `ego_network_mr_minimal_hybrid.json`, 센서는 `Sensor_competition_local.json`.
- 12초 연결 점검: LiDAR 12.89 Hz, front camera 29.94 Hz, Ego/Object 49.61 Hz.
- 초기 ego speed 0, static actor stability gate 통과, 시나리오 사이 자동 재연결 성공.
- 정지 online capture 지원을 수집기에 추가했다. 동기 writer 호출이나 drive/control 명령은 하지 않는다. 각 trigger의 Ego/Object가 0.25초 이내, ego XY drift가 0.25 m 이내여야 한다.
- 첫 실행은 metadata dict/path 계약 오류로 **capture 전에 실패**했다. 생성기가 기존 path/runtime_actors 계약을 사용하도록 수정한 후 7/7 성공했다. 실패를 성공 샘플로 포함하지 않았다.
- raw audit: `passed=true`, warnings/errors 없음, 7개 완료 marker window.
- 카메라-LiDAR nearest stamp p95는 38.3–40.2 ms, 최대 90.8 ms. 0.10초 기준 초과 없음.
- rosbag 약 302 MiB, 변환 데이터 약 11 MiB. 전체 변환 GT Obstacle 9/Vehicle 12, unknown assets 0.

## GT/입력 진단

MR Ego heading -1.347°를 변환 시 한 번 적용하고 measured asset catalog를 사용했다.
raw cloud 보존, scan deskew OFF, state interpolation 사용. 이 정지 실험으로 deskew 우열은 판단하지 않는다.

| Scene | 배치 | 대표 프레임 corrected GT 내부 LiDAR points |
| --- | --- | ---: |
| b5vp01 | CargoBox 전방 5 m | 0 |
| b5vp02 | CargoBox 왼쪽 3 m | 0 |
| b5vp03 | Concrete_Barricade01 왼쪽 3 m | 0 |
| b5vp04 | Ioniq 전방 30 m | 16 |
| b5vp05 | Ioniq 전방 40 m | 9 |
| b5vp06 | Ioniq 전방 60 m | 6 |
| b5vp07 | Ioniq 전방 80 m | 4 |

이는 scene마다 한 대표 frame이며 거리별 통계가 아니다. Ioniq geometric center는 배치 pivot보다 약 1.25 m 전방이므로 실제 LiDAR center 거리는 약 30.47/40.51/60.43/80.43 m이다.
GT 내부 점 0개는 객체가 화면에 없다는 뜻이 아니다. CargoBox 전방 카메라 투영 이미지를 직접 확인했으며 객체가 보였다. 센서 FOV/가림/GT 정합을 추가 구분해야 한다.
근접 camera-visible 보강 샘플을 `MIN_GT_POINTS=5`로 모두 버리지 않도록 이 진단 변환은 **minimum 0, distance-filter all**로 수행했다. 이를 production 공통 필터 변경으로 해석하지 않는다.

## 동기 캡처 A/B 상태

기존 6-interface `ego_network_mr_calibration_hybrid.json`을 probe sidecar에만 적용했다.
SaveSensorData 연결 없이 시작하면 runner가 scenario를 로드하기 전에 기다리므로, online stationary load로 먼저 6-interface를 활성화했다.
이후 `/SaveSensorData` subscriber가 연결된 상태에서 CargoBox 1회 요청을 실행했다.

- 파일 7개, CAMERA_1–4 / LIDAR_5 / GPS_6 / IMU_7 저장은 확인했다.
- Camera Depth/Instance/Semantic, LiDAR Instance bin/txt/8Points/Semantic annotation이 없어 completeness gate가 10초 후 실패했다.
- 현재 Capture Mode에서 GT-paired 비교가 미완료이다. **MR에 저장 기능이 없다**는 뜻이 아니며, sensor capture mode를 GT 출력 포함으로 설정하고 다시 검증해야 한다.
- 시험 후 7개 probe sidecar와 live network를 5-interface 최소 프로필로 복원했다. 사용자 MORAI/ROS는 종료하지 않았다. 내가 시작한 recorder/runner는 종료했다.

## 산출물과 재현

- generator: `dataset/morai_od/scripts/generate_base5_visibility_probe.py`
- ignored manifest/runtime/scenarios: `artifacts/datasets/base5_visibility_probe_261004/`
- 원본 7 bag: `rosbags/datasets/base5/261004_visibility_probe/`
- metadata: `/home/luv/morai_cap/261004_base5_visibility_probe/dynamic_metadata/`
- 변환: `artifacts/datasets/base5_visibility_probe_261004/parsed/`
- raw audit: 동일 artifact root의 `raw_audit.json`
- 7개 대표 BEV 및 4-camera GT 투영: `gt_audit/gt_alignment_audit.json`, `gt_audit/*jpg`, `gt_audit/*png`
- synchronous partial capture 실패는 유효 학습 데이터에 포함하지 않았다.

```bash
ajou
/usr/bin/python3 dataset/morai_od/scripts/generate_base5_visibility_probe.py \
  --output artifacts/datasets/base5_visibility_probe_261004/scenarios
# 설치/실행은 기존 hybrid runbook + stationary online 모드 사용.
# 기존 수집 중 실행하지 않는다. ScenarioLoad는 사용자 시뮬레이터 장면을 변경한다.
```

검증: generator 계약 test 1개 + 기존 dynamic capture tools 52개 통과, py_compile 및 git diff --check 통과.
이번 단계에서 학습·모델 추론·production 승격은 수행하지 않았다.
