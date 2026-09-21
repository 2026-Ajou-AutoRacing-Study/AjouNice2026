# MR heading과 동기 annotation 교차검증

- 상태: **EVIDENCE / CURRENT CONVERSION CONTRACT**
- 검증일: 2026-09-21
- 대상: competition MR rosbag 기반 Base3-Online/Base4 변환

## 결론

Competition MR의 raw `/Ego_topic.heading`은 scenario 및 sensor annotation 좌표계보다
약 `+1.35 deg` 회전되어 있다. MR rosbag을 LiDAR/Ego 좌표 학습 GT로 변환할 때는
다음을 사용한다.

```bash
--ego-heading-correction-deg -1.347
```

Personal MRP에는 이 보정을 적용하지 않는다. MRP Base3 캡처에서 scenario yaw와
`/Ego_topic.heading` 차이는 약 `0.009 deg`였으므로 MRP 기본값은 `0.0`이다.

## 검증 방법과 결과

MR Capture Mode의 `/SaveSensorData`를 짧은 calibration oracle로만 사용했다. 8개 차량
asset을 두 위치/yaw에서 배치하고, trigger 시점 `/Ego_topic`, `/Object_topic`과 동기
LiDAR Instance annotation을 함께 저장했다. 본 데이터 수집은 계속 UDP sensor rosbag을
사용하며 Capture Mode를 입력 경로로 사용하지 않는다.

| 검사 | raw heading | `-1.347 deg` |
|---|---:|---:|
| 9 asset, 32 observation center residual p50 | 0.192 m | 0.046 m |
| 같은 residual p95 | 0.228 m | 0.081 m |
| MR 2-scene rosbag corrected-box 내부 point | 79 | 147 |
| MR 고주로 20-scene corrected-box 내부 point | 815 | 896 |

별도 시나리오 비교에서 scenario Ego yaw `60.306 deg`에 대해 MR Ego heading은 약
`61.651 deg`였고, 16개 scene의 필요한 보정 중앙값은 `-1.350 deg`였다. 기존 competition
localization의 `-1.347 deg`와 일치하므로 새 상수를 만들지 않고 같은 값을 사용한다.
같은 scene의 Grandeur와 K5도 scenario yaw 대비 각각 약 `+1.348 deg`, `+1.356 deg`로
나타나 actor와 Ego가 공통 heading convention을 사용한다는 점도 확인했다.

## 적용 범위

보정 대상:

- map delta를 Ego/LiDAR frame으로 회전하는 GT center 계산
- `ego2global`, `lidar2global`
- causal sweep의 `sweep2lidar`
- Ego-pose 기반 scan deskew pose history

추가 보정하지 않는 값:

- actor heading과 Ego heading의 차인 box relative yaw
- actor body velocity를 Ego frame으로 회전한 relative velocity

두 값은 actor와 Ego에 공통 heading convention이 들어가므로 상수 편향이 상쇄된다.

## 데이터 상태

- raw MR rosbag: 재취득 불필요
- 기존 `p24v1`, `hwy3h1` info/checkpoint: 보정 발견 전 EVIDENCE
- 다음 학습: raw bag에서 corrected info를 재생성한 뒤 수행
- Base3-Static/MRP 및 기존 MRP-derived catalog: 변경하지 않음

MR 동기 calibration 산출물:

```text
/home/luv/morai_cap/260921_mr_vehicle_bbox_calibration_canonical
artifacts/datasets/mr_vehicle_bbox_calibration_260921_heading_corrected.json
```

MR asset별 크기 차이는 launcher asset mapping 차이일 수 있어 기존 Base3 catalog를
자동 덮어쓰지 않는다. 현재 catalog는 center residual과 point support를 검증하는 기준으로
유지하고, MR 전체 asset coverage가 확보된 뒤 별도 version으로 승격한다.

재현 시에만 다음 profile을 설치한다.

```text
configs/network/ego_network_mr_calibration_hybrid.json
```

이는 minimal hybrid에 `/SaveSensorData` 한 항목만 더한 profile이다. Capture Mode에서
필요한 Intensity/Instance/BBox3D modality를 GUI에서 켜면 live camera/LiDAR ROS stream이
중단되므로 calibration이 끝나는 즉시 Capture Mode를 끄고, scenario sidecar도
`ego_network_mr_minimal_hybrid.json`으로 복원한다. 2026-09-21 검증 종료 후 설치된
52개 asset-revalidation scenario sidecar는 minimal profile로 복원했다.
