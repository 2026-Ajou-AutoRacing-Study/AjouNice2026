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

Personal MRP에는 이 보정을 적용하지 않는다. 이는 단일 scene 추정이 아니라 기존
Base3-Static 동기 annotation 전체 `3,680` metadata, `31,644` observation, `75` asset을
다시 계산해 확인했다. MRP 기본값은 `0.0`이다.

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

MRP 전체 동기 캡처에서는 반대로 잘못된 `-1.347 deg`를 넣으면 정합이 크게 악화됐다.

| MRP 31,644 observation | `0.0 deg` | 잘못된 `-1.347 deg` |
|---|---:|---:|
| center residual p50 | 0.126 m | 0.319 m |
| center residual p95 | 0.259 m | 0.834 m |
| center residual max | 1.236 m | 5.787 m |

따라서 launcher 계약은 확정적으로 `MR=-1.347 deg`, `MRP=0.0 deg`다. 두 launcher의
raw bag이나 info를 이름만 보고 섞지 않는다.

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

기존 MR `hwy3h1` raw bag 전체를 다음 corrected dataset으로 재변환했다.

```text
artifacts/datasets/morai_base4_temporal_highway_hwy3h1_mrhc_v2
PKL serialized root: /datasets/base4_hwy3h1_mrhc_v2
```

| 항목 | 보정 전 hwy3h1 | MR heading-corrected V2 |
|---|---:|---:|
| 유효 frame/Vehicle GT | 13,248 | 13,749 |
| train / val / test frame | 10,723 / 2,525 / 0 | 11,116 / 2,633 / 0 |
| sparse GT rejection | - | 1 |
| distance rejection | - | 4,222 |

구조 validator와 temporal quality audit는 모두 통과했다. 공통 13,232 instance의 GT
center 이동량은 p50 `0.283 m`, p95 `0.634 m`, max `0.719 m`다. 거리별 p50은
`0--10 m: 0.139 m`, `10--20 m: 0.334 m`, `20--30 m: 0.554 m`, `30 m+: 0.699 m`로,
원거리일수록 heading 편향의 영향이 커졌다. 따라서 기존 checkpoint를 수정된 GT의 결과로
간주하지 않고 새 V1/T1 A/B를 수행한다.

Corrected info fingerprint:

```text
train 159cf1740c5cb85214dc17bc7e6105bb0629632a084e8eaee8d688d6ddfd57af
val   acc729f1054cb3d37b8a596b70897c92df9dc1de629546f5de17154f7f952dbb
test  39e794985ab316598b8d9a0329771abe067b1581c3fdbacb73e4a89d26016dda
```

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
