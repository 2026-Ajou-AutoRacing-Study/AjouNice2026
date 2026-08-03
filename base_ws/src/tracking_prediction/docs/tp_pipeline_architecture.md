# T&P 파이프라인 구조: Tracking / Extrapolation / Prediction

`tracking_prediction` 패키지의 런타임 노드를 세 단계(Tracking → Extrapolation / Prediction)로 나눠 정리한 문서. 평가/시각화용 스크립트는 마지막 절에 별도로 분류함.

## 전체 흐름

```
[GT 또는 실제 검출기]
    /Object_topic (MORAI GT)           /pointpainting/detections (실제 OD, 아직 미연동)
            |                                        |
            v                                        v
  ===================== Tracking =====================
  (A) GT+EKF 경로 (현재 메인, tracking_and_prediction_ekf.launch)
      /Object_topic
        -> gt_to_jsk_bbox_bridge.py       (GT -> jsk_recognition_msgs/BoundingBoxArray)
        -> ekf-multi-object-tracking(C++, 서드파티) (실제 EKF 데이터연관 트래킹)
        -> ekf_tracking_node.py           (EKF 출력 -> TrackedObjectArray로 포맷 변환)

  (B) 단순 경로 (dummy tracking, tracking_and_prediction.launch)
      /Object_topic -> tracking_node.py               -> TrackedObjectArray
      /pointpainting/detections -> pointpainting_tracking_node.py -> TrackedObjectArray
                                        |
                                        v
                          /tracking/objects (TrackedObjectArray)
                                        |
                    +-------------------+-------------------+
                    |                                       |
                    v                                       v
  ===== Extrapolation =====                    ===== Prediction =====
  extrapolation_node.py                        prediction_node.py
  (20~50Hz 고rate 출력)                         (0.3~3.0s horizon 미래예측)
        |                                             |
        v                                             v
  /tracking/objects_highrate                   /prediction/objects
  (TrackedObjectArray)                         (PredictedObjectArray)
  -> 판단/제어 소비용                                  |
                                                        v
                                    ===== Autoware 브릿지(선택적) =====
                                    predicted_objects_autoware_bridge.py
                                    (+ /tracking/objects 최신상태 매칭)
                                                        |
                                                        v
                                    /perception/object_recognition/objects
                                    (autoware_perception_msgs/PredictedObjects)
                                    -> 판단팀(behavior_path_planner 등) 실제 구독처
```

핵심 설계 원칙: **extrapolation과 prediction은 둘 다 `/tracking/objects`를 직접 구독하고, 서로의 출력을 참조하지 않는다.** extrapolation이 만든 고rate 값을 prediction이 다시 예측에 쓰면 "외삽 오차 + 예측 오차"가 누적되기 때문 (`docs/tp_final_report.md` 4-1절에서 사전 검증됨).

---

## 1. Tracking — 검출 결과를 표준 인터페이스로 통일

목적: 입력 소스(GT/실제 OD/실제 트래커)가 무엇이든 동일한 `/tracking/objects` (`TrackedObjectArray`)로 통일해서, 뒤단(extrapolation/prediction)은 소스가 바뀌어도 코드를 안 고쳐도 되게 함.

### 메시지: `TrackedObject.msg` / `TrackedObjectArray.msg`
```
track_id, class_label, position(x,y,z), heading(rad), velocity(vx,vy,vz), acceleration, size
```

### (A) EKF 경로 — 현재 메인, `tracking_and_prediction_ekf.launch`

**`gt_to_jsk_bbox_bridge.py`**
GT(`/Object_topic`, MORAI 전용 메시지)를 서드파티 EKF 트래커가 요구하는 입력 포맷(`jsk_recognition_msgs/BoundingBoxArray`)으로 변환. 실제 OD로 교체할 땐 이 노드 대신 같은 출력 포맷을 내는 다른 브릿지로 바꿔 끼우면 됨. `exclude_ids`(junk NPC 제외)와 `teleport_guard_dist_m`(순간이동 프레임 스킵) 필터를 갖고 있음 — 로터리 bag의 NPC 순간이동 아티팩트 대응용.

**`ekf-multi-object-tracking`** (서드파티 C++ 서브모듈, 이 패키지 밖)
실제 데이터연관 기반 EKF 트래킹. 진짜 지속되는 track_id, yaw_rate/가속도 추정을 제공. 빌드 환경이 막혀 있어 소스 직접 수정 불가(알려진 제약, `docs/tp_final_report.md` 7절).

**`ekf_tracking_node.py`**
EKF의 메인 출력(`jsk_all_track`, position/heading/속도 크기만 있고 track_id/yaw_rate 없음)과 시각화용 출력(`vis_all_track`의 track_info 마커, 진짜 track_id/yaw_rate가 텍스트로 들어있음)을 위치로 매칭해서 합친 뒤 `TrackedObjectArray`로 재발행. `~gap_warn_threshold` 파라미터로 콜백 간격이 비정상적으로 벌어지면 경고 로그를 남김(EKF 내부 ~3초 처리 지연 진단용, 2026-07-31 추가).

### (B) 단순 경로 — `tracking_and_prediction.launch` (dummy tracking)

**`tracking_node.py`**
GT를 "이미 검출된 3D bbox"인 것처럼 취급해서 재발행. 데이터연관 없이 프레임마다 보이는 순서로 1,2,3... 임시 ID 부여(dummy). EKF 연동 이전 초기 버전이자, EKF 없이 빠르게 테스트할 때 쓰는 대체 경로.

**`pointpainting_tracking_node.py`**
실제 PointPainting 검출기 연동용(아직 실사용 안 함). PointPainting은 위치만 주고 속도/가속도가 없어서, 직전 프레임과 최근접 매칭한 뒤 유한차분으로 속도/yaw_rate를 직접 추정해서 채움. 출력 인터페이스는 위와 완전히 동일해서 prediction_node.py는 무수정으로 그대로 동작.

---

## 2. Extrapolation — 판단/제어용 고rate 출력

목적: OD/트래킹 rate(8~15Hz)를 판단/제어가 요구하는 20~50Hz로 채워서 별도 토픽에 발행. prediction의 입력에는 영향 없음(설계 원칙 참고).

**`extrapolation_node.py`**
`/tracking/objects`를 구독해서 트랙별 최신 상태(위치/heading/v/omega)를 저장해두고, `~target_hz`(기본 30Hz) 주기의 별도 타이머에서 CTRV 공식으로 "지금까지의 경과시간만큼" 짧게 앞으로 굴려서(rollout) `/tracking/objects_highrate`에 발행. `~max_extrapolation_gap`(기본 0.3s)보다 오래된 트랙은 그 틱에서 제외(무리하게 오래 외삽하지 않음). yaw_rate 추정은 prediction_node.py와 동일한 최근접 매칭 방식.

---

## 3. Prediction — 미래 위치 예측

목적: 단일 고정 horizon(1.5s)에서 각 트랙의 미래 위치를 예측해서 판단(안전마진 산정)에 제공.

**`prediction_node.py`**
`/tracking/objects`를 구독, 트랙별로 CTRV(기본) 또는 CTRA(`~model` 파라미터) 모델로 `~horizons`(기본 `[1.5]`, 2026-08-01 최종 결정 — 미션 무관 단일 고정값) 시점의 미래 위치를 계산해서 `/prediction/objects`(`PredictedObjectArray`)로 발행. `trajectory` 배열에는 점이 정확히 1개만 들어있어 판단은 `trajectory[0]`을 그대로 쓰면 됨.

- **CTRV** (기본, 2026-07-30 확정): 등속+등회전속도. `docs/ctrv_vs_ctra_decision_summary.md` 근거로 CTRA보다 최악의 경우(p90/p95) 오차가 항상 작아서 채택.
- **omega(yaw_rate)**: GT/트래커가 직접 안 주므로, 직전 프레임과 최근접 매칭한 뒤 heading 변화량을 유한차분으로 추정 (`MATCH_DIST_THRESHOLD=3.0m`).
- **horizon 단일화 배경 (2026-08-01):** 처음엔 미션(로터리/하이웨이)별로 다른 "3.0m 이내 90% 확률 보장" 안전마진 horizon(로터리 0.3s / 하이웨이 1.7s)을 쓰려 했으나 지나치게 보수적이라 판단됨. 로터리 진입구간만 좁혀 느슨한 커버리지 기준으로 재평가하니 2.0s가 거의 손실 없었지만(`docs/entry_zone_ctrv_ctra_summary.md`), 같은 방식으로 하이웨이를 재평가하니 2.0s는 하이웨이에서 손실이 있었음(`docs/highway_ctrv_ctra_summary.md`). `~mission`/`~safety_horizon` 파라미터와 `mission_switching` 노드 구상은 모두 폐기하고, 로터리·하이웨이 둘 다 손실 없는 **1.5s 단일 고정값**으로 통일함.

### 메시지: `PredictedObject.msg` / `PredictedObjectArray.msg` / `PredictedTrajectoryPoint.msg`
```
PredictedObject: track_id, class_label, trajectory[](PredictedTrajectoryPoint 배열)
PredictedTrajectoryPoint: time_offset(초), position(x,y,z), heading(rad)
```

---

## 4. Autoware 브릿지 — 판단팀 인터페이스 변환 (2026-08-02 추가)

목적: 판단팀 코드(`behavior_path_planner`/`behavior_velocity_planner`/`motion_velocity_planner`, 총 59개 파일)를 직접 읽어서 확인한 결과, 이들은 전부 `/perception/object_recognition/objects`를 `autoware_perception_msgs/PredictedObjects`로 구독하고 있어 우리 `/prediction/objects`(`PredictedObjectArray`)와 토픽명·메시지 타입이 둘 다 다름. 이 브릿지가 그 변환을 담당.

**`predicted_objects_autoware_bridge.py`**
`/prediction/objects`(미래 궤적)와 `/tracking/objects`(현재 상태 - pose/twist/accel/shape/classification)를 track_id로 매칭해서 `autoware_perception_msgs/PredictedObjects`로 합쳐 재발행. track_id→UUID는 결정적 변환(같은 track_id는 항상 같은 UUID). twist/accel은 판단팀 코드(`bpp_dynamic_avoidance_tools.py`)가 pose의 orientation과 결합해서 쓰는 방식에 맞춰 heading 기준 전진/횡 성분(body frame)으로 투영해서 채움. covariance는 전부 0(불확실성 추정치 없음).

**주의(경로 다중점 필요) — `tracking_and_prediction_ekf_for_planning.launch` 사용:**
판단팀의 cut-in 감지 로직(`bpp_dynamic_avoidance_tools.py`의 `_predicted_path_cuts_in`)이 `kinematics.predicted_paths[].path`를 시간에 따라 이어진 경로로 순회하기 때문에, 우리가 최종 채택한 단일 horizon(1.5s) 점 하나만으로는 부족함. 이 브릿지를 실사용하려면 `prediction.launch`의 `~horizons`를 여러 값의 등간격 리스트로 띄워야 하며, 이를 위해 `tracking_and_prediction_ekf.launch`는 그대로 두고 별도로 `tracking_and_prediction_ekf_for_planning.launch`를 만들어 이 오버라이드 + 브릿지 포함을 함께 처리함.

2026-08-03부터 판단 연결용 기본값은 장기 오차와 RViz 형상을 함께 확인하기 위해 `[0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3.0]`으로 확장함. `t=0.0` 현재 위치를 첫 점으로 포함해서 Autoware `PredictedPath` 소비자가 `path[index]`를 `index * time_step` 시각으로 올바르게 해석하게 함. **이는 "실사용 horizon=1.5s" 안전성 결정을 3.0s로 변경했다는 뜻이 아니다.** 1.8~3.0초 점은 검토용이며, 판단에서 안전성이 검증된 범위를 사용할 때는 기존 1.5초 기준을 유지해야 함.

---

## Launch 구성 요약

| Launch 파일 | 구성 |
|---|---|
| `tracking_and_prediction_ekf.launch` | gt_to_jsk_bridge → EKF(서드파티) → ekf_tracking → prediction(horizons=[1.5]) → extrapolation (**현재 메인**, `trajectory[0]`만 쓰는 판단/제어 코드 전제) |
| `tracking_and_prediction_ekf_for_planning.launch` | 위와 동일 + prediction의 horizons를 다중점으로 오버라이드 + `predicted_objects_autoware_bridge` 포함 (판단팀이 실제 구독하는 `/perception/object_recognition/objects`까지 발행) |
| `tracking_and_prediction.launch` | tracking_node(dummy) → prediction → extrapolation (EKF 없이 빠른 테스트용) |
| `tracking.launch` / `tracking_from_pointpainting.launch` | tracking 단계만 단독 실행 |
| `ekf_tracking.launch` | ekf_tracking_node만 단독 (EKF 자체는 `ekf_multi_object_tracking` 패키지의 launch로 별도) |
| `prediction.launch` / `extrapolation.launch` / `predicted_objects_autoware_bridge.launch` | 각 단계 단독 실행 |
| `gt_to_jsk_bridge.launch` | GT→jsk bridge 단독 |

---

## 파이프라인 외 도구 (참고용, 런타임에 안 올라감)

- **시각화(RViz 확인용, bag 테스트 전용)**: `visualization_node.py`(tracking/prediction → MarkerArray), `gt_trajectory_markers_node.py`(GT 실제경로 → MarkerArray). GT 토픽 자체가 bag/시뮬레이터 전용이라 실차에는 해당 안 됨. (구버전 `future_rollout_node.py` — EKF 마커에서 직접 CTRV 롤아웃하던 prediction_node.py 이전 임시 확인용 — 는 미사용 확인 후 2026-08-02 삭제함.)
- **오프라인 평가/진단**: `eval_*.py`, `analyze_*.py`, `diagnose_*.py` 수십 개 — CTRV/CTRA 비교, horizon 결정, rate 민감도, gap 원인 진단 등. 전부 launch에 안 물려있는 독립 스크립트(bag 직접 읽어서 분석), 결과는 `docs/` 아래 문서로 정리됨.
