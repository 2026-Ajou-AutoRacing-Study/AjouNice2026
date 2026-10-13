# BEVFusion ROS bridge 실행 기준

[모델 CURRENT](../../../../docs/bevfusion/CURRENT.md#runtime)에서 production과 시험 모델을 먼저 선택한다.
이 패키지는 호스트 ROS 입력/후처리를 담당하며 GPU 서버·checkpoint는 별도로 준비한다.

## Production E24와 E25 복귀

[E24 production handoff](../../../../docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
`bevfusion`은 E24 고정 weights·Base5 transport·AMP·병렬화와 canonical `base5_production.launch`를 사용한다.
`bevfusion --model e25`는 [E25 복귀 계약](../../../../docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)을 선택한다.
`base3_online.launch`는 canonical detection topic을 사용하며 class threshold와 rotated BEV NMS를 적용한다.
기존 `bevfusion` 실행기는 서버 준비와 중복 canonical publisher를 검사한다.
`zero_shot.launch`는 호환 경로이며 학습되지 않은 nuScenes를 현재 모델로 고르는 안내가 아니다.

## Base5 시험

`base5_trial.launch`는 production을 자동 교체하지 않는다. 별도 서버 29515와
`/perception/base5_trial/detection/objects`를 사용한다.
status·latency·marker·projection도 `/perception/base5_trial/`로 분리한다.
`base5_e24_calibrated_trial.launch`의 선정값은 E24 전용이며 E9나 다음 학습 모델에 자동 적용하지 않는다.
2026-10-13 동일 조건 비교의 Base5 최종 시험 선택은 E24다. E9 비교용 선정값은
`base5_e9_calibrated_trial.launch`의 0.30/0.20/0.12이며 각 모델의 서버 가중치는 별도로 확인한다.
정확한 모델·threshold·최적화 선택은 위 CURRENT와 실제 launch/config를 확인한다.

학습된 512×1056/resize 0.825 입력과 카메라별 보정 정보를 사용한다.
호스트는 Base5 요청의 원본 JPEG와 0–255 XYZI를 정규화·재압축하지 않는다.
서버의 `--base5-runtime --point-intensity-source ros255`가 강도를 정확히 한 번 ÷255 한다.
`--fp32-reference`는 정합 대조용이며 최적화된 실시간 옵션을 의미하지 않는다.

[ROS 입력/TCP 정합 근거](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012)는
함수·메시지·저장된 입력 검증이다. 실제 24GB·LAN·full-stack queue 인증과 구분한다.

## 입력과 후단 경계

Sensor config가 토픽·보정·camera order를 정한다.
LiDAR 기준으로 카메라 buffer의 가까운 이미지를 선택하며 preprocessing과 추론은 최신 완료 frame을 유지한다.
그 정책을 바꾸면 sensor stamp·age·drop·정합을 함께 확인한다.
출력은 RViz MarkerArray와 Autoware DetectedObjects이며
[후단 CURRENT](../../perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 따른다.
MORAI GT·ego oracle을 대회 detector 입력이나 production fallback으로 쓰지 않는다.

단계별 JSON 진단은 `publish_frame_timing:=true`로 선택한다. 기본 OFF이며 production
실행기는 `BEVFUSION_PUBLISH_FRAME_TIMING=true`를 명시한다. canonical/trial 토픽과
raw/prepared queue·server·publish 범위는 [진단 계약](../../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#diagnostics)을 따른다.
기존 latency Float32나 입력 stamp·payload를 재정의하지 않으며 실제 server 비용은 별도로 기록한다.

박스 원점 시험은 calibrated trial의 `box_center_output_origin:=gravity_center_v1`을 선택한다.
기본은 `legacy_passthrough`이며 원점 metadata가 없는 구 서버는 시험 변환을 거부한다.
새 서버 raw tensor는 바닥 기준 그대로 보존하고 ROS adapter에서 높이 절반을 한 번만 더한다.
production 실행기는 자동 적용하지 않는다. 실제 3D TF와 tracker 정지/reset·planner 기억까지
[출력 중심 계약](../../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#box-origin)을 검증한 뒤 사용한다.

기존 Base3 실행 설명과 내부 구현 시점 기록은
[원문 스냅샷](../../../../docs/perception/legacy/context_261012/README.md)에 보존했다.
