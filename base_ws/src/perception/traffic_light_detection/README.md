# Traffic light detection runtime

현재 launch는 `launch/traffic_light_node.launch`다. 신호등 작업은 이 README와 실제 호스트의 resolved 인자에서 시작한다.
BEVFusion 학습·dataset 문서를 먼저 읽을 필요는 없다.

## 프로세스·Python 경계

| 프로세스 | 현재 계약 |
|---|---|
| `traffic_light_route_group_id` | ROS Noetic `/usr/bin/python3`·system Lanelet2로 active-route regulatory-element ID 선택 |
| `traffic_light_node` | `inference_python` interpreter의 YOLO GPU 추론 |
| Output | AW149 표준 `/perception/traffic_light_recognition/traffic_signals` |

기본 `inference_python`은 `~/.local/share/ajounice/traffic_light_gpu/bin/python`이다. 설치기를 먼저 실행한다.
이 RTX5060Ti 호스트는 Python3.11 CUDA runtime과 Python3.8 Lanelet2 binding을 분리한다.
planning 전체를 Conda에서 실행하지 말고 정상 sourced ROS shell을 사용한다.

```bash
roslaunch traffic_light_detection traffic_light_node.launch \
  device:=0
```

## 환경·입력

- GPU에 맞는 PyTorch wheel을 먼저 설치한 후 `requirements-runtime.txt`를 설치한다. CUDA/PyTorch는 rosdep 외부 계약이다.
- 이 호스트의 확인 이력은 Python3.11.15·PyTorch2.7.1+cu128·Torchvision0.22.1+cu128·NumPy1.26.4·OpenCV4.8.1이다. 다른 GPU에 동일 wheel을 강제하지 않는다.
- 팀 Ultralytics **8.4.143**을 유지한다. interpreter는 `scripts/workspace/source_aw149.sh` source 후 `rospy`·generated Autoware messages를 import해야 한다.
- 설치기는 고정 v5 두 모델의 CUDA load와 관련 C++/launch 설치본을 확인한다. MORAI front1280×720·10Hz cadence·confidence는 유지한다.
- 이 호스트의 warm-up 추론 35–45ms는 과거 실측이며 다른 GPU·LAN 전체 지연 보증이 아니다.
- map 기본은 v87다. 부모 launch의 map/projector override도 확인한다. planning UDP time은 `use_sim_time=false`·기존 `is_simulation=false` 계약을 따른다.

## Competition signal policy

`config/traffic_signal_profiles.yaml`은 route matcher가 선택한 regulatory-element ID를 재사용 가능한 신호 program에 연결한다.
Profile은 supported lamps·유효 Autoware element 조합·detector correction·fallback phase를 정의한다.
출력은 표준 `TrafficLightGroup.elements[]`를 유지하며 A/B/C/D 동작을 detector code 변경 없이 설정한다.

Route helper의 invalid match는 group ID `0`을 발행하며 지도 모드에서 다른 신호등으로 대체하지 않는다.
별도 2초 message timeout은 helper process 상실에만 적용한다. 옛 route ID를 계속 유지하지 않는다.
Planning 준수 가드 ON(현재 기본)은 missing/stale/unknown을 새 GO로 풀지 않는다.
기존 `competition_fail_open`은 가드 OFF 대조군에서만 적용한다. 실제 도로용 안전 인증은 아니다.
원시 correction 이유는 `/traffic_light_node/debug_policy_reason`에서 확인한다.

환경 설치 명령과 정리 전 상세는 [보존 원문](docs/legacy/context_261024/README.md)에 있다.

## 강건성·학습 작업

운영 기준·후보 상태·구간별 관찰 계약은
[신호등 CURRENT](../../../../docs/perception/traffic_light/CURRENT.md)에서 관리한다.
이 README에는 과거 실험 결과를 누적하지 않는다.
v5 시험 배포는 [설치·GUI 선택 안내](../../../../docs/perception/traffic_light/V5_CANDIDATE_HANDOFF.md)를 따른다.
`install_traffic_light_candidate.sh`는 두 가중치·전용 환경을 준비한다. 사용자 요청으로 v5가 현재 기본이다.

## 지도 선택·CNN 실행 옵션

기본은 `selection_mode:=map_association classifier_mode:=cnn detector_imgsz:=960`다.
명시적 복귀는 `traffic_light_legacy.launch`(또는 `traffic_legacy`)다. 설치본이 옛 파일인지 확인한다.

```bash
roslaunch traffic_light_detection traffic_light_node.launch \
  inference_python:=$HOME/miniconda3/envs/traffic_light_gpu/bin/python device:=0 \
  selection_mode:=map_association classifier_mode:=cnn \
  weights_path:=/absolute/path/to/explicit_research_detector.pt detector_imgsz:=960 \
  cnn_checkpoint_path:=/absolute/path/to/explicit_research_checkpoint.pt cnn_device:=cuda:0 \
  output_topic:=/trial/traffic_light_recognition/traffic_signals
```

`map_association`은 `/localization/kinematic_state`와 **기존** TF를 읽는 system-Python
helper를 추가한다. `morai_map → map → base_link → camera_front` 연결이 필요하며,
동적 TF는 JPEG stamp로 조회한다. TF/측위/route가 없거나 stale이면 명시적 missing이다.
helper는 TF를 발행하거나 Ego/native API를 구독하지 않는다. 원본 camera profile의
고정 extrinsic과 실제 camera TF가 다르면 자동 재보정하지 않는다.
운영 제공자는 `bringup/sensing.launch → tf_pub/sensor_tf.launch →
ioniq5/sensor_static_tf.launch → morai_sensor_static_tf.py`의 camera static TF와,
`localization_gnss_imu_gt_ctra.launch → tf_pub/map_tf.launch`의 `map → morai_map`,
localization EKF의 `map → odom → base_link`다. sensing은 camera TF 기본 ON이고
localization의 sensor TF 기본은 OFF다. 설치 helper는
`install_aw149/lib/ioniq5/morai_sensor_static_tf.py`이며 별도 LiDAR 전용 TF가
camera frame을 대신 제공하지 않는다. 이름만 맞추는 새 정적 TF를 임의 추가하지 않는다.

정적 geometry는 출처 SHA가 있는 `config/traffic_light_map_geometry.json`이다.
네 미션 그룹 A077/B091/C016+C018/D052만 선택 가능하다. C의 두 head는 같은 프로그램이다.
지도 모드에서는 route helper도 `allowed_group_ids`로 이 네 그룹만 남겨,
옛 D-1이 '다음 신호' 자리를 점유하지 않게 한다. legacy 모드의 helper는 기존 그대로다.
다른 차량 head는 identity 모호성을 거부하는 비교 대상일 뿐이며, 보행자·옛 D-1은
미션 대상으로 승격하지 않는다. 다른 화면 박스로 전체 fallback하지 않는다.
정면 각도·깊이·최소 투영 크기·공분산 기반 bounded padding을 검사한다. 투영은
가림·실제 점등·완벽한 identity의 GT가 아니므로 실제 YOLO/측위 입력 검증이 남는다.

지도 모드의 검색 상한은 수평 90px·수직 200px로 분리한다. XY 최대 고유축 표준편차와
GNSS Z 표준편차를 별도로 투영하며 Z가 수평 검색을 넓히지 않는다. 현재 Z 분산 1은
EKF placeholder가 아니라 `vertical_stddev_m=1.0`의 명시적 GNSS 설정이므로 유지한다.
기존 개발 val 66장의 저장 검출 박스 재생에서 수직 90px는 42개 정상 검출을 잃었고,
180px는 66/66을 유지했다. 가로 housing의 종횡비/투영 종횡비도 0.4~2.5로 제한한다
(동일 val의 실제 범위 0.830~0.966). 독립 폭/높이 비율만으로는 세로 보행자 모양의
합성 박스가 통과했으며, 이 guard는 해당 합성 오류를 차단했다. 다른 head의 실제 승인
박스를 사용한 음성 challenge는 **18개뿐**이며 여기서 오연결이 없었다는 한계가 있다.
근거는 `artifacts/evaluation/traffic_light_robustness_261025/live_covariance_axis_review_v1.json`이다.
이는 실제 ROS match율·미관측 주행 test 성능이 아니다. helper와 node는 같은
`selector_config()` 계약과 packet version 2를 사용하며, 구버전 helper와 섞이면 missing으로
거부하므로 시험 배치 전 둘 다 설치·재시작해야 한다.

근접 재검출은 `near_rescue_enabled:=true`가 기본이다. 960에서 현재 미션 신호를 연결하지
못했고 지도 투영 깊이3~15m·폭60px 이상인 경우에만 같은 원본을640으로 한 번 더 본다.
동일 이미지 stamp·지도 freshness·현재 미션을 확인하며, 재검출 도중 미션이 바뀌거나
packet이 만료되면 결과를 버린다. 다른 head 모호성·기하 검사·confidence0.25는 그대로다.
이미 연결된 관측·CNN UNKNOWN·지도 누락은 재시도 대상이 아니다. raw Python과 legacy는
기본 OFF다. 가중치·CNN·황색/신호 상실 정책은 바꾸지 않는다.
[근접 보완 근거](../../../../docs/perception/traffic_light/261025_NEAR_SIGNAL_RECOVERY.md)를 본다.
기존 방식과 대조할 때만 `near_rescue_enabled:=false map_max_vertical_padding_px:=180`을 쓴다.
`debug_selection.near_rescue`와 `debug_runtime.frame_decision.near_rescue`에 시도/승인을 남긴다.
추가 비용 `near_rescue_detector_and_association_sec`는 기존 selection 시간의 부분집합이다.

helper packet은 기본적으로 **동일 JPEG stamp**여야 하며 최대 30ms만 기다린다.
실 CPU 부하·TF 후속 표본 지연에서 이 대기값의 실제 match/missing 비율은 별도 측정한다.
현재 순수 테스트 통과만으로 실 운용 match율이 보장되지는 않는다.
`debug_selection`에서 source stamp/head ID/기하 비용/누락 사유를 본다.
`debug_classifier`는 CNN UNKNOWN과 확신 있는 소등을 구분하며 원본 detector box와
실제 crop 좌표를 기록한다. CNN만 기본 12px crop margin을 적용한다
(`cnn_crop_margin_px`); HSV의 기존 crop은 바꾸지 않는다. CNN 파일·계약·확신 실패는
빈 관측이며 HSV로 몰래 전환하지 않는다. 현재 launch는 `planner_compliance_observations:=true`로
상류 7/4 대신 원래 stamp의 관측을 전달하고 판단에서 허가 관측 3개를 확인한다. 원시 missing/UNKNOWN과 정책 출력은 구분한다.
`detector_imgsz` 기본은 960이며 비교 실험에서 다른 해상도를 명시할 수 있다.
`debug_runtime`은 모델 경로/SHA·실제 device·imgsz·CNN 설정과 CUDA 메모리를 기록한다.
장치는 실제 predictor/backend에서 읽고 출처도 기록한다. Ultralytics outer model이
CPU에 남았다는 이유만으로 실제 추론을 CPU로 단정하거나 CUDA 메모리를 0으로 쓰지 않는다.
또한 같은 source stamp로 원시/보정/최종 tuples·정책 이유와 decode/검출·박스 읽기/
지도 대기/분류/출력의 monotonic callback wall time을 남긴다. CUDA event 시간과 다르다.

오프라인 단계 분리는 저장소 `scripts/evaluation/evaluate_traffic_light_pipeline.py`를
사용한다. `--dataset`의 JPEG/개발 pose, `--native-evidence`의 정확 head별 전후 응답,
`--housing-qc`의 SHA-bound 승인 native ID+2D box를 따로 받는다. 지도 투영을 identity
정답으로 재사용하지 않는다. 옛 C016 단독 강제 사진에서 C018을 선택하면 해당 GT는
unknown이며 성공에 넣지 않는다. 원본 detector는 SHA를 고정하고 다른 모델에는
`--allow-candidate-detector`가 필요하다. `--imgsz`와 `--cnn-checkpoint`를 명시하고,
`--output`은 `artifacts/evaluation/` 아래 새 JSON만 허용한다. 이 도구는 ROS/RPC/학습을
실행하지 않는다. 실제 native 상태 일치율·arrow·UNKNOWN·잘못된 대상·현재 정책 proxy를
분리하지만 sparse 사진으로 smoother 지연이나 공식 황색 통행 규칙을 평가하지 않는다.

## 2026-10-26 관측 보존과 정차 시야 보완

운영 direct 경로는 `observed_phases`의 실제 램프만 전달한다. C의 화살표를 보고 직진G를
추가하거나 D의Y를R로 바꾸지 않는다. 모순·무점등·저확신은 UNKNOWN이며 legacy 보정과 구분한다.
`detection_miss_hold_sec:=0.2`는 같은 미션/head가 현재 지도에 유효하게 투영되는 단발 detector
누락만 완충한다. 원본 stamp를 유지하고, 실제 R/Y·CNN UNKNOWN·TF/지도/identity 실패는 즉시 반영한다.

팀과 합의하여 정차 위치는 기존 지도와 planner 설정으로만 관리한다. 운영 기준은
`planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml`의
`traffic_light.stop_margin: 0.75`이며 A/B/C/D에 공통 적용한다. normal 모드와 독립 모듈 기본도
동일 값이다. 앞범퍼가 지도 ref_line보다0.75m 앞에서 정차하도록 계산하며 지도 OSM은 바꾸지 않는다.
PR54의 별도 `traffic_signal_stop_lines.yaml`·추가 margin·통과선 분리 코드와 launch 인자는 제거했다.
지도 선의 위치는 지도 담당자가 관리한다. 기존 지도선 기준 GO_OUT과 실제 도색선의 차이,
이동 중 신호등 화각 이탈 문제는 여전히 남으며 황색 통과 기본 OFF·허가3회는 유지한다.
[변경 결정과 이전 실험](../../../../docs/perception/traffic_light/261026_TIMED_YELLOW_CANDIDATE.md#standard-stop-margin)을 본다.
