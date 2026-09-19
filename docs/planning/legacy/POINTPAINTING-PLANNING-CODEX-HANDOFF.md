# PointPainting–Planning production 통합: Codex 인계 문서

최종 갱신: 2026-09-13

대상 브랜치: `planning/autoware-universe-to-ros1`

통합 PR: `#20`

planning 병합 커밋: `8bc438fe`

## 이 문서를 읽는 AI에게

이 문서는 AjouNice의 현재 PointPainting production 인지 결과를 ROS1
Autoware planning에 연결하기 위한 **최신 실행 계약**이다. 과거 문서와 범용 launch의
기본값이 다르면 이 문서와 아래 production launch 두 개를 우선한다.

- `morai_pointpainting_ros/pointpainting_production.launch`
- `perception_object_pipeline/tracking_prediction.launch`

임의로 S1/S1g/E65/FastBEV, CTRV, temporal sweep 또는 detector velocity로
되돌리지 않는다. 변경이 필요하면 현재 기준선을 먼저 재현하고 동일 rosbag/실주행
조건으로 A/B한 뒤 결정한다.

이 문서의 수신자는 v66 지도 관리자이며 AW149 planning 개발자다. 따라서 다음은
이미 갖춰진 전제로 취급한다.

- 최신 v66 OSM과 projector 설정을 보유하고 있다.
- 본인 PC에서 AW149 underlay/overlay를 빌드하고 관리할 수 있다.

v66 지도 전달이나 AW149 개념 설명을 통합 장애물로 취급하지 않는다. 이 인계에서
새롭게 확인할 핵심은 PointPainting 전용 Python/OpenPCDet/model bundle과 ROS object
topic 경계다.

## 결론부터

현재 production object pipeline은 다음과 같다.

```text
MORAI 4 cameras + LiDAR
          |
          v
B3S-PP1 E34 PointPainting detector
  DetectedObjects, lidar1 frame
  /perception/object_recognition/detection/objects
          |
          v
EKF multi-object tracker, internal 30 Hz prediction publish
  /perception/object_recognition/tracking/measurement_objects
          |
          v
UNKNOWN/Obstacle stationary policy
  /perception/object_recognition/tracking/objects
          |
          v
Lanelet2 v66 Map-CV predictor
  PredictedObjects, map frame, 0.1 s x 3.0 s
  /perception/object_recognition/objects
          |
          v
Autoware planning
```

Planning이 소비해야 하는 객체 topic은 오직 다음이다.

```text
/perception/object_recognition/objects
autoware_perception_msgs/PredictedObjects
```

검출 box, PointPainting legacy message, RViz marker를 planning 입력으로 직접 연결하지
않는다.

## 확정한 production 값

| 구분 | 값 |
|---|---|
| 3D detector | `B3S-PP1-GTDB-V20P10O4 E34` |
| semantic model | `Base3 LRASPP E30`, 4-class |
| 입력 | single-frame, 9 features |
| 전처리 | OpenCV, camera workers 4 |
| Vehicle threshold | `0.40` |
| Pedestrian threshold | `0.15` |
| Obstacle threshold | `0.10` |
| detector velocity | 발행하지 않음 |
| LiDAR measurement stamp | `object_points_median` |
| scan 내부 deskew | `off` |
| temporal sweep | 사용하지 않음 |
| tracker | EKF, vehicle yaw canonicalization, 내부 30 Hz |
| detector velocity fusion | `off` (`mode=0`) |
| stale limit | `0.25 s` |
| Obstacle 표현 | Autoware `UNKNOWN` |
| UNKNOWN motion policy | velocity/acceleration 0 강제, 관측 pose는 계속 갱신 |
| predictor | Lanelet2 v66 Map-CV |
| prediction | `0.1 s` 간격, `3.0 s`, 최대 6 hypotheses |

PointPainting의 세 번째 class는 Autoware 메시지에서 `UNKNOWN`으로 전달된다. 이는
분류 미확정 객체라는 일반 의미가 아니라 현재 시스템에서는 **정적 장애물** 계약이다.
검출 위치 jitter가 EKF 속도와 긴 predicted path로 증폭되어 planning을 방해했던 문제를
막기 위해 tracker 뒤에서 UNKNOWN의 운동 상태를 0으로 만든다. Vehicle/Pedestrian까지
무조건 정지시키면 안 된다.

## 모델 선택 근거와 남은 약점

같은 현재 evaluator의 7개 rosbag target-class macro center mAP는 PP1 `0.7073`,
S1g E37 `0.5574`였고 PP1이 모든 평가 bag에서 같거나 높았다. 팀원의 live full-stack
관찰도 PP1 우세였으므로 현재 기본값은 PP1이다.

단, 이것이 모든 obstacle asset에서 PP1이 최고라는 뜻은 아니다.

- 별도 multi-asset obstacle benchmark에서는 PP0가 PP1보다 좋았다.
- PP1은 CargoBox와 긴 barrier에서 confidence가 낮았다.
- Base3에서 학습 대상에서 제외한 극소형/저점수 obstacle까지 대회 production 목표로
  간주하지 않는다.
- 현재 obstacle 안전성은 detector 성능만이 아니라 UNKNOWN stationary policy까지
  포함한 system contract로 판단한다.

S1g는 비교/fallback checkpoint이지 최신 production 기본값이 아니다. 모델 bundle의
과거 `MANIFEST.json`, `QUICKSTART.md` 또는 `run_model.sh`가 S1g를 default라고
표현하더라도 현재 planning 통합에서는 이 문서와 `pointpainting_production.launch`를
따른다.

## 이번 인계에서 전달되는 것과 기존 planning 자산

서로 성격이 다른 네 항목을 구분해야 한다.

1. Git 소스
   - `base_ws/src`, `planning_ws/src`, launch와 문서.
   - planning 브랜치를 pull하면 받는다.
2. v66 지도
   - `kcity_map/lanelet_map/lanelet2_map_v67.osm`.
   - 수신자가 직접 관리하는 기존 자산이다. 새로 전달하거나 다운로드할 대상으로
     간주하지 않는다.
   - production launch가 기대하는 경로와 파일명이 일치하는지만 확인한다.
3. PointPainting model bundle
   - `artifacts/team_handoffs/260909_pointpainting_production_3model`.
   - checkpoint가 `.gitignore` 대상이므로 별도 전달이 필요하다.
4. AW149 install overlay
   - `planning_ws/install_isolated`, `base_ws/install_aw149`.
   - 수신자가 개발·관리하던 기존 planning 빌드 결과다.
   - PointPainting 인계에서 새로 제공할 파일이나 선행 장애물이 아니다.
   - 이번 Git 변경을 반영하기 위해 필요한 범위만 본인의 기존 절차로 증분 빌드한다.

PP1 핵심 파일 SHA256:

```text
493da1203083b73cefaf31f73f0e646f7665c688030f5d3da8fb7aed8ba99e21  models/b3s_pp1_e34/detector.pth
465da8d907cc05e6e5e4891fa65125892a136f7ade967d14c49c941bec0acbf4  models/b3s_pp1_e34/model.yaml
acf9981070a323b234056ee725998ea9059ca3457940c22f3395c16c280c0f64  segmentation/base3_lraspp_e30/segmentation.pth
```

## 기존 planning 작업공간에 변경 반영

작업 중인 변경을 먼저 보존한다.

```bash
cd ~/AjouNice2026
git status --short --branch
git switch planning/autoware-universe-to-ros1
git pull --ff-only origin planning/autoware-universe-to-ros1
git submodule update --init --recursive
```

수신자는 기존 AW149 개발 환경을 그대로 사용한다. source 변경이 반영되지 않았다면
본인의 기존 증분 빌드 절차를 사용한다. 저장소 표준 명령은 다음이다.

```bash
cd ~/AjouNice2026
bash scripts/workspace/build_all.sh
source scripts/workspace/source_aw149.sh
```

`setup_ws.sh`는 clean 초기화용이므로 이 인계 때문에 무조건 실행하지 않는다. 기존
정상 install이 있으면 `build_all.sh`의 증분 빌드 또는 팀원이 쓰던 동등한 빌드
절차를 사용한다.

기존 overlay가 이번 변경을 보고 있는지 확인:

```bash
test -f ~/AjouNice2026/base_ws/install_aw149/setup.bash \
  && echo "AW149 overlay ready"

source ~/AjouNice2026/scripts/workspace/source_aw149.sh
rospack find ajounice_planning_integration
rospack find perception_object_pipeline
rospack find morai_pointpainting_ros
rospack find ekf_multi_object_tracking
```

모든 경로는 `base_ws/install_aw149/share/...` 또는 planning
`install_isolated`를 가리켜야 한다. 옛 `base_ws/devel`을 AW149 production 실행에
섞지 않는다.

모델 bundle을 전달받은 뒤 OpenPCDet patch와 환경을 준비한다.

```bash
cd /path/to/260909_pointpainting_production_3model
bash scripts/install_openpcdet_patch.sh /path/to/OpenPCDet
python3 scripts/verify_bundle.py
```

`verify_bundle.py`의 과거 manifest 기본 모델 검사는 S1g를 기대할 수 있지만, 파일
hash 검증에는 사용할 수 있다. 실제 production 모델 선택은 launch가 PP1으로
고정한다.

## Python 환경을 분리한 이유

PointPainting은 PyTorch/OpenPCDet/spconv가 있는 Conda Python을 사용한다. EKF는
C++이고 motion-policy/Map-CV는 system ROS Python 3.8 환경을 사용한다. 이들을 한
Conda shell의 같은 roslaunch process tree로 묶었을 때 Python 3.11 경로가 system
ROS Python 노드까지 전파되어 downstream 노드가 죽는 문제가 있었다.

따라서 detector와 downstream은 ROS message를 경계로 **서로 다른 프로세스
트리**에서 실행한다. launch 자체는 시스템 ROS shell에서 실행하고 detector 노드에만
Conda Python의 절대 경로를 넘기는 방식이 가장 명확하다. Conda 환경 이름이나 GPU
번호를 launch에 고정하지 않는다.

## 권장 실차/MORAI 실행: 통합 planning 방식

아래 방식이 planning 팀원이 전체 stack을 돌릴 때 권장된다.

### Terminal 1: ROS master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal 2: MORAI UDP bridge

MORAI에서 Sensor/Ego network를 연결한 뒤 실행한다.

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
rosparam set /use_sim_time false

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  network_config:="$HOME/AjouNice2026/configs/network/ego_network_1.json" \
  enable_control:=false \
  lidar_rpm:=900.0
```

환경에 맞는 sensor/network JSON을 사용한다. 파일명을 무조건 복사하지 말고 MORAI
설정과 일치하는지 확인한다.

### Terminal 3: PP1 detector 전용

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh

roslaunch morai_pointpainting_ros pointpainting_production.launch \
  python_executable:="$HOME/miniconda3/envs/openpcdet_rtx50/bin/python" \
  openpcdet_root:="$HOME/OpenPCDet" \
  bundle_root:="/path/to/260909_pointpainting_production_3model"
```

팀원 환경 이름이 다르면 `python_executable`만 해당 환경의 Python으로 바꾼다.
`which python`을 launch 내부에서 추측하게 하지 않는다.

### Terminal 4: localization + sensing TF + tracking/prediction + planning

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh

roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_detector:=false \
  start_controller:=false
```

이 launch는 기본적으로 다음을 소유한다.

- MORAI GT localization.
- sensor static TF.
- EKF tracker.
- UNKNOWN stationary policy.
- v66 Map-CV predictor.
- traffic-light detector.
- v67 map loader와 planning graph.
- route requester.

`start_controller=false`가 안전한 초기값이다. trajectory, validator, object topic과
TF를 확인한 뒤에만 controller를 켠다.

### Terminal 5: RViz

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
roslaunch bringup aw149_planning_rviz.launch
```

객체 pipeline만 별도로 보고 싶으면:

```bash
roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

## 분리 진단 방식

planning full-stack과 독립적으로 인지 chain을 디버깅할 때만 사용한다.

```bash
# Detector는 위 PP1 production launch 그대로 실행

# 별도 system ROS terminal
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
roslaunch perception_object_pipeline tracking_prediction.launch

# localization/sensing/traffic 등이 이미 별도로 실행 중인 경우의 순수 planning
roslaunch ajounice_planning_integration competition_planning.launch
```

중요: `morai_planning_stack.launch`는 tracking/prediction을 기본 소유한다.
따라서 full-stack과 `tracking_prediction.launch`를 동시에 실행하면 EKF와 predictor가
중복 기동한다. 둘 중 하나만 선택한다.

## launch 이름이 비슷해서 생기는 혼동

- `pointpainting_production.launch`
  - 현재 PP1 detector-only production entrypoint.
- `tracking_prediction.launch`
  - detector 없이 EKF → UNKNOWN policy → v66 Map-CV만 실행.
- `competition_planning.launch`
  - localization/perception을 소유하지 않는 순수 planning entrypoint.
- `morai_planning_stack.launch`
  - 통합 편의 entrypoint. tracking/prediction은 기본 실행하지만 detector는 기본 off.
- `pointpainting_online.launch`
  - 범용 저수준 launch. 기본 checkpoint를 production 선택 근거로 쓰지 않는다.
- `pointpainting_with_tp.launch`, `object_pipeline.launch`
  - 호환/연구용. production 운영자가 직접 선택할 entrypoint가 아니다.

`morai_planning_stack.launch`의 `detector=fastbev` 기본값은
`start_detector=false`인 동안 사용되지 않는다. 이를 보고 실제 detector가 FastBEV라고
판단하면 안 된다. 실제 detector는 별도 `pointpainting_production.launch`가 발행한다.

## 정상 topic과 소유 노드

```bash
rostopic info /perception/object_recognition/detection/objects
rostopic info /perception/object_recognition/tracking/measurement_objects
rostopic info /perception/object_recognition/tracking/objects
rostopic info /perception/object_recognition/objects
```

정상 publisher:

| Topic | Publisher |
|---|---|
| detection | `/pointpainting_online` |
| measurement tracks | `/ekf_multi_object_tracking` |
| policy-applied tracks | `/tracked_object_motion_policy` |
| final predictions | `/map_based_prediction` |

입력과 출력 rate:

```bash
rostopic hz --wall-time /sensor/lidar1
rostopic hz --wall-time /sensor/camera/front/compressed
rostopic hz --wall-time /perception/object_recognition/detection/objects
rostopic hz --wall-time /perception/object_recognition/tracking/objects
rostopic hz --wall-time /perception/object_recognition/objects
```

Detection은 fresh LiDAR 입력을 따라 약 12--15 Hz다. Tracking/prediction은 최근
detection age가 0.25초 이하인 동안 30 Hz로 EKF 상태를 예측 발행한다. 30 Hz가 새
measurement가 30 Hz라는 뜻은 아니다.

TF 확인:

```bash
rosrun tf tf_echo map base_link
rosrun tf tf_echo map lidar1
```

RViz fixed frame은 `map`이다.

## 자주 발생한 실패와 진단 순서

### Detection log에는 boxes가 있는데 downstream topic이 없음

1. 실제 canonical output을 확인한다.
   ```bash
   rostopic info /perception/object_recognition/detection/objects
   rostopic echo -n 1 /perception/object_recognition/detection/objects
   ```
2. `/pointpainting/detections`와 marker는 진단용이지 planning 입력이 아니다.
3. publisher가 둘 이상이면 중복 detector/launch를 종료한다.

### Tracking/Map-CV가 안 나옴

1. `map -> base_link -> lidar1` TF 연결 확인.
2. detection header stamp와 현재 ROS time의 차이 확인.
3. live 실행에서는 `/use_sim_time=false` 확인.
4. full-stack과 standalone `tracking_prediction.launch`가 중복인지 확인.
5. v66 OSM과 projector 파일 존재 확인.

### Python import, torch, spconv, checkpoint shape 오류

1. system Python으로 detector를 실행하지 않았는지 확인.
2. `python_executable`이 팀원 Conda 환경의 절대 경로인지 확인.
3. bundle의 OpenPCDet patch를 설치했는지 확인.
4. PP1 model YAML과 PP1 checkpoint, Base3 LRASPP를 함께 사용했는지 확인.
5. CUDA/PyTorch/spconv 조합은 전달된 environment 자료와 비교한다.

### Obstacle이 움직이거나 긴 path를 생성함

정상 계약에서는 Obstacle이 `UNKNOWN`이고
`force_unknown_stationary=true`다.

```bash
rosparam get /tracked_object_motion_policy/force_unknown_stationary
rostopic info /perception/object_recognition/tracking/objects
```

false이거나 motion-policy 노드를 우회해 measurement track을 planning에 직접 넣고
있는지 확인한다.

### RViz에는 안 보이는데 detector는 동작함

표준 RViz adapter는 canonical Autoware object topic을 사용한다. 과거
PointPainting marker 설정을 재사용하지 말고
`object_pipeline_visualization.launch`를 사용한다. fixed frame과 TF도 확인한다.

### rosbag 평가 결과가 이상함

live launch를 그대로 두고 `rosbag play --clock`만 실행하지 않는다. 일부 노드가 wall
time, 일부가 bag time을 쓰면 EKF stale guard가 출력을 버린다. 전용 evaluator를 쓴다.

```bash
perception/pointpainting/tools/run_rosbag_pipeline_evaluation.sh \
  INPUT.bag OUTPUT_DIR LABEL
```

## 검증된 범위

- production launch/policy Python tests 통과.
- `morai_pointpainting_ros`, `perception_object_pipeline`,
  `ajounice_planning_integration` build 통과.
- 표준 object pipeline rostest 통과.
- EKF → UNKNOWN policy → Map-CV 세 노드 실제 기동 확인.
- PointPainting launch가 Base3 LRASPP와 PP1 checkpoint `127/127` 로드하는 것 확인.
- AW149 overlay 생성 후 v66를 사용하는 `morai_planning_stack.launch` 정적 파싱 확인.

이 결과가 실제 MORAI 폐루프 주행 승인까지 뜻하지는 않는다. 팀원 PC에서는 실제
sensor rate, TF, detection/tracking/prediction topic, planning trajectory와 validator를
확인한 뒤 controller를 활성화해야 한다.

## 추가로 읽을 문서

- `docs/runbooks/legacy/morai_pointpainting_object_pipeline_live.md`
- `perception/pointpainting/docs/legacy/production_runtime_contract_260913.md`
- `base_ws/src/perception_object_pipeline/README.md`
- `base_ws/src/perception_object_pipeline/docs/map_cv_production_handoff_260820.md`
- `docs/planning/INTEGRATION-HANDOFF.md`

모델 실험의 세부 metric과 역사적 후보 비교는 production runtime contract와
`perception/pointpainting/docs/legacy/rosbag_vehicle_leaderboard.md`를 참고한다. 역사적
문서의 과거 기본값을 현재 launch보다 우선하지 않는다.
