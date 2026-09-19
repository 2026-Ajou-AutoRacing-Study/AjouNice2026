# PointPainting production runtime contract — 2026-09-13

## 결론

현재 전체 pipeline의 기준은 아래 한 조합으로 고정한다.

- Detector: `B3S-PP1 E34`.
- Semantic painting: `Base3 LRASPP E30`.
- Input: single-frame 9-feature PointPainting.
- Detector velocity: 발행하지 않음.
- Class score: Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10`.
- LiDAR timestamp: `object_points_median`.
- Scan 내부 deskew: `off`.
- Tracking: EKF measurement update와 내부 30 Hz prediction publish.
- Detector velocity fusion: `off`.
- UNKNOWN/Obstacle motion policy: stationary 강제.
- Prediction: Lanelet2 v62 Map-CV, 0.1초 간격, 3초 horizon, 최대 6 paths.

`S1g E37`은 역사적 fallback/comparator로 보존한다. 같은 7-bag detector
평가에서 PP1의 macro center mAP가 `0.7073`, S1g가 `0.5574`였고 PP1이 7개
평가 bag 모두에서 같거나 높았다. 팀원의 live full-stack 관찰도 PP1 우세였다.

다만 PP1이 모든 obstacle slice에서 우세하다는 뜻은 아니다. 새 multi-asset
obstacle benchmark에서 PP0가 PP1보다 좋았고 PP1은 CargoBox와 긴 barrier에
약하다. 현재 production 판단은 전체 Vehicle/Pedestrian/Obstacle 및 live 결과의
종합이며, UNKNOWN stationary policy로 obstacle의 가짜 속도와 경로 폭주를 별도로
막는다.

## 통합 뒤 발견한 회귀

planning 통합 뒤 실행 경로마다 서로 다른 기본값이 선택될 수 있었다.

1. `morai_planning_stack.launch`는 S1 E38과 CTRV를 기본으로 갖고 있었다.
2. 과거 `pointpainting` alias는 PP1을 가리키면서, 특정 사용자의
   `~/Downloads/260909_pointpainting_production_3model`에 의존했다.
3. 범용 rosbag evaluator는 모델을 명시하지 않아 E65 runtime 기본값으로 내려갈
   수 있었다.
4. detector와 T&P를 한 roslaunch에 넣으면 PointPainting Python 3.11 Conda
   `PYTHONPATH`가 Python 3.8 ROS 노드에 전파되어 Map-CV와 motion-policy가 죽었다.
5. AW149 포팅에서 evaluator가 ROS2 Duration의 `sec/nanosec`만 읽어 ROS1
   `secs/nsecs` bag 평가가 실패했다.
6. offline evaluator가 먼저 `/use_sim_time=true`를 설정해도
   `localization_morai_gt.launch`를 인자 없이 시작하면 그 launch의 기본값
   `false`가 전역 parameter를 다시 덮어썼다. EKF는 wall time과 과거 bag의
   detection timestamp를 비교해 모든 30 Hz 출력을 stale로 폐기했다. evaluator는
   localization에도 `use_sim_time:=true`를 명시하고 재생 전 전역 값을 검증한다.

현재 코드는 production entrypoint에서 PP1과 Map-CV를 명시하고, 모델 bundle은
저장소 내부 `artifacts/team_handoffs/260909_pointpainting_production_3model`을
사용한다. Duration evaluator는 ROS1/ROS2 양쪽 layout을 처리한다.

## 반드시 지킬 프로세스 경계

PointPainting과 downstream을 같은 roslaunch process tree로 실행하지 않는다.

```text
Python 3.11 Conda
PP1 detector-only
  -> /perception/object_recognition/detection/objects
                |
                v
System ROS Python 3.8 / C++
EKF -> UNKNOWN stationary policy -> Map-CV
  -> /perception/object_recognition/tracking/objects
  -> /perception/object_recognition/objects
```

두 프로세스는 같은 ROS master의 메시지로만 연결한다. `pointpainting_with_tp.launch`
같은 호환 entrypoint의 범용 기본값을 production 판단 근거로 사용하지 않는다.

## 모델 파일

- Config:
  `artifacts/team_handoffs/260909_pointpainting_production_3model/models/b3s_pp1_e34/model.yaml`
- Detector:
  `artifacts/team_handoffs/260909_pointpainting_production_3model/models/b3s_pp1_e34/detector.pth`
- Segmentation:
  `artifacts/team_handoffs/260909_pointpainting_production_3model/segmentation/base3_lraspp_e30/segmentation.pth`

`pointpainting_production.launch`는 위 모델과 consequential parameter를 명시한다.
머신별 Python, OpenPCDet, bundle 경로만 launch argument로 넘긴다.

## Live 실행

아래 세 터미널이 PointPainting production object-pipeline 실행법이다.
`pointpainting_with_tp.launch`나 범용 `object_pipeline.launch`를 직접 실행하는 것은
production 실행법이 아니다.

Terminal 1:

```bash
roscore
```

Terminal 2, detector-only:

```bash
conda activate <pointpainting-environment>
source /opt/ros/noetic/setup.bash
source /path/to/AjouNice2026/base_ws/devel/setup.bash
roslaunch morai_pointpainting_ros pointpainting_production.launch \
  python_executable:="$(command -v python)" \
  openpcdet_root:=/path/to/OpenPCDet \
  bundle_root:=/path/to/260909_pointpainting_production_3model
```

Terminal 3, EKF와 Map-CV:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/AjouNice2026/base_ws/devel/setup.bash
roslaunch perception_object_pipeline tracking_prediction.launch
```

Conda 환경명, checkout 위치와 GPU는 저장소가 강제하지 않는다. Detector와
downstream을 별도 터미널로 분리해 Python 환경이 전파되지 않게 한다.
분리 실행과 함께 Planning을 켤 때는 `competition_planning.launch`를 사용한다.
`morai_planning_stack.launch`가 downstream object pipeline을 기본 소유한다.
따라서 full-stack 실행과 standalone `tracking_prediction.launch`를 동시에
실행하면 안 된다.

현재 PC는 planning AW149 전체 install이 완료되지 않았으므로 perception runtime은
`base_ws/devel/setup.bash`를 사용한다. 이는 AW149 planning build의 성공 여부와
독립적으로 EKF와 Map-CV를 실행할 수 있다.

## 고주로1 재현 결과

입력:
`rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag`.

출력:
`artifacts/qualitative/pp1_production_contract_260913/highway1_fixed30_simtime_fix`.

- Raw duration: 52.21초, LiDAR 452 frame.
- Production detection: 441 frame.
- Tracking/Map-CV prediction: 각각 1,456 frame. 입력 시작·검출 공백의 0.25초
  stale gate 때문에 전체 bag 평균은 약 27.9 Hz이고, 유효 구간은 30 Hz다.
- v61 map hash prefix: `c8ee07fb7033`.
- Vehicle tracking precision/recall/F1: `0.9792 / 0.9212 / 0.9493`.
- Vehicle tracking bbox-center p50/p90: `0.464 / 0.679 m`.
- Vehicle tracking speed error mean/p50/p90: `0.449 / 0.332 / 1.000 m/s`.
- Top-1 FDE p50: 1초 `0.530 m`, 2초 `1.152 m`, 3초 `1.988 m`.

같은 v61 실행의 S1g 결과는 F1 `0.9164`, bbox p50/p90
`0.430 / 0.716 m`, speed error mean/p50 `0.426 / 0.337 m/s`, FDE p50
`0.507 / 1.083 / 1.871 m`였다. 같은 v61·30 Hz downstream 계약에서 PP1은
precision, recall, F1, bbox p90은 더 좋고 bbox p50·속도·FDE는 소폭 불리하다.
이는 detector 7-bag macro AP와 live 비교에서 PP1을 선택한 근거를 대체하지 않으며,
두 모델의 downstream trade-off로 함께 기록한다.

Map-CV log의 `objects`와 `fallback`은 UNKNOWN/Pedestrian도 포함한다. 예를 들어
`objects=38, mapped=2, fallback=36`은 차량 두 대가 map matching되고 나머지
UNKNOWN이 CV fallback된 경우일 수 있다. 이 숫자만 보고 Map-CV가 꺼졌다고
판정하면 안 된다. 실제 PP1 고주로 출력에는 Vehicle prediction 1,271개 중 238개
(`18.7%`)가 복수 hypothesis를 가졌으므로 Map-CV branch가 실행됐다.

Map-CV path는 객체 위치에서 시작해 `lateral_convergence_time_sec=1.5` 동안
centerline으로 수렴한다. 첫 점부터 중심선에 강제로 snap되지 않는 것이 정상이다.

### Map-CV path rank 의미

Vehicle의 Top-K rank는 CTRV 오차와 비교해서 정하지 않는다.

1. 현재 위치에서 3 m 이내이고 yaw 차이가 45도 이내인 시작 Lanelet 후보를 찾는다.
2. 횡방향 거리와 yaw 오차의 Gaussian score로 시작 후보를 정렬한다.
3. routing graph의 following Lanelet을 확장하면서 분기 prior와 연결부 yaw 변화
   score를 곱한다.
4. 거의 같은 궤적은 합치고 최대 6개의 기하학적으로 다른 경로만 남긴다.
5. `start-lane score * route score`를 합이 1이 되도록 정규화한 값이 path
   confidence이고, 이것이 rank가 된다.

즉 tracker의 현재 위치·yaw·speed와 Lanelet geometry를 사용하며, CTRV는 rank
feature가 아니다. Vehicle이 map-match되지 않거나 경로 길이가 부족하면 그때만
CTRV로 fallback한다. Pedestrian과 UNKNOWN은 map routing 대상이 아니며 kinematic
fallback을 사용한다. UNKNOWN은 앞단 motion policy에서 속도를 0으로 강제하므로
결과적으로 현재 위치에 정지한 path가 생성된다.

## 고주로1 RViz

```bash
cd ~/AjouNice2026
bash perception/pointpainting/tools/play_pp1_production_highway1_rviz.sh
```

느리게 보려면 마지막 인자로 배속을 준다.

```bash
bash perception/pointpainting/tools/play_pp1_production_highway1_rviz.sh 0.5
```

표시는 LiDAR, front cameras, v61 Lanelet map, detection, tracking, prediction,
MORAI GT box, matched GT future path를 포함한다.

## 문서 우선순위

현재 runtime을 판단할 때 이 문서와 실제 production launch를 우선한다.

- `base_ws/src/perception_object_pipeline/docs/legacy/map_cv_production_handoff_260820.md`는
  당시 알고리즘·tracker 계약의 역사 기록이다.
- 그 문서의 v60 경로는 historical이다. 통합 runtime map은 v62이다.
- `base3_static_pointpainting_training.md`, `rosbag_vehicle_leaderboard.md`의
  PP0/PP1 결과는 후보 비교 근거이며 production 기본값을 뜻하지 않는다.
- `artifacts/.../production_3model/README.md`의 S1g 기본 선택은 bundle 제작 당시의
  보수적 판단으로 historical이다. 이후 class-threshold 재평가와 live 비교를 반영한
  현재 기본값은 PP1이다.
