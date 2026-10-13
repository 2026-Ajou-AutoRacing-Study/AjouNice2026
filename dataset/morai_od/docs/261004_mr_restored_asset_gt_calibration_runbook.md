# MR 복구 객체 geometric GT 보정 준비

상태: EVIDENCE / MR 20 scene, 40회 Instance GT 캡처 완료. 2026-10-04.

## 완료 결과

- 사용자가 캡처 출력을 변경한 뒤 CargoBox control에서 annotation 포함25개 파일 저장 PASS. 전체20/20 scene, 40/40 capture, failure0.
- 보정용 동기 캡처에서는 UDP 센서 스트림이 중단되어 `--no-timing-sidecar`를 명시했다. 정지 Ego/Object와 동기 annotation을 짝지은 보정 자료이며 rosbag 학습 입력용 수집으로 보고하지 않는다. 이후 rosbag 수집 전 UDP camera/LiDAR 연결 복구가 필요하다.
- Raw: `/home/luv/morai_cap/261004_base5_mr_gt_restored`. 측정 reference: `../configs/morai_asset_bbox_offsets_mr_restored_261004.json`.
- 거리8/12m, yaw0/90도, 각 asset8회. 전체 center residual p50=0.00478m, p95=0.01199m, max=0.04626m. 이는 보정값을 산출한 동일 관측에서의 일관성 검사이지 held-out 정확도나 LiDAR mesh 정합 평가가 아니다.
- WoodBox/Scooter_01/Boy2/police_f_1의 측정 reference를 확보했다. 각각 center residual p95=0.00476/0.00581/0.01161/0.03665m. MR catalog의 pending 상태를 measured reference로 갱신했다.
- CargoBox control size는 이번 MR에서 0.748630×0.748630×0.434416m, 역사적 reference는 0.847357×0.847357×0.491706m. 크기 차이의 원인을 아직 단정하지 않는다. 기존75-asset production catalog를 덮어쓰지 않고 이번5-asset reference를 분리 보존했다.
- 추가 학습은 아직 실행하지 않았다. 새 객체를 포함한 rosbag 보강과 명시적인 reference 선택·변환 검증 뒤 기존 replay pilot에 추가해야 한다.

## 앞선 실패 이력

- UDP 센서5종과 ROS Ego/Object publisher 및 실제 메시지 수신은 PASS.
- 처음에는 SaveSensorData subscriber가 없었다. 보정용 sidecar를 사용하는 b5gt01을 online stationary 모드로 한 번 로드한 뒤 ROS bridge subscriber가 연결됐다.
- CargoBox b5gt01 동기 캡처 요청은 원본 센서7개(Camera4/LiDAR/GPS/IMU)를 저장했지만, LiDAR Instance txt/bin/8Points 및 camera annotation은 저장하지 않았다. 10초 completeness gate에서 실패했으며 전체20 scene 실행은 중단했다.
- 실패 control 경로: `/home/luv/morai_cap/261004_base5_mr_gt_control`. 캡처 ID: `r1004210353_b5gt01_f00`.
- MR의 Camera/LiDAR 캡처 출력에서 Instance annotation을 활성화한 뒤 단일 control부터 재검증해야 한다. 파일 존재 여부만으로 GT 수집 성공으로 처리하지 않는다. 새4종 보정값이나 학습 데이터는 아직 만들지 않았다.
- stationary 실행도 공통 CLI 검증을 통과하도록 `--speed-kph 2 --minimum-curve-speed-kph 1`을 명시한다. stationary 모드는 이 값으로 주행하지 않는다.

## 준비 결과와 현재 제한

- CargoBox control + WoodBox/police_f_1/Boy2/Scooter_01, 각각 거리8/12m × yaw0/90°, 총20 scene. scene당2회 동기 캡처, 최대40 frame. Ego/객체 정지, scale1.
- `generate_mr_restored_asset_calibration.py`가 MR 원본 template을 사용한다. 초기속도0, 단일 객체, 무스폰, 기존 manifest overwrite 거부 테스트 PASS.
- 시나리오 b5gt01–20과 6-interface EgoNetwork sidecar를 MR Scenario 폴더에 설치했다. 이 변경으로 사용자 MR을 실행하거나 scene을 로드하지는 않았다.
- 준비 시점에는 ROS master/rosbridge/Simulator process가 없었다. 이후 연결해 위 단일 control을 수행했다. 과거 WoodBox capture 근거는 rosbag이며, 해당 복구 네 종의 측정 LiDAR Instance GT reference는 아직 없다.
- 종전 동기 probe에서 sensor payload7개만 저장되고 Instance annotation은 누락됐다. 이번에는 GT annotation 저장 여부를 먼저 확인한다. Capture Mode UI 설정을 확인하지 않고 성공을 가정하지 않는다.
- 기존75-asset measured reference와 production E25는 변경하지 않았다.

## MR 재연결

각 터미널에서 `ajou`부터 실행한다. MRP overlay나 full `udp` alias를 섞지 않는다.

```bash
# 터미널1
ajou
rc

# 터미널2
ajou
roslaunch rosbridge_server rosbridge_websocket.launch

# 터미널3
ajou
rosparam set /use_sim_time false
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false enable_control:=false
```

MR GUI에서는 기존 UDP Sensor_competition_local.json과 이번 보정용 network를 사용한다.

```text
Network: ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/
         25.S4.MolitComp03/ss_ego_gt_calibration_hybrid.json
Sensor:  ~/AjouNice2026/configs/sensors/Sensor_competition_local.json
```

보정용 network는 평소5-interface에 SaveSensorData만 추가한6-interface이다. 전체 ROS profile을 켜지 않는다. 센서 동기 캡처에서 LiDAR Instance annotation 저장이 포함돼 있어야 한다. 먼저 CargoBox1회로 실제 `LIDAR_5/*_Instance.txt`의 생성·nonempty 여부를 검증한다. GUI label-output 옵션의 정확한 이름은 현재 자동 검증되지 않았으므로 임의 JSON field를 만들지 않았다.

## 에이전트가 연결 확인 후 수행할 순서

1. 단일 UDP sensor publisher / ROS Ego·Object / ScenarioLoad·SaveSensorData 연결 확인.
2. CargoBox b5gt01 한 번만 아래와 같이 캡처. 실패하면20 scene 수집으로 확대하지 않는다.
3. 캡처 Instance GT·unique ID·ObjectStatus snapshot이 함께 있는지 확인.
4. 통과한 경우 전체20 scene을 별도 run directory로 캡처하고 distance/yaw별 GT offset·size의 안정성을 확인.
5. `build_morai_asset_bbox_offset_catalog.py`로 MR heading -1.347° 한 번 적용, asset-local center offset과 capture size 산출. CargoBox control을 기존 reference와 비교한다.
6. 새 네 종만 측정값·출처·오차와 함께 catalog에 추가한다. 기존75종을 전체 덮어쓰지 않는다. 보정 여부와 실제 mesh/LiDAR 잔차는 별도 지표이다.
7. 완료 후5-interface 최소 network로 복귀한다. Raw dataset/main training에는 동기 캡처 profile을 그대로 쓰지 않는다.

```bash
# 아래는 연결·annotation 설정 완료 뒤 실행하는 CargoBox 한 번 검증 명령이다.
ajou
/usr/bin/python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest artifacts/datasets/base5_mr_restored_gt_calibration_261004/runtime.json \
  --scenario b5gt01 --captures-per-scene 1 \
  --capture-dir "$HOME/morai_cap/261004_base5_mr_gt_control" --no-timestamp-prefix \
  --morai-scenario-dir "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --morai-sensor-data-dir "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData" \
  --motion-mode stationary --speed-kph 2 --minimum-curve-speed-kph 1 --capture-timeout-sec 10 \
  --load-timeout-sec 10 --scenario-load-attempts 1
```

MR catalog에는 현재 네 종 외에도 일부 NCAP·police1·조건부 obstacle의 geometric reference가 없다. 이번20 scene은 복구 네 종 우선 보정이며 전체 eligible107종의 GT coverage 완료로 보고하지 않는다. Missing reference인 visible actor를 background로 묵살하지 않는 converter error gate를 유지한다.
