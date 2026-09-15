# configs
- 차량, 센서, tf 구성에 사용할 파라미터들

## configs 관리의 필요성
- 설정 값들을 모두 코드에 하드코딩하면, 비교와 실험 재현, 설정 관리가 매우 어려워짐.
- 따라서 기본 차량, 센서, tf 설정뿐만 아니라, 각 스택별 사용할 config 또한 따로 yaml 파일로 분기하여 저장하면 관리가 쉬워짐.
- 반복 실험을 위해 yaml 파일은 여러 개를 만들 수도 있음. 활용하기 나름.

## configs list
- `vehicle/ioniq5.yaml`: 우리 차량 구성 파일
- `sensors/sensor_layout.yaml`: 우리 센서 구성 파일
- `tf/static_tf.yaml`: 우리 좌표계 구성 파일
- `morai_field_reference/`: 현장 MORAI PC에 실제 로드해 풀주행을 검증한
  Sensor/Network 원본. 알고리즘 PC의 `udp_lan` 입력과 역할이 다르므로 해당
  디렉터리의 README를 먼저 확인함.
- 
## Notes
- 현재(260511) 있는 `vehicle/ioniq5.yaml`, `sensors/sensor_layout.yaml`, `tf/static_tf.yaml`은 **예시 파일들임.**

## TODO
- [ ] `localization/fast_lio2.yaml` 생성
- [ ] `perception/bevfusion.yaml` 생성
- [ ] `planning/lane_driving.yaml` 생성
- [ ] `control/mpc.yaml` 생성
