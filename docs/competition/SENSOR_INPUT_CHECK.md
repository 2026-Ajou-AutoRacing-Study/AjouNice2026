# 네트워크·센서 입력 점검

운영 절차 4번에서 사용한다. `rc`와 해당 환경의 UDP 수신기를 먼저 켜고,
MORAI에서 Sensor Load·Connect 및 네트워크 연결을 확인한다. `sensing_lan`은
센서 TF용이며 실제 데이터 수신은 UDP 수신기가 담당한다.
인지·측위·플래닝·제어기는 이 검사에 필요 없다. 차량은 Keyboard·P·정지로 둔다.

## 통합 / 개별 실행

각 PC의 실제 저장소 경로에서 alias를 불러온다. 재빌드는 필요 없다.

```bash
source ~/AjouNice2026/docs/common_alias.txt
sensor_check
```

기본 환경은 본선 ROS IP `192.168.0.10`이다. 팀장님 개발망은
`sensor_check --profile teamlead` (`10.42.0.1`), 한 PC 로컬은
`sensor_check --profile local`로 선택한다. 설정을 변경하지 않고 검사 기준만 바꾼다.

항목별로도 같은 명령을 쓴다. 네트워크 외 항목은 IP 검사 없이 해당 수신만 확인한다.

```bash
sensor_check network --profile competition
sensor_check cameras
sensor_check lidar
sensor_check gnss
sensor_check imu
sensor_check vehicle
```

모든 토픽을 동시에 기본 5초 관찰한다. `--seconds 10`으로 1~30초 범위에서 바꿀 수 있다.
`PASS`: 최소 두 번 새 메시지 수신, 마지막 수신 1초 이내, 예상 타입 일치.
`WARN`: 기준 주기의 절반 미만 수신률 또는 GNSS 좌표 무효/음영; GNSS 무수신과 구분한다.
`FAIL`: 무수신·단일/오래된 메시지·타입 불일치·선택 프로필의 로컬 IP 부재.
`age`는 이 검사기가 마지막 메시지를 받은 뒤 경과 시간이며 센서 timestamp 지연이 아니다.
성공 시 0, 입력 실패 시 1, ROS master/환경 실패 시 2를 반환한다. WARN은 출력 검토가 필요하다.

출력 예시(실측 아님):

```text
PASS NET           local IP 192.168.0.10 OK
PASS Front         29.9Hz age=0.02s receiving
PASS LiDAR         14.9Hz age=0.03s receiving
WARN GNSS          29.8Hz age=0.01s invalid / shadow coordinates
CHECK: flow present; review warnings
```

## 점검기를 못 쓸 때: 기본 ROS 명령만으로 개별 확인

별도 구현 `sensor_check_raw`는 통합 점검기 Python·rospy 로직·설치 overlay를
사용하지 않고 ROS 기본 `rostopic`만 실행한다. 다음 항목을 각각 선택한다.

```bash
sensor_check_raw network
sensor_check_raw vehicle
sensor_check_raw front
sensor_check_raw left
sensor_check_raw right
sensor_check_raw rear
sensor_check_raw lidar
sensor_check_raw gnss
sensor_check_raw imu
```

이것은 자동 PASS/FAIL 판정기가 아니라 원문 Hz 확인용이다. alias까지 안 되면
`bash ~/AjouNice2026/scripts/runtime/check_sensor_inputs_raw.sh lidar`로 직접 실행한다.

ROS 환경을 불러온 별도 터미널에서 아래 각 줄을 실행한다. 카메라·LiDAR의
전체 payload를 `echo`하지 않는다. 화면 폭이 작아도 평균 수신 Hz만 볼 수 있다.

```bash
ip -br -4 addr
timeout 5s rostopic hz /Competition_topic
timeout 5s rostopic hz /sensor/camera/front/compressed
timeout 5s rostopic hz /sensor/camera/front_left/compressed
timeout 5s rostopic hz /sensor/camera/front_right/compressed
timeout 5s rostopic hz /sensor/camera/back/compressed
timeout 5s rostopic hz /sensor/lidar1
timeout 5s rostopic hz /sensor/gps
timeout 5s rostopic hz /sensor/imu
timeout 3s rostopic echo -n 1 /sensor/gps
```

`timeout` 종료 코드 124는 측정 시간을 제한한 정상 동작이다. `no new messages`는
수신 안 됨이다. `/sensor/gps` 위도·경도가 0이면 음영 가능성을 보되 무수신과 혼동하지 않는다.
CLI·rospy 자체가 없다면 ROS 설치/환경 문제이며, 이 대체 명령도 동작하지 않는다.

## 검사 범위와 현장 주의

- `/Competition_topic`은 현행 adapter에서 `morai_msgs/EgoVehicleStatus` 타입이다.
  `morai_msgs/Competition`은 다른 메시지다. `/Ego_topic`·`/Object_topic`은 요구하지 않는다.
- Collision은 이벤트 채널이라 무충돌 상태의 무출력을 오류로 판정하지 않는다.
- Competition은 운영측 판정프로그램이 중계한다. 4번 단계에서 아직 중계가 시작되지
  않았다면 운영 요원에게 확인하고, 5번 준비 후 재검사한다. 센서 정상과 별개다.
- NET은 선택한 로컬 IPv4 존재와 수신기의 config 경로를 보여준다. MORAI/판정 PC의
  실제 설정·제어 송신 경로·방화벽·패킷 출처를 검증했다는 뜻은 아니다.
- 네트워크/센서 경로가 예상과 다르면 [설정 안내](OPERATOR_NETWORK.md)와 대조한다.
- 이미지 색상/해상도·카메라 실제 장착·TF·동기화·모델 준비·주행 안전은 별도 확인이다.
  5초 정상 수신이 장시간 무단절을 보장하지 않는다.
- 노드 시작·종료, ROS param 변경, 제어/기어/ScenarioLoad 메시지 송신은 하지 않는다.
  이 도구를 사용하려고 운영 절차 7번 이후 PC 조작 금지 규정을 어기지 않는다.
