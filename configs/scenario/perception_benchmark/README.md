# MORAI raw-sensor benchmark scenarios

These scenarios are deterministic derivatives of
`configs/scenario/2026_molit_comp_sample_scene.json` for reusable rosbag
recording. Regenerate them with:

```bash
python3 dataset/morai_od/scripts/build_raw_benchmark_scenarios.py
```

- `bench_roundabout_raw.json`: Ego is in Keyboard mode and parked at the
  roundabout approach. Only official roundabout spawn point `4` is retained.
- `bench_highway_merge_raw.json`: Ego is in Keyboard mode and parked at the
  highway merge approach. Only official highway spawn points `6`, `8`, and
  `10` are retained.

Both scenarios remove saved NPC instances, pedestrians, obstacles, pedestrian
spawners, and actor waypoint data. Traffic-light state is retained because it
is map infrastructure rather than a perception actor.

## 2026-10-14 원시 LiDAR 패킷 실험 준비

학습용 수집이 아니라 기존 51패킷 입력을 그대로 보존하는 짧은 녹화다. MORAI의
시나리오 목록에 아래 이름으로 원본과 동일한 파일을 복사했다. 위치는
`~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/`다.
로드만으로 Ego가 자동 주행하지 않으며 세 파일 모두 Keyboard·P·정지 상태로 시작한다.

| 로드할 이름 | 원본 | 권장 녹화 | 목적 |
|---|---|---|---|
| `0_lidar_probe_1_static` | `bench_static_obstacle_base3_assets_raw.json` | 30초, Ego 정지 | 정적 벽·장애물의 방위각 중복과 scan 경계 기준선 |
| `0_lidar_probe_2_highway` | `bench_highway_merge_raw.json` | 60초, 수동 직진·가능하면 50~80 km/h | Ego 이동·NPC 상대 이동에서 시간 차이 확인 |
| `0_lidar_probe_3_curve` | `bench_roundabout_raw.json` | 45초, 수동 저속 회전 | yaw 변화에서 scan 경계·왜곡 비교 |

속도는 시험 권장값이지 통신 설정이나 필수 통과 기준이 아니다. NPC와 접촉하지 않고
주행하며, 정지 기준선부터 먼저 확보한다. 기존 static benchmark의 여섯 장애물을
재사용한다. 새 Base5 학습용 배치를 만들거나 대회 분포를 바꾸는 작업은 아니다.

### MORAI와 네트워크

1. 대회 맵·Sunny13 유지. 일시정지/Capture freeze가 아니라 실시간 실행 상태로 둔다.
2. 첫 번째 시나리오를 로드하고 P·Keyboard·정지를 확인한다.
3. 센서 설정은 `Sensor_competition_local`, Ego Network는
   `ego_network_competition_local`을 Load한다. SaveFile의 두 파일은 저장소 원본과
   SHA256이 같음을 준비 시 확인했다. 시나리오 변경 후 수신이 멈추면 설정을 다시 확인한다.
4. 전송은 UDP, IP는 모두 `127.0.0.1`이다. LiDAR `2368`, 전방/좌전방/우전방/후방
   카메라 `9291/9293/9295/9297`, GPS `9301`, IMU `9303`, Ego `9110`,
   Competition `9099`, Object `7505`, Collision `9092`를 사용한다.
5. LiDAR 15Hz 설정, 카메라 30Hz·1280×720·JPEG Q90을 유지한다. ROS 드라이버도
   900 RPM·첫 패킷 stamp·cut_angle 비활성 상태를 유지한다. `npackets=50`으로
   바꾸거나 deskew를 켜지 않는다. native SaveSensorData/annotation 저장과
   rosbridge는 이번 UDP 녹화의 필수 경로가 아니다.

### 실행할 최소 ROS 구성

각 터미널에서 저장소 루트로 이동하고 runtime을 source한다. `ajou` alias와 `rc`를
이미 사용하는 경우 첫 번째 터미널은 기존 `rc`로 대신해도 된다. 다른 master/UDP
수신기를 중복 실행하지 않는다. 풀스택·validator 실행·BEVFusion·tracker·RViz는 필요 없다.
다른 에이전트의 세션이 활성화되어 있으면 먼저 조율한다.

```bash
# 터미널 1
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
roscore
```

```bash
# 터미널 2: 제어 송신기를 제외한 기존 UDP 수신 구성
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  network_config:="$AJOUNICE_ROOT/configs/network/ego_network_competition_local.json" \
  enable_control:=false
```

정적 센서 TF는 저장한 센서 설정으로 복원할 수 있으므로 별도 `sensing`/localization
기동은 필수가 아니다. 이미 정상 TF/localization이 실행 중이면 토픽도 함께 기록한다.
추가 제어 노드나 전역 시간 설정을 자동으로 변경하지 않는다.

```bash
# 터미널 3: 우선 수신 검사만. 파일 생성·시나리오 로드·운전 없음.
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
python3 scripts/rosbag/record/record_lidar_packet_probe.py --check

# 해당 시나리오를 로드하고 3~5초 안정화한 뒤 각각 녹화
python3 scripts/rosbag/record/record_lidar_packet_probe.py --case static --duration 30 --run
python3 scripts/rosbag/record/record_lidar_packet_probe.py --case highway --duration 60 --run
python3 scripts/rosbag/record/record_lidar_packet_probe.py --case curve --duration 45 --run
```

마지막 세 명령을 한꺼번에 실행하지 않는다. 각 시나리오를 로드하고 다음 명령을
실행한다. 이동 두 장면은 녹화 동안 Keyboard로 수동 운전해야 한다. 실행기는 운전하지
않는다. 운전을 별도 자동화하려면 제어 방법·안전 정지부터 따로 검증한다.

### 저장·검사와 이후 순서

`rosbags/eval/perception/lidar_packet_probe/<시간_case>/`마다 `raw.bag`,
`capture.json`, `audit.json`을 보존한다. 필수 입력은 raw `VelodyneScan`, PointCloud2,
카메라 4개 JPEG, IMU, Ego, Competition, Object GT다. Collision/GPS/TF 및 양쪽
perception diagnostic namespace는 발행 중이면 추가 기록한다. CollisionData는
이벤트성일 수 있어 무수신만으로 오류라고 하지 않는다.

수신 검사는 토픽 이름만 보는 것이 아니라 실제 메시지 두 개와 타입을 확인한다.
현재 live 드라이버 설정, 센서·네트워크 JSON, 선언한 시나리오, calibration 원문과
SHA256도 저장한다. 파일은 덮어쓰지 않는다. 로드한 시나리오와 날씨는 사용자 선언이며
실시간으로 읽어 검증한 값이 아니다. 종료 검사는 raw scan의 51×1206 byte 및 첫 패킷
stamp 계약을 확인한다. 이는 최소 입력 검사이고 주행 성공·센서 무누락 인증은 아니다.

녹화 후 MORAI와 UDP 수신기를 끄고 기존 bag 시간축 조사 → 동일 패킷의 50/51/회전각
재조립 비교 → 같은 가중치로 검출·추적 비교 → 변경 채택 여부를 정한다. 기존 bag에는
원시 패킷이 없는 것이 있어 이번 녹화가 필요하다. 로컬 단일 PC 시험은 팀장님
4090·LAN 풀스택의 237ms 지연 원인을 확정하는 시험이 아니다. 기존 학습 데이터와
production 설정은 이 준비 작업으로 변경하지 않았다.

준비 검증: 단위·합성 rosbag 검사 9개 통과, 문서 링크 검사 통과, MORAI에 복사한
세 시나리오의 원본 일치 확인. 실제 센서 녹화는 아직 하지 않았다. 준비 시 ROS master가
꺼져 있어 live 검사 실행기는 노드를 시작하지 않고 안내와 함께 종료했다.
