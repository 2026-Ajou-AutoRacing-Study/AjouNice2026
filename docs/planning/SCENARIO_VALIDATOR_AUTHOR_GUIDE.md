# 시나리오랑 YAML 직접 만드는 법

설치가 끝난 팀원이 새 검증 케이스를 만드는 안내입니다. 처음 세팅하는 경우에는 [설치·실행 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)부터 보면 됩니다.

우리 목적은 데이터셋 수집이 아니라 **일반 상황에서도 잘 도는지, 어떤 상황에서 못 도는지 찾아 녹화와 근거를 남기는 것**입니다. 실패하는 시나리오도 필요한 결과입니다.

## 1. 파일 두 종류만 구분하면 됩니다

- **시나리오 JSON:** 시작 위치, 객체 배치, NPC 속도·spawn 같은 MORAI 장면입니다.
- **캠페인 YAML:** 어떤 JSON들을 실행할지, 어디까지 갈지, 몇 초 안에 갈지, 어떤 스택으로 녹화할지 정합니다.

**YAML 하나에서는 모든 장면이 같은 목적지·경유점·제한시간·도착 방식으로 실행됩니다.** 객체 위치나 NPC 조건만 달라지는 JSON들은 같은 YAML로 묶어도 됩니다. 다른 목적지·제한시간·스택을 원하면 다른 이름의 YAML을 하나 더 만드세요. `campaign.yaml` 하나만 계속 덮어쓸 필요 없습니다.

만든 YAML 여러 개를 밤새 실행하려면 GUI `실행 목록 / 배치`에 추가하면 됩니다. 배치를 위한 별도 YAML은 만들지 않습니다. 공통 Network를 연결하고 순서대로 실행·중단·재개하는 방법은 [짧은 사용법](SCENARIO_VALIDATOR_RUNBOOK.md)에 정리했습니다. 실행한 배치의 JSON이나 YAML을 바꾸면 재개가 거절되므로 수정 실험은 새 이름으로 분리합니다.

장면마다 `.case.yaml`을 만드는 방식은 없앴습니다. JSON 폴더에 남아 있어도 설정을 덮어쓰지 않고 경고 후 무시합니다. GUI에는 **완성된 캠페인 YAML**을 열어야 합니다. 목적지 두 줄만 적힌 YAML이나 빈 YAML은 열 수 없습니다.

## 2. 내 작업 폴더부터 만들기

저장소 루트에서 실행합니다.

```bash
cd ~/AjouNice2026
./planning_validator_gui
```

1. 담당 미션의 `missions/<미션>/base/example.yaml`을 선택합니다.
2. **베이스 → 내 작업 폴더 복사**를 누릅니다.
3. 같은 미션의 `workspace/`에 JSON 사본과 개인 YAML이 생기고 바로 열립니다.
4. 캠페인 이름을 알아보기 쉽게 정하고 저장합니다. 예: `high_gap_test_01`.

폴더는 이 정도면 충분합니다. 아래 이름은 예시라 그대로 맞출 필요 없습니다.

```text
config/missions/highway/
├─ base/                         # 팀 공유 원본. 직접 수정하지 않기
│  ├─ example.yaml
│  ├─ README.md                  # 시작/끝점·설계 의도·확장 예시·미션 AI 프롬프트
│  ├─ manifest.json
│  └─ scenarios/*.json
└─ workspace/                    # 내 PC 작업. Git 제외
   ├─ .gitignore
   ├─ high_gap_test_01.yaml      # 목적지 A, BEVFusion
   ├─ high_gap_gt_01.yaml        # 목적지 A, GT 대조
   ├─ high_other_goal_01.yaml    # 목적지 B
   ├─ scenarios/*.json          # 목적지 A로 갈 장면들
   └─ other_goal/*.json         # 목적지 B로 갈 장면들
```

YAML은 같은 JSON 폴더를 공유해도 됩니다. 예를 들어 GT 대조는 JSON을 다시 만들지 않고 다른 YAML에서 탐지기만 바꾸면 됩니다. 시작점이 달라도 같은 경유점·목적지까지 정상적으로 갈 수 있는 장면이라면 하나로 묶을 수 있습니다.

`workspace/` 안의 개인 YAML과 JSON은 `.gitignore`로 제외됩니다. 여기에 파일을 만들었다고 팀원 PC에도 생기는 건 아닙니다. 공유할 케이스는 따로 전달하거나, 검토 후 공유 base/생성기에 반영해야 합니다. 녹화는 workspace가 아니라 `planner_logs/validation/<캠페인 이름>/`에 남습니다.

## 3. 시나리오 JSON 만들기

가장 쉬운 방법은 같은 미션의 기존 JSON을 복사한 다음 한두 조건씩 바꾸는 겁니다.

### MORAI에서 직접 만들 때

1. 기준 시나리오를 MORAI에서 Load합니다. 기존 샘플 spawn 경로와 도로 위치를 기준으로 시작하세요.
2. 객체 위치·개수·속도 등을 바꾸고 새로운 이름으로 Scenario를 Save합니다.
3. 저장된 **시나리오 JSON**을 내 `workspace/scenarios/` 또는 별도 장면 폴더에 복사합니다.
4. GUI에서 그 폴더와 파일 선택 패턴을 지정합니다.

원본 공유 base는 그대로 둡니다. 실행기는 시나리오를 별도 설치 사본으로 준비하고 ExternalCtrl 정지 시작을 설정하므로 원본 JSON을 직접 덮어쓰지 않습니다.

시나리오 폴더에는 실제 Scenario JSON만 넣으세요. Sensor JSON, Ego Network JSON, `_MN.json`, manifest는 실행할 시나리오가 아닙니다. `_MN.json`은 MORAI의 네트워크 sidecar이며 폐지한 `.case.yaml`과는 다른 파일입니다. 자동화의 네트워크 설치 처리는 그대로 필요합니다.

### AI로 만들 때도 지킬 것

- 해당 미션 `base/manifest.json`과 기존 JSON을 먼저 읽히세요. 좌표·spawn 링크·DataID를 추측해서 만들지 않도록 합니다.
- 같은 폴더의 `base/README.md`도 읽히세요. 기본 시작점·목적지와 유지/변경 기준, 미션별 추가 실험 순서가 있습니다. GUI 복사 시 workspace README에 공유 안내 링크를 만들며 기존 개인 메모는 보존합니다.
- Ego 시작점은 도로 위, 진행 방향과 일치하도록 둡니다. 목적지까지 연결되는지 먼저 봅니다.
- 한 장면 안 객체 unique ID가 중복되지 않게 합니다. 지원 객체는 기존 예제나 `dataset/morai_od/configs/morai_asset_catalog_260807.json`을 참고합니다.
- 정적 장애물은 실제 정적 미션 구간에서 좌우 위치·종류·개수를 바꿉니다. 음영 구간은 정적 장애물과 정지 차량 중심입니다.
- 동적 미션은 샘플 사거리에서 보행자뿐 아니라 도로를 주행하는 차량도 시험할 수 있습니다. 속도 값을 넣었다고 이동이 보장되지는 않습니다. 경로·trigger·animation이 실제로 동작하는지 확인하세요.
- 고주로·로터리는 샘플의 spawn 위치·주행/종료 경로를 기준으로 속도와 간격을 바꿉니다. 학습 데이터셋처럼 객체를 아무 도로에나 무작위로 깔지 않습니다.
- NPC 초기 속도와 목표 속도는 다릅니다. 초기 0이면 목표 속도까지 가속하는 시간이 있고, spawn 간격은 실제 교통 밀도에 영향을 줍니다. 화면에서 움직임과 간격을 확인합니다.
- 일반 장면을 충분히 넣고 어려운 장면을 섞으세요. 공유 베이스는 기준선 2개·일반 14개·stress 8개입니다. 이 비율을 반드시 맞출 필요는 없지만 극단적인 케이스만 만들지는 않습니다.

주최측은 본선 속도가 샘플과 같지 않다고 답했습니다. 고주로 RandomMode는 미정이며 사용 시 MiddleSize만 예정, 로터리는 비공개 고정 차종입니다. 예제의 속도 범위·차종은 검증용 가정이지 공식 답안이 아닙니다. WoodBox는 별도 확인 전까지 공유 베이스에서 제외합니다.

파일 이름은 짧은 영문·숫자·밑줄을 권장합니다. 예: `soa_left_car_01.json`, `do_cross_vehicle_01.json`, `high_short_gap_01.json`, `ra_slow_dense_01.json`, `hid_soa_pair_01.json`. 파일명에 조건을 넣으면 실패 영상을 다시 찾기 쉽습니다.

## 4. YAML에서 바꿀 부분

새로 빈 YAML을 짜지 말고 **공유 example.yaml 전체를 복사하거나 GUI에서 저장**하세요. 네트워크·녹화·준비 설정까지 포함되어 있어야 합니다.

우선 바꿀 부분은 캠페인 이름, JSON 폴더·패턴, 공통 목적지·시간, 선택 스택입니다. 아래는 고주로 예제의 일부이며 이것만 저장한 파일은 완성된 캠페인 YAML이 아닙니다.

```yaml
schema_version: 2
campaign_name: high_gap_test_01
scenarios:
  directory: base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/workspace/scenarios
  glob: 'high_normal_*.json' # 이 폴더의 파일만 선택. 하위 폴더 자동 재귀 실행 아님
default_case:
  goal:                     # 선택된 모든 장면의 공통 map 목적지
    x: 197.5177
    y: 653.498242
    z: 28.264104
    yaw_deg: -86.688598
  waypoints: []             # 경유점도 모든 장면에 동일하게 적용
  timeout_sec: 90           # 제어기 준비 후 주행 제한시간
  arrival_mode: stop
stack:
  pose_source: localization
  detector: bevfusion       # GT 대조는 od_gt로 바꾸고 새 캠페인 저장
  traffic_light: false
```

경로는 **저장소 루트 기준 상대경로**입니다. YAML 파일이 들어 있는 폴더 기준이 아닙니다. 위 고주로 목적지는 현재 base의 예시이므로 다른 미션에 그대로 쓰면 안 됩니다.

GUI에서 선택한 YAML의 설정이 그대로 불러와지고 저장됩니다. 캠페인은 “YAML로 정의한 한 묶음의 실행”을 뜻할 뿐, 별도의 숨겨진 설정 파일이 아닙니다. GUI 설정 파일에는 마지막에 열었던 YAML 경로만 기억합니다.

### 목적지 좌표가 헷갈릴 때

목적지와 경유점은 **planning의 map 좌표**, 시나리오 Ego/NPC 배치는 **MORAI 좌표**입니다. 시나리오의 객체 x/y를 목적지에 그대로 복사하면 안 됩니다.

처음에는 같은 미션 base 목적지를 유지하세요. 다른 종료점을 만들 때는 캠페인·제어기가 실행 중이지 않은 상태에서 Planviz의 `2D Nav Goal`로 도로 위 목표를 잡아 `/move_base_simple/goal` 메시지를 확인할 수 있습니다. 이 도구는 실제 goal을 발행하므로 주행 중에 좌표 확인용으로 누르지 마세요.

```bash
# 목표를 찍기 전에 실행. 확인 후 Ctrl+C로 종료
rostopic echo -n 1 /move_base_simple/goal
```

메시지의 map position이 목적지 x/y/z이고, orientation은 quaternion입니다. YAML의 `yaw_deg`는 quaternion의 z값이 아니라 주행 방향 각도(deg)입니다. 방향 변환이 어려우면 AI에게 이 메시지와 현행 TF 설정 `configs/tf/morai_lanelet_static_tf.yaml`을 읽고 목표를 구성하도록 요청하세요. map 버전이 바뀌면 도로 위 위치·방향을 다시 확인합니다.

### 제한시간·녹화

- `default_case.timeout_sec`는 준비 후 실제 주행 시간입니다. 스택·시나리오·route 준비 timeout은 별개라 이 값만 줄여도 준비가 빨라지지는 않습니다.
- `stop`은 목표 반경 안에서 속도·유지시간 조건까지 만족해야 도착입니다. `pass_through`는 목표 영역 통과를 보는 별도 모드입니다. 같은 YAML에서 장면마다 다르게 지정하지 않습니다.
- 기본은 `basic` 토픽 + `include_sensors: false` + 영상 on입니다. **센서를 끄라는 뜻이 아니라 bag에 카메라/LiDAR를 안 넣겠다는 뜻**입니다.
- 기본 설정 탭에서는 신호등 on/off만 선택합니다. Python 방식·실행 경로·GPU/CPU 장치는 신호등 환경 탭에서 설정합니다. 팀원 가상환경 설치는 [설치 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)의 7-1을 참고하세요.
- 업로드는 기본 off입니다. 필요한 공유 범위만 고릅니다. 로컬 bag·영상은 업로드 선택과 별개로 보존됩니다.

## 5. 새 케이스는 먼저 한두 개만 실행

1. JSON 한두 개만 선택하는 패턴을 넣고 새 캠페인 이름으로 저장합니다.
2. **설정 검사**를 누릅니다. 이것은 파일·설정 검사이지 실제 주행 성공 보장이 아닙니다.
3. MORAI 네트워크·센서가 연결돼 있고 센서 주기가 정상인지 확인하고 실행합니다.
4. 시작 위치, 객체 크기·위치, 동적 객체가 실제로 움직이는지, 목표 경로가 의도와 같은지 봅니다.
5. 맞으면 전체 JSON을 선택하는 새 YAML/캠페인으로 저장해서 돌립니다.

CLI로는 다음처럼 검사·실행할 수 있습니다. `high_gap_test_01.yaml`은 위에서 만든 개인 파일 이름으로 바꾸세요.

```bash
cd ~/AjouNice2026
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/workspace/high_gap_test_01.yaml --dry-run

# 설정과 MORAI 연결 확인 후 실제 실행
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/workspace/high_gap_test_01.yaml
```

PASS는 현재 충돌·목적지·시간 기준을 통과했다는 뜻입니다. 양보, 차선 침범, 추월 적절성, 속도 규정까지 전부 자동 판정하는 건 아닙니다. 움직여야 하는 배우가 멈춰 있었다면 동적 대응 성공으로 해석하지 마세요. FAIL은 JSON·이벤트·영상으로 원인을 보고, 설치/연결/배우 오작동과 planning 문제를 구별합니다. 성공률을 높이려고 판단 파라미터나 장애물을 임의로 바꾸지 않습니다.

## 6. 중단·수정·재개

같은 v2 YAML과 같은 JSON으로 다시 실행하면 완료한 PASS/FAIL은 건너뛰고 중단·ERROR 장면을 재시도합니다. 기존 FAIL까지 다시 볼 때는 GUI의 **기존 FAIL도 다시 실행**을 선택합니다.

실행 후 목적지·시간·JSON 배치를 바꿨다면 **새 캠페인 이름**을 사용하세요. YAML 내용과 시나리오 hash를 비교하므로 입력을 바꾼 채 기존 결과에 덧붙이는 건 허용하지 않습니다. JSON도 수정 전 사본을 보관하거나 새 장면 폴더로 복사하면 이전 케이스를 다시 확인하기 쉽습니다.

예전 `schema_version: 1` YAML은 GUI에서 열어 확인한 뒤 새 이름의 v2 캠페인으로 저장합니다. 예전 결과는 지우지 않지만, 예전 보조 YAML 실행 계약의 state를 새 계약과 섞어 resume할 수는 없습니다.

## 7. 담당 AI에게 그대로 전달할 내용

```text
planning_scenario_validator로 검증할 개인 시나리오와 캠페인 YAML을 만들어 줘.
먼저 docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md,
패키지 config/missions/README.md, 담당 미션 base/README.md,
base/example.yaml과 manifest.json을 읽어 줘.

공유 base는 수정하지 말고 해당 미션 workspace 아래에 작성해 줘.
YAML 하나는 공통 목적지·경유점·제한시간·스택 조건 하나야.
목적지나 실행 조건이 다르면 다른 캠페인 이름의 YAML로 분리하고,
장면별 .case.yaml은 만들지 마. 예제 전체를 복사해서 완성된 v2 YAML을 만들어 줘.

기존 미션 구간·spawn/종료 경로를 유지하며 일반 상황과 어려운 상황을 섞어 줘.
단순 랜덤 학습 데이터 생성이 아니라 실제 시스템의 실패 케이스를 찾는 목적이야.
지원 asset과 좌표를 확인하고, 정적/동적 배우가 의도대로 동작하는지 구별해 줘.
목적지는 planning map 좌표, 시나리오 객체 배치는 MORAI 좌표라는 점을 지켜 줘.
처음에는 한두 장면으로 설정 검사·실제 동작을 확인할 수 있게 준비해 줘.
planning 파라미터를 바꾸거나 장애물을 옮겨서 성공하도록 맞추지는 마.
기존 실행 결과를 삭제하거나 수정된 입력으로 resume하지 마.
파일 경로, 실행 방법, 각 장면의 검증 의도와 미검증 사항을 함께 알려 줘.
```
