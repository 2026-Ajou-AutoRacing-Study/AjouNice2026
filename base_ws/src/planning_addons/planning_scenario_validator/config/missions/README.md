# 팀 미션별 검증 스타터

목적은 학습 데이터 생성이 아니라 **정상 상황의 회귀와 대응하지 못하는 상황 발견**이다.
사람 이름 대신 미션 폴더를 공유한다. 담당자가 바뀌어도 그대로 이어받는다.

새 PC 세팅부터 필요한 팀원은 [첫 설치·실행 안내](../../../../../../docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)를 먼저 참고한다.
시나리오와 YAML을 직접 만드는 방법은 [제작 가이드](../../../../../../docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md)에 정리했다.

**YAML 하나는 공통 목적지·경유점·제한시간·스택 설정 하나다.** 선택된 모든 JSON에 같은 설정을 적용한다. 다른 목적지나 조건을 시험하려면 YAML을 하나 더 만든다. 장면별 `.case.yaml`은 사용하지 않는다.

| 폴더 | 내용 | 시나리오 | 주행 제한/장면 |
| --- | --- | ---: | ---: |
| `roundabout` | 샘플 spawn 경로, 고정 차종 변경, 속도·간격·횡 offset | 24 | 75초 |
| `static_obstacle` | 샘플 정적 구간 좌우/다객체, 정지 차량, 스쿠터 | 24 | 65초 |
| `dynamic_obstacle` | 샘플 사거리 양방향 보행자·스쿠터, 도로 교차 차량 | 24 | 70초 |
| `gps_shadow` | 공식 음영 영역의 정적 장애물·정지 차량 회피 | 24 | 90초 |
| `highway` | 원본 3개 spawn 위치·종료 링크, MiddleSize RandomMode | 24 | 90초 |

미션 폴더마다 `base/`와 `workspace/` 두 폴더를 둔다.

```text
config/missions/highway/
├─ base/                 # 공유 베이스: Git 추적, GUI에서 덮어쓰지 않음
│  ├─ example.yaml
│  ├─ manifest.json
│  └─ scenarios/         # 실행할 MORAI JSON만
└─ workspace/            # 개인 작업: .gitignore만 Git 추적
   ├─ .gitignore
   ├─ highway_starter_<시간>.yaml
   ├─ highway_other_goal.yaml # 다른 목적지/조건은 다른 YAML
   └─ scenarios/         # 베이스 복사 버튼으로 준비 후 자유롭게 수정
```

`base/`는 `example.yaml`, `manifest.json`, `scenarios/*.json`으로 구성한다.
각각 baseline 2개 → 일반 변형 14개 → stress 8개 순서다. 이름은 검증 분류이며 성공 보장이 아니다.
Manifest는 실행용 시나리오가 아니므로 `base/scenarios/` 밖에 둔다.
공유 YAML·JSON·manifest는 Git에 추적한다. 개인 YAML과 녹화 결과는 Git에서 제외한다.

## 가장 쉬운 실행

1. 저장소 루트에서 패키지 `install.sh`를 실행하고, MORAI에 validator 전용 Ego Network와 `Sensor_competition_local.json`을 Load·Connect한다. 카메라와 LiDAR는 **실제로 켜둔다**.
2. `./planning_validator_gui`를 열고 예제 선택 목록의 `missions/<미션>/base/example.yaml`을 고른 뒤 **베이스 → 내 작업 폴더 복사** 버튼을 누른다. 시나리오 JSON을 workspace로 복사하고 새 이름의 YAML을 만들어 바로 연다. 같은 이름의 기존 시나리오는 덮어쓰지 않는다.
3. 처음에는 파일 패턴을 `*_baseline_*.json`으로 바꿔 두 기준선을 실행한다. 실제 배우 위치/움직임·목적지·센서 주기가 맞는지 본다. 복사 없이 바로 저장/실행하면 workspace에 YAML만 만들고 공유 base 시나리오를 참조한다. 시나리오를 수정하려면 복사 버튼부터 사용한다.
4. 확인 후 패턴을 `*.json`으로 바꾸고 새 캠페인으로 저장·설정 검사·실행한다. 기존 결과는 자동 resume된다. 기준선을 통과시키려고 판단 파라미터나 장애물 위치를 억지로 바꾸지 않는다.
5. FAIL은 조사 대상이다. `summary.csv`, 해당 run JSON/이벤트/영상으로 관측 근거를 확인한다. 시스템 오류나 시나리오 배우 오작동은 planning 실패와 구별한다.

기본은 localization + BEVFusion + 신호등 off, basic bag + **원시 센서 녹화 제외** + 10fps 전체 화면 split 영상이다.
신호등 자체가 아닌 해당 미션을 격리해 보는 예제이며, 신호등 on 통합 시험을 대체하지 않는다.
GT 대조는 GUI 탐지기를 `od_gt`로 바꿔 별도 캠페인으로 저장한다. 측위 원인 분리는 `maptf_morai`를 사용하되 대회용 측위 검증으로 오해하지 않는다.
GUI 없이도 저장소 루트에서 `./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/base/example.yaml`로 공유 기준선을 실행한다. 개인 변형은 workspace의 YAML을 지정한다.

### YAML 저장 규칙

- 미션 base를 열면 저장은 같은 미션 workspace에 하고, base 원본은 보호한다. workspace YAML은 미실행 상태에서는 같은 캠페인 이름으로 수정한다.
- 이미 실행한 YAML을 설정 그대로 열면 동일 캠페인을 재개한다. 설정 변경 시 확인 후 새 캠페인 이름의 YAML을 같은 workspace에 만든다. `campaign.yaml` 하나를 계속 덮어쓰는 구조가 아니다.
- 현재 실행 계약은 `schema_version: 2`다. 이전 v1 YAML은 GUI에서 열어 새 캠페인으로 저장할 수 있지만, 예전 결과와 새 계약을 섞어서 resume하지 않는다. 기존 결과는 삭제하지 않는다. 남아 있는 `.case.yaml`은 실행 시 경고하고 무시하며 GUI에서 실행용 YAML로 열 수 없다.
- ‘베이스 복사’를 다시 누르면 기존 시나리오는 유지하고 누락된 베이스 파일만 복사하며, 새 캠페인 YAML을 하나 더 만든다. 기존 사용자 설정을 초기화하고 싶다면 새 시나리오 폴더를 직접 준비하고 새 YAML을 사용한다.
- `내 YAML 열기 / resume`는 선택한 미션의 workspace에서 시작한다. 기존 `config/users/<사용자>/` YAML도 열 수 있고 그곳의 기존 저장 방식은 유지한다. 기존 파일을 강제로 이주시키지 않는다.
- 마지막으로 연/저장한 YAML 기억은 workspace에서도 동일하다. 예전 `config/examples/missions/...` 경로는 해당 공유 base의 새 위치로 읽기 호환 처리한다. 기존 YAML·결과 스냅샷을 소급 수정하지 않는다.
- 팀원마다 별도의 로컬 workspace를 가진다. 개인 YAML·시나리오는 자동으로 Git에서 제외된다. 팀에 공유할 사례는 검토 후 base/생성기에 선별 반영한다. 녹화 결과는 workspace가 아닌 기존 `planner_logs/validation/`에 남는다.

## 주최측 답변 반영과 가정

2026-09-21 주최측 답변: 고주로·로터리 속도는 샘플과 동일하지 않음. 고주로 RandomMode 여부는 미정이며 사용한다면 MiddleSize만 사용. 로터리는 고정 차종이며 차종은 비공개.

- 고주로: 원본 3개 spawn/종료 링크 그대로, `listVehicleLengthType: [2]` 유지. 목표 30–80 km/h, 초기 0/10/20/목표속도, spawn 간격 3–14초를 재현 가능한 시나리오별 값으로 변형한다. 같은 차선이 항상 같은 속도가 아니게 한다.
- 로터리: 원본 spawn/종료 링크 그대로. 시나리오 안에서는 한 차종으로 고정하고 K5/Ioniq/Ioniq5 등으로 시나리오 사이의 차종만 바꾼다. 목표 15–40 km/h와 spawn 간격을 바꾼다.
- 위 수치·차종은 **검증용 가정이며 공식 범위·MiddleSize 전체 목록을 확정한 것이 아니다**. 샘플 값의 기준선도 별도로 포함한다.
- 음영: 동적 끼어들기는 넣지 않는다. 차량도 **정지 객체**로 배치한다. 동적 장애물 미션은 음영이 아니라 샘플 사거리다.
- 정적 미션: 샘플·기존 판단 검증 구간을 사용한다. 일반 객체는 scale 1, 원본 scaled CargoBox 기준선은 그대로 유지한다. WoodBox는 별도 확인 전까지 넣지 않는다. 스쿠터/자전거는 학습 포함 여부와 무관하게 인지·판단 robustness 사례다.
- 목적지는 map 좌표, 배우 배치는 MORAI 좌표다. 좌표 변환은 저장소의 TF 설정을 사용한다. 로터리·고주로 목표는 기존 코스상의 점, 사거리 목표는 지도 도로의 점이다. 지도가 바뀌면 다시 검사한다.

## 검증 범위와 후속 제작

**현재 이 120개 묶음은 오프라인 형식·spawn 링크/인덱스·설정 로딩 검증만 완료했고 live 주행은 미검증이다.**
기존 정적 예제의 live 검증을 새 변형 전체의 검증으로 확대 해석하지 않는다.
특히 이동 스쿠터, 다른 pedestrian model의 animation, 교차 차량의 `activeDistance`와 도로 경로 동작은 MORAI에서 먼저 확인한다.
이동 배우가 안 움직였는데 정적 회피 PASS한 경우 동적 대응 성공으로 집계하지 않는다.
현재 validator는 충돌·시간·목적지로 판정하며 미션별 양보/차선침범/배우 작동/속도 규정 준수의 자동 판정을 모두 제공하지는 않는다.
회피/추월 성공을 주장하려면 영상과 궤적을 함께 확인해야 한다.

팀원은 workspace의 JSON을 복사하여 한두 조건씩 바꾸고, 공통 조건을 담은 캠페인 YAML에서 해당 폴더·패턴을 선택한다. 장면마다 보조 YAML을 만들지 않는다. 목적지나 제한시간이 다르면 새 캠페인 이름의 YAML로 분리한다.
새 시나리오는 workspace/scenarios 또는 그 아래 별도 실험 폴더에 두고 GUI의 시나리오 폴더를 바꾼다. 하위 폴더를 자동 재귀 실행하지는 않는다. 실행 후 같은 캠페인 입력을 수정하지 말고 새 이름으로 저장한다.
담당 AI에게는 이 README와 해당 base/manifest.json을 먼저 읽히고, 원본 sample anchor와 일반/스트레스 균형을 유지하도록 요청한다.
공유 스타터를 수정하려면 생성기를 수정한 뒤 재생성한다. 기존 JSON 손수 수정 후 생성기를 돌리면 덮어써지므로 개인 변형은 별도 폴더에 보관한다.

```bash
python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_mission_starters.py
python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_mission_starters.py --check
```

## 시간·저장 공간

24개를 모두 timeout까지 돌리면 주행과 기존 준비/정리 시간 합계는 미션별 대략 40–55분이다. PASS가 빨리 나면 더 짧아지고, 오류·업로드·첫 모델 시작은 추가 시간이 든다.
반나절 검토는 여러 미션, GT/BEV 대조, 실패 영상 검토를 합친 작업량이지 동일 장면을 무의미하게 반복하는 설정이 아니다.
이전 센서 제외 녹화의 관측치인 bag 약 50 MiB/분 + 영상 약 10 MiB/분을 적용하면 40–55분은 약 2.3–3.2 GiB다. 실제 토픽·화면·정지시간에 따라 달라지며 새 미션의 실측치는 아니다.
처음 두 장면을 녹화해 실제 크기를 확인한다. 기본 업로드는 off이며 공유 시 basic(CSV·JSON) 또는 video를 선택한다.
시작과 장면 사이에 여유 공간 10 GiB 미만이면 시스템 오류로 멈추고 결과를 보존한다. 이는 **장면 사이 검사**이지 녹화 중 디스크 소진을 완전히 막는 보장은 아니다. 긴 단일 장면·다른 프로그램의 디스크 사용은 별도로 관리한다. 공간 확보 후 동일 YAML로 재개한다.
