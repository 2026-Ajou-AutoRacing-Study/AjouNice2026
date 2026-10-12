# 시나리오 검증기 짧은 사용법

## rosbag ON/OFF·토픽 선택 (2026-10-13)

GUI 실행 탭의 **rosbag 녹화**는 `끄기 / 경량 / 기본 / 인지 분석 / 직접 선택`입니다. **화면 영상 녹화**는 별도 체크입니다. 직접 선택은 **토픽 선택… → 검색 또는 그룹 → 표시된 토픽 일괄 선택/해제 → 적용 → YAML 저장** 순서입니다. 빈 목록 대신 bag 끄기를 사용하세요.

bag OFF에서는 rosbag 프로세스를 띄우지 않으며 성공/실패 판정·JSON·CSV는 유지합니다. 설정은 다음 새 실행에 적용됩니다. 기존 기록 재개는 당시 사본의 설정을 사용하고, 주행 중 변경은 막습니다. bag OFF가 MORAI 센서 생성·인지 추론 부하를 없애지는 않습니다.

YAML에서 직접 수정할 때:

```yaml
recording:
  bag_enabled: false   # 생략 시 기존처럼 true. 영상과 독립.
  video:
    enabled: false
```

직접 선택은 `bag_enabled: true`와 `selected_topics: [/Ego_topic, /CollisionData, /planning/trajectory]`를 사용합니다. 이 목록은 `topic_profile` 대신 기록 대상을 지정하며, `include_sensors: true`라면 원본 센서 5개가 추가됩니다. 인지 분석 모드는 기존처럼 원본 센서+영상 필수이고 실제 메시지 수를 검사합니다. 일반/직접 선택 목록에 토픽이 있다는 것만으로 발행·녹화됐다고 판단하지 마세요.

**이 변경은 dataset/base5 작업 브랜치 기준입니다. 공유 planning 브랜치에 통합하기 전에는 그쪽에도 배포됐다고 가정하지 않습니다.**

설치가 끝나고 **작업하면서 볼 문서는 [딸깍 작업 순서](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md)**입니다. 이 문서는 연결·입력 계약·문제 해결을 조금 더 자세히 다룹니다. [자주 쓰는 버튼을 표시한 화면 5장](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_GUI_HIGHLIGHTS.md)도 참고하세요.

처음 세팅하는 PC는 [설치 가이드](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)부터 따라 해 주세요. 여기서는 ROS 빌드·맵·Docker/E25 준비가 끝난 상태를 기준으로 설명합니다. 공유·시나리오 실행 브랜치는 `planning/autoware-universe-to-ros1`입니다. 검증기를 쓰려고 다른 개발 브랜치로 이동할 필요 없습니다.

**이미 Docker·BEVFusion 이미지·E25까지 준비한 분은 재설치·재다운로드 없이 아래 1번부터 하면 됩니다.** 기존 `.env.bevfusion.local`과 가중치는 유지하고 GUI 환경 검사에서 실제 사용 경로를 확인합니다. 설치 가이드 5·6장을 다시 실행하지 마세요. 첫 실행 전에 담당 미션의 `base/README.md`를 한 번 읽어 주세요. [미션별 안내 목차](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/base_ws/src/planning_addons/planning_scenario_validator/config/missions/README.md)에서 고르면 됩니다.

ROS·맵은 준비했는데 Docker/E25만 없는 Ubuntu 20.04 PC는 `bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --bevfusion`으로 설치·Drive 다운로드·이미지 등록·GPU 검사를 한 번에 진행합니다. 본인 Google 로그인과 sudo가 필요하고, Docker 그룹 신규 추가 시 GUI 사용 전 재로그인합니다. 자세한 제한은 설치 가이드 5-0을 참고하세요.

## 1. MORAI 연결하고 GUI 켜기

업데이트가 올라왔으면 캠페인·GUI·수동 스택을 정상 종료한 뒤 이 브랜치를 pull하고 **planning → base 둘 다 증분 빌드**합니다. 실패 시 중단하는 복붙 명령은 [설치 가이드 4-1](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md#4-1-다음부터는-pull-후-증분-빌드)에 있습니다. build_all.sh만으로는 이미 설치된 planning이 갱신되지 않을 수 있습니다. 평상시 build/install을 지우지 않습니다.

증분 빌드는 변경된 파일과 영향받는 의존 대상을 다시 컴파일하며 패키지 순회·설치 복사는 다시 할 수 있습니다. 빌드 후 환경을 다시 source하고 노드를 재실행하세요. 업데이트 전후 비교는 `새 실험 시작`으로 결과를 분리하며 기존 녹화는 남겨둡니다.

```bash
cd ~/AjouNice2026
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --gui
# 작은 정적 장애물 연결 확인 예제도 쓸 경우 한 번 실행
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
./planning_validator_gui
```

`install.sh`가 출력한 validator 전용 Ego Network와 YAML의 Sensor JSON을 MORAI에서 Load·Connect합니다. Sensor는 보통 `configs/sensors/Sensor_competition_local.json`입니다. 생성된 Ego Network는 MR SaveFile에도 복사됩니다. 일반 주행용 Network로는 자동 시나리오 전환이 안 됩니다.

수동으로 켜놓은 udp·측위·인지·planning·제어 터미널은 먼저 종료해 주세요. 검증기가 스택을 켜고 끕니다. MORAI 자체는 켜둡니다. GUI의 `연결 JSON 경로 복사`로 정확한 파일 경로를 확인할 수 있습니다.

## 2. 내 YAML 준비하기

**일상 옵션은 GUI에서 수정·저장하고, 회색 고정값만 VSCode에서 수정합니다.**

1. **내 미션 선택하기 → 담당 미션 → 실험 YAML의 예제 / example.yaml → 베이스 → 내 작업 폴더 복사**입니다. 복사 후 목록은 내 작업 파일로 바뀝니다. base에서 바로 실행하지 마세요. 옆 README에 시작점·목적지·배우 배치 의도가 있습니다.
2. workspace에 주석 있는 YAML·JSON이 준비됩니다. 재복사는 기존 파일을 덮어쓰지 않고 엽니다.
3. 기본 탭의 이름·파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유를 수정하고 상단 왼쪽 **YAML 저장**을 누릅니다. 같은 개인 파일에 저장하고 고정값·주석은 유지합니다. 처음에는 파일 패턴을 `*_baseline_*.json`으로 두 장면만 고를 수 있습니다. 패턴을 바꾸면 기존 개별 선택 목록은 해제됩니다.
4. **YAML은 선택한 미션의 목록에서만 고릅니다.** 경로 입력·붙여넣기는 없습니다. 새 파일은 해당 미션 workspace에 저장 → 불러오기/새로고침 → 실험 YAML에서 선택합니다. VSCode에서 현재 파일을 Ctrl+S로 수정한 경우에도 새로고침을 눌러 표시를 갱신합니다.
5. **YAML 저장 → 설정 검사 → 새 실험 시작**. 검사·새 실행 전에도 GUI 변경분을 저장합니다. **‘여기부터 수정 X’ 아래 목적지·시간·네트워크는 그대로 두세요.** 랜덤 범위·Python 환경·회색 고정값은 YAML 편집 (VSCode)에서 수정·저장 후 다시 불러옵니다. 같은 항목을 VSCode와 GUI 양쪽에서 바꾸면 재불러오도록 안내합니다.

상단 왼쪽 세 버튼은 YAML 저장·불러오기/새로고침·새 실험 시작입니다. 설정 검사·실험 기록·결과 폴더는 오른쪽 보조 기능 메뉴에서 선택합니다. 중단·모니터 모드는 바로 누를 수 있습니다.

일부 JSON은 YAML의 scenarios.include에 파일명을 적습니다. include가 없으면 glob과 일치한 모든 파일을 실행합니다. 빈 목록·없는 파일·하위경로는 오류이며 전체로 대체하지 않습니다.

고주로 반복용 **highway/base/random_example.yaml**은 기존 JSON 2개 × repeat 10 = 총 20회입니다. NPC 목표 속도 50–60km/h, path offset −10–10%를 회차별 추출하며 Ego/객체 배치·경로·차종 조건은 바꾸지 않습니다. 공식 본선 범위가 아니라 시작용 값입니다. randomization.enabled=false면 원본 그대로 반복하고 seed 변경은 다른 추출 조합입니다. 같은 seed/JSON/회차는 재시도해도 같은 파라미터지만 MORAI의 native 차종/physics는 seed로 고정되지 않습니다. 각 회차의 input.json/결과 JSON에 추출값, scenario.json에 실제 로드 입력을 남깁니다.

YAML 하나의 목적지·시간·스택은 모든 장면에 공통입니다. 목적지가 다른 미션을 섞지 마세요. 개인 YAML·JSON은 workspace에 두면 Git 제외입니다. base의 example·공유 비교 YAML은 추적됩니다. 파일 삭제·이름 변경은 **내 작업 폴더 열기**에서 하고 열린 파일을 지웠으면 다른 YAML을 여세요. 기존 결과는 남습니다.

### planning 팀의 한두 장면 대조

개인 YAML을 불러온 뒤 **planning 빠른 테스트** 탭에서 이름 검색·Ctrl/Shift로 JSON 1–2개를 선택합니다. od_gt/bevfusion, maptf_morai/localization, 신호등 on/off를 대조한 뒤 **선택한 JSON 빠른 실행**입니다.

원본 YAML은 바꾸지 않고 별도 임시 설정과 결과를 만듭니다. 반복은 1회, 랜덤 파라미터·Drive 업로드는 off입니다. 목적지·시간·네트워크·녹화는 저장된 YAML을 그대로 씁니다. 본 캠페인의 랜덤 검증은 일반 새 실험 시작으로 실행하세요. 환경 검사·ROS 수신 검사·연결 JSON 경로 복사도 이 탭에 있습니다.

## 3. YAML 여러 개 걸어놓기

1. 각 조건을 개인 YAML로 저장합니다. 예: 같은 정적 장면의 GT 대조, BEVFusion, 다른 목적지.
2. `실행 목록 / 배치` 탭에서 `YAML 여러 개 추가`로 저장한 개인 파일들을 선택합니다. 현재 저장된 파일을 넣으려면 `현재 YAML 추가`를 누릅니다.
3. 필요하면 목록에서 항목을 선택하고 `목록 관리` 메뉴의 ↑/↓로 순서를 바꿉니다. `전체 검사`로 경로·설정·중복 여부를 확인합니다.
4. 같은 Sensor/Ego Network를 MORAI에 연결한 상태에서 `새 배치 시작`을 누릅니다. 기존 배치를 이어갈 때만 `배치 재개`를 누릅니다.

배치는 YAML들을 **순서대로 한 번에 하나씩** 실행합니다. 다른 배치를 넣는 구조는 아닙니다. 목적지·시간·GT/BEVFusion·신호등 설정은 YAML마다 달라도 되지만, 배치 중 MORAI Network나 시뮬레이터를 바꾸는 조합은 거절합니다. 같은 YAML은 중복 추가하지 않습니다. 다른 YAML의 실험 이름이 같아도 실행 결과 폴더는 별도로 생성됩니다.

모니터 바에는 `배치 2/5 · 캠페인 이름`, 장면 진행, 단계·시간·거리·속도·충돌이 나옵니다. RViz는 왼쪽 위, 모니터 바는 왼쪽 아래, MORAI는 오른쪽 전체 높이입니다. 영상은 전체 화면이므로 다른 창으로 가리거나 화면을 잠그지 마세요.

예상 시간·용량은 추정입니다. 제한시간에 준비·마감 시간을 더한 값이며, 용량은 센서 제외 기본 녹화의 과거 비율을 사용합니다. 센서를 포함하면 훨씬 커지므로 첫 두 장면을 보고 공간을 잡으세요.

### 전체 코스 주행

전체 코스 시험은 GUI에서 **전체 코스 주행 → 예제 / sample_only.yaml → 베이스 복사**부터 시작합니다. 샘플 하나를 확인한 뒤 같은 미션의 `example.yaml`을 복사하면 객체 종류 변형 포함 4장면입니다. 신호등 on·localization·BEVFusion, 주행 제한 600초/장면입니다. [전체 코스 README](../../base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/base/README.md)의 경유점·출발선 직전 종료와 live 미검증 범위를 먼저 확인하세요.

## 4. 중단·재개·다시 처음부터

- `중단`을 누르고 bag·영상 마감과 스택 정리가 끝날 때까지 기다립니다. 그 뒤 다음 YAML은 시작하지 않습니다.
- 단일 실험은 `실험 기록 / 재개` 탭에서 시간·진행률·P/F/E를 보고 기록을 선택한 뒤 `선택 실험 재개`를 누릅니다. 현재 화면의 편집값이 아니라 보관된 입력 사본을 사용합니다.
- 다시 켜도 실행 목록은 남습니다. `배치 재개`를 누르면 완료한 YAML과 PASS/FAIL 장면은 건너뛰고 미완료부터 진행합니다.
- 주행 FAIL은 조사할 결과라 다음 YAML로 넘어갑니다. 시스템 오류나 미완료 종료는 배치를 멈추므로 로그 확인 후 재개합니다.
- 원본 YAML·JSON·Network·토픽 파일은 수정해도 됩니다. 이미 시작한 실험/배치는 **입력 사본**으로 재개합니다. 결과 폴더의 사본이나 실행 목록을 변조하면 재개가 거절됩니다. 실제 MORAI 연결은 재개할 실험의 Network와 맞춰야 합니다.
- `목록 초기화`는 **목록만** 비웁니다. 기존 YAML·bag·영상·판정은 삭제하지 않습니다. 이전 배치는 `이전 목록 열기`에서 결과 폴더의 `queue.json`을 골라 다시 열 수 있습니다.
- 같은 조건을 처음부터 다시 시험하려면 `새 실험 시작` 또는 `새 배치 시작`을 누릅니다. 개인 YAML은 복사하지 않고 새 결과와 입력 사본만 생성합니다. `새 배치 준비`는 현재 YAML 목록을 재사용하도록 재개 경로만 해제합니다.

단일 실험에서만 실패 장면도 다시 돌리려면 기록을 선택하고 `재개 시 FAIL도 재실행`을 체크한 뒤 재개합니다. 새 실험은 모든 장면을 처음부터 실행하므로 이 체크를 사용하지 않습니다. 배치에는 적용되지 않습니다.

이전 고정 이름 결과는 그대로 보존합니다. 목록에 `(이전)`으로 표시되며, 당시 원본 YAML/JSON이 변경되지 않았을 때만 재개할 수 있습니다. 새 실험부터 입력 사본 보호가 적용됩니다.

## 5. 결과 확인·공유

```text
planner_logs/
├─ validation/<실험 이름>_<YYMMDD_HHMMSS>/
│  ├─ execution.json            # 실행 ID·원본 경로·입력 hash
│  ├─ config_snapshot.yaml      # 실제 실행/재개할 설정
│  ├─ source_config.yaml        # 원본 YAML 그대로 보관
│  ├─ scenarios/ · network/ · topic_profile.json # 입력 사본
│  ├─ summary.csv               # 장면별 결과
│  ├─ campaign_state.json
│  └─ success/ · failure/ · error/  # bag·MP4·JSON·events
└─ validation_batches/<배치 ID>/
   ├─ summary.csv               # YAML별 진행·P/F/E 합계
   ├─ state.json
   └─ queue.json                # 이전 배치 열기에 사용
```

GUI에서 `배치 CSV`·`배치 결과 폴더`로 열 수 있습니다. CSV는 Google 스프레드시트의 파일 → 가져오기로 공유하면 됩니다. 실패 장면을 더블클릭하면 관측 근거와 조사 우선 단계를 볼 수 있습니다. **의심 단계는 확정 원인이 아닙니다.**

Drive 업로드는 각 YAML의 옵션입니다. 기본은 꺼짐이며, 켜면 `AjouNice2026/planning_validator/<실험 이름>_<YYMMDD_HHMMSS>/`에 선택한 자료와 실험 CSV가 올라갑니다. basic/bag/video/full은 업로드 대상만 바꾸며 로컬 원본은 보존합니다. **배치 CSV는 자동 업로드 대상이 아니므로 필요하면 별도 공유합니다.** 업로드 오류와 주행 FAIL은 구분됩니다.

실행 목록은 `~/.config/ajounice/planning_scenario_validator/batch.json`에 자동 저장됩니다. 이 파일은 원본 YAML 경로와 배치 결과 위치를 기억하며, 실제 재개 조건은 결과 폴더의 입력 사본에 있습니다. `XDG_CONFIG_HOME`이 있으면 그 경로를 사용합니다.

## 안 되면

### 모델 선택·인지 분석 녹화

GUI `객체 탐지: bevfusion` 아래의 **BEVFusion 모델**에서 선택합니다. 기존 YAML에 `stack.bevfusion_model`이 없으면 **E25 production**입니다. E24는 `base5_e24`를 명시적으로 선택해야 하며 V/P/O threshold는 0.40/0.10/0.10입니다. `base5_e9`는 0.40/0.15/0.10의 **미보정 비교 후보**입니다. GUI 수정 후 YAML 저장·재불러오기·배치·재개에도 선택이 유지됩니다.

E25의 `run_bevfusion.sh`와 SHA 보호는 유지합니다. Base5는 별도 실행기가 검증된 checkpoint·상속 config·센서 hash를 검사하고 Docker 서버 + ROS bridge를 소유합니다. AMP·camera 병렬화 on, inference compile off, single-frame·OD velocity off입니다. 원본 compressed JPEG와 ROS 0–255 강도를 전달하고 서버에서만 강도를 한 번 ÷255합니다. Sensor JSON은 실제 실행 사본을 서버와 bridge에 동일하게 전달합니다. canonical detection publisher가 중복되면 실행을 거부하거나 중단합니다.

**전체 코스 주행 → 예제 / base5_e24_analysis_sample.yaml → 베이스 복사 → YAML 저장 → 설정 검사 → 새 실험 시작**으로 샘플 하나부터 봅니다. 연결·route·배우·bag·영상 마감이 확인된 뒤 `base5_e24_analysis_variants.yaml`을 복사해 나머지 3장면을 시험합니다. 기존 4개 JSON의 경로·속도·spawn은 바꾸지 않습니다. 원본 example/sample_only는 E25 기본값입니다.

GUI **녹화 모드 → 인지 분석 (센서+영상)**은 별도 `perception_analysis.json`, LiDAR + compressed 카메라 4개, MORAI/RViz 화면 영상을 켭니다. 기존 basic 프로파일은 그대로 둡니다. 분석에서는 실제 planner avoidance/info/debug 토픽도 정규식으로 수집하고 `/clock`은 발행될 때 기록합니다. **원본 센서 없는 bag은 사후 재추론 불가**입니다.

종료 후 각 장면의 `recording_audit.json`에서 **실제 메시지 수**, 누락 핵심 토픽, 센서 재추론 가능 여부를 확인합니다. 목록에만 있는 토픽은 기록 성공으로 세지 않습니다. route/trajectory 준비 전에 실패한 장면은 해당 후단 데이터가 없어도 그 상태를 그대로 남깁니다. 주행 PASS/FAIL과 녹화 완전성은 별개이며 CSV에도 `recording_complete`, `sensor_replayable`, `missing_recorded_topics`가 있습니다.

결과 폴더의 `model_contract.json`/`model_runtime.json`, `model_configs/`, `model_process_receipt.json`, `model_launch.json`, `stack_logs/commands.json`에 checkpoint 경로·SHA, 실제 config/센서와 hash, threshold·NMS·AMP·병렬화·compile·Docker image ID·실행 명령이 남습니다. 장면별 input.json에는 Git SHA가, ros_parameters.yaml에는 실제 ROS 설정이 남습니다. YAML·원본/실제 로드 시나리오·녹화 목록도 보존합니다. 모델·core 설정·Docker image가 달라지면 같은 결과에 resume할 수 없습니다. 새 실험을 시작하세요.

5060 Ti + MORAI 로컬 검증은 연결·기능 검사입니다. 대회용 4090/LAN의 처리율·지연·주행 안전성을 인증한 결과가 아닙니다. tracker의 0.5/1초 기억과 planner 설정은 이번 연결 작업에서 바꾸지 않습니다.

- 설정 오류: `설정 검사` / 배치 `전체 검사`부터 봅니다.
- 설치·가중치·녹화 오류: `환경 검사`의 FAIL과 해결 방법을 봅니다. 기본 검사는 실제 GPU 추론이나 MORAI 연결을 보장하지 않습니다.
- 수신 오류: 스택 수신 중 ROS 토픽과 포트를 확인합니다. 실행 중에는 GUI가 실시간 주행 값을 보여주고, 별도 `ROS 수신 검사`는 다른 실행이 없을 때만 켤 수 있습니다.
- TIMEOUT·충돌·경로 실패: 없애거나 성공으로 덮지 말고 JSON·로그·영상을 남깁니다. 그 실패를 찾는 것이 검증기의 목적입니다.

새 장면을 만드는 방법은 [제작 가이드](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md), 다른 AI가 이어서 볼 구현 계약·검증 범위는 [구현 문서](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/scenario_validator_automation.md)를 참고하세요.
