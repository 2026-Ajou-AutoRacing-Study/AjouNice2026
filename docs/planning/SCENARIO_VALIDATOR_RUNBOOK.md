# 시나리오 검증기 짧은 사용법

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

1. 담당 미션의 base/example.yaml을 선택하고 **베이스 → 내 작업 폴더 복사**를 누릅니다. base에서 바로 실행하지 마세요. 옆 README에 시작점·목적지·배우 배치 의도가 있습니다.
2. workspace에 주석 있는 YAML·JSON이 준비됩니다. 재복사는 기존 파일을 덮어쓰지 않고 엽니다.
3. 기본 탭의 이름·파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유를 수정하고 상단 왼쪽 **YAML 저장**을 누릅니다. 같은 개인 파일에 저장하고 고정값·주석은 유지합니다. 처음에는 파일 패턴을 `*_baseline_*.json`으로 두 장면만 고를 수 있습니다. 패턴을 바꾸면 기존 개별 선택 목록은 해제됩니다.
4. VSCode의 YAML **경로 복사 → GUI 상단 Ctrl+V → Enter/불러오기**. 경로 붙여넣기 버튼도 되며 공백·따옴표·~/ 경로를 받습니다.
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

- 설정 오류: `설정 검사` / 배치 `전체 검사`부터 봅니다.
- 설치·가중치·녹화 오류: `환경 검사`의 FAIL과 해결 방법을 봅니다. 기본 검사는 실제 GPU 추론이나 MORAI 연결을 보장하지 않습니다.
- 수신 오류: 스택 수신 중 ROS 토픽과 포트를 확인합니다. 실행 중에는 GUI가 실시간 주행 값을 보여주고, 별도 `ROS 수신 검사`는 다른 실행이 없을 때만 켤 수 있습니다.
- TIMEOUT·충돌·경로 실패: 없애거나 성공으로 덮지 말고 JSON·로그·영상을 남깁니다. 그 실패를 찾는 것이 검증기의 목적입니다.

새 장면을 만드는 방법은 [제작 가이드](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md), 다른 AI가 이어서 볼 구현 계약·검증 범위는 [구현 문서](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/scenario_validator_automation.md)를 참고하세요.
