# 시나리오 검증기 짧은 사용법

처음 세팅하는 PC는 [설치 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)부터 따라 해 주세요. 여기서는 ROS 빌드·맵·Docker/E25 준비가 끝난 상태를 기준으로 설명합니다. 공유 브랜치는 `planning/scenario-validator-automation`입니다.

**이미 Docker·BEVFusion 이미지·E25까지 준비한 분은 재설치·재다운로드 없이 아래 1번부터 하면 됩니다.** 기존 `.env.bevfusion.local`과 가중치는 유지하고 GUI 환경 검사에서 실제 사용 경로를 확인합니다. 설치 가이드 5·6장을 다시 실행하지 마세요. 첫 실행 전에 담당 미션의 `base/README.md`를 한 번 읽어 주세요. [미션별 안내 목차](../../base_ws/src/planning_addons/planning_scenario_validator/config/missions/README.md)에서 고르면 됩니다.

ROS·맵은 준비했는데 Docker/E25만 없는 Ubuntu 20.04 PC는 `bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --bevfusion`으로 설치·Drive 다운로드·이미지 등록·GPU 검사를 한 번에 진행합니다. 본인 Google 로그인과 sudo가 필요하고, Docker 그룹 신규 추가 시 GUI 사용 전 재로그인합니다. 자세한 제한은 설치 가이드 5-0을 참고하세요.

## 1. MORAI 연결하고 GUI 켜기

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

1. 예제 목록에서 `missions/<담당 미션>/base/example.yaml`을 선택합니다.
   **먼저 같은 폴더의 `README.md`를 읽어 주세요.** 기본 시작점·목적지와 미션별 배치 의도·확장 방법·AI 프롬프트가 있습니다. 처음에는 좌표를 그대로 사용합니다.
2. `베이스 → 내 작업 폴더 복사`를 누릅니다. 같은 미션의 `workspace/`에 개인 YAML과 시나리오 사본이 생깁니다.
3. 첫 시험은 파일 패턴을 `*_baseline_*.json`으로 두고 두 장면만 실행합니다. 새 배우가 실제로 움직이는지도 확인합니다.
4. 이름·공통 목적지·시간·측위·탐지기를 확인하고 `개인 YAML 저장`을 누릅니다. 기본은 basic 토픽, 원시 LiDAR·카메라 녹화 제외, 영상 켜짐입니다. 센서 제외는 **녹화만** 바꾸며 BEVFusion 입력을 끄지 않습니다.
5. `설정 검사` 후 고급/네트워크 탭의 `환경 검사`를 실행합니다. BEVFusion은 최초 한 번 `실제 GPU 검사`도 권장합니다. WARN 내용을 확인하고, 연결됐으면 `실행 / resume`을 누릅니다.

빠른 연결 확인은 `static_obstacle/base/static_pair_mission_gt_split.yaml` 한 장면으로 시작해도 됩니다. 위 정적 예제 생성 명령이 필요합니다. BEVFusion 준비 후 탐지기를 바꾸고 새 이름으로 저장하면 같은 장면을 비교할 수 있습니다.

**YAML 하나의 목적지·시간·스택은 그 YAML이 선택한 모든 장면에 적용됩니다.** 목적지가 다르면 YAML을 하나 더 만듭니다. 장면별 보조 YAML은 사용하지 않습니다. 공유 `base/`는 직접 바꾸지 않고 `workspace/`에서 작업합니다. 개인 YAML과 JSON은 Git에서 제외됩니다.

## 3. YAML 여러 개 걸어놓기

1. 각 조건을 개인 YAML로 저장합니다. 예: 같은 정적 장면의 GT 대조, BEVFusion, 다른 목적지.
2. `실행 목록 / 배치` 탭에서 `YAML 여러 개 추가`로 저장한 개인 파일들을 선택합니다. 현재 화면 설정을 저장하며 넣으려면 `현재 YAML 추가`를 누릅니다.
3. 목록에서 항목을 선택해 ↑/↓로 순서를 바꿉니다. `전체 검사`로 경로·설정·중복 여부를 확인합니다.
4. 같은 Sensor/Ego Network를 MORAI에 연결한 상태에서 `배치 실행 / 재개`를 누릅니다.

배치는 YAML들을 **순서대로 한 번에 하나씩** 실행합니다. 다른 배치를 넣는 구조는 아닙니다. 목적지·시간·GT/BEVFusion·신호등 설정은 YAML마다 달라도 되지만, 배치 중 MORAI Network나 시뮬레이터를 바꾸는 조합은 거절합니다. 같은 YAML이나 같은 결과 폴더를 두 번 넣을 수 없습니다.

모니터 바에는 `배치 2/5 · 캠페인 이름`, 장면 진행, 단계·시간·거리·속도·충돌이 나옵니다. RViz는 왼쪽 위, 모니터 바는 왼쪽 아래, MORAI는 오른쪽 전체 높이입니다. 영상은 전체 화면이므로 다른 창으로 가리거나 화면을 잠그지 마세요.

예상 시간·용량은 추정입니다. 제한시간에 준비·마감 시간을 더한 값이며, 용량은 센서 제외 기본 녹화의 과거 비율을 사용합니다. 센서를 포함하면 훨씬 커지므로 첫 두 장면을 보고 공간을 잡으세요.

## 4. 중단·재개·다시 처음부터

- `중단`을 누르고 bag·영상 마감과 스택 정리가 끝날 때까지 기다립니다. 그 뒤 다음 YAML은 시작하지 않습니다.
- 다시 켜도 실행 목록은 남습니다. `배치 실행 / 재개`를 누르면 완료한 YAML과 PASS/FAIL 장면은 건너뛰고 미완료부터 진행합니다.
- 주행 FAIL은 조사할 결과라 다음 YAML로 넘어갑니다. 시스템 오류나 미완료 종료는 배치를 멈추므로 로그 확인 후 재개합니다.
- 실행한 목록·YAML·JSON·Network·토픽 설정을 바꾸면 재개가 거절됩니다. 입력을 복원하거나 새 실행으로 분리하세요.
- `목록 초기화`는 **목록만** 비웁니다. 기존 YAML·bag·영상·판정은 삭제하지 않습니다. 이전 배치는 `이전 목록 열기`에서 결과 폴더의 `queue.json`을 골라 다시 열 수 있습니다.
- 같은 조건을 처음부터 다시 시험하려면 `새 실행으로 복사`를 누릅니다. 새 개인 YAML과 캠페인 이름을 만들어 기존 증거와 구분합니다.

단일 캠페인에서만 일부 실패 장면을 다시 돌리려면 해당 YAML을 열어 `기존 FAIL도 다시 실행`을 선택합니다. 이 체크는 배치 실행에는 적용되지 않습니다.

## 5. 결과 확인·공유

```text
planner_logs/
├─ validation/<캠페인 이름>/
│  ├─ summary.csv               # 장면별 결과
│  ├─ campaign_state.json
│  └─ success/ · failure/ · error/  # bag·MP4·JSON·events
└─ validation_batches/<배치 ID>/
   ├─ summary.csv               # YAML별 진행·P/F/E 합계
   ├─ state.json
   └─ queue.json                # 이전 배치 열기에 사용
```

GUI에서 `배치 CSV`·`배치 결과 폴더`로 열 수 있습니다. CSV는 Google 스프레드시트의 파일 → 가져오기로 공유하면 됩니다. 실패 장면을 더블클릭하면 관측 근거와 조사 우선 단계를 볼 수 있습니다. **의심 단계는 확정 원인이 아닙니다.**

Drive 업로드는 각 YAML의 옵션입니다. 기본은 꺼짐이며, 켜면 `AjouNice2026/planning_validator/<캠페인 이름>/`에 선택한 자료와 캠페인 CSV가 올라갑니다. basic/bag/video/full은 업로드 대상만 바꾸며 로컬 원본은 보존합니다. **배치 CSV는 자동 업로드 대상이 아니므로 필요하면 별도 공유합니다.** 업로드 오류와 주행 FAIL은 구분됩니다.

완료 목록은 `~/.config/ajounice/planning_scenario_validator/batch.json`에 자동 저장됩니다. 설정값 자체가 별도로 복제되는 것은 아니며 각 YAML이 실행 조건의 원본입니다. `XDG_CONFIG_HOME`이 있으면 그 경로를 사용합니다.

## 안 되면

- 설정 오류: `설정 검사` / 배치 `전체 검사`부터 봅니다.
- 설치·가중치·녹화 오류: `환경 검사`의 FAIL과 해결 방법을 봅니다. 기본 검사는 실제 GPU 추론이나 MORAI 연결을 보장하지 않습니다.
- 수신 오류: 스택 수신 중 ROS 토픽과 포트를 확인합니다. 실행 중에는 GUI가 실시간 주행 값을 보여주고, 별도 `ROS 수신 검사`는 다른 실행이 없을 때만 켤 수 있습니다.
- TIMEOUT·충돌·경로 실패: 없애거나 성공으로 덮지 말고 JSON·로그·영상을 남깁니다. 그 실패를 찾는 것이 검증기의 목적입니다.

새 장면을 만드는 방법은 [제작 가이드](SCENARIO_VALIDATOR_AUTHOR_GUIDE.md), 다른 AI가 이어서 볼 구현 계약·검증 범위는 [구현 문서](scenario_validator_automation.md)를 참고하세요.
