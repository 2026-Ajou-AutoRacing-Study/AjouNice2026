# 설치 끝났으면 이것만 보고 돌리면 됩니다

설치·맵·ROS 빌드·BEVFusion 준비가 끝난 분용입니다. 처음 설치는 [설치 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)를 보세요. **일상 설정은 GUI에서 수정 → YAML 저장 → 실행합니다.** 회색 목적지·시간·네트워크 등 고정값은 확인용입니다.

## 1. 연결하고 GUI 켜기

MORAI MR의 K-City에서 아래 파일을 Load·Connect합니다. 수동 udp·측위·인지·planning·제어는 끄고 MORAI만 켜둡니다.

- Sensor: configs/sensors/Sensor_competition_local.json
- Ego Network: base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json

일반 주행용이 아니라 검증기 전용 Ego Network입니다. 경로가 헷갈리면 planning 빠른 테스트 탭의 연결 JSON 경로 복사를 누르세요.

```bash
cd ~/AjouNice2026
./planning_validator_gui
# alias를 쓰면 pvgui
```

## 2. 예제를 개인 작업 폴더로 복사

담당 미션의 **base/example.yaml 선택 → 베이스 → 내 작업 폴더 복사**부터 합니다. base에서 바로 실행하지 마세요. 같은 폴더 README에 시작점·목적지·시나리오 의도가 있습니다.

고주로의 소수 장면 반복 시험은 **highway/base/random_example.yaml**을 복사하면 됩니다. JSON 2개를 각 10번, 총 20회 실행하며 NPC 목표 속도 50–60 km/h와 차로 내 offset −10–10%를 회차마다 추출합니다. 대회 공식 범위가 아니라 첫 시험용 값입니다.

복사 후 열린 경로가 workspace인지 확인하세요. 기존 파일이 있으면 덮어쓰지 않고 다시 엽니다. 개인 파일은 Git 제외입니다.

## 3. 기본 탭에서 수정 → 파란 YAML 저장 → 실행

기본 탭에서 이름, 파일 패턴/장면 선택, JSON당 반복 횟수, 측위·탐지, 신호등·영상·센서 bag·Drive 업로드와 공유 모드를 바꿉니다. **상단 파란 YAML 저장**을 누르면 지금 열린 개인 YAML에 저장됩니다. 파일을 새로 만들거나 이름에 timestamp를 붙이지 않습니다. 고정값과 주석은 유지합니다.

패턴은 `*.json`이면 전체, `high_baseline_*.json`이면 해당 장면들입니다. 장면 선택 버튼에서는 검색·일괄 체크로 일부 JSON만 고를 수도 있습니다. **패턴을 직접 바꾸면 기존 개별 선택 목록이 해제됩니다.** 반복 횟수는 선택한 JSON 하나당 횟수입니다.

목적지·시간·네트워크 같은 회색 값과 랜덤 seed/범위·Python 환경은 필요할 때만 YAML 편집 (VSCode)에서 수정합니다. **‘여기부터 수정 X’ 아래는 기본값을 유지합니다.** Ego 시작점은 시나리오 JSON에 있습니다.

VSCode에서 YAML 파일 우클릭 → 경로 복사 → GUI 상단 칸에 Ctrl+V → Enter 또는 불러오기입니다. 경로 붙여넣기 버튼도 됩니다. 경로에 공백·따옴표가 있어도 받습니다.

**YAML 저장 → 설정 검사 → 새 실험 시작.** 검사·새 실행 전에도 GUI 변경분을 저장합니다. VSCode에서 바꾼 고정값은 보존하며, 같은 항목을 양쪽에서 바꿨다면 다시 불러오도록 안내합니다. 실행 직전에도 JSON 개수 × 반복 횟수를 확인하세요. YAML 하나에는 목적지·시간·스택 하나만 사용합니다.

첫 두 장면만 확인하고 싶으면 planning 빠른 테스트 탭에서 JSON 1–2개를 선택해 실행하세요. 실제 장면·객체·bag·영상이 잘 나오는지 보고 본 시험을 걸어두면 됩니다.

## 4. 고주로 반복 시험

개인 YAML의 scenarios.repeat가 JSON 한 개당 횟수입니다. randomization.enabled가 false면 JSON 값 그대로 반복하고, true면 지정 범위에서 매회 추출합니다. seed를 바꾸면 다른 조합을 시험합니다.

원본 JSON은 보존합니다. 실제 로드한 scenario.json과 추출값이 적힌 input.json/결과 JSON을 각 회차에 남깁니다. 같은 seed·JSON·회차는 재시도해도 같은 파라미터지만 **MORAI RandomMode 차종과 physics까지 재현하는 seed는 아닙니다.** PASS/FAIL은 회차별로 별도 집계합니다.

## 5. planning 팀이 한두 장면만 빠르게 볼 때

개인 YAML을 불러온 뒤 기본 탭 옵션을 저장하고 **planning 빠른 테스트** 탭에서 JSON 1–2개를 선택합니다. 파일명으로 검색하고 Ctrl/Shift로 함께 선택할 수 있습니다.

필요하면 od_gt / bevfusion, maptf_morai / localization을 대조합니다. **선택한 JSON 빠른 실행**은 원본 YAML을 바꾸지 않고 임시 설정으로 1회씩 실행합니다. 랜덤 파라미터와 Drive 업로드는 꺼집니다. 목적지·시간·네트워크·녹화 설정은 불러온 YAML 그대로입니다. 회차 랜덤 검증은 일반 새 실험 시작을 사용하세요.

환경 검사·ROS 수신 검사·연결 경로 복사도 이 탭에 있습니다. 도착 실패나 궤적 미생성도 결과로 남기는 게 목적입니다.

## 6. 밤새 여러 YAML 돌리기 / 중단·재개

실행 목록 / 배치에서 **YAML 여러 개 추가 → 전체 검사 → 새 배치 시작**입니다. 같은 Sensor/Ego Network를 사용하는 개인 YAML만 넣으세요. 순서 변경·목록 초기화·이전 목록 열기는 목록 관리 메뉴에 있습니다.

- 중단은 GUI 버튼을 누르고 bag·영상과 ROS 정리가 끝날 때까지 기다립니다.
- 단일 실험: 실험 기록 / 재개 → 기록 선택 → 선택 실험 재개.
- 배치: 배치 재개. 이전 목록은 목록 관리 → 이전 목록 열기에서 queue.json을 고릅니다.
- 다시 처음부터: 새 실험 시작 / 새 배치 시작.

재개는 당시 보관한 입력 사본으로 완료된 PASS/FAIL 회차를 건너뛰고 ERROR·미완료부터 진행합니다. 현재 YAML 수정값을 과거 실험에 섞지 않습니다.

## 7. 결과 보기

결과 폴더의 summary.csv와 회차별 bag·영상·JSON을 봅니다. 실패 행을 더블클릭하면 관측 근거와 조사 우선 단계가 나옵니다. 의심 단계가 확정 원인은 아닙니다.

단일 결과는 planner_logs/validation/실험이름_날짜시간/, 배치 요약은 planner_logs/validation_batches/배치ID/에 남습니다. PASS는 도착, FAIL은 충돌·시간 초과 같은 주행 실패, ERROR는 수신·스택·녹화·중단 등 시스템 문제입니다. 실패를 없애려고 시나리오나 판단 코드를 임의로 고치는 시험이 아닙니다.

기본은 basic 토픽 + 카메라/LiDAR 원본 제외 + 화면 영상입니다. 센서 입력 자체는 켜둡니다. 영상은 전체 화면이므로 다른 창으로 가리거나 화면을 잠그지 마세요. 모니터 모드에서 현재 회차·배치 순서·주행 상황을 봅니다.

Drive 공유가 필요할 때만 GUI에서 자동 업로드와 공유 모드를 선택하고 YAML 저장을 누릅니다. basic/bag/video/full은 업로드할 자료만 고르며 로컬 원본은 남습니다. rclone 인증은 먼저 필요하고, 배치 CSV는 별도 공유합니다.

업데이트는 실행·GUI 종료 → pull → planning/base 증분 빌드 → GUI 재실행입니다. Docker·가중치를 매번 다시 설치하지 않습니다.
