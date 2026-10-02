# 설치 끝났으면 이것만 보고 돌리면 됩니다

시나리오 검증기 설치·ROS 빌드·맵·BEVFusion 준비까지 끝난 분용입니다. GUI 전체 설명이 아니라 실제 작업 순서만 적었습니다. 처음 설치는 [설치 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md), 버튼 위치는 [화면 5장 안내](SCENARIO_VALIDATOR_GUI_HIGHLIGHTS.md)를 보세요.

## 1. MORAI 연결하고 GUI 켜기

MORAI MR에서 K-City를 켜고 아래 파일을 Load·Connect합니다. 전에 연결했던 그대로면 다시 바꿀 필요 없습니다.

- Sensor: configs/sensors/Sensor_competition_local.json
- Ego Network: base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json

일반 주행용 Ego Network가 아니라 **검증기 전용 파일**입니다. 경로가 헷갈리면 GUI 고급 / 네트워크 탭의 **연결 JSON 경로 복사**를 누르세요. YAML의 파일과 MORAI에 실제로 로드한 파일이 같아야 합니다. 센서도 실제로 켜두세요.

수동으로 켜놓은 udp·측위·인지·planning·제어는 종료합니다. 검증기가 켜고 끕니다. MORAI는 켜둡니다. Conda가 꺼진 터미널에서:

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
source docs/common_alias.txt
pvgui
```

## 2. 처음이면 예제를 내 폴더로 복사

**담당 미션의 base/example.yaml을 선택 → 베이스 → 내 작업 폴더 복사. 예제에서 바로 실행하지 마세요.**

예를 들어 고주로면 config/missions/highway/base/example.yaml을 고르고 복사합니다. 열린 경로가 highway/workspace/highway_starter.yaml인지 확인하세요. 앞의 config는 검증기 패키지 안의 폴더입니다. 미션 옆 README도 한 번 읽어 주세요.

이미 개인 YAML이 있으면 **내 YAML 열기**로 그 파일을 고르면 됩니다. GUI는 마지막에 연 파일을 기억합니다. 작업 파일 이름은 매일 바꾸지 않아도 됩니다.

## 3. 첫 두 장면 확인

기본 설정에서 아래만 확인합니다.

- 장면 선택: **장면 선택 → 기본 장면 → 검색 결과 전체 체크 → 체크한 장면 적용**. 처음부터 24개 전부 돌리지 않습니다. 기존처럼 파일 패턴 *_baseline_*.json만 써도 됩니다.
- 목적지: 처음에는 예제 값 그대로. 다른 목적지가 필요할 때만 YAML을 따로 만듭니다.
- 측위 localization / 객체 탐지 bevfusion. GT 대조를 할 때만 od_gt.
- 화면 영상 녹화 ON / bag에 카메라·LiDAR OFF / Drive 자동 업로드 OFF.
- 신호등은 미션에 필요할 때만 ON. 그때 신호등 환경 탭에서 설치해 둔 가상환경 또는 시스템 Python을 고릅니다.

**개인 YAML 저장 → 설정 검사 → 고급 / 네트워크의 환경 검사 → 새 실험 시작.** 처음 BEVFusion을 준비한 PC는 실제 GPU 검사도 한 번 합니다. 검사 FAIL은 해결하고, WARN은 내용을 확인합니다.

처음 두 장면에서 연결·배우 움직임·bag·영상이 실제로 잘 나오는지 봅니다. PASS만 떴다고 모든 기능이 맞다는 뜻은 아닙니다. 영상은 전체 화면이므로 다른 창으로 가리거나 화면을 잠그지 마세요. 모니터 모드로 진행 상황을 볼 수 있습니다.

## 4. 장면을 늘리고 다시 시험

**내 작업 폴더 열기**로 workspace/scenarios 안에 JSON을 추가·수정합니다. 기존 JSON 하나를 복사해서 한두 조건씩 바꾸면 편합니다. 일반적인 장면도 넣고 어려운 장면도 섞으세요. 실패를 없애려고 장애물을 치우거나 판단 파라미터를 억지로 바꾸는 게 목적은 아닙니다.

전체를 돌릴 때는 **장면 선택 → 전체 실행(패턴)**으로 돌아간 뒤 패턴을 *.json으로 바꿉니다. 일부만 돌릴 때는 이름을 검색하고 **검색 결과 전체 체크 / 체크 해제**를 쓰세요. baseline,stress처럼 쉼표로 여러 묶음을 한 번에 찾을 수도 있습니다. 기존 체크는 검색을 바꿔도 유지됩니다. 새 조합으로 고르려면 먼저 **모든 체크 지우기**를 누르세요.

**개인 YAML 저장 → 설정 검사 → 새 실험 시작**하면 선택한 장면을 다시 처음부터 돕니다. 선택 목록도 이 YAML 안에 저장됩니다. 목록 모드에서는 나중에 추가한 JSON이 자동 포함되지 않으며, 패턴 모드에서는 패턴에 맞으면 포함됩니다. 같은 YAML의 목적지·제한시간·스택이 선택한 모든 장면에 적용됩니다. 자세한 제작법은 [시나리오·YAML 제작 가이드](SCENARIO_VALIDATOR_AUTHOR_GUIDE.md)에 있습니다.

개인 YAML·JSON은 workspace에 두면 Git에 안 올라갑니다. base는 공유 원본입니다. 파일 탐색기에서 삭제·이름 변경도 가능하지만, 삭제한 YAML이 GUI에 아직 열려 있으면 다른 파일을 여세요. 그대로 저장하면 다시 생길 수 있습니다.

## 5. YAML 여러 개 걸어놓기

각 조건을 개인 YAML로 저장한 뒤 **실행 목록 / 배치 → YAML 여러 개 추가 → 순서 정리 → 전체 검사 → 새 배치 시작**으로 갑니다. 현재 열린 파일 하나를 넣을 때는 현재 YAML 추가를 누르면 됩니다.

같은 Sensor/Ego Network로 돌아가는 YAML만 함께 넣습니다. 목적지·탐지기 등은 YAML마다 달라도 됩니다. 모니터 바에서 배치 2/5처럼 현재 순서를 봅니다. 배치 안에 또 배치를 넣는 구조는 아닙니다.

## 6. 중단했으면 재개, 다시 시험이면 새 시작

- **중단**: 녹화 마감·스택 정리가 끝날 때까지 기다립니다. 강제 종료하지 마세요.
- 단일 실험 이어가기: **실험 기록 / 재개 → 기록 선택 → 선택 실험 재개**.
- 배치 이어가기: 기존 목록에서 **배치 재개**. 목록이 없으면 이전 목록 열기에서 배치 결과의 queue.json을 고릅니다.
- 같은 조건을 처음부터 다시: **새 실험 시작 / 새 배치 시작**. YAML을 매번 새로 복사할 필요 없습니다.

새 시작은 timestamp 결과 폴더를 따로 만들고, 재개는 당시 보관된 입력 사본으로 이어갑니다. **재개 시 FAIL도 재실행**은 단일 실험을 재개할 때만 사용합니다. 배치에서는 주행 FAIL도 완료된 결과라 넘어가고 ERROR·미완료를 다시 시도합니다.

## 7. 결과 보고 공유

**결과 폴더**에서 영상과 summary.csv를 봅니다. 실패 행을 더블클릭하면 관측 근거·조사 우선 단계가 나옵니다. 의심 단계는 확정 원인이 아닙니다. 배치는 배치 CSV로 전체 진행을 봅니다.

결과는 planner_logs/validation/실험이름_날짜시간/에 남고 배치 요약은 planner_logs/validation_batches/배치ID/에 남습니다. PASS는 도착, FAIL은 충돌·시간 초과 같은 주행 실패, ERROR는 설치·수신·녹화·중단 등 시스템 문제를 포함합니다. ERROR를 곧바로 판단 알고리즘 실패로 보지는 마세요.

Drive로 공유할 때만 기본 설정의 자동 업로드를 켜고 공유 모드를 고릅니다. 기본 공유는 CSV·JSON, bag 공유는 여기에 bag, 영상 공유는 MP4, 원본 공유는 둘 다입니다. 기본 경로는 AjouNice2026/planning_validator/입니다. 배치 CSV는 자동 업로드하지 않으므로 필요하면 별도로 공유하세요.

업데이트가 올라오면 실행·GUI 종료 후 **pull → planning/base 증분 빌드 → GUI 다시 실행**합니다. 설치를 다시 할 필요는 없습니다. 복붙 명령은 [설치 가이드 4-1](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md#4-1-다음부터는-pull-후-증분-빌드)에 있습니다.
