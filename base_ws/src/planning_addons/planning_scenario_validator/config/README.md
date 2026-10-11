# Validator 설정 폴더

팀원이 새로 시작할 때는 `missions/<미션>/base/example.yaml`을 GUI에서 연 뒤 **베이스 → 내 작업 폴더 복사**를 누르면 된다.

**example.yaml을 선택한 상태에서 바로 실행하지 않는다. 반드시 복사 버튼부터 누르고, GUI에 열린 경로가 `missions/<미션>/workspace/…yaml`인지 확인한다.** 설정·실행·배치 추가는 개인 복사본을 사용한다.

```text
config/
├─ missions/<미션>/
│  ├─ base/                 # Git 공유: README.md, example.yaml, manifest.json, scenarios/*.json
│  └─ workspace/            # 개인 YAML·JSON: .gitignore만 공유
├─ internal/
│  ├─ generated/            # 설치기가 생성하는 validator 전용 Ego Network
│  └─ topic_profiles/       # basic/lightweight/full 녹화 토픽
└─ users/                   # 이전 개인 설정 위치; 기존 파일 유지
```

미션은 `roundabout`, `static_obstacle`, `dynamic_obstacle`, `gps_shadow`, `highway`다. 사용자 이름 대신 미션으로 구분한다.

GUI 상단 **내 미션 선택하기 → 담당 미션 → 실험 YAML**로 고른다. 목록은 그 미션의 base/예제와 workspace/내 작업 파일만 표시하고 자유 경로 입력·붙여넣기는 받지 않는다. 기본 탭에서 일상 옵션·파일 패턴/장면 선택·반복·측위/탐지·녹화/공유를 수정하고 `YAML 저장`을 누른다. 회색 목적지·시간·네트워크·Python 환경은 확인용이며 필요한 경우만 VSCode에서 수정·저장한 뒤 새로고침한다. 새 YAML도 해당 미션 workspace에 저장한 뒤 새로고침한다. 주석의 ‘여기부터 수정 X’ 아래는 미션 고정 설정이다.

고주로 반복용은 `missions/highway/base/random_example.yaml`이다. JSON 2개 × 10회 = 20회이며 repeat/seed/범위를 위쪽에서 바꾼다. planning 팀의 JSON 1–2개 대조는 GUI `planning 빠른 테스트` 탭에 별도로 둔다.

**캠페인 YAML 하나 = 공통 목적지·시간·스택·녹화 조건 하나.** 시나리오 JSON 여러 개를 선택할 수 있지만, 장면별 조건을 덮어쓰는 보조 YAML은 없다. 다른 조건은 다른 캠페인 이름의 YAML로 만든다.

여러 개인 YAML은 GUI `실행 목록 / 배치`에서 순서대로 실행한다. 별도의 배치 YAML을 작성할 필요는 없다. 목록과 재개 경로는 홈 설정 폴더의 `batch.json`에 자동 저장하며 결과는 `planner_logs/validation_batches/`에 보관한다. 목록 초기화는 기존 YAML·녹화 결과를 지우지 않는다.

작업용 YAML 이름에는 timestamp를 붙이지 않는다. 베이스 복사 시 `highway_starter.yaml`처럼 만들고, 이미 있으면 그대로 연다. 개인 YAML은 계속 수정·재사용할 수 있다. `새 실험 시작`/`새 배치 시작`마다 별도 timestamp 결과 폴더에 입력 사본을 보관한다. 재개는 결과 폴더의 사본을 사용하며 원본 YAML·JSON 수정과 구분한다. 이전 고정 이름 기록은 당시 원본 입력을 유지해야 재개할 수 있다.

`base/manifest.json`은 제작 의도와 조건 설명이며 실행 설정이 아니다. 시나리오 폴더에는 실행할 MORAI JSON만 넣는다. 생성된 Network JSON도 시나리오가 아니다.

자기 미션 `base/README.md`에는 실제 시작점·목적지, 배치 근거, 조건을 바꿔 볼 순서와 담당 AI 프롬프트가 있다. 처음에는 좌표를 유지하고 객체 조건부터 바꾸면 된다. GUI 복사 시 workspace README에 공유 안내 링크도 만들며, 개인 메모는 덮어쓰지 않는다.

개인 `workspace/`의 YAML·JSON은 Git 제외된다. 공유할 입력은 검토 후 base/생성기에 반영한다. 녹화 결과는 별도 `planner_logs/validation/`에 저장된다.

`example.yaml` 이외의 모든 YAML을 제외하는 규칙은 아니다. `base/`의 공유 GT/BEVFusion 비교 예제 YAML도 Git 추적 대상이다. 개인 파일은 이름과 무관하게 `workspace/`에 둔다.

작은 smoke·GT/BEV 비교 예제도 해당 미션의 `base/`에 모았다. `static_obstacle_*.yaml` 두 파일은 이름과 달리 음영 구간이므로 `gps_shadow/base/`에 있다. 예전 examples·generated·topic_profiles 경로를 담은 개인 YAML은 내용 수정 없이 읽기 호환한다. 기존 `users/` 개인 파일도 이동하지 않는다.

- [미션별 베이스·실행 안내](missions/README.md)
- [설치 후 실제 작업 순서](../../../../../docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md)
- [자주 쓰는 버튼 표시 화면](../../../../../docs/planning/SCENARIO_VALIDATOR_GUI_HIGHLIGHTS.md)
- [설치 후 짧은 사용법·배치 실행](../../../../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)
- [첫 설치부터 실행까지](../../../../../docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)
- [시나리오·YAML 직접 제작](../../../../../docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md)

이전 v1 YAML은 GUI에서 공통 설정을 확인만 하고 주석 있는 v2 예제로 옮겨 사용한다. 이전 실행 결과는 유지하지만 v2 캠페인과 섞어서 resume하지 않는다. 이전 `.case.yaml`은 실행 시 무시되고 실행용 YAML로 열 수 없다.
