- Git 협업이지만 사실상 Branch 전략 위주로 얘기할 듯 싶음.

# Git 협업 전략의 필요성

- 여러 사람이 같은 `main` 브랜치에서 동시에 개발하면 다음 문제가 생긴다.
    - 아직 검증되지 않은 코드 때문에 전체 시스템이 실행되지 않을 수 있다.
    - **한 팀의 revert가 다른 팀의 인지·측위·판단 작업까지 함께 지울 수 있다.**
    - 어떤 변경이 어떤 목적에서 들어왔는지 추적하기 어렵다.
    - 충돌이 발생했을 때 어떤 결과를 남겨야 하는지 판단하기 어렵다.
    - 대회 직전에 안정적으로 실행되는 기준 버전을 찾기 어렵다.
- 이를 방지하기 위해 안정 버전, 통합 시험 버전, 개별 작업 버전을 분리한다.

```
main
└─ 대회 실행이 가능한 안정 버전

develop
└─ 각 팀의 기능을 합쳐 통합 시험하는 버전

perception/pointpainting-online
localization/gnss-imu-fusion
planning/behavior-path
control/adaptive-pure-pursuit
└─ develop에서 분기한 개별 작업 브랜치
```

- 핵심 원칙은 다음과 같다.
    1. `main`과 `develop`은 장기 유지한다.
    2. 평소 개발은 `main`이 아닌 `develop`에서 분기한 **작업 브랜치**에서 한다.
    3. **작업 브랜치**는 Github의 Pull Request를 통해 `develop`에 합친다.
    4. `develop`의 최신 변경을 작업 브랜치로 가져올 때는 일반 merge를 사용한다.(Pull Request X)
    5. `develop`에서 통합 시험을 통과한 버전만 주기적으로 `main`에 반영한다.
    6. `main`에는 실행이 깨진 코드나 개인 실험 코드를 직접 push하지 않는다.
- Git 명령이 익숙하지 않거나 충돌 상황이 애매하면 `git branch 결과`,
`git status 결과`, `실행하고자 하는 내용`을 복사해서 GPT나 Codex 등에 붙여넣고 “어떻게 터미널에 내가 입력하면 될까?” 라고 물어본다.
    - 단, `reset --hard`, 강제 `push --force`, `브랜치 삭제` 같은 명령은 의미를
    확실히 이해하기 전에는 실행하지 않는다.
- 이는, Git branch 전략의 고전적이고 가장 안정적인 전략인 Git Flow 전략을 우리 대회에 맞게 간소화시킨 것이다.

# Branch

!image.png

!image.png

### 1. 장기 유지 브랜치

#### `main`

- 대회 또는 팀 통합 시연에 사용할 수 있는 안정 브랜치
- `develop`에서 통합 시험을 통과한 변경만 Pull Request로 반영
- 개인 기능 개발 및 직접 push 금지

#### `develop`

- 인지·측위·판단·제어 결과를 모으는 통합 브랜치
- 개별 브랜치의 Pull Request 대상
- `main`에 올리기 전 전체 launch를 수행하여 제대로 작동하는지 확인
    - 최소한 에러 없이 파이프라인이 잘 작동하는 경우에만 main에 반영

### 2. 작업 브랜치 이름

- 기존의 범용 `feature/*` 대신 담당 영역을 prefix로 사용한다.

```
perception/pointpainting-online
perception/bevfusion-finetuning
localization/gnss-imu-fusion
planning/behavior-path
control/adaptive-pure-pursuit
sensing/sensor4-noise-test
map/kcity-lanelet-update
```

- 브랜치 이름은 영문 소문자와 `-`를 사용하고, 수행할 작업을 짧게 표현한다.

### 2.1. 예외)

- 만약 작업 단위로 브랜치를 나누는 게 어렵다면, 그냥 part 단위로 나누는 것도 방법이다.
    - ex. `control` 브랜치, `planning` 브랜치 등.

### 3. 새 작업 시작

- 항상 최신 `develop`에서 분기한다.
    - 어려우면 gpt나 codex에게 물어보자!!

```bash
# 최신 develop을 기준으로 새 작업 브랜치를 생성하는 방법 예시
git switch develop
git pull --ff-only origin develop

git switch -c perception/example-task
git push -u origin perception/example-task
```

### 4. develop 변경을 내 브랜치로 가져오기

- 이 방향은 Pull Request를 만들지 않고 작업 브랜치에서 직접 merge한다.

```
develop → perception/example-task
```

```bash
# 최신 develop을 현재 작업 브랜치로 병합하는 방법 예시
git fetch origin
git switch perception/example-task
git merge origin/develop
```

- 충돌이 발생하면 어떤 코드를 남길지 해당 파일을 커밋한 사람에게 연락하여 확인한다. 충돌을
해결한 뒤 빌드와 실행 검증을 하고 merge commit을 push한다.
    - 그러나 작업을 잘 나눠서만 하면 충돌이 일어날 일은 거의 없을 것이다.

### 5. 작업 브랜치를 develop에 반영

- 이 방향은 반드시 Pull Request를 사용한다.
    - 명시적으로 기록을 남기기 위함

```
perception/example-task → develop
```

- **웬만해선, 작업 브랜치에서 먼저 최신 `develop`을 merge하고 로컬 검증을 마친 뒤 PR을 생성하는 것이 좋다.**
    - **즉 5번(작업 브랜치 → develop PR) 전에 4번(develop → 작업 브랜치 merge)를 (웬만해선) 먼저 수행해하는 것이 좋다.**
    - **그러나, 수정한 내용이 적거나, 내 브랜치에서 독립적으로 길게 무언가를 개발해야 한다면, merge는 생략하고 바로 PR해도 좋다.**
- 다른 팀원이 개발중인 내용과 내가 개발중인 내용이 많이 독립적이면, 꼭 5번 전에 4번을 할 필요는 사실 없다.
    - 그러나, develop 브랜치의 최신 내용과 내가 수행한 브랜치의 최신 내용이 conflict가 나거나, conflict는 안 나도 호환이 안 될 수 있다.
    - 따라서 4번을 먼저 하여 내 branch에서 conflict가 안 났고, launch가 잘 되는지 확인 후 develop 브랜치로 PR하는 것이 안전하다.
    - (그냥 rebase를 써버리는 방법도 있지만, 협업에서 rebase를 쓰면 conflict가 나면 --force-with-lease를 써야 돼서 귀찮아진다)

### 6. develop을 main에 반영

- `develop`은 매 커밋마다 `main`에 합치지 않는다. 팀에서 정한 주기 또는
주요 통합 기능이 완성된 시점에 다음을 확인한다.
    - 전체 workspace 빌드 성공
    - 필수 launch 파일 실행 가능
    - MORAI 센서 입력 정상
    - 인지·측위·판단·제어 핵심 토픽 정상
    - TF 충돌 및 중복 publisher 없음
    - 담당자별 통합 시험 결과 공유
- 시험을 통과한 `develop`만 `main`에 반영한다. 웬만해선
`develop → main`도 PR로 기록하고 팀장 또는 담당자 승인을 받은 뒤 merge한다.

# Pull Request(PR)

!image.png

!image.png

### 1. 언제 사용하는가

- 기본 PR 방향:

```
개별 작업 브랜치 → develop
```

- 예:

```
perception/pointpainting-online → develop
localization/gnss-imu-fusion → develop
planning/behavior-path → develop
```

- 반대 방향인 `develop → 개별 작업 브랜치`는 PR을 만들지 않고 개별 브랜치에서
일반 merge한다.

### 2. PR을 만들기 전

1. 작업 트리가 깨끗한지 확인한다.
2. 최신 `develop`을 작업 브랜치에 merge한다.
3. 충돌을 해결한다.
4. 담당 기능을 빌드하고 실행한다.
5. 임시 파일과 대용량 결과물이 포함되지 않았는지 확인한다.
6. 작업 브랜치를 원격에 push한다.

```bash
# PR 생성 전 기본 확인(필요 시)
git status
git fetch origin
git merge origin/develop
git push
```

### 3. PR 제목

- 한 눈에 알아보기 쉽도록 한글로, 본론만 작성하는 것을 권장한다.

```
제어 revert 이후 인지와 판단 수정 커밋 cherry pick 후 conflict 수정
BEVFusion from mmdetection 기본 세팅 with docker 및 일부 버그 수정
lidar default hz 10에서 15로 변경
```

### 4. PR 본문

- 본문은 필수는 아니나, 적는다면 아래와 같이 간단하게 적으면 좋다.
    - 필요에 따라 목적/결과/확인 필요 등의 내용을 자유롭게 적어도 된다.
- **만약 해당 PR과 관련된 Issue를 Git에 작성했었다면, 변경 사항 내용들을 생략하고 그냥 Issue만 Tagging해도 된다.**

```
## 주요 변경 사항
- bevfusion docker 세팅
- ros와의 통신 스크립트 및 launch 작성
- 관련 docs 문서화

(Issue Tagging)
Closes #12
또는
Related to #15
```

- Reviewers를 등록할 수 있는데, 내 PR을 다른 사람이 확인할 필요가 있다면 등록하자.
    - 등록 시 상대방에게 메일이 간다.
- Assignees에는 자기 자신 또는 같이 해당 PR에 기여한 사람들을 추가한다.
- Labels에는 이 PR이 어떤 영향을 주로 미치는지 그 범주를 적는다.
- **PR merge 후에는 필요 없는 원격 작업 브랜치를 삭제할 수 있다. (원래는 삭제하는 것이 권장된다.) 다만 후속 개발이 바로 이어지거나 다른 팀원이 사용 중이면, 브랜치 삭제 결정을 공유한 후, 확인받은 후에 삭제한다.**

# **Commit**

!image.png

### 1. 형식

- 커밋 메시지는 다음 형식을 사용한다.

```
type(part): 간단한 변경 내용
```

- 예시:

```
feat(perception): add PointPainting live inference
fix(localization): convert MORAI velocity from kmph to mps
docs(planning): document K-City route validation
build(perception): add reproducible BEVFusion container
refactor(control): split longitudinal controller parameters
chore(sensing): update Sensor_4 config path
```

### 2. type

| Type | 사용 시점 |
| --- | --- |
| `feat` | 새로운 기능 추가 |
| `fix` | 버그 수정 |
| `build` | 빌드, 패키지, Docker, 환경 설정 |
| `refactor` | 동작은 유지하면서 코드 구조 개선 |
| `docs` | 문서 추가 또는 수정 |
| `test` | 테스트 코드나 검증 절차 추가 |
| `chore` | 분류하기 어려운 작은 유지보수 작업 |

### 3. part

- 현재 권장 part:

```
perception
localization
planning
control
sensing
map
common
```

- part는 변경의 주된 영향을 기준으로 하나를 선택한다. 여러 영역을 동시에
수정해야 한다면 가능하면, 되도록 목적별 커밋으로 분리하거나, 주된 part의 이름으로 커밋한다.

### 4. 커밋 작성 원칙

- 하나의 커밋에는 하나의 설명 가능한 목적을 담는다.
    - 원칙은 위와 같으나, 어렵게 느껴진다면 여러 목적을 함께 커밋해도 큰 문제는 없다(그러나 이 경우 후에 이력 관리 및 복구가 어려워 질 수 있다).
- 실행 가능한 중간 지점에서 커밋한다.
    - 즉, 되도록이면 어떤 작업을 수행하기 전에 커밋하고, 수행하여 최종 실행 테스트까지 완성된 후 한번 더 커밋하면 좋다.
- log, rosbag, checkpoint, 개인 IDE 설정을 무심코 `git add .`로 올리지 않는다.
    - 이러한 파일들은 전부 `.gitignore`에 올려서 Git이 추적하지 못하게 한다.
- 커밋 전, 다음 내용을 확인하면 도움이 된다.

```bash
# 현재 변경과 스테이징 범위 확인
git status
git diff
git diff --cached
git diff --cached --check
```

- 명령을 잘 모르겠다면 GPT/Codex에 다음 정보를 같이 전달하면 비교적 정확한
도움을 받을 수 있다.

```
내가 하려는 작업
현재 브랜치 이름
git status
git log -5 --oneline --decorate
발생한 오류 원문
```

- 커밋 메시지는 원래 여러 줄을 적을 수 있게 되어 있는데, 여러 줄을 적으면 가독성도 안 좋고 과하게 길어지니,
    - 한 줄만 적는 걸로 통일한다.

# Issue

!image.png

!image.png

- 현재는 팀원들이 Commit, Branch, Pull Request 흐름에 먼저 익숙해지는 것을
우선한다.
- 8월부터는 다음 목적을 위해 Issue 사용을 해 보는 것을 권장한다.
    - 해야 할 작업과 담당자 기록
    - 버그 재현 조건과 로그 보존
    - 실험 가설, 조건, 결과 추적
    - PR이 해결한 작업 연결
    - 대회 전 남은 위험과 우선순위 관리
- 그러나, 우리는 Git 만으로 협업하는 것이 아닌, Notion을 함께 사용해 협업하므로 Issue 사용은 자율로 맡긴다. 그러나 Issue를 생성하고, 이를 PR에 Tagging하는 식으로 기록하면 Notion을 일일이 확인하는 것보다 협업이 편해진다.

### 1. 제목 형식

```
[Feature(Perception)] PointPainting 처리속도 최적화
[Fix(Localization)] GNSS 초기 yaw 오차 분석 및 수정
[Docs(Common)] 대회 통합 파이프라인 계획 문서화
[Task(Control)] develop 통합 주행 시험 및 검증, 에러 확인 및 수정 시도
```

### 2. 권장 Issue 내용

```
## 배경/문제상황
왜 필요한 작업인가

## 목표
완료 조건은 무엇인가

## 수행 계획
어떤 방법으로 수행할 것인가
```

- Issue를 도입한 이후에는 관련 PR 본문에 다음처럼 연결한다.

```
Closes #12
또는
Related to #15
```

- `Closes #12`가 포함된 PR이 merge되면 Issue가 자동으로 닫힌다.

---

# 전체 작업 흐름 요약

```
최신 develop 받기
        │
        ▼
영역/작업명 브랜치 생성
perception/example-task
        │
        ▼
작은 단위로 commit 및 push
        │
        ▼
origin/develop을 작업 브랜치에 merge
        │
        ▼
빌드·실행·rosbag/MORAI 검증
        │
        ▼
작업 브랜치 → develop Pull Request
        │
        ▼
리뷰 및 develop 통합
        │
        ▼
주기적인 전체 시스템 시험
        │
        ▼
develop → main 반영
```

이 전략의 목적은 절차를 복잡하게 만드는 것이 아니라, 한 팀의 수정이나
revert가 다른 팀의 작업을 지우는 일을 막고 언제든 실행 가능한 기준 버전을
유지하는 것이다.

# 계획

- 오늘 12시 이전까지 모두 다 main 브랜치에 커밋 및 push를 일단 완료해 주시면,
    - 12시에 main 브랜치에서 develop 브랜치를 파겠다.
    - 그럼 이후부터는 develop 브랜치에서 작업 브랜치를 새로 파서 작업해 주면 좋겠다.