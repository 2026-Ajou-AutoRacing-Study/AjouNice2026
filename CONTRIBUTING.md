# Contributing Guide
- 다른 프로젝트에서 사용했던 convention을 그대로 가져온 것(임시)
  - 추후 우리 대회에 맞게 convention, examples 수정 예정


## 1. (Git) Convention
- 원활한 Git 협업을 위해 약속한 규약
- `Commit`: 
  - 말 그대로 커밋
- `Issue`: 
  - 무언가 수행해야 할 일이 있을 때, issue로 작성. issue를 자주 작성해야 개발 현황을 트래킹하기가 편함. 
  - 어떤 commit이 특정 issue에 관련되어 있을 경우, commit 메시지 작성 시 issue를 태깅하는 것이 일반적. 
  - 모든 commit에 대해 issue를 생성하는 방법도 있고, 굵직한 것들만 issue로 생성하는 방법도 있음.
- `Pull Request`:
  - Merge를 수행할 때, "Merge 했음!" 이라고 기록을 남기기 위한 용어
- `Branch`:
  - main branch를 두고, 개발하고자 하는 기능 단위로 branch를 분기해서 개발 후 다시 mearge하는 것이 원칙
  - 1기능 - 1branch로 long-live branch를 지양하는 것이 원칙이나, 지키기 어려운 경우가 많음.


## 2. Commit
### 기본 사용법:
- `type: 간단한 설명`

### type 종류:
- `feat:`: 새로운 기능 추가
- `refactor:`: 코드 리팩토링
- `docs:`: 문서 작성/수정
- `fix:`: 버그 수정
- `chore:`: 진짜 자잘한 것들, 그다지 안 중요한 커밋들(직역: 잡일)
- 헷갈리면 대부분 `feat:` 사용해도 ok

### 예시:
- `feat: add HRNet model support from mmsegmentation`
- `fix: resolve inference resize issue`
- `refactor: group wandb metrics with train/val prefixes`

### issue-commit 태깅:
- 해결한 issue가 있을 경우, commit에 `#(이슈 넘버)`로 태깅해도 좋음.
- ex. `fix: #8 - add typecasting code to float32 before data aug in dataset.py`

- `main`: stable, runnable
- `dev`: integration branch
- `feature/*`: module development
- `exp/*`: experiments
  

## 3. Issue
### 기본 사용법:
- `[Type] 간단한 설명`

### Type 종류:
- `[Feature]`: 새로운 기능 추가
- `[Refactor]`: 코드 리팩토링
- `[Docs]`: 문서 작성/수정
- `[Fix]`: 버그 수정
- `[Task]`: 작업/태스크 -> 범용적으로 사용 가능
- `[Experiment]`: 실험
- 헷갈리면 대부분 `[Feature]` 사용해도 ok

### 예시
- `[Refactor] .yaml 파일에 checkpoint_name_format을 추가`
- `[Feature] HRNet support from mmsegmentation`
- `[Fix] inference.py에서 resize 문제 해결`


## 4. Pull Request
### 기본 사용법:
- `(바로) 간단한 설명`

### 예시:
- `yaml 파일에 checkpoint.pt 이름을 지정 가능하게 함 + 학습 자동화 scripts 초안 작성`
- `HRNet 기본 baseline 작성 + wandb logging 리더보드 Refactoring`

### PR 생성 전 확인사항:
- 관련 Issue가 있다면 Description에 언급
  - PR 이후 해당 Issue close하기
- 관련 Issue가 없다면 간단하게 글로 설명 적기
- (필요시) Reviewer에 인원 추가해 코드 리뷰 요청


## 5. Branch
### 기본 사용법:
- `Type/브랜치 이름`

### Type 종류:
- `Feature/`: 새로운 기능 추가
- `Refactor/`: 코드 리팩토링
- `Docs/`: 문서 작성/수정
- `Fix/`: 버그 수정
- `Task/`: 작업/태스크

### 예시
- `Refactor/added_ck_format`
- `Feature/HRNet_imp`
- `Fix/resize_in_infer`
 
 ### Branch 생성 전 확인사항:
 - 작업과 관련된 이름이되, 최대한 간단히 적기