# BEVFusion documentation agent context

이 디렉터리에서는 `README.md`가 문서 router다. CURRENT 문서를 실행 기준으로,
EVIDENCE를 수치 근거로, HISTORICAL을 과거 재현용으로만 사용한다. Dataset 경로가
관련되면 `../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`를 함께 확인한다.

2026-10-11부터 Base5 모델·학습·최적화의 현재 결정과 남은 순서는
`261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub`에서
먼저 확인한다. README는 인덱스, EVIDENCE는 실측, Vast 계획은 원격 상세다.
현재 계획을 여러 문서에 길게 중복 복사하거나 동일 목적의 계획 파일을 추가하지 않는다.
depth 25cm는 계획 후보이고 실제 준비본은 아직 50cm다. 다음 임대용 고정 팀 이미지 게시는
계획 단계이며 현재 설치된 인스턴스 교체나 이미지 업로드를 수행한 것으로 기록하지 않는다.

문서는 정상적인 한국어 띄어쓰기와 일관된 ‘~다’ 문체로 작성한다. 짧게 적더라도 단어를
붙이거나 영어 키워드만 나열하지 않는다. 코드·명령·식별자·경로·해시·측정 수치는 교정 중에도 보존한다.
계획, 로컬 검증 완료, 실제 원격 검증 완료를 구분하며 날짜가 다른 기록을 현재 상태처럼 섞지 않는다.
스크린샷과 가격은 관측 시점·원본 출처를 기록하고, 민감 정보나 ignored 산출물을 강제로 추적하지 않는다.

2026-10-10 사용자 정정에 따라 여러 대화에 걸친 하나의 task가 완료되면 구현·검증·문서를
모아 커밋·push한다. 매 응답이나 작은 검사마다 수행하지 않는다. 미전송 변경을 장기간
누적하지 않으며 현재 브랜치에서 일반 push만 사용한다.
