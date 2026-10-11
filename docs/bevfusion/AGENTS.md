# BEVFusion documentation agent context

이 디렉터리에서는 `README.md`가 문서 router다. CURRENT 문서를 실행 기준으로,
EVIDENCE를 수치 근거로, HISTORICAL을 과거 재현용으로만 사용한다. Dataset 경로가
관련되면 `../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`를 함께 확인한다.

2026-10-11부터 Base5 모델·학습·최적화의 현재 결정과 남은 순서는
`261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub`에서
먼저 확인한다. README는 인덱스, EVIDENCE는 실측, Vast 계획은 원격 상세다.
현재 계획을 여러 문서에 길게 중복 복사하거나 동일 목적의 계획 파일을 추가하지 않는다.
최신 15:49 KST 학습 실행은 EVIDENCE 25절이다. 사용자 선택의 공간 5/20cm·depth 25cm에서
실제 4090 OFF/ON·gradient·batch 4→3 재개·FP32 val 검사를 완료하고 새 fine 준비 계약을
만들었다. 학습 camera backbone compile ON을 추천하지만 본학습·백업 watcher는 아직이다.
추론 compile/병렬화의 기본 OFF와 학습 ON 추천을 구분한다. 이전 준비 폴더를 강제 resume하지 않는다.
아래의 ‘학습 미실행/4090 학습 비용 미확인’은 짧은 학습 승인 이전의 이력이다.
최신 분리 stream 결과는 EVIDENCE 24절이다. sparse count의 조기 읽기를 재현하고
camera-side·default-stream LiDAR로 우회했다. 추론 전용 opt-in이며 기본 OFF다.
비용 단축·진단 통제·학습된 모델의 출력/정확도 gate를 구분한다. 기존 준비 계약과 학습은
변경하지 않았으며 22절의 실패 prototype 제거는 수정 전의 이력으로 보존한다.
최신 원격 실행은 EVIDENCE 23절의 2026-10-11 학습 제외 검증이다. 25cm runtime 후보의
새 prepare-only를 만들었지만 실제 학습 비용·저장/재개와 본학습은 실행하지 않았다.
compiler 출력 gate 실패, query 고정 진단 통과, 빈 출력의 형식적 일치를 구분한다.
추론 compile 기본값은 OFF이며 비용용 checkpoint는 학습 초기화에 사용하지 않는다.
아래의 ‘원격 미접속·아직 50cm 준비’는 과거 이력이다. 다음 임대용 고정 팀 이미지 게시는
계획 단계이며 현재 설치된 인스턴스 교체나 이미지 업로드를 수행한 것으로 기록하지 않는다.
추가 로컬 학습 가속의 최신 근거는 EVIDENCE 21절이다. 기본 config와 opt-in 후보,
5060 Ti 실측과 아직 수행하지 않은 4090 재확인을 구분한다.
후속 로컬 전체 추론 비교는 EVIDENCE 22절이다. 300장 비용 이득을 확인했지만 decoded
출력 gate가 미완료라 compile 기본값은 OFF다. 사용자 재개 전 Vast에 접속하지 않는다.
같은 모델·입력의 수치 통제와 재로드 비용 비교를 구분하며 실패한 prototype은 배포하지 않는다.

문서는 정상적인 한국어 띄어쓰기와 일관된 ‘~다’ 문체로 작성한다. 짧게 적더라도 단어를
붙이거나 영어 키워드만 나열하지 않는다. 코드·명령·식별자·경로·해시·측정 수치는 교정 중에도 보존한다.
계획, 로컬 검증 완료, 실제 원격 검증 완료를 구분하며 날짜가 다른 기록을 현재 상태처럼 섞지 않는다.
스크린샷과 가격은 관측 시점·원본 출처를 기록하고, 민감 정보나 ignored 산출물을 강제로 추적하지 않는다.

2026-10-10 사용자 정정에 따라 여러 대화에 걸친 하나의 task가 완료되면 구현·검증·문서를
모아 커밋·push한다. 매 응답이나 작은 검사마다 수행하지 않는다. 미전송 변경을 장기간
누적하지 않으며 현재 브랜치에서 일반 push만 사용한다.
