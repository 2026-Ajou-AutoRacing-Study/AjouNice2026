# BEVFusion documentation agent context

2026-10-12 최신은 EVIDENCE 38절의 전체 객체 분포 재분석이다. 37절의 저장된 출력을
사용해 병렬화 고정 compile OFF/ON의 평균·중앙값·p90/p95/p99와 미대응을 구분했다.
검출 중심은 대체로 sub-mm이며 30m 장애물 코어 평균/p95는 0.699/1.288cm다. 약49cm는
0.5m 대응 gate 안의 최대이고 절대 상한이나 GT 오차가 아니다. 2m 민감도·추적 생애
구간별 집계도 보존했다. 일반 compile 기본 ON은 보류하며 병렬화 우선이다. 상태 차이를
GT AVE/ID switch/live ROS 인증으로 읽지 않는다. production·훈련은 유지한다.

2026-10-12 최신은 EVIDENCE 36절이다. 학습된 E9/E24·4090 300장 비용·전체 val 745장
FP32/AMP/OFF/ON과 20GiB cap 검사를 완료했다. AMP 기본/병렬화/compile 결합은 약
79/72/66ms다. AP 유지와 고정 수치 gate 일부 실패를 구분하며 production 기본값은 유지한다.
병렬화 우선 실전 시험·실제24GB·live ROS/LAN/tracker·100m가 다음이다. 빈 출력 COST
fixture를 완성 모델 근거로 사용하지 않는다. 아래 35절은 앞선 입력·연속 bag 검사다.

2026-10-12 최신은 EVIDENCE 35절이다. Base5 별도 ROS trial·서버 강도 ÷255·원본
JPEG 전달을 구현하고 실제 메시지→bridge→TCP→학습 E24의 12장 정합을 통과했다.
대표 dev bag의 모든 적격 1,570프레임 AP·threshold·연속 실패 진단을 완료했다.
고주로 1m 실패와 실제 출력 부재를 구분한다. production E25·기존 threshold는 유지한다.
다음은 학습된 출력 최적화/4090·실제24GB, live ROS queue·tracker·100m 확인이다.
함수/TCP 입력 정합은 live full-stack 인증이 아니다. 아래 34절은 앞선 표본 검사다.

2026-10-12 앞선 결과는 EVIDENCE 34절이다. 대표 dev bag 4개·256프레임의 동일
30m 입력/GT에서 E25/E9/E24를 실제 추론하고 공통 ROS NMS로 비교했다. Base5의
입력 tensor에만 intensity ÷255를 적용했으며 production adapter는 아직이다.
전체 연속 tracking·live ROS·24GB 인증이 아니다. 다음은 실제 ROS 입력 동등성,
연속 bag의 threshold·누락 검사와 학습된 출력 최적화다. 아래 33절은 수집 데이터 비교다.

2026-10-12 최신 결과는 EVIDENCE 33절이다. 24 epoch·E9/E24 Drive 백업을 확인했고
동일 Base5 745장·radial 30m에서 E25/E9/E24를 실제 추론했다. native intensity의
0~1/0~255 불일치를 보정하지 않은 E25 결과는 비교 결론으로 쓰지 않는다. E25 원래
val control도 재현했다. 다음은 Base5 ROS 강도 adapter·동등성과 공통 bag·threshold,
학습된 출력의 최적화/24GB gate다. production·데이터·훈련은 유지하며 아래 실행은 이력이다.

2026-10-12 로컬 후속은 EVIDENCE 32절이다. 현재 5K/11,052 GT의 native 점·실제 crop
foreground·E15 분모를 전수 대조하고 미촬영 25K metadata와 bag 22개를 조사했다.
약한 proxy 1.06%와 복합체 unknown을 구분한다. 사람 원거리 train 희소성과 장애물
30~50m의 낮은 AP를 무조건 GT 오류로 단정하지 않는다. 다음 수집의 현재 명세와 원격
학습은 변경하지 않았다. 실제 prediction 분해·100m bag 변환/평가는 아직이다.

2026-10-12 후속 v2는 EVIDENCE 31절의 구간 mAP 누락 보완이다. 같은 run의 E11~15
클래스 AP에서 정확한 mAP를 복원했고 E15 체크포인트에서 E16으로 재개했다. 원거리 mAP는
세 클래스 모두 GT가 있을 때의 동일 비중 평균이며 여섯 구간 mAP를 visible로 정의한다.
동결 훈련 source·계약·optimizer·스케줄은 유지한다. 아래 E12 진행은 최초 확장 당시 이력이다.

2026-10-12 최신 실행·W&B 패널 구성은 EVIDENCE 31절이다. 본학습은 사용자 실행으로
이미 시작됐다. 요청에 따라 원거리 평가만 추가하고 같은 훈련 계약과 W&B run으로 재개한다.
동결 source를 수정하거나 모델·LR 스케줄을 바꾸지 않는다. 실제 중단·재개 근거와 현재
상태는 허브·31절을 따른다. 아래 ‘본학습 미시작’은 해당 시점의 이력이다.

2026-10-11 최신 실행 결정은 EVIDENCE 30절이다. 동일 3클래스 E1 비교를 완료해 호환
nuScenes BEV·fusion·head 추가 재사용을 채택하고 E2 비교는 생략했다. 사용자가 후보
E1에서 `--resume`으로 E2~24를 이어간다. 본학습·상주 watcher는 미시작이다. 아래
29절의 fresh 시작·4클래스 재개는 이전 이력이며 현재 허브와 Vast 최신 실행 절을 우선한다.

2026-10-11 최신 클래스 결정은 EVIDENCE 29절의 Animal → Pedestrian 통합이다.
원본 의미·BBox·수집은 보존하고 3클래스 파생 infos와 별도 fresh 실행을 사용한다.
아래 4클래스 E2 재개는 비교 이력이다. 명령은 Vast 문서의 최신 3클래스 절을 따른다.

이 디렉터리에서는 `README.md`가 문서 router다. CURRENT 문서를 실행 기준으로,
EVIDENCE를 수치 근거로, HISTORICAL을 과거 재현용으로만 사용한다. Dataset 경로가
관련되면 `../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`를 함께 확인한다.

2026-10-11부터 Base5 모델·학습·최적화의 현재 결정과 남은 순서는
`261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub`에서
먼저 확인한다. README는 인덱스, EVIDENCE는 실측, Vast 계획은 원격 상세다.
현재 계획을 여러 문서에 길게 중복 복사하거나 동일 목적의 계획 파일을 추가하지 않는다.
2026-10-11 저녁 학습 상태 점검·FP32 matching·제한 복구와 optimizer 이름/순서 재개
검사는 EVIDENCE 28절이 최신이다. 원본 실패 실행과 검증 완료 실행을 구분하고, 아래의
‘본학습 미시작’은 사용자 실행 전 준비 이력으로 읽는다. 현재 재개 경로·검증 결과는
계획 허브와 Vast 실행 절을 먼저 확인한다.
최신 16:45 KST CPU 경합 검사는 EVIDENCE 27절이다. 로컬 raw bag의 실제 측위·tracking·
planning·제어와 신호등을 실행했지만 GT-driven planning과 독립 비용용 인지를 결합한
시험이다. 학습된 OD의 ROS bridge·LAN·실제 4090 24GB·13Hz 인증과 구분한다.
전체 출력 활성화는 모든 trajectory validator flag 통과를 뜻하지 않는다. 원본 입력 공백,
로터리 목표 실패·기록된 목표로 재실행과 초기 harness 실패를 보존했다. 생산 설정과 Vast,
본학습은 변경하지 않았고 다음은 승인 후 본학습·백업이다.
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
