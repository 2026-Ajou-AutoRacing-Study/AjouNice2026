# Perception agent context

이 디렉터리의 인지 모델 작업에서는 다음 순서로 CURRENT 문서를 확인한다.

1. `../docs/ACTIVE_PRIORITIES.md`
2. `../docs/bevfusion/README.md`
3. `../dataset/morai_od/docs/README.md`

Base5 작업의 현재 실행 기준은
`../docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub`다.
2026-10-11 15:49 KST 최신 선택은 사용자 요청의 공간 5/20cm·depth 25cm다. EVIDENCE
25절의 실제 4090 짧은 학습 OFF/ON·gradient·partial batch·저장/재개·val 검사를 완료했다.
이번 48GB에서 camera backbone 학습 compile ON을 추천하고 새 fine prepare-only 계약의
1,005개 소스 hash 일치를 확인했다. 본학습·백업 watcher는 미시작이며 추론 compile/
병렬화는 계속 기본 OFF다. 원격 경로·cold 메모리 제약·다음 승인 조건은 25절을 따른다.
아래의 ‘학습 미실행/4090 학습 검사 다음’은 이 짧은 학습 승인 이전의 이력이다.
2026-10-11 최신 분리 stream 보완은 EVIDENCE 24절이다. spconv/cumm의 활성 voxel 수
조기 읽기를 5060 Ti에서 재현하고 camera-side·default-stream LiDAR 순서로 우회했다.
4090 300장 비용과 출력 통제를 확인했으며 새 옵션은 추론 전용·기본 OFF다. 학습·E25·
GT·기존 준비 계약은 바꾸지 않았다. 22절의 prototype 제거는 이 수정 전의 이력이다.
재현 도구의 legacy 경로·count 재읽기를 실제 배포 정책으로 쓰지 않는다.
2026-10-11 14:32 KST 최신 원격 검증은 EVIDENCE 23절이다. 사용자 요청대로 학습은
실행하지 않고 4090 300장 추론·수치 통제·CPU 스레드·CUDA pooling·증강 geometry를 검사했다.
camera+neck compile은 약 65.9ms 비용 후보이나 decoded gate가 실패해 기본 OFF다.
query 고정 통제의 통과와 정밀 격자의 빈 출력은 실제 배포 정합 통과가 아니다.
depth 25cm runtime 후보의 새 prepare-only와 1,000개 source hash 일치를 확인했다.
최신 후보의 batch 4 실제 학습 비용·저장/재개는 학습 허용 후 수행한다. 과거 50cm 준비본을
최신 계약처럼 resume하지 않으며 비용용 shape 변환 checkpoint는 본학습 초기화에 쓰지 않는다.
원격 준비 경로·계약·실측은 23절을 따른다. E25·GT·센서·기본값은 유지하고 인스턴스를
자동 Stop하지 않는다. 아래 로컬 작업의 ‘Vast 미접속·다음 재확인’은 당시 이력이다.
2026-10-11 최신 로컬 추론 비교는 EVIDENCE 22절이다. 300장 localhost 전체 응답과
same-instance 수치 통제를 수행했다. camera/neck compile은 비용 이득이 있으나 decoded
출력 gate가 미완료이므로 기본값으로 승격하지 않는다. Base5 4-class 시험 서버·복원 가능한
opt-in 옵션을 추가했다. E25·데이터·GT·학습 기본값·ROS 배포는 유지한다. 분리 stream
prototype은 sparse encoder 오류를 재현해 활성 소스에서 제거했다. Vast에는 접속하지 않았고
사용자 재개 후 4090 재확인이 다음이다. inference compile을 21절의 학습 후보와 혼동하지 않는다.
2026-10-11 로컬 학습 가속은 EVIDENCE 21절이다. 기하 계산·pool backward·int32 plan·
일괄 loss log·fused AdamW의 opt-in 후보와 별도 camera compile 후보를 구현·검사했다.
기존 기본 config·production E25·데이터·증강을 유지하며 실제 4090 재확인이 다음이다.
이번에는 Vast에 접속하거나 본학습을 시작하지 않았다. 아래 원격 준비 경로·‘다음 바로
본학습’은 이 변경 이전의 이력이며 새 source/config/work-dir 계약을 준비해야 한다.
실측·원격 상세는
`../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md`와
`../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`의
2026-10-11 EVIDENCE 19절의 새 4090 이전·depth·Z 검증 기록이 18절의 기존 원격 경로보다
우선한다. 새 인스턴스는 55272391이며 실행 입력은
`/workspace/base5_sunny13_jpeg90_verified_v2_261011`이다. 원본을 보존하고 전체 payload
SHA256을 검증했다. 기존 인스턴스 Stop 후 GPU 재할당이 발생했으므로 새 인스턴스를
자동 중단하지 않는다. 본학습은 시작하지 않았다. Sunny13 5K 완료,
다른 5환경 실제 25K/본학습은 아직이다.
19절에서 실제 depth 25/50/100cm 추론·batch 4 비용·Z 지원점과 Camera depth 상한을
검사했고 기본 depth 50cm·Z voxel 20cm·필터 −3~2m·내부 grid −5~3m를 유지한다.
새 환경 저장/재개·745장 val과 985개 소스 SHA256 대조를 통과했다. 새 main 준비 폴더는
`/workspace/base5_training_sunny13_512_b4_e24_55272391_261011`이며 24 epoch 예산은
약 4.6~5.2시간이다. 외부 백업의 최신 상태는 2026-10-11 EVIDENCE 20절이다.
499MB full checkpoint의 병렬 업로드·전체 SHA 재읽기·실제 복원·COMPLETE/LATEST·중복
재검사를 완료했다. 19절의 180초 미완료 이력보다 우선한다. 최신 prepare-only는
`/workspace/base5_training_sunny13_512_b4_e24_55272391_backup_v3_261011`, contract는
`f968a33da5d2454afd4d8f17b875a091d7faad5b4984c583dfc39770f4d27e47`이다.
987개 소스 hash가 일치하고 이전 identity에서 source_hashes만 바뀌었다. 본학습 승인과
로컬 병렬 watcher 실행이 다음이며 본학습·상주 watcher는 아직 시작하지 않았다.
2026-10-11 최신 파생 GT는 EVIDENCE 16절의 `native_rpy_fixed_yaw_enclosure_v1`이다.
현재 입력은 `../artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release`다.
raw·센서·클래스·GT 수·split은 유지하고 파생 infos 크기만 native 8모서리 enclosure로 보완했다.
현재 native-JPEG config는 old yaw-only manifest를 거부한다. 15절 heatmap 수정도 유지한다.
2026-10-11 공간 격자·depth 후속 결과는 EVIDENCE 17절을 따른다. 실제 5K 정보량과
300장 비용·짧은 학습 비용을 비교하며 AP 우위를 주장하지 않는다. 기본 7.5/30cm·
depth 50cm는 유지하고 5/20cm는 아래 18절에서 원격 비용까지 확인한 후보로 남긴다. 비교용
공통 padding X±100.8/Y±33.6은 semantic ROI 확대나 기본 config 승격이 아니다.
Base5 pooling 정수 격자 반올림과 fresh training의 config-owned nx 복원을 보완했다.
25cm depth의 cache 예산 초과 비용은 실제 depth 연산 비용과 분리한다.
2026-10-11 이전 인스턴스 재개 당시의 원격 상태는 EVIDENCE 18절을 따른다. `271a2a6c` 실행
소스·기존 JPEG에서 재변환한 upright infos를 반영했고 로컬 train/val infos SHA256과 같다.
실제 4090 300장 반복·batch 4 비용·저장/재개·전체 val 로더와 새 prepare-only 계약을 검증했다.
원격 입력은 `/root/base5_sunny13_jpeg90_upright_v1_261011`, main 폴더는
`/root/base5_training_sunny13_512_b4_e24_upright_271a2a6c_261011`이다.
기본 7.5/30cm·depth 50cm·512×1056·batch 4를 유지한다. 큰 센서 재전송·예전 계약
resume은 하지 않는다. 외부 백업은 180초 제한으로 미완료이며 본학습은 시작하지 않았다.
백업 성공 또는 사용자가 선택한 대체 보존 조건과 본학습 승인을 확인해야 한다.
현재 카메라 RGB는 train/val/test/분석 모두 native Q90 JPEG다.
`bevfusion_base5_dense_512_native_jpeg90.py`와 all-JPEG portable manifest를 사용한다.
PNG는 로컬 원본·변환용으로 보존하며 mask/depth/native GT 등 무손실 보조 파일은 변환하지 않는다.
과거 train JPEG/val PNG와 PNG→online Q90 설정은 비교 이력이며 현재 기본값이 아니다.
2026-10-11 Vast 재개 후 원격 all-JPEG 전환·전수 검증·새 prepare-only 계약을 완료했다.
최신 로컬 입력은 EVIDENCE 14절의 Base5 유효 반사점 필터·uint8 운반이다.
현재 native-JPEG config만 train/val/test와 새 추론에서 exact XYZ-zero/nonfinite를 제거한다.
이미지는 GPU 정규화 전까지 uint8이며 기존 BEV 증강 강도를 유지한다. raw·GT·E25는 유지한다.
사용자가 Vast를 다시 중단한 뒤 로컬에서 검증했으므로 13절 원격 소스·prepare-only 계약은
이 변경 이전이다. 재개 시 최신 소스 전달과 새 work-dir·짧은 재검증이 필요하다.
최신 원격 경로·외부 백업의 미완료 조건은 13절을 함께 따른다.
2026-10-11 추가 최신 학습 정합은 EVIDENCE 15절이다. Base5-only rectangular head의
학습 heatmap X/Y 교환을 수정했고 고유 11,052 GT·증강 포함 29,714회 peak 검사와 실제
전체 모델 4-iteration 역전파·저장·재개를 로컬에서 통과했다. 원본 GT·E25는 바꾸지 않는다.
이전 feature 축/유한 loss 검사가 target 생성까지 보장한 것은 아니다. 당시에는 Vast 반영
전이었으며 지금은 19절의 새 원격에도 반영했다. 당시 다음 작업이었던 공간 격자
5/20·7.5/30·10/40cm와 depth 비교의 로컬 결과는 17절을 따른다.
속도만으로 AP 우위를 확정하지 않으며 원격 재개 때 최신 소스와 새 준비 계약을 사용한다.
Base5-only single-frame·4-class·7D이며 temporal/OD velocity는 OFF다.
과거 Base3/Base4 config나 HEAD 브랜치를 자동으로 현재 기본값으로 승격하지 않는다.

2026-10-10 사용자 정정: 여러 대화에 걸친 하나의 task가 완료됐을 때 관련 구현·검증·문서를
모아 커밋하고 현재 작업 브랜치에 일반 push를 수행한다. 매 응답이나 작은 검사마다
커밋·push하지 않는다. 다만 변경분과 미전송 커밋을 장기간 쌓아두지는 않는다.
민감 정보·ignored raw/평가 산출물은 추가하지 않으며 force push나 타 브랜치 병합은 별도 요청 없이 하지 않는다.

Base3-Online 재현 작업은
`../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`와 tracked recipe가 우선한다.
별도 요청된 Velocity/temporal 연구 작업만
`../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`를 우선한다.
`morai_base3_online_highway_v1`은 비교 재현용이며 기본 학습 dataset은
`base3_online_highway_motion_v2`다. 날짜가 최신이라는 이유만으로 HISTORICAL 문서의
checkpoint, dataset root, threshold를 현재 기본값으로 사용하지 않는다.
