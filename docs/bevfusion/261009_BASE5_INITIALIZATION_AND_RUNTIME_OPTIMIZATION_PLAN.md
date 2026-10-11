# Base5 학습 초기화와 BEV 연산 최적화 계획

2026-10-09 KST 계획과 당시 미구현 상태를 보존한다. **현재 순서는 아래 현재 계획 허브가 우선**한다.
Sunny13 수집, 원본 annotation, collector runtime과 production E25는 변경하지 않는다.
설명용 원리와 개발 계약을 구분하며, 실측은 연결된 EVIDENCE 문서를 따른다.

<a id="base5-current-plan-hub"></a>

## 현재 계획 허브 — 2026-10-12 학습 완료 모델의 4090 추론 검증

**10-12 추가 컴파일 검토 완료:** [37절 전달·연속 bag·실제 EKF 코어 영향](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-delivery-tracking-review-261012)을 우선한다.
AP가 거의 같아도 장애물 일부에서 연속 association/coasting 출력 차이가 커졌다.
일반 compile 기본 ON은 보류하고 병렬화 우선 후보를 유지한다. 연산/레이아웃 유지는
완전한 해결이 아니며 CUDA Graph 병렬 prototype은 약 72ms로 추가 속도 이득이 없다.
이 코어 시험은 live ROS/실제24GB/UNKNOWN 후처리/GT 속도 평가 인증이 아니다.
다음은 실제24GB Base5 ROS trial의 queue·LAN·tracker다. 훈련·production E25는 유지한다.

**10-12 06:24 KST 최신 완료:** 학습된 E9/E24를 원격 4090에서 실제 출력이 있는 상태로
300장 비용/출력 비교와 전체 val 745장의 FP32·AMP·병렬화·compile 검증에 사용했다.
학습 geometry는 유지했고 20GiB allocator cap에서 AMP 기본 약 79ms, 병렬화 약 72ms,
compile+병렬화 약 66ms를 확인했다. AP는 거의 유지되지만 고정 decoded 수치 gate는
일부 실패하며 기본 재실행에도 작은 차이가 있다. 같은 모델/고정 raster 통제에서는
병렬화 32/32를 통과하고 compile은 추가 수치 차이가 남았다.
실패 장면 8장의 추가 진단에서도 고정 raster 병렬화는 통과했고, 같은 입력의 1픽셀
depth 변화가 기본 출력 흔들림 일부와 함께 나타났다. production raster 정책은 미변경이다.
[36절 실측·수치 통제·메모리·제한](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-learned-runtime-vast-261012)을 따른다.
병렬화 우선 실전 시험·compile 추가 후보로 두며 production 기본값과 훈련은 변경하지 않는다.
다음은 실제 24GB·live ROS queue/LAN/tracker와 100m·새 환경/자산별 확인이다.
아래 05:50의 ‘학습된 출력 최적화가 다음’은 이번 검사 이전의 이력이다.

**10-12 05:50 KST 최신 완료:** Base5 별도 ROS trial 경로를 구현했다. 원본 JPEG를
Docker에서 전처리하고 ROS 강도는 서버에서만 고정 ÷255 처리한다. 실제 메시지→bridge→
private TCP→학습된 E24의 12장 입력·출력 정합 검사를 통과했다. 같은 대표 dev bag의
모든 적격 1,570프레임을 E25/E9/E24로 비교했고 threshold·연속 매칭 실패도 분석했다.
장애물·보행자 개선은 유지됐지만 고주로 1m 매칭 실패와 실제 출력 부재는 다르다.
[35절 구현·AP·threshold·제한](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012)을 따른다.
다음은 학습된 출력의 최적화 정합·4090/실제 24GB 비용, live ROS queue·LAN·tracker,
고주로 위치 오차 원인 분해와 100m 확인이다. production E25·기존 threshold는 유지한다.
아래 04:53의 ‘입력 adapter·연속 비교가 다음’은 이미 완료한 앞선 이력이다.

**10-12 04:53 KST 최신 완료:** 대표 dev bag 4개에서 동일 256프레임·30m를 추출해
E25/E9/E24를 실제 추론했다. ROS 원본 강도는 보존하고 Base5 입력 tensor에만 ÷255를
적용했다. 공통 ROS NMS 이후 장애물 AP는 65.86/76.61/79.03%, 보행자는
55.04/81.70/81.91%이며 차량은 비슷하다. 전체 연속 프레임·tracker·실시간 ROS 시험은
아니다. [34절 입력·GT·결과·제한](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-rosbag-quick-common30-261012)을 따른다.
다음은 실제 Base5 ROS adapter와 입력 동등성, 연속 bag의 threshold·누락 검사,
학습된 출력의 최적화·24GB 비용이다. production·훈련·데이터·다른 agent 작업은 유지했다.

**10-12 04:30 KST 최신 완료:** 24 epoch 학습과 E9 best/E24 마지막 체크포인트의
Drive readback을 확인했다. 같은 Base5 val 745장·radial 30m에서 강도 스케일을 맞춘
E25/E9/E24를 실제 추론했고, 장애물 AP는 60.76/81.95/83.49%다. E25는 원래 Base3
val 약 98%도 재현했다. 서로 다른 데이터·거리의 AP를 그대로 비교하지 않는다.
전체 100m best E9와 근접 평가의 E24를 모두 보존하며 production은 아직 E25다.
[33절 계약·결과·보존](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-common30-e25-comparison-261012)을 따른다.

**배포 전 입력 검사 완료:** Base5 native intensity는 0~1, Base3 및 확인한 기존 ROS bag은
0~255다. Base5-only `ros255` 서버 adapter·실제 메시지/오프라인 동등성을 통과했고
E25는 raw intensity를 유지한다. 원본·훈련·production 정책은 변경하지 않았다.
연속 dev bag 검출/threshold 진단도 완료했으며 다음은 학습된 출력의 compile/
병렬 추론 정합·실제 24GB 비용과 live ROS/tracker다. 아래 ‘진행 중’은 당시 실행 이력이다.

**10-12 로컬 원거리 조사 완료:** Sunny13 5K·현재 GT 11,052개를 실제 crop foreground와
native LiDAR 지원점으로 전수 대조했다. 약한 단서 proxy는 1.06%이며, 안 보이는 GT가
전반적으로 학습을 망친다는 근거는 없다. 다만 사람 50~80m는 train 25 / val 46개이고,
장애물 30~50m는 카메라에 보이는 val이 많아도 AP가 낮아 실제 prediction 원인 분해가 필요하다.
나머지 25K 명세와 기존 bag 22개도 확인했다. 기존 bag 평가기의 30m/marker/GT 정책을
새 모델에 그대로 쓰지 않는다. [32절의 수집 방향·근거·평가 준비 제한](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-range-evidence-local-261012)을 따른다.
이번 조사는 로컬 CPU 전용이며 원격 학습·완료 5K·frozen 25K 명세는 변경하지 않았다.

**10-12 후속 로깅 보완 완료:** 평가 확장 v2에서 누락된 50~80/80~100m 구간 전체 mAP를
추가하고 기존 네 구간 mAP도 W&B에 노출한다. 저장된 E11~15 클래스 AP의 정확한 평균을
같은 run의 원래 epoch 좌표로 복원했고 온라인 history를 확인했다. E15 완료 체크포인트에서
E16으로 재개해 정상 학습을 확인했다. 훈련 계약·모델·optimizer·24 epoch 스케줄은 유지한다.
새 자동 계산은 E16 이후 validation에 적용되며 아래 E11 추가·E12 진행은 최초 적용 이력이다.
지원 GT와 키·복원 방식·실제 재개는 [31절 후속 보완](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-distance-resume-261012)을 따른다.

**2026-10-12 최신 작업 완료:** 사용자가 시작한 3클래스 본학습은 진행 중이다. 사용자 요청으로
10에폭 전체 검증·체크포인트 완료 후 잠시 중단하고, 기존 0~50m 거리 진단에 50~80m와
80~100m AP를 추가해 같은 W&B run에서 11에폭부터 재개했다. 11에폭 전체 검증과 새 지표의
온라인 반영을 확인했고 12에폭이 진행 중이다. 전체 AP·모델·optimizer·24 epoch
LR 스케줄·데이터·초기화는 유지한다. 동결 학습 소스는 수정하지 않고 별도 SHA256으로
검사하는 평가 전용 실행 확장을 사용한다. 실제 중단·재개 확인은
[31절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-distance-resume-261012)을 따른다.
아래 10-11의 ‘본학습 미시작’은 초기화 채택 당시의 이력이다.

**최신 결론: 호환 nuScenes 가중치의 추가 재사용을 채택한다.** 같은 3클래스 데이터·설정에서
camera+sparse 기준과 호환 융합·BEV·검출 부분까지 재사용하는 후보를 각각 1 epoch 학습하고
745장 전체 validation을 완료했다. 평균 center AP는 49.01%→70.19%, 장애물은 28.93%→43.78%다.
차이가 명확해 2 epoch는 생략했다. 24 epoch LR 스케줄을 유지한 후보의 E1 체크포인트에서
사용자가 `--resume`으로 E2~24를 이어간다. 장시간 본학습과 상주 watcher는 시작하지 않았다.
[30절 초기화 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-full-initializer-trial-261011)가
아래의 이전 새 시작·4클래스 재개 안내보다 우선한다. 기존 실행·데이터·production E25는 보존한다.

**사용자 결정으로 이번 Base5 학습은 Vehicle/Pedestrian/Obstacle 3클래스로 진행한다.**
Animal의 학습 라벨만 Pedestrian으로 변환하며 원본 audit, 자산명, 원래 클래스와 개별
BBox는 보존한다. 재수집과 동물 quota 증량은 하지 않는다. 학습 완료 후 사람·동물·자산별
검출 결과를 확인하고 필요한 보강만 결정한다. [29절 구현·검증 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-animal-pedestrian-merge-261011)가
아래의 4클래스 E2 재개 안내보다 우선한다. 4클래스 E2는 비교 이력으로 보존하고 새
3클래스 work-dir에서 시작한다. 클래스 변경은 optimizer까지 이어가는 resume가 아니다.

**제한 E1/E2 검증을 완료했고 장시간 본학습은 사용자 재개를 기다린다.** 사용자 실행의 첫 학습은 E1을 완료한 뒤
E2 중반 Hungarian 비용 행렬의 비유한값으로 종료됐다. Base5-only FP32 target matching을
추가하고, 원본 E1·optimizer·24 epoch LR 스케줄을 보존한 명시적 migration으로 E2와 E3의
전체 train/745장 val을 완료했다. matching 복구만으로는 E3 mean AP 11.31%였다.
fresh query classifier prior의 실제 누락을 수정하고, nuScenes camera+sparse LiDAR의
축 변환 초기화를 검증했다. 새 E1/E2 전체 검증의 mean AP는 35.16%→45.57%,
E2 차량/보행자/장애물 AP는 72.67/72.40/37.20%다. Animal AP 0과 최종 수렴은 남은 과제다.
이 4클래스 실행의 E2는 보존한다. 최신 3클래스 학습은 별도 초기화·실행 계약을 사용하며
agent는 본학습을 자동 시작하지 않는다.
[중단 원인과 실제 복구 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-training-health-repair-261011)을
최신 실행 근거로 삼는다. 아래 17:46의 준비 이력과 짧은 cost gate를 수렴 인증으로 해석하지 않는다.

Base5 모델·학습·최적화의 **현재 결정, 남은 순서와 상태는 이 절을 기준으로 관리한다.**
`README.md`는 문서 인덱스, EVIDENCE는 실측 근거, Vast 계획은 원격 실행 상세다.
세부 계획·명령·실측을 여러 문서에 복사하지 않고 해당 절을 연결한다.
아래 날짜별 결정은 이 허브의 근거이며, 과거의 ‘다음 단계’를 현재 지시로 재사용하지 않는다.
생성기·수집기와 다른 팀의 전체 우선순위는 각 기존 CURRENT 문서를 유지한다.

| 항목 | 현재 결정·상태 | 다음 할 일 / 상세 근거 |
|---|---|---|
| 학습 입력·모델 | **공간 5/20cm·depth 25cm, 512×1056 Q90 JPEG, ROI X±100/Y±30m를 유지한다. 학습은 Animal을 Pedestrian에 통합한 3-class single-frame 7D다.** temporal/OD velocity는 OFF다. | [29절 클래스 전환](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-animal-pedestrian-merge-261011). 기존 4-class와 production E25는 보존한다. 통합의 정확도 이득은 아직 미검증이다. |
| 로컬 가속 | 실제 5060 Ti 학습 step에서 depth 25cm 약 32.8%, 50cm 약 21.4% 단축을 확인했다. 원래 학습 조건은 유지하며 opt-in 후보로 제공한다. 카메라 compile을 더한 별도 후보는 기존 대비 약 37.6~45.8% 단축이며 수치·저장/재개 검사도 완료했다. | 로컬 근거는 [EVIDENCE 21절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-training-runtime-local-261011), 후속 4090의 실제 fine batch 4 검사는 [25절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-fine-training-compile-vast-261011)이다. 로컬 가속률을 원격에 그대로 적용하지 않는다. |
| 로컬 추론 | 실제 300장 localhost 전체 응답에서 camera+neck compile의 비용 단축은 약 20.1%, 정밀 격자 안에서는 약 15.3%다. 단, decoded 출력 gate가 모두 통과하지 않아 **기본값으로 승격하지 않는다.** | [EVIDENCE 22절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-inference-runtime-local-261011). 비컴파일 추론을 대안으로 유지한다. Base5 4-class 시험 서버를 구현했지만 ROS 배포·학습 완료 정확도·4090 70ms 달성은 아직이다. |
| 최신 4090 재확인 | **학습된 출력의 비용·전체 val에 이어 컴파일 연산/레이아웃 유지·Graph prototype 검토까지 완료.** 일반 compile 약 66ms지만 장애물의 연속 EKF 코어 예외가 있어 기본 ON은 보류한다. 병렬화 약 72ms를 우선한다. | [EVIDENCE 37절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-delivery-tracking-review-261012). 20GiB cap의 48GB와 실제 24GB·ROS/LAN 인증을 구분한다. Graph 약72ms는 추가 이득이 없고 production 조합 거부도 유지한다. 학습은 실행하지 않았다. |
| 학습 compile 선택 | **이번 48GB 4090의 fine 후보에서는 camera backbone 학습 compile ON을 추천한다.** 실제 OFF/ON 각 64 update와 역순 24 update, 수치·gradient·batch 4→3 저장/재개·FP32 val 경로를 통과했다. | [EVIDENCE 25절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-fine-training-compile-vast-261011). 기본값 전체 승격이 아니라 새 opt-in config다. 첫 준비 메모리 약 45.71GiB를 감안한다. 추론 compile/병렬화는 계속 기본 OFF다. |
| 학습·추론 GPU와 배치 | **학습은 48GB·물리 batch 4·누적 1을 유지한다. 대회 추론은 24GB·batch 1을 별도로 검증한다.** 학습을 24GB 예산으로 제한하거나 48GB를 채우기 위해 배치를 늘리지 않는다. | [메모리·배치 결정과 남은 검사](#base5-training-inference-memory-decision-261011). 추론 compile ON도 20GiB 예산에서 실행 근거가 있지만 실제 24GB 카드·학습 완료 가중치 검사는 남아 있다. |
| ROS CPU 경합 | **로컬 6코어/12스레드에서 실제 측위·tracking·planning·제어와 신호등 YOLO를 실행한 부하 검사를 완료했다.** 정상 CPU 평균 약 33~37%, 추가 점유 시 약 76%에서 제어·측위 50Hz·planning 약 10Hz를 유지했다. | [EVIDENCE 27절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-cpu-contention-local-261011). GT로 planning을 구동한 독립 인지 비용 검사이며 ROS detector bridge·LAN·학습된 모델의 정확도·4090 13Hz 인증은 아니다. 배포 시험의 Torch 4·OpenCV 1 제한을 명시한다. production 기본값·Vast·본학습은 변경하지 않았다. |
| 분리 stream 오류 보완 | 설치 라이브러리의 활성 voxel count 조기 읽기를 재현했다. LiDAR를 기본 stream에 두고 camera-side 병렬 실행으로 우회했다. 4090 300장 전체 응답은 비컴파일 약 71.3→65.8ms, compile 결합 비용은 약 59.2ms다. | [EVIDENCE 24절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-parallel-stream-repair-261011). 추론 전용 opt-in이며 기본 OFF다. sparse 예외 해결·입력 통제 검사와 학습된 모델의 실제 출력 gate·AP를 구분한다. 학습은 실행하지 않았다. |
| 이번 학습·분할 | Sunny13 5K의 train 4,255 / val 745 유지. 이번 학습에 rosbag val을 추가하지 않는다. | SaveSensorData val로 checkpoint 후보 선정. rosbag-to-val 구현과 추적·예측 평가 실험은 후속이다. 기존 val 점수와 rosbag 점수를 하나로 합치지 않는다. |
| 장기 데이터 계획 | 나머지 5 환경은 각각 새 배치 5K, 총 30K 계획 유지 | 85/15를 우선 기준으로 두되 전체 장면·공간 group 누출을 검사해 실제 장수를 확정한다. [환경별 배치](../../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md) |
| 초기화·재개 | **camera+sparse에 더해 호환 nuScenes BEV·fusion·head까지** 축·물리 단위를 변환한 초기화를 채택한다. 3클래스 동일 조건 E1 비교를 완료했으며 후보 E1에서 E2~24를 이어간다. | [30절 비교·변환 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-full-initializer-trial-261011). E25/Base4/COST-only 가중치는 사용하지 않는다. 4클래스 E2는 별도 보존한다. |
| 체크포인트 보존·사용자 실행 | **24 epoch 학습과 E9 best/E24 마지막 체크포인트의 Drive 백업·readback을 확인했다.** 원거리 평가 확장도 같은 훈련 계약과 W&B run을 유지했다. | [33절 완료·보존](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-common30-e25-comparison-261012). 과거 실행/재개 명령은 [Vast 운영 절](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011)에 보존한다. 새 학습을 자동 시작하지 않는다. |
| 거리별 평가 | 기존 0~10/10~20/20~30/30~50m에 **50~80/80~100m 두 구간**을 추가한다. 원거리 클래스 AP·구간 전체 mAP·차량 전방/후방 AP·GT 수를 기록하며 여섯 구간 mAP를 노출한다. E11~15의 누락된 mAP 복원을 완료했다. | [31절의 키·W&B 패널 배치와 v2 보완](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-distance-resume-261012). 거리 기준은 ego/LiDAR XY 방사 거리다. 새 mAP는 세 클래스 모두 GT가 있을 때만 동일 비중 평균으로 기록한다. GT 20개 미만은 소표본으로 표시한다. 방사 거리 100m 이상 모서리는 전체 AP에 유지하고 GT 수만 따로 기록한다. |
| 다음 환경 수집과 원거리 정보량 | 완료 5K와 현재 25K 계약은 유지한다. 실제 GT·crop 정보량 조사와 다음 25K의 primary 분포 검사를 완료했다. 먼 차량은 유지하고, 모든 사람·장애물을 최대 거리까지 늘리는 변경은 하지 않는다. | [32절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-range-evidence-local-261012). 보강 우선은 장애물 20~50m, 식별 가능한 사람·장애물 30~60m train 사례다. 최종 모델의 clear/weak·자산별 누락 확인 후 별도 보강을 설계한다. Foggy 첫 pilot에서는 RGB 대비를 실제 확인한다. |
| 다음 임대의 실행 환경 | **의존성·CUDA 연산을 고정한 팀 이미지를 registry에 게시해 재사용하기로 결정. 빌드·게시·Vast pull 검증은 아직이다.** | 로컬 최적화 정리 후 이미지 준비. 현재 설치된 인스턴스는 유지하고 게시 때문에 이번 학습을 지연시키지 않는다. [게시·검증 계획](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fixed-image-publication-plan-261011) |
| ROS 입력·연속 검출 | **별도 Base5 trial의 실제 메시지→TCP 입력/학습 출력 12장 정합, 대표 dev bag 1,570프레임 AP·threshold·연속 실패 진단을 완료했다.** E25 기본값은 유지한다. | [35절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012). 장애물 0.15는 후속 tracker 시험 후보일 뿐 기본값 승격이 아니다. 고주로 1m 실패 중 위치 오차를 검출 부재와 구분한다. |
| 성능 확인·배포 | 장애물·보행자 개선은 공통 30m dev bag에서도 확인했다. **컴파일 영향의 1,570프레임 재추론·실제 EKF 코어 비교까지 완료. depth/정밀 격자 개별 효과·GT 속도 MAE, 실제 24GB·LAN·ROS 전체 비용은 아직 미검증.** production E25는 유지한다. | [37절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-delivery-tracking-review-261012)의 병렬화 우선 후보 → 실제24GB live ROS queue/tracker → 100m·미사용 bag·환경별 확인. 코어 상태 차이를 GT AVE/ID switch나 full-stack 인증으로 읽지 않는다. |

현재 우선 순서는 **학습 완료·E9/E24 보존·공통30m 검출 비교·ROS 입력/TCP 정합·
연속 dev bag 진단 완료 → 학습된 출력의 추론 OFF/ON·4090 비용·전체 val·수치 통제 완료 →
실제 24GB·ROS queue·LAN·tracker와 100m/환경/자산별 확인**이다.
최초 본학습은 사용자가 직접 시작했으며 10-12에는 지표 수정 후 agent가 재개하도록 승인했다. 원격 학습은 tmux로
실행하며 로컬 PC의 통신 중계·토큰 갱신을 유지한다. 상세 명령은 위 Vast 링크를 따른다.
추론 compile은 비용 후보이며 출력 gate가 실패해 기본 OFF다. query 선택을 고정한 원인 분석을
실제 배포 정합 통과로 읽지 않는다. 21절의 학습 gradient 검사 통과를
23절의 전체 decoded 출력 동일성이나 ROS 배포 통과로 바꾸어 해석하지 않는다.
고정 이미지 게시 준비와 rosbag-to-val 구현은 별도 후속 작업이며 이번 요청으로 자동 실행하지 않는다.
다음 임대 전에는 고정 이미지 게시·새 환경 pull 검증을 완료한다.

### 문서 갱신 규칙

- 결정이나 우선순위가 바뀌면 먼저 이 허브의 해당 행을 갱신하고 날짜를 기록한다.
- 실험 결과는 EVIDENCE에 기록하고 허브에는 완료/미완료와 근거 링크만 남긴다.
- 원격 명령·이미지 digest·백업 운영 상세는 Vast 문서에 둔다. 코드·실행 계약과 계획을 구분한다.
- `README.md`와 `docs/ACTIVE_PRIORITIES.md`에는 이 허브의 링크·다음 단계만 갱신한다.
  새 결정을 매번 여러 곳에 길게 복사하거나 동일 목적의 날짜별 계획 파일을 추가하지 않는다.

<a id="base5-training-inference-memory-decision-261011"></a>

### 학습 48GB·batch 4와 대회 추론 24GB의 분리 — 2026-10-11 16:03 KST 합의

사용자는 학습 GPU 메모리와 대회 추론 GPU 메모리를 분리하고, 검증된 **48GB·batch 4·
누적 1·학습 camera backbone compile ON**을 유지하는 방향에 동의했다. 공간 5/20cm,
depth 25cm, 입력 512×1056, LR 1e-4, warm-up 500, 24 epoch와 기존 준비 계약은 그대로다.
이번 갱신은 문서 정리이며 새 GPU 실험·본학습·원격 설정 변경을 실행하지 않는다.

- 학습은 역전파용 중간값·gradient·optimizer 상태와 batch 4가 필요하다. 이번 ON의
  정상 peak allocated 약 26.95GiB / 최초 준비 약 45.71GiB는 **학습 측정값**이다.
  batch 4가 일반 24GB 카드에도 들어간다는 뜻이 아니다.
- 대회 추론은 batch 1이며 학습용 gradient·optimizer 상태가 필요 없다. 5/20cm·D25의
  이전 추론 시험은 allocator 예산 20GiB에서 compile OFF와 camera+neck compile·병렬화
  결합 후보 모두 OOM 없이 실행했다. 정상 allocated 약 1.55~1.61GiB, 시작 피크 약
  17.08~17.14GiB다. **48GB 카드의 예산 제한 시험**이지 실제 24GB 카드 인증은 아니다.
  비용용 가중치의 decoded 출력이 0개였다는 한계도 유지한다. [원자료와 해석](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-memory-batch-decision-261011).
- 학습 compile ON의 가중치는 추론 compile OFF/ON 양쪽에서 사용할 수 있다. compile은
  실행 방식이며 학습 때의 45.71GiB 피크가 가중치에 포함되어 따라오지 않는다.
  추론 compile·병렬화 기본 OFF의 이유는 학습 완료 모델의 출력·정확도 gate가 남아 있기
  때문이지, 24GB에서 실행 불가능하다고 확인했기 때문이 아니다.
- VRAM 점유율 자체를 목표로 batch 6~8을 선택하지 않는다. 과거 더 가벼운 조건의
  batch 4→8 처리량 이득은 약 2.7%였고 현재 fine 후보에서는 재측정하지 않았다.
  batch 확대는 모델 파라미터 개수를 바꾸지 않지만 같은 epoch의 optimizer 갱신 수,
  gradient 통계·학습률·warm-up 해석을 바꾸므로 무료 속도·정확도 향상으로 취급하지 않는다.
  향후 필요할 때만 별도 비용·OOM·학습 설정 검사로 결정한다.
- 나중에 24GB에서 추가 학습해야 한다면 작은 물리 batch와 gradient accumulation을
  검토할 수 있다. 유효 batch를 맞춰도 BatchNorm·난수·부동소수점 연산 때문에 완전히
  같은 학습 경로를 보장하지 않는다. 현재 계약을 그런 설정으로 임의 변경하지 않는다.

추가 최적화를 본학습의 새 선행 조건으로 늘리지 않는다. 큰 병목 개선과 짧은 학습 검사는
완료했으며, 남은 순서는 **승인 후 본학습·외부 백업 → 완성 가중치의 추론 OFF/ON·병렬화
정합/비용 비교 → 실제 24GB 카드에서 cold start·rosbag 연속 입력·ROS 풀스택 확인**이다.
13Hz·장기 수렴·실전 정확도는 아직 보장하지 않는다. TensorRT는 필요하면 수행할 후속 후보다.

<a id="base5-training-compile-short-gate-261011"></a>

### 학습 compile의 짧은 4090 검사 — 2026-10-11 14:45 KST 합의

학습 기회가 적다는 이유로 compiler를 무조건 제외하거나, 검증 없이 본학습에 적용하지 않는다.
카메라 backbone 학습 compile은 짧은 검사 후 선택 가능한 후보로 유지한다.
로컬 검사·추가 약 19~21% step 단축은 EVIDENCE 21절의 결과이고 4090에서의 보장은 아니다.
14:45 당시에는 계획만 반영했다. 이후 사용자가 **5/20cm를 우선으로 짧은 학습을 승인**했고
[EVIDENCE 25절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-fine-training-compile-vast-261011)의
실제 검사를 완료했다. 아래는 최초 7.5/30cm 검사 계획을 보존한 것이며 현재 선택은 허브를
따른다. 본학습은 아직 시작하지 않았다.

1. 현재 512×1056·7.5/30cm·depth 25cm·batch 4·workers 4 조건에서 동일한 runtime
   개선 묶음의 **compile OFF/ON**만 비교한다. camera backbone만 compile하며 sparse
   encoder·head까지 동시에 변경하지 않는다. 같은 camera-only 초기화·입력 순서·증강
   GT·seed·AMP scale·optimizer를 사용하고, 측정 중 소스는 고정한다.
2. 후보당 64 update 이내의 실제 5K 로더 검사로 loader 대기·전체 step·forward/backward·
   메모리를 비교한다. warm-up과 첫 compile 비용을 정상 step 통계와 분리한다.
   loss·gradient·parameter 유한값, 누락 gradient, AMP overflow/skip을 검사한다.
   고정 upstream의 기존 gradient 정합 검사와 실제 loss 검사를 함께 사용한다.
   DropPath 등의 RNG 실행 차이를 장기 수렴 손상으로 단정하거나 bitwise 일치를 강제하지 않는다.
   실제 MMEngine AMP forward/backward 문맥도 확인한다.
3. compile ON 후보에서 별도 2→4 update의 atomic 저장·재개를 확인한다.
   모델 key·optimizer·scheduler·AMP scaler·메인 RNG·resume contract를 검사한다.
   검사 통과와 의미 있는 전체 비용 이득이 있으면 학습 ON을 채택할 수 있다.
   누락 gradient·불안정한 갱신·저장/재개 실패가 있거나 이득이 없으면 OFF로 진행한다.
   유한한 짧은 loss나 정합 통과는 장기 수렴·AP 동등성의 인증이 아니다.
4. 선택 후 새 config/source/work-dir 계약과 예산을 보고하고 별도 승인 후 본학습한다.
   기존 prepare-only의 compile 플래그를 임의로 덮어쓰거나 다른 계약으로 강제 resume하지 않는다.
   학습 시 compile 여부와 무관하게 같은 가중치의 **추론 compile ON/OFF**를 후속 비교할 수 있다.
   학습된 모델의 val·근접/원거리 recall·rosbag 위치 흔들림으로 추론 적용을 결정한다.
   추론만 변경하는 이 비교 때문에 두 번 본학습할 필요는 없다.

분리 CUDA stream의 camera/LiDAR 병렬 prototype은 별개 후보다. sparse encoder 오류로
기각한 것은 그 scheduling 구현이지, sparse encoder 자체나 학습 compiler가 아니다.
관측·미확정 원인은 [EVIDENCE 22절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-inference-runtime-local-261011)을 따른다.

<a id="base5-depth25-local-optimization-plan-261011"></a>

## 2026-10-11 최신 합의: depth 25cm 후보와 본학습 전 로컬 최적화

기록 시각: 2026-10-11 06:43 KST. 사용자는 학습 비용 증가를 감수하되 추론 속도를
가능한 한 유지하는 방향에 동의했고, 본학습 전에 로컬에서 학습·추론 가속을 더 검토하기로 했다.
06:43 갱신 당시에는 **계획 기록만** 수행했다. 후속 로컬 구현·실험 결과는
[EVIDENCE 21절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-training-runtime-local-261011)을
따른다. 원격 준비 계약·본학습은 아직 변경하지 않았다. 외부 백업·복원 완료 상태는
[EVIDENCE 20절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-checkpoint-parallel-backup-261011)을 따른다.
이 절의 후속 순서는 이전의 ‘백업 완료 후 바로 본학습’보다 우선한다.

### 학습 후보와 성능 기대의 경계

- 우선 후보는 입력 512×1056, LiDAR voxel XY 7.5cm / Camera BEV 30cm / fusion·head 60cm,
  depth `[1,100)`m의 **25cm 간격**이다. semantic ROI는 X±100m / Y±30m로 유지한다.
- depth 50cm는 검증된 비용 대조군이자 대안이다. 25cm는 실제 거리 오차를 25cm로
  보장하지 않으며, 원거리 차량의 위치·탐지 개선은 소폭 또는 거의 없을 수도 있다는 가설이다.
  학습 후 AP·rosbag 지표의 우위는 아직 측정하지 않았다.
- 정밀 5/20cm 공간 격자는 근거리 형상 보존·밀집 객체 분리에 더 직접적인 개선 근거가 있지만,
  이번 우선 후보에는 합치지 않는다. 더 작은 voxel은 원거리 LiDAR 점을 새로 만들지 않는다.
  최종 박스 중심은 연속 회귀이므로 head 간격 감소를 위치 오차 감소율로 환산하지 않는다.
- tracker 속도에는 프레임 간 검출 중심의 흔들림·누락·연결 안정성이 중요하다. depth 간격을
  줄였다는 사실만으로 속도 MAE 개선을 보장하지 않는다. temporal/OD velocity는 계속 OFF다.
- 기존 Q90 JPEG 입력·upright GT·split·증강·초기화 정책은 유지한다. 재수집하지 않는다.

| 개별 변경 | 실제 4090 비용 근거 | 해석과 적용 조건 |
|---|---|---|
| depth 50→25cm, 공간 7.5/30cm 고정 | 전처리 포함 추론 평균 67.49→68.71ms, batch 4 학습 570.10→684.77ms/step, 약 +20.1% | 추론 차이는 작지만 무료가 아니다. 추론 cache 예산 512MiB가 조건이다. 25cm plan 약 308.8MiB는 기존 256MiB를 초과한다. |
| 공간 7.5/30→5/20cm, depth 50cm 고정 | 공통 padding 비교 추론 평균 68.57→78.42ms, 학습 약 +21.2%, 메모리 21,340→28,679MiB | 전처리 포함 평균이 13Hz 주기 76.92ms를 넘는다. 일반 24GB GPU의 batch 4 적합성을 가정하지 않는다. |

표는 [18절 공간 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-grid-qualification-261011)와
[19절 depth 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-depth-z-qualification-261011)의
서로 다른 실행이다. 두 변경의 동시 적용 비용이나 학습 완료 정확도를 뜻하지 않는다.

### 다음 실행 순서

| 순서 | 로컬에서 검토할 내용 | 채택·종료 기준 |
|---|---|---|
| 1 | 25cm 후보와 50cm 대조군에서 기존 최적화 상태를 확인하고 학습·추론 병목을 다시 분리한다. | 같은 장면·batch·증강·정밀도·워밍업 조건으로 모델/전처리/loader 대기/forward/backward/optimizer 비용과 메모리를 구분한다. warm 추론과 cold cache 비용을 섞지 않는다. |
| 2 | 측정된 병목부터 CPU decode·운반·loader 반복 비용, GPU의 반복 기하/index 생성과 pooling backward 등 개선 후보를 좁힌다. | 이미 적용한 uint8·분리 pooling·추론 cache를 새 개선으로 중복 계산하지 않는다. 학습 랜덤 증강에 추론 고정 cache를 그대로 재사용하지 않는다. |
| 3 | 이득이 예상되는 후보만 구현·짧은 A/B·정합 회귀를 수행한다. 필요할 때 카메라 부분 compile·연산 실행 방식도 검토한다. | 픽셀·투영·GT·출력·gradient·AMP 갱신·저장/재개의 정합을 지킨다. 모델만 빨라지고 전체 step/service 비용이 늘면 채택하지 않는다. |
| 4 | 채택한 로컬 변경만 원격 4090에 반영하고 비용·메모리·새 학습 준비 계약을 확인한다. | 5060 Ti의 가속률을 4090에 그대로 적용하지 않는다. 25cm 출력 shape·512MiB cache·초기화를 명시하고 새 work-dir/config/source hash를 만든다. |
| 5 | 최종 구성과 시간 예산을 보고하고 승인 후 백업 watcher·본학습을 시작한다. | 기존 50cm prepare-only를 25cm 학습 계약처럼 사용하거나 자동 resume하지 않는다. 최적화 이득이 없으면 검증된 경로로 진행하고 본학습을 무기한 미루지 않는다. |

학습 후에는 동일 mission rosbag에서 원거리 차량의 첫 전달거리·recall, 근거리 누락,
검출 중심의 시간적 안정성, tracking·속도 지표를 확인한다. 비용 시험을 이 성능 검증의 대체로 쓰지 않는다.
Vast 인스턴스의 Stop/destroy·임대 변경은 이 문서 갱신으로 수행하지 않는다.

## 2026-10-10 상태 갱신

[실제 입력·직사각형 축·cache A/분리 B 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)을 완료했다.
[Vast 준비 결과](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과)에
상대경로 기반 5K 전달본, 직접 실행 학습, 실제 4-class AMP 저장·재개,
로컬 백업 검증과 원격 실행 순서를 정리했다. 4090 연산 비용 재측정, 최종 입력 선정,
장기 학습과 체크포인트의 Drive 외부 백업 검증은 아직 수행하지 않았다.
데이터셋 자체의 Drive 백업은 완료했으며 체크포인트 백업과 구분한다.

초기화 대안으로 nuScenes 카메라 backbone/neck의 가중치를 실제로 로딩하고 동일성을 확인했다.
기존 정사각형 체크포인트의 LiDAR/BEV 방향은 새 [Y,X] 계약과 달라 텐서 크기만 맞는다고
전체 가중치를 로드하지 않는다. 이 대안의 준비가 전체 사전학습 가중치 적용을 완료했다는
뜻은 아니다. 원격 학습 전에 전체 축 변환의 검증 결과 또는 대안 선택을 명시한다.
이 대안을 사용하면 depth/class/head와 LiDAR/BEV는 새로 초기화한다.
아래의 'cache/adapter/번들 미구현·GPU 미실행'은 10-09 시점의 이력이며 최신 상태가 아니다.

## Sunny13 수집 종료 후 묶음 실험 합의

2026-10-09 사용자 합의. 이번 Sunny13 5K 수집을 먼저 마치고 MORAI를 끈 뒤,
입력·pooling·초기화 실험을 묶어서 진행한다. 20:49 KST 대화에서 남은 약 1시간 30분은
사용자 예상이며 완료 시각이나 현재 진행률을 인증하는 값이 아니다. 이 합의는 실험 순서의 결정이지
지금 GPU 작업·본학습·다음 환경 수집을 자동 시작하라는 요청이 아니다.

다음 작업은 이 절부터 읽고 아래 입력/crop·4090 비용·CUDA-BEVFusion 조사 링크로
세부 context를 복구한다. 수집 종료와 MORAI 종료를 확인한 뒤 사용자의 실행 요청에 따라
진행한다. Sunny13 5K 완료와 6 환경 30K 완료를 혼동하지 않는다.

원격 학습을 선택하면 [Vast.ai 단일4090 실행 계획](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md)을
함께 따른다. 로컬 검증으로 후보를 좁힌 뒤 임대 4090에서 추론·학습 비용을 재확인하고 최종 설정을
동결한다. SSH/direct runner·전달 용량·외부 checkpoint 백업 및 재개 검증이 선행 조건이며,
임대·업로드·장기 학습은 아직 시작하지 않았다.

| 순서 | 수행할 일 | 완료 조건 |
|---|---|---|
| 1 | 성공/실패·raw/annotation/metadata 무결성, 자산·거리·근접 coverage 및 geometry group split 감사 | 사용 가능한 데이터와 부족분을 분리; 원본 보존 |
| 2 | 학습/Docker/ROS 전처리와 투영 정합, 384×896/448×896/512×1056 crop 비교 | 잘림·가시성·픽셀 밀도로 후보 축소; 기존 320 기준도 보존 |
| 3 | geometry/index cache A → depth/feature 분리 pooling B 구현·검증 | 기존 출력과 gradient 정합, cache 무효화·경계·직사각형 축·stream 검사; 기존 경로 fallback |
| 4 | 동일 조건의 baseline/A/A+B latency·VRAM 및 짧은 학습 step 측정 | 추천 입력·격자·depth·연산 구성 하나와 fallback 하나 선정; 학습시간 실측 갱신 |
| 5 | 공식 nuScenes 호환 초기화 adapter·로딩 감사, 4-class single-frame 학습 smoke 및 resume 확인 | 본학습 준비 결과를 사용자에게 보고; 장기 학습은 별도 요청에 따라 시작 |

해상도 비교에서는 voxel/camera BEV/depth 간격과 class/query·입력 장면·정밀도 등을
맞추고 변경 요인을 분리한다. 새 해상도·depth shape에 필요한 초기화는 명시하며
비용 측정을 학습 완료 모델의 정확도 평가로 해석하지 않는다. CPU 준비 포함 시간과
모델 시간, ROS 전달 지연을 구분하고 p95·drop·peak VRAM을 확인한다.

### 선택과 종료 기준

- 모든 기술의 적용이 목표가 아니다. 출력 정합 실패·이득 부족·과도한 구현 비용은
  근거를 남기고 기존 경로로 돌아간다. 최적화 미완료 때문에 본학습을 무기한 미루지 않는다.
- GPU 전처리·v3 추가 최적화·부분 TensorRT는 앞 단계의 잔여 병목과 일정에 따라 선택한다.
  전체 CUDA-BEVFusion 이주는 후순위이며 이번 묶음 실험의 필수 성공 조건이 아니다.
- 학습 전에 잘림·좌표·속도·메모리 문제는 검증할 수 있지만 해상도별 탐지 성능 우열은
  학습 및 mission rosbag 평가가 필요하다. 본학습 전에 정확도 최적 구성을 확정했다고 주장하지 않는다.
- 원본 이미지·LiDAR·annotation을 보존하면 crop/해상도 및 visibility/info를 다시 만들 수 있다.
  같은 연산을 보존하는 pooling/cache 교체 자체 때문에 재수집할 필요는 없다.
  누락 센서·잘못된 원본 GT·부족한 배치 분포는 별도 감사 후 해당 조건을 보강한다.
- 보고는 수행 내용·측정 결과·불확실성·추천 구성·fallback·남은 과제로 묶는다.
  본학습 시작 직전에 사용자에게 검토받으며, production E25와 기존 tracker는 유지한다.
- 이후 5 환경 수집 전에는 기존 [collector 반복 비용 A/B](../../dataset/morai_od/docs/261009_base5_collection_cost_and_next_experiments.md)
  계획도 유지한다. 이번 GPU 실험과 별개이며 완료된 Sunny13 raw를 바꾸지 않는다.

## 결정과 남은 선택

아래 표는 초기 계획 당시의 후보 상태를 보존한다. 이후 검증한 입력·정밀도·연산 구성과 초기화 준비 상태는
상단의 상태 갱신 및 연결된 최신 EVIDENCE 문서를 따른다.

| 항목 | 현재 상태 | 다음 기준 |
|---|---|---|
| 데이터 | Sunny13 5K 보존, 나머지 5 환경은 각각 새 배치 5K로 총 30K | Base3를 기본 학습본에 섞지 않음; 구성표·주요자산 할당 유지 |
| 모델 | 4-camera + LiDAR single-frame, 7D BBox | Temporal/query memory/OD velocity 회귀 미사용 |
| 이미지 | 코드 후보 320×896, 최종 입력 미확정 | 448×896 아래 정렬 우선, 384×896 비용 대안, 512×1056 원거리 밀도 대안 |
| 범위 | label 중심 ROI 종방향 ±100m, 횡방향 ±30m | encoder padding 범위와 label ROI를 구분 |
| 공간/depth 간격 | 코드 후보 voxel 0.10m / camera BEV 0.40m / depth 1.0m | 최종 선택 전 비용 대조의 0.075/0.30/0.50m와 혼동 금지 |
| 초기화 | 현재 Base5 config는 load_from=None | 호환되는 공식 nuScenes 가중치 재사용을 추천, 새 adapter 검증 후 반영 |
| 연산 최적화 | 기존 CUDA pooling 사용 | A: 고정 기하/인덱스 cache → B: BEVPoolv2 방식, 독립 검증 |
| threshold | production E25는 0.40/0.15/0.10 | 새 모델은 rosbag dev-val에서 별도 선택 |

crop 결과는 [입력 후보 분석](261009_BASE5_CAMERA_CROP_INVESTIGATION.md),
비용과 전달 번들은 [4090 시험 계획](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md)을 따른다.
최적화 성공을 본학습의 필수조건으로 만들지 않는다. 일정상 필요하면 검증된 기존 pooling으로 학습한다.

2026-10-09 [CUDA-BEVFusion·v2/v3 배포 조사](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md)를
반영했다. PyTorch 학습 체계는 유지하고 필요한 연산을 선별 이식한다. CUDA-BEVFusion의
depth-outer pooling과 NVIDIA가 소개한 v3는 다르며, 전체 저장소 교체나 v3 이름만으로
속도·정확도·학습 호환을 보장하지 않는다. 아래 B와 후속 TensorRT는 아직 미구현이다.

## Base3/E25 초기화의 실제 이력

Base3 production E25를 만든 `bf-sw-fresh-e40-rtx5060ti-260918-005536`은 완전한
랜덤 초기화가 아니다. `fresh`는 MORAI 호환 nuScenes checkpoint를 로드하고
optimizer/scheduler를 새로 시작했다는 뜻이다. E40 실행 중 E25를 선정했다.

- 실행 설정: [resolved_experiment.yaml](../../artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/resolved_experiment.yaml).
  checkpoint는 `bevfusion_nuscenes_morai3_compatible.pth`, lr 1e-4, 물리 batch 4였다.
- [run_manifest.json](../../artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/run_manifest.json)의
  checkpoint SHA256은 `2a9ce3e01a723581f8e76b247887a704c4751f2ebcc9b36884c266466700a247`이다.
- [Base3 adapter](../../perception/detection3d/bevfusion/tools/adapt_nuscenes_checkpoint.py)는
  nuScenes 10-class를 Vehicle/Pedestrian/Obstacle로 평균 집계했다. 분류 가중치를 전부
  랜덤으로 초기화한 것이 아니다. LiDAR 입력은 5→4채널, velocity head와 grid buffer는 제외했다.
- [현재 Base5 후보](../../perception/detection3d/bevfusion/configs/bevfusion_base5_independent_100m.py)는
  load_from=None이며 상속된 camera init_cfg도 None이다. 별도 override 없는 후보는
  checkpoint-free 초기화다. 이 문서의 추천 초기화가 코드에 이미 반영됐다고 해석하지 않는다.

공식 BEVFusion의 [학습 절차](https://github.com/mit-han-lab/bevfusion#training)도
camera nuImages pretrained와 LiDAR checkpoint를 사용하는 fusion 학습을 안내한다.
Base5-only 학습은 데이터 독립성의 결정이지 가중치까지 랜덤이어야 한다는 뜻은 아니다.

## Base5 초기화 추천과 로딩 검증

기본 추천은 **공식 nuScenes BEVFusion 가중치에서 호환 부분을 가져와 Base5 전체를 학습**하는 것이다.
E25/Base4/Q1 가중치를 기본 초기값으로 쓰지 않는다. 나머지 5 환경은 새 geometry로 생성할
계획이지만 완전 랜덤 초기화가 더 유리하다고 가정하지 않는다. backbone freeze는 기본 계획에 넣지 않는다.

| 부분 | 추천 처리 | 검증할 사항 |
|---|---|---|
| Camera backbone/neck | shape 호환 가중치 로드 | 색상·정규화·채널·stage 구성, 위치 관련 buffer |
| LiDAR sparse encoder | 호환 가중치 로드, XYZI 4채널 적용 | spconv layout와 state_dict의 _metadata 보존, 실제 로딩률 |
| Fusion/BEV decoder | 호환 learned weight 로드 | 직사각형 축 순서, stride와 channel 호환 |
| 분류 heatmap/class encoding | Vehicle/Pedestrian/Obstacle/Animal 4-class로 새 초기화 우선 | 3-class adapter 그대로 사용 금지, 클래스 순서와 loss |
| BBox 회귀/공통 head | 호환 부분만 재사용 검토 | center/size/yaw 정의, 새 rectangular head의 key/shape |
| Depth 출력 | depth bin 변경으로 불일치하는 부분 새 초기화 | depth와 image feature가 합쳐진 출력층은 명시적으로 분리 검사 |
| Geometry/frustum/grid | 새 config/calibration으로 재생성 | dx/bx/nx뿐 아니라 모든 좌표 buffer, nuScenes 배치 재사용 금지 |
| Velocity/temporal | 로드/활성화하지 않음 | 7D loss와 single-frame 실행 확인 |
| Optimizer/scheduler | 새 시작 | 과거 epoch/step/optimizer state를 초기 학습에 resume하지 않음 |

로드 전에 CPU에서 source/target key와 shape를 대조하고 loaded/reset/dropped/unexpected
목록과 모듈별 tensor/parameter 로딩률을 저장한다. strict=False의 경고만 보고 성공 처리하지 않는다.
source checkpoint 경로·SHA256·adapter revision·config hash·클래스 순서·전처리 계약을 기록한다.
4-class adapter는 아직 없으며 Base3 adapter를 수정 없이 적용하지 않는다.

GPU가 분리되거나 수집이 끝난 뒤 실제 forward/backward/loss 및 AMP step을 확인한다.
pretrained/fresh 모듈의 LR 차등은 후보이며 수치 미확정이다. epoch 수와 batch도 짧은 실제 학습
비용 측정 후 동결한다. 20epoch와 과거 학습시간 외삽을 필수 수렴 시간으로 사용하지 않는다.

## BEVPoolv2 검토 근거와 한계

[BEVFusion](https://arxiv.org/abs/2205.13542)은 2022-05-26,
[BEVPoolv2](https://arxiv.org/abs/2211.17111)는 2022-11-30 최초 제출됐다.
BEVFusion은 처음부터 자체 최적화 pooling을 사용했고
[공식 구현](https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/ops/bev_pool/bev_pool.py)도
CUDA interval reduction이다. 이후 미채택 이유나 저자의 인지 여부는 공식 근거 없이 추측하지 않는다.

BEVPoolv2의 핵심은 depth 확률과 image feature를 분리 입력하여 거대한 frustum feature의
중간 생성/저장을 생략하는 것이다. 새 탐지 모델이나 학습된 정확도 개선 기법이 아니다.
논문의 특정 pooling latency를 우리 전체 모델 latency로 사용하지 않는다.

- [참조 구현](https://github.com/HuangJunJie2017/BEVDet/blob/dev3.0/mmdet3d/ops/bev_pool_v2/bev_pool.py)은
  depth/feature를 FP32로 변환하며 backward에서도 ranks_feat 정렬을 한다. AMP의 dtype와
  학습 비용을 추론 cache 효과로 대신 인증하지 않는다.
- CUDA extension/배포 plugin 호환, XYZ/B/C 축·z collapse·정수 인덱스 정합이 필요하다.
  합산 순서 변화에 따른 수치 오차와 depth/feature 양쪽 gradient를 검증해야 한다.
- 고정 인덱스 cache와 v2 kernel은 별도 변경이다. calibration/crop/augmentation이 변하면
  cache를 재생성한다. 동적 calibration이 v2 자체를 사용할 수 없다는 뜻은 아니다.
- 입력·격자·장면 밀도에 따라 효과가 다르며 backbone/sparse encoder/head 비용은 남는다.
  구현이 정확할 경우 탐지 능력을 희생하는 것이 목표는 아니지만 속도와 정확도 모두 검증 전이다.

## 최적화 A/B 구현 순서

현재 [DepthLSS](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/depth_lss.py)는
매 forward geometry를 계산하고 depth×feature 중간 tensor를 만든다.
[pooling](../../third_party/mmdetection3d/projects/BEVFusion/bevfusion/ops/bev_pool/bev_pool.py)은
ranks 정렬 후 CUDA로 합산한다. 따라서 CPU pooling을 처음 CUDA로 옮기는 작업이 아니다.

| 단계 | 구현 범위 | 승격 조건 |
|---|---|---|
| A | inference-only 고정 geometry/valid index/rank/interval cache, 기존 pooling 유지 | 기존과 출력 정합, cache 무효화와 메모리 상한, 실제 단계 비용 감소 |
| B | 공개 v2를 backward 기준으로 삼은 depth/feature 분리 pooling, 기존 경로 fallback | CPU 기준/기존 CUDA와 forward·gradient 정합, A와 A+B 분리 측정 |
| B 추론 후보 | CUDA-BEVFusion depth-outer, FP16 load/FP32 누적 및 명시적 index 개선 | 같은 dtype/shape의 추가 이득, channel/stream/경계/직사각형 정합; 공식 v3 이식이라고 혼칭하지 않음 |
| 후속 | GPU 이미지 전처리, camera 부분 compile/TensorRT/CUDA graph | 앞 단계 실측 후 병목이 남을 때, 배포와 수치 회귀 검증; 전체 CUDA-BEVFusion 이주는 후순위 |
| 별도 모델 변경 | backbone/channel 축소·distillation 등 | 재학습 필요, 이번 초기 최적화와 분리 |

A/B 모두 opt-in/default OFF로 시작한다. 최종 옵션명은 구현 시 정하며 현재 존재하는 옵션인 것처럼
명령어에 쓰지 않는다. cache는 camera 순서, K/extrinsic/img_aug/lidar_aug, feature shape,
depth/grid bounds, batch, device/dtype을 포함해 무효화하며 엔트리와 메모리를 제한한다.
고정 camera→LiDAR 상대기하와 변화하는 world pose를 구분한다. 학습의 랜덤 augmentation에
추론 cache를 그대로 사용하지 않는다. 현재 `.long()`의 음수 경계 처리도 비교 중 임의 변경하지 않는다.

포인트 deepcopy는 DepthLSS의 좌표 변형과 연관되어 무조건 제거하지 않는다.
CUDA synchronize를 지워 측정 숫자만 줄이는 것도 최적화로 취급하지 않는다.
현재 online은 CPU OpenCV decode/resize/crop, 카메라 병렬 처리, uint8 전송과 GPU 정규화,
FP16 inference를 이미 사용한다. OpenCV CUDA 도입은 host 재다운로드/재업로드 없이 실제
pipeline 비용이 줄어드는지 확인한다. camera/LiDAR branch 병렬화도 GPU 경합 때문에 자동 이득은 아니다.

## 수집 중 가능한 검증과 종료 후 비용 시험

이전 합의에서는 작은 연산 단위 GPU 검증을 제한적으로 허용했다. 최신 합의는
위의 묶음 실험 순서대로 이번 수집 종료/MORAI 종료 후 GPU 검증을 진행하는 것이다.
아래 항목은 추후 사용자가 수집 중 단위 시험을 다시 요청할 때의 안전 조건으로 보존한다.
전체 모델/학습/latency benchmark를 수집 중 임의 실행하지 않는다. 아직 GPU 시험은 미실행이다.

1. CPU 먼저: 수동 합산 기준, 중복 rank, 빈/범위 밖 입력, 직사각형 XYZ와 batch/camera 순서,
   z 축 결합, 경계값, depth/feature gradient와 cache hit/invalidation/메모리 상한을 검사한다.
2. CUDA 최소 시험: 작은 tensor의 forward/backward, 제한된 반복부터 시작한다.
   추가 VRAM 1~2GiB 이내를 초기 예산으로 잡되 실제 context/peak를 확인하고 확대 여부를 판단한다.
3. 캡처 진행률·timeout·재시도와 GPU 여유를 전후/중간 확인한다. 영향/OOM 징후가 있으면
   **시험 프로세스만 종료**하고 collector는 건드리지 않는다. 예상보다 큰 모델을 임의 로드하지 않는다.
4. 동시 실행의 시간은 공식 latency가 아니다. 전체 모델, 학습 step과 열 안정성 측정은
   수집 종료 후 또는 별도 4090에서 실시한다.

과거 17:43 GPU snapshot의 총 사용 약 6.7GiB는 여유/peak 인증이 아니다.
4090 기존 확대 모델 warmup peak allocated 약 11.0GiB도 같은 GPU 병행의 안전 근거로 쓰지 않는다.
허용 범위가 실제 구현된 watchdog이나 수집 무영향 인증을 뜻하지는 않는다.

팀원 딸깍 번들은 미구현이다. 같은 raw·checkpoint·crop·BEV/depth에 baseline/A/A+B를
비교하고 correctness, cold/warm, 모델 단독/CPU 준비 포함 p50/p95/p99, steady/peak VRAM,
짧은 backward/optimizer step을 기록한다. 조합 전체를 본학습하는 실험은 아니다.
13Hz≈76.9ms이며 모델 평균 70ms만으로 ROS/full-stack 실시간성을 인증하지 않는다.

## 수집 종료부터 학습·배포까지

1. 성공 raw/annotation/metadata와 부족분을 확정하고 자산·거리·근접·환경 coverage를 감사한다.
2. 전처리 세 경로의 crop/resize/img_aug_matrix 정합 및 비용으로 입력·격자·depth를 동결한다.
   새 crop 기준으로 visibility/info를 재생성한다. 기존 320 camera positive를 재사용하거나 3D GT를 자르지 않는다.
3. Base5 adapter와 로딩 manifest를 검증하고 4-class single-frame 학습 smoke를 통과한다.
4. [split 계약](../../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md)의
   Sunny13 train 4255/val 745를 유지한다. 나머지 25K는 새 geometry의 공간 group 일관성과
   누출을 검사해 split을 재계산한다. 4255/745를 6배한 값을 최종 30K 분할로 단정하지 않는다.
   학습시간은 실측 step으로 갱신한다.
5. offline val로 후보를 줄이고 동일 mission rosbag의 detection/tracking/Map-CV, 첫 전달거리,
   근접 누락과 ID 연속성을 평가한다. 새 threshold를 선택하고 독립 미사용 bag으로 최종 확인한다.
6. 기존 E25 fallback을 유지하며 production 변경 시 CURRENT/EVIDENCE와 runtime 전처리도 함께 반영한다.

로컬/외부 단일 GPU 학습 모두 주기적인 resumable checkpoint를 사용한다. model뿐 아니라
optimizer/scheduler/AMP scaler/epoch/global step 및 가능한 RNG/sampler 상태를 보존하고
중단→재개 smoke로 이어지는 LR와 step을 확인한다. 정확한 mid-epoch batch 재현 여부는
실행기 지원을 검증하며 무조건 보장하지 않는다. 외부 GPU를 선택하면 checkpoint 완료 후
비동기 Drive 업로드·checksum·latest 완료 파일과 실패 기록을 추가 검증한다. 아직 구현/업로드된 상태가 아니다.

## TensorRT 후속 시험의 범위와 작업 예산

2026-10-09 합의는 기존 최적화의 효과를 먼저 측정한 뒤 부분 TensorRT를 선택적으로 검토하는 것이다.
ONNX는 모델 교환 형식이며 내보내기만으로 가속을 보장하지 않는다. CUDA-BEVFusion은 주로
CUDA/TensorRT 추론 배포 참조이며 현재 PyTorch 학습 체계를 대체하는 기본 학습기로 채택하지 않는다.
호환 구조·전처리·출력 계약을 맞춘 후 학습 가중치를 내보내는 경로를 검증한다.

| 범위 | 개략 작업량 추정 | 종료 판단 |
|---|---|---|
| Camera backbone/neck 부분 export와 비용 탐색 | 반나절~하루 | 먼저 export·수치 정합·실제 이득 확인 |
| 부분 engine 연결과 실제 추론·rosbag 회귀 | 추가 1~2일 | 이득이 충분할 때 진행, PyTorch fallback 유지 |
| sparse encoder/pooling/head를 포함한 전체 전환 | 수일~1주 이상 가능 | 이번 본학습의 필수 조건으로 두지 않음 |

직접 이식한 실측 작업 시간이 아닌 일정 추정이다. 현재 spconv와 custom pooling,
카메라별 crop·4-camera·4-class·7D·직사각형 BEV export 및 CUDA/TensorRT 호환 때문에
전체 전환은 불확실성이 크다. 카메라 부분 탐색에서 이득이 작거나 호환 문제가 크면
기존 경로로 학습을 시작한다. GPU 임대 비용보다 구현과 수치·mission 회귀 검증 시간이 주요 부담이다.

학습 해상도는 [13Hz 처리 예산과 후보 예상](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md#수집-종료-후-해상도-선택-기준)을
따른다. 학습 전에 입력·crop·grid/depth·class를 동결하되 동등한 출력의 추론 최적화는
학습 중/후에도 검증해 적용할 수 있다. TensorRT는 추론 최적화이며 본학습 시간 단축으로 계산하지 않는다.
