# BEVFusion 문서 index

마지막 정리: 2026-10-11.

## CURRENT

현재 실행 순서는 **Sunny13 5K 완료·JPEG 전환 → 로컬 입력/정답 축/upright 라벨·전수·재개 검증 완료 →
로컬 공간 격자/depth 비용·위험 비교 완료 → 원격 최신 소스·upright 라벨·비용/재개 검증 완료 →
새 4090 이전·depth 25/50/100cm·높이 범위 후속 검증 완료 →
외부 백업·실제 복원 검증 완료 → depth 25cm 후보로 로컬 추가 가속 검토 →
채택 변경의 4090 확인·새 학습 계약 → 승인 후 본학습**이다.
원본 5K·유효 rosbag·production E25는 유지한다. 최신 상태는 아래 표의 링크부터 읽는다.

2026-10-11 06:43 KST 합의는 문서에만 반영했다. 공간 7.5/30cm·512×1056을 유지하고
depth 25cm를 우선 학습 후보로 삼되, 현재 config와 원격 prepare-only는 아직 50cm다.
25cm의 AP·tracker 속도 개선은 미검증이며 학습 비용 증가를 감수하는 선택이다.
아래의 50cm 기본 유지·백업 직후 본학습 안내는 이전 시점 기록으로 읽는다.

| 목적 | 최신 기준 |
|---|---|
| depth 25cm 우선 후보 / 본학습 전 로컬 학습·추론 추가 가속 / 원격 재확인·새 계약 / 계획만 갱신 | [10-11 06:43 최신 합의와 실행 순서](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-depth25-local-optimization-plan-261011) |
| 외부 full checkpoint 백업 완료 / 실제 복원·전체 SHA·완료 표시·중복 재검사 / 새 prepare-only | [10-11 병렬 백업 완료와 운영 경로](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-checkpoint-parallel-backup-261011) |
| 새 4090 이전 / depth 25·50·100cm·높이 점검 완료 / 저장·재개·val·새 준비 완료 / 외부 백업 아직 | [10-11 새 원격 경로와 후속 검증](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-depth-z-qualification-261011) |
| 최신 소스·upright 라벨의 실제 4090 / 300장 반복·실제 batch 4 학습·재개 / 본학습 아직·외부 백업 미완료 | [10-11 원격 비용·검증과 새 준비 계약](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-grid-qualification-261011) |
| 공간 5/20·7.5/30·10/40cm / depth 25·50·100cm / 실제 5K 정보량·비용과 선택 근거 | [10-11 로컬 공간 격자·depth 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-grid-depth-local-261011) |
| 기울어진 native RPY 박스 → upright 7D / 최신 파생 라벨 입력·전수·실제 점·재개 | [10-11 upright 라벨 보완](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-upright-native-rpy-261011) |
| Base5 학습 heatmap 축 수정 / 실제 5K·증강·전체 모델 역전파/재개 / 다음 격자·depth 실험 | [10-11 학습 정답 정합 검증과 후속 순서](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-heatmap-target-alignment-261011) |
| Base5 유효 반사점 필터·uint8 운반 / 기존 증강 유지 / 로컬 검증 | [10-11 정합성·CPU 비용·원격 재개 조건](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-valid-returns-uint8-261011) |
| Vast 재개·전체 JPEG 전환·새 prepare-only 계약 / 외부 백업 아직 | [원격 전수 동등성·실제 입력 검증과 남은 조건](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#vast-jpeg-migration-261011) |
| 현재 카메라 RGB: train·val·분석 모두 Q90 JPEG / PNG는 로컬 원본 / Drive 교체 | [10-11 입력 통일·전수 검증·전달](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#all-jpeg-policy-261011) |
| 이전 mixed 전달본 검증 / PNG vs JPEG 짧은 학습 비용 | [20,000개 JPEG·당시 독립 전달·짧은 실제 학습 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#jpeg-full-build-261011) |
| 학습용 JPEG Q90 100% / 사전 생성 캐시 추천 / blur·노출·noise OFF | [256장면 픽셀·GT 동등성과 CPU 로더 비교](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#jpeg-cache-decision-261010), [실제 JPEG 관측 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#image-domain-decision-261010) |
| Base5 전후방 ±100m·좌우 ±30m 유지 / ±40m 확대 기각 | [5K·고주로 합류·로터리 경로 근거와 재검토 조건](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#lateral-range-decision-261010) |
| 21시 학습 전 확대 검증 / 512·batch 4 추천 / 외부 백업 미완료 | [300장·실제 5K 로더·745장 val·저장/재개·네트워크 gate](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#6-4090-확대-표본학습-전-검증--2026-10-10-2023-이후), [Vast 후속 절차](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#확대-검증-완료와-네트워크-gate--2026-10-10-21시) |
| 실제 임대 4090 비용 검증 / Sunny13 5K 전달·전수 검증 완료 | [10-10 원격 실측](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#5-vast-4090-실제-검증--2026-10-10-17311734-kst), [19:49 복구·전체 검증 완료](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst), [후속 Cloud Sync 선택](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의) |
| Vast 템플릿·GPU 후보 추천·블로그용 캡처 원본 | [10-10 설정·캡처 보관 기록](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-vast-템플릿-안내와-블로그용-증거-기록) |
| 로컬 입력·직사각형 축·CUDA 분리 pooling 검증 | [10-10 실측 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md) |
| 원격 전달본·direct 학습·resume/백업·정리 결과 | [Vast 계획의 10-10 준비 결과](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과) |
| Sunny13 원본 PNG 백업·분할복원·전송시간 이력 / 현재 로컬 보존 | [10-10 당시 백업 기록](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-sunny13-5k-drive-원본-백업) |
| 다른 5 환경 생성/수집·resume·대시보드 | [새 환경 런북](../../dataset/morai_od/docs/261010_base5_fresh_environment_runbook.md) |

현재 연구는 Base5-only·4-class·single-frame·decoded 7D이며 OD velocity/temporal OFF다.
production은 E25·threshold 0.40/0.15/0.10이다. 새 학습은 512×1056·batch 4와 감사한
camera-only 초기화를 추천한다. 20절에서 실제 외부 백업·복원 검증을 완료했으며,
사용자 승인 전에는 본학습을 시작하지 않는다. 아래 외부 백업 미완료 문구는 이전 시점 기록이다.
2026-10-10 미션 경로 검토 후 Base5의 semantic ROI는 전후방 ±100m·좌우 ±30m로 유지한다.
±40m 비용 후보는 비교 이력으로 보존하며 이번 학습·다음 환경 배치를 확대하지 않는다.
2026-10-11 현재 config는 `bevfusion_base5_dense_512_native_jpeg90.py`다.
카메라 RGB는 train·val·test·분석 모두 원본 해상도 Q90 JPEG로 통일하고 blur·노출·noise는 끈다.
`artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release/`의 all-JPEG/upright manifest를 사용한다.
RGB PNG는 로컬 백업·변환용이며 mask·depth·native GT 등 보조 PNG는 무손실로 보존한다.
LiDAR·native GT·보정·split·production E25는 바꾸지 않았다. 파생 infos의 크기만 native RPY를
포함하는 upright enclosure로 보완했다. Base5만 0 기록·nonfinite 행을
증강 전에 제거하고 이미지를 GPU 정규화 전까지 uint8으로 유지한다. 로컬 정합 검사와
비용 측정을 통과했고 18절의 Vast 재개 후 최신 소스와 upright 라벨도 반영했다. 기존 원격
prepare-only 계약은 유지하고 새 work-dir을 사용했다. val 픽셀 분포는 의도적으로 Q90으로
변경했으므로 과거 PNG val 수치를 같은 조건의 결과처럼 합치지 않는다.
독립 전달본에는 `--jpeg-cache`나 JPEG 재압축을 추가하지 않는다. 기존 mixed 전달본·online
변환 설정은 이력이며 현재 학습의 fallback이 아니다. CPU 로더 개선을 전체 학습 가속으로 해석하지 않는다.
5060 Ti의 원본 PNG vs 사전 JPEG 짧은 실제 학습 비교는 약 0.47% 차이로 사실상 같았다.
종전 약 4.8시간은 JPEG 증강 이전 실측 기반 추정이다. Vast 재개 후 새 준비 계약을
만들었다. 19절 당시 미완료였던 외부 백업은 20절에서 완료했다.
새 인스턴스의 24 epoch 운영 예산은 약 4.6~5.2시간이다.
JPEG 정책의 AP 향상은 미검증이다. 10-10 저장 방식 비교는 CPU만 사용했고 10-11 PNG/JPEG
짧은 학습 비교는 로컬 GPU로 수행했다. 최신 원격 검증은 19절이며 본학습은 아직이다.
새 인스턴스에서는 `/workspace`의 검증본·새 준비 계약을 사용한다. 기본 depth 50cm와 높이
범위는 유지한다. 외부 full checkpoint 백업·복원 검증은 20절에서 완료했다.
최신 준비 경로는 `/workspace/base5_training_sunny13_512_b4_e24_55272391_backup_v3_261011`이며,
사용자 승인 후 병렬 백업 watcher와 본학습을 시작한다.
아래 이전 상태의 '수집 중/미구현/미측정'을 현재 실행 지시로 재사용하지 않는다.

2026-10-11 추가로 Base5 `[Y,X]` feature에 맞는 학습 heatmap 정답을 수정했다. 이전 feature
축 검사에서 빠졌던 target 생성 경로이며 원본 GT 오류가 아니다. 실제 11,052개 GT와 반복
증강을 포함한 29,714개 정답 위치, 4-iteration 전체 모델 역전파·재개를 로컬에서 통과했다.
본학습 성능 개선을 측정한 것은 아니다. E25와 prediction 경로는 유지하고 Vast에도 수정 소스를 반영했다.
후속 공간 격자·depth 비교의 결과는 17절을 따른다. 비용 검사를 본학습 AP 개선으로
해석하지 않는다. 기본 공간 7.5/30cm·depth 50cm를 유지하며, 더 정밀한 5/20cm는
실제 4090에서 추론·짧은 학습 비용을 확인한 선택 후보로 남긴다. AP 우위는 미검증이며
coarse 10/40cm를 단순 속도만으로 승격하지 않는다. 모델 비용·높이 검증은 19절,
최신 준비 계약과 외부 백업 완료 상태는 20절을 따른다.

## 이전 시점의 진행 기록 — 현재 실행 기준 아님

[2026-10-10 실제5K 입력·격자·pooling 검증 완료](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)가
아래 10-09 미구현·미측정 계획보다 우선한다. 전수 crop·448/512 파생 infos·실제 loader/online 정합,
직사각형 sparse coordinate 축 수정, opt-in cache A/분리 B, 실제 14장 full-model 동등성 및
짧은 역전파를 검증했다. 5060 Ti의 448 모델은 162.67→74.91ms, 512 모델은 216.75→93.52ms다.
4090·ROS/full-stack 검증, 최종 해상도 선정과 pretrained 본학습은 아직이며 production E25와 수집 원본은 불변이다.
구 4090 확대 비용은 sparse 축 수정 전 이력이므로 새 번들로 다시 측정해야 한다.

[2026-10-10 수집기 후속작업 완료](../../dataset/morai_od/docs/261010_base5_postcollection_collector_work.md):
Sunny13 최종 5K 보존, 다른 5개 환경의 새 배치 생성, 수집기 비용·실제 정적/이동 보충·resume 검증은 끝났다.
[새 수집 런북](../../dataset/morai_od/docs/261010_base5_fresh_environment_runbook.md)은 준비됐지만
나머지 실제 25K와 본학습은 미시작이다. GPU/crop/cache/분리 pooling은 위 10-10 근거를 따른다.
수집기 비용 22.55% 단축은 BEVFusion 추론 지연이나 모델 성능 개선이 아니다.

[2026-10-09 수집 종료 후 해상도 선택](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md#수집-종료-후-해상도-선택-기준):
crop/투영과 cache·분리 pooling을 검증한 뒤 세로×가로 입력을 비용 실측으로 확정한다.
13Hz≈76.9ms에서 전처리 포함 평균 50~60ms는 탐색 목표이며 p95/발행 지연을 별도 확인한다.
448×896 약 50ms·512×1056 약 60ms·576×1152 약 70ms는 20~30% 최적화 가정의 미측정 예상이다.
TensorRT는 후속 선택이며 예상 가속을 전제로 큰 입력을 고르지 않는다. 수집·GPU 작업은 이번 정리로 변경하지 않는다.

[2026-10-09 환경별 새 공간 배치](../../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md):
현재 Sunny13 5K를 완료·보존하고 나머지 5 환경은 각각 새 배치 5K로 생성한다.
구성표/주요자산 할당 유지·연속 Ego 샘플링·공간 split 검증이 선행하며 기존 복사 manifest를 바로 수집하지 않는다.

[2026-10-09 최신 실행 합의](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#sunny13-수집-종료-후-묶음-실험-합의):
이번 Sunny13 5K 수집/MORAI 종료 후 사용자 요청으로 무결성·split → 입력/crop → cache/분리 pooling →
latency/VRAM·짧은 학습 → 초기화/smoke/resume을 묶어서 수행한다. 추천 구성 하나와 fallback 하나로
좁혀 본학습 직전에 보고한다. TensorRT/v3 전체 적용은 필수가 아니며 최적화 때문에 학습을 무기한
미루지 않는다. 이 합의만으로 GPU 작업·본학습·다음 환경 수집을 자동 시작하지 않는다.

[2026-10-09 Vast.ai 단일4090 학습·추론 계획](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md):
로컬 입력·연산 검증 후 임대한 4090에서 latency/VRAM·짧은 학습·재개를 확인하고 Sunny13 5K를 학습한다.
첫 5K는 On-demand/200GB, 같은 임대로 30K까지 이어가면 500GB를 검토한다. 실시간 가격 관측과
디스크·SSH·직접 실행·외부 checkpoint 백업 계획을 보존한다. 임대·전달·remote runner는 아직 미실행/미구현이다.

[2026-10-09 CUDA-BEVFusion과 BEVPool 배포 검토](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md):
우리도 이미 CUDA 신경망·voxelization·sparse encoder·pooling을 사용한다. PyTorch 학습은
유지하고 geometry cache와 분리 pooling을 선별 이식하는 방향을 추천한다. 공식 코드의
카메라별 전처리·4채널 LiDAR·7D/class/직사각형 계약 차이 때문에 전체 교체는 후순위다.
CUDA-BEVFusion depth-outer와 NVIDIA v3를 구분하며 이번에는 조사·문서화만 했다.

[2026-10-09 초기화와 BEV 연산 최적화 실행 계획](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md):
Base3/E25의 fresh는 nuScenes 호환 가중치 + 새 optimizer였으며 완전 랜덤 초기화가 아니다.
현재 Base5 후보의 load_from=None과 향후 호환 pretrained 추천을 구분한다. 4-class/depth/grid
로딩 감사 후 본학습하며, 다음 연산 작업은 기존 CUDA pooling의 기하 cache A → v2 방식 B다.
수집 중 작은 CUDA 연산 단위 시험만 제한적으로 검토한다. 전체 모델/학습/공식 비용 시험은
별도 4090 또는 수집 종료 후다. adapter/cache/v2/새 번들은 아직 미구현이며 수집·production 유지다.

[2026-10-09 카메라 crop 분석과 입력 후보](261009_BASE5_CAMERA_CROP_INVESTIGATION.md):
2,586장·121종의 실제 instance mask를 비교했다. 우선 후보는 448×896 아래 정렬,
비용 대안 384×896, 원거리 밀도 확대 대안 512×1056이다. crop 위치만 올리면 근접
장애물 아래쪽 손실이 커진다. 최종 입력은 미확정이며 다음은 전처리 정합과 GT 재선별이다.
수집과 production은 유지한다. 분석 수치와 그래프도 같은 문서에 보존한다.

[2026-10-09 4090 추가 비용 시험 계획](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md):
crop 분석의 384×896/448×896/512×1056과 320×896 기준을 추론·짧은 학습으로 비교할 계획이다.
종전 비례 해상도 표는 초기 비용 비교 기록이다. 새 번들은 미구현이며 풀스택은 보류한다.
기존 실측과 새 해상도 및 학습시간 추정을 구분하고 본학습 여러 회를 전제하지 않는다.

[2026-10-09 학습split·가시성·temporal 결정](../../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md):
Sunny13 train 4255/val 745 유지, 나머지 25K 새 geometry는 공간 group 누출을 검사해 split 재계산한다.
반복 평가 bag은 dev-val, 새 미사용 주행은 test다.
30K 수집 계획을 유지하고 이후 필요한 조건만 보강한다. E25와 Base5는 single-frame이며 OD velocity/temporal 연구 분기는 사용하지 않는다.
tracker의 시간 기반 추정·NMS·lifecycle은 유지한다. BabyStroller3의 3/21은 camera 대응 모호성을 포함한 component 집계다.
이번 Base5 본학습은 아직 미시작이며 관측 단서 proxy를 모델 성능이나 학습 충분성으로 해석하지 않는다.

Sunny13 v132 본 5K는 사용자가 시작해 수집 중이다. 이번 runtime/input은 고정하고 종료 후
[반복 준비 비용 최적화 A/B](../../dataset/morai_od/docs/261009_base5_collection_cost_and_next_experiments.md)를 수행할 계획이다.
목표는 전체 20% 이상 단축이며 GT/안전/resume을 유지한다. 아래 미시작/승인 대기는 과거 준비 상태다.
OBJ 차량은 Vehicle, Animal은 독립 라벨로 보존한다. 재수집 없이 Pedestrian으로 합칠 수 있지만 후처리·모델 설정 변경이 필요하다.

Base5/E25의 [batch1·ROS 포화 처리량](261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)을 측정했다.
5060 Ti에서 E25는 offline 13.90FPS, 원속도 ROS 11.47Hz/드롭 0, 약 98Hz stress 입력에서 출력 12.97Hz였다.
320×896/±100m fresh 후보는 offline 10.58FPS 비용 시험이며 정확도·4090·확대 후보 ROS 결과는 아니다.
[단계별 비용·격자/depth 대조](261006_BASE5_LATENCY_STAGE_BREAKDOWN.md)에서 비용 상쇄를 실측했다.
기존 격자와 0.5m depth로 100m 고해상도 입력을 처리하는 비용은 E25 대비 약 1.98배였다. 후보의 속도를 추가 비용 없는 정확도 개선으로 해석하지 않는다.
고정 간격 4-case는 3-class/200 query까지 통일했다. 기존 66.48ms,
이미지 확대 90.46ms, 범위 확대 96.27ms, 둘 다 확대 130.59ms(1.96×)다.
0.5m depth 간격을 유지하므로 범위 확대에 따라 118→198 bin이다.

[4090 팀원 실측 결과](261009_RTX4090_CAPACITY_RESULTS.md)를 2026-10-09 확인했다.
동일 고정간격 4-case 모델 평균은 기존 27.66ms/이미지 34.35ms/범위 36.19ms/둘다 45.86ms다.
둘 다 확대한 경우 CPU 이미지 준비를 포함한 평균은 50.93ms, 모델 p95는 46.71ms이며 8/8 case가 통과했다.
5060 Ti 대비 2.40~2.85배이며 ROS/full-stack·밀집 장면·확대 모델 정확도 인증은 아니다.
기존 5060 Ti 문서의 ‘4090 미측정’ 표현은 해당 측정 시점의 기록이다.

독립 Base5 생성/학습 후보는 [Base5 v2 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 우선한다.
2026-10-09 현재 [v132 최종 외관/배치 설계](../../dataset/morai_od/docs/261009_base5_final_appearance_v132.md)를
사용한 기록이다. Base5-only 5K→6개 환경 30K, 정지 자유 yaw·배치 형상 보완이며 native GT와 production E25는 그대로다.
현재는 `final_appearance_5k_v132_release`의 Sunny13 직접 5K 수집 중이며 로터리 미션 350셀을 포함한다.
로터리 실제 경로 관측과 정적 NPC Park 초기화의 이전 [검증/한계](../../dataset/morai_od/docs/261008_base5_roundabout_route_mission_evidence.md)를
유지하고 [본수집 런북](../../dataset/morai_od/docs/261007_base5_5k_capture_runbook.md)을 따른다. 새 학습은 아직 시작하지 않았다.
구 10K/왕복/v25 yaw 근거는 이력이다. 간헐 통신 timeout의 내부 원인은 미해결이며 실패를 보존한다.
100m 직사각형 BEV 및 4-class 후보의 단일 GPU loss/역전파 smoke와 production E25를 구분한다.

Tracker 속도 평가의 GT 기준은 [velocity GT 재검토·실행 설정](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md)를 함께 읽는다. 보행자 ROS 메시지 우선/이동-zero fallback은 opt-in이며 차량 세 기준 비교와 H01/R01 baseline 재현을 보존한다. 2026-10-06 정리한 MORAI 추가 답변에서 **UDP 차량 velocity가 후륜 차축 중심의 순간 선속도**임을 확인했고 메시지 GT 사용을 권장했다. BBox 중심 변환의 좌표계/시간/yaw-rate 검증은 남으며 evaluator 기본값과 production은 변경하지 않았다.

[최신 MORAI 출제 범위·G80 답변](../../dataset/morai_od/docs/260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준): 대회 F2 전체 객체가 후보, 기존 정적 자산의 이동 예외 없음. `OBJ_G80_Open_Door`는 공식 출제 제외/목록 삭제 예고다. 다른 자산 부유는 재현 자료 요청 단계이며 해결됐다고 보지 않는다.

Base5 E25-init E2 학습과 [E1 보행자·장애물9-bag rosbag A/B](261005_BASE5_E1_MISSION_ROSBAG_AB.md)는 완료됐다. 장애물 소폭 개선이지만 초근접·원거리 보행자 공백은 미해결이다. Production은 E25 그대로다. [Base3 공백·학습 결과·후속 보강](261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md)을 함께 읽는다.

| 목적 | 문서 |
|---|---|
| Production E25 실행·판단팀 전달 | [`BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| Vast.ai 단일 4090 임대·5K 전달·원격 학습과 재개 | [`261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md) |
| Base5 가중치 초기화·pooling/cache 최적화 순서 | [`261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md`](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md) |
| 카메라 crop 크기와 위치 조사 | [`261009_BASE5_CAMERA_CROP_INVESTIGATION.md`](261009_BASE5_CAMERA_CROP_INVESTIGATION.md) |
| 팀원 4090 입력 후보 비용 시험 | [`261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md`](261009_BASE5_CAMERA_INPUT_AND_4090_BENCH_PLAN.md) |
| CUDA-BEVFusion 차용·BEVPoolv2/v3배포 판단 | [`261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md`](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md) |
| 현재 개선 우선순위: 근접·원거리·차량 승격·고해상도 | [`261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md) |
| Production E25 rosbag Tracking/Prediction RViz 재생 | [`E25_ROSBAG_RVIZ_RUNBOOK.md`](E25_ROSBAG_RVIZ_RUNBOOK.md) |
| 팀 공용 Docker 학습 | [`TEAM_DOCKER_TRAINING_HANDOFF.md`](TEAM_DOCKER_TRAINING_HANDOFF.md) |
| Base4 temporal 팀원 인계·재현 | [`BASE4_TEMPORAL_TEAM_HANDOFF.md`](BASE4_TEMPORAL_TEAM_HANDOFF.md) |
| Base3-Online dataset 현재 상태 | [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md) |
| 다음 velocity/temporal 통제 실험과 architecture ladder | [`260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md) |
| Base4 24-scene 수집·9D/3-sweep·학습 smoke 결과 | [`260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md`](260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md) |
| Base4 V1/T1 E4 고주로 detection·tracking·prediction 판정 | [`260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md`](260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md) |
| Base4 velocity head learnability·LR 진단 | [`260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md`](260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md) |
| Base4 center-motion/association 오차 분해 | [`260923_BASE4_CENTER_MOTION_PROBE.md`](260923_BASE4_CENTER_MOTION_PROBE.md) |
| Base4 T2 sweep-separated temporal BEV 구현·smoke | [`260922_BASE4_T2_TEMPORAL_BEV_P0.md`](260922_BASE4_T2_TEMPORAL_BEV_P0.md) |

Base4 수집·학습·Q1 실험 코드는 최신 production 기준 위의
`perception/base4-temporal-competition-v2`에 복구했다. 모든 temporal/velocity 옵션은
기본 OFF이며, 이 복구가 production E25를 교체한다는 뜻은 아니다.

Production 표준은 `B3-BF0-F40 E25`, class threshold는 `0.40/0.15/0.10`이다.
Offline 최고 epoch만으로 production을 바꾸지 않고 detection, tracking, Map-CV를 같은
mission rosbag에서 비교한다.

## EVIDENCE

- [`261009_RTX4090_CAPACITY_RESULTS.md`](261009_RTX4090_CAPACITY_RESULTS.md)
  - 팀원 4090 반환 ZIP 무결성, 고정 간격 4-case 실측, 5060 Ti 대응 비교와 단계별 비용. 수집/runtime 변경 없음.

- [`261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md`](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md)
  - 새 H01/R01 속도 GT A/B 및 최신 공식 후륜 기준 확인. 과거 중심 속도 가설과 회전 보정 잔차 관측을 구분한다.

- [MR VLP16 실제 출력100m 제한](261005_MR_VLP16_100M_RANGE_EVIDENCE.md): fireDistance150 저장 유지에도 원본 UDP/pointcloud는 약 100m 이내. 원거리 7대와 55.6m control 차량 live 대조. 내부 원인/대회 규정 제한을 확정한 것은 아니다.

- [`261005_BASE5_E1_MISSION_ROSBAG_AB.md`](261005_BASE5_E1_MISSION_ROSBAG_AB.md)
  - E1 기존/새 보행자·장애물 9-bag 완료, 동일 E25 계약 Detection/Tracking/Map-CV 및 거리별 검출률. E2·차량 미션은 이번 범위 밖.
- [`261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md`](261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md)
  - 실제 Base3 train 거리/point 분포, Base5 E1/E2 완료 결과와 미검증 경계, 다음 고정 계약 A/B·표적 보강 순서.
- [`261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md`](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md)
  - 기존 6-bag도 현재 E25/NMS·tracker·GT catalog·v85로 재실행한 기존/새 미션 대응 표. 아래 historical-baseline 직접 비교를 대체한다.
- [`../../dataset/morai_od/docs/261004_base5_reinforcement_smoke_runbook.md`](../../dataset/morai_od/docs/261004_base5_reinforcement_smoke_runbook.md)
  - Base5 근접·실제 고주로 전후방 24 scene + 저속 통과 2 scene, 84-frame 변환과 sparse/ROI 경계 감사.
- [`261004_BASE5_LIVE_ACQUISITION_PROBE.md`](261004_BASE5_LIVE_ACQUISITION_PROBE.md)
  - MR 근접/원거리 7-scene rosbag 수집·21-frame 변환, 동기 캡처 annotation 미완료와 최소 프로필 복원 기록.
- [`261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md)
  - 16-bag 근접/전달 감사, opt-in 승격 단위 검증·H03 replay, 5060 Ti 고해상도 GPU smoke.
- [`261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md`](261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md)
  - 기존 E25 미션 평가와 새 팀원 bag 대응, 거리별 누락·center 정밀도·미래 경로 오차 분석.
- [`261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md`](261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md)
  - UDP 차량 기본 제원→geometric BBox 대응 복구와 재평가. 아래 첫 평가의 차량 결과를 대체한다.
- [`261004_TEAMMATE_MANUAL_FULL_PIPELINE_EVALUATION.md`](261004_TEAMMATE_MANUAL_FULL_PIPELINE_EVALUATION.md)
  - 수동주행 E25 16-bag / Q1-P7 고주로 7-bag detection·tracking·Map-CV 평가와 GT 미식별 프레임 감사.
- [`261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md`](261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md)
  - 팀원 수동주행 16-bag 원본 검증, 고주로 실제 속도 분포 및 정식 평가 전 GT/시간축 조건.
- [`260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)
  - Base3-Online Vehicle-only E25 fine-tuning의 7-bag 및 tracking/Map-CV 결과.
  - Vehicle detection·center·2D AVE는 개선됐지만 Obstacle 망각 때문에 E25 유지.
- [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
  - Tracker motion model/noise, 올바른 2D AVE 계약, GT-velocity oracle 근거.
- [`260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md`](260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md)
  - 학습 변환기와 rosbag 평가기의 asset별 BBox center/size 계약 통합 및 기존 고주로 A/B.
- [`260918_base3_e40_production_evaluation_results.md`](260918_base3_e40_production_evaluation_results.md)
  - E25를 production으로 선택한 offline/rosbag/tracking/Map-CV 근거.
- [`260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md`](260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md)
  - Rotated BEV NMS와 tracker lifecycle/duplicate-birth 개선의 saved-bag 근거.
## HISTORICAL

- [`legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`](legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md): 이전 BF1 image-domain augmentation 실험.
- [`legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md): 완료된 fresh/E25-init 팀 학습 절차.
- [`legacy/260918_base3_e40_post_training_checklist.md`](legacy/260918_base3_e40_post_training_checklist.md): 완료된 E40 사후평가 절차.
- `legacy/260720_morai_four_camera_zero_shot_*`: 초기 zero-shot feasibility 기록.

HISTORICAL 문서의 checkpoint, image tag, dataset root, threshold를 현재 기본값으로
사용하지 않는다.
