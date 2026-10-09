# Base5 Vast.ai 4090 학습과 추론 시험 계획

2026-10-09 KST 기준. Sunny13 독립5K를 먼저 학습하고 이후 나머지5환경의 새 배치25K를
더해6환경30K로 확장한다. [환경별 새 공간 배치 합의](../../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md)가
과거 같은 geometry 반복 계획보다 우선한다. 첫 임대는 단일 RTX4090 On-demand를 추천한다.
로컬5060Ti에서 입력·pooling 정합과 후보 비용을 확인한 뒤, 원격4090에서 최종 비용을
재확인하고 학습한다. 임대·결제·이미지 게시·데이터 업로드·원격 실행은 아직 하지 않았다.
현재 수집·production E25·tracker는 유지한다.

## 임대 사양 추천

| 항목 | 첫5K 추천 | 이유와 확인 사항 |
|---|---|---|
| GPU | RTX4090 24GB 한 장 | 대회 GPU와 같은 계열에서 학습·추론 비용 확인; batch와 최대 메모리는 별도 smoke |
| 임대 유형 | On-demand, verified host 우선 | 첫 환경 구축·재개 검증 중 회수 위험을 줄임; 장애·잔액 부족·계약 만료까지 없어지는 것은 아님 |
| CPU | 할당8 vCPU 이상, 가능하면16 이상 | 이미지 decode/augmentation과 loader 병목 확인; 전체 호스트 core 수와 할당분 구분 |
| RAM | 32GB 최소 후보, 64GB 이상 권장 | loader worker/prefetch 및 가공·평가 여유; shared memory도 검사 |
| 디스크 | 첫5K는200GB, 같은 임대로30K까지 계속하면500GB부터 검토 | 완성 학습본·이미지/환경·checkpoint·압축 중복·평가 subset까지 계산 후 확정 |
| 디스크·네트워크 | 빠른 로컬 SSD, direct SSH, 낮은 전송 단가 | GPU 가격뿐 아니라 업로드 시간과 loader I/O도 비교 |
| 안정성 | reliability0.99 이상 선호, 계약 잔여기간 넉넉히 | 표시 점수는 장애 보장이 아님; 실제 1차 smoke/네트워크/지속 실행 확인 |
| CUDA 환경 | 팀 PyTorch2.7.1/CUDA12.8 계열과 호환 | host driver와 CUDA extension sm89 지원, 설치 버전/digest 기록 |

On-demand와 Interruptible은 다른 임대 유형이며 Interruptible은 회수로 정지될 수 있다.
첫 실행은 On-demand로 하고, resumable checkpoint와 외부 백업이 검증된 뒤 중단형의
가격 이득을 검토한다. 계약 만료일·잔액·host 안정성은 On-demand에서도 확인한다.
[공식 임대 유형](https://docs.vast.ai/guides/instances/choosing/instance-types).

## 2026년10월9일 가격 관측

18:20 KST 무렵 공식 search offers API를 읽기 전용 조회했다.
필터는 verified/rentable/not-rented, 단일4090, RAM32GB 이상, 할당8 vCPU 이상,
disk200GB 이상, reliability0.98 이상, On-demand, allocated_storage200GB이며
dph_total 오름차순12개다. 따라서 전체 시장의 평균·최저가가 아닌 해당 조건의 후보 관측이다.
[공식 검색 API](https://docs.vast.ai/api-reference/search/search-offers),
[실시간 선택 화면](https://cloud.vast.ai/create/).

| 관측 후보 | 지역 | 할당 vCPU / RAM 표시 | GPU 기본 $/h | 200GB 포함 dph_total $/h |
|---|---|---|---:|---:|
| 39308422 | 일본 | 24 / 약129GB | 0.3867 | 0.4126 |
| 46384000 | 한국 | 16 / 약64GB | 0.4133 | 0.4504 |
| 45473516 | 일본 | 16 / 약97GB | 0.4133 | 0.4626 |

조회12개 total은 약$0.413~0.552/h였다. 저가 후보의 storage_cost는 각각
$0.0933/$0.1333/$0.1773 per GB/month로 표시됐다. dph_total에는 이번200GB 할당 비용이
이미 반영되어 있으므로 디스크를 다시 더하지 않는다. 전송은 별도이며 한국 후보의
inet_down_cost/inet_up_cost 표시값은 각각 약$0.0002604/$0.0003906 per GB였다.
최종 checkout에서 할당량·전송 단위와 총액을 다시 확인한다. 한국 후보는 가용 disk가
206GB로 표시되어500GB 장기 구성에는 해당 시점 사용 불가였다. 후보ID는 예약이나 추천 확정이 아니다.

같은 기본 조건의3090 저가3개는200GB 포함 약$0.202~0.256/h였다. 가격 대안이지만
우리 확대 모델의4090 대비 학습 속도·시간당 효율은 미측정이다. 대회 추론 비용까지
동일 GPU에서 확인하려는 첫 실행은4090을 우선한다.

예산은 여유를 두고 **4090+200GB $0.45~0.65/h**로 잡는다. 이는 추천 예산이지 확정 요금이 아니다.

| 가동 시간 가정 | GPU와200GB 예산 | 별도 항목 |
|---|---:|---|
| 24시간 | $10.8~15.6 | 데이터 전송·유휴 저장·세금/결제 비용 |
| 48시간 | $21.6~31.2 | 동일 |
| 72시간 | $32.4~46.8 | 동일 |

설치·업로드·실험 중 실행 상태의 시간도 예산에 포함한다. 실제 학습시간은 아래 짧은
step 실측으로 갱신한다. 전체 비용은 실행시간×해당 총시급 + 정지 중 저장 + 전송 등이며,
가격·재고는 변한다. [공식 비용 구성](https://docs.vast.ai/guides/instances/pricing).

## 저장공간과 업로드 범위

당일 대시보드 기존 저장량 표본은 승인3208장에21.84GiB,5K 전체 추정34.05GiB였다.
이는 실패·재시도까지 포함한 캠페인 저장량 외삽이며 최종 학습 전달본의 실측이 아니다.
30K 단순6배는 약204GiB지만 환경별 이미지 크기·불필요 artifact·후처리에 따라 달라진다.
5K 종료 후 사용할 파일 목록과 byte 합계를 확정하고 GB/GiB 차이를 고려해 임대한다.

- 원격에는 선택한 성공5K의 학습 입력·native annotation/필요 metadata·고정 split/info,
  호환 초기 weight, 코드 revision 및 소량의 실제 평가 raw를 우선 전송한다.
- 전체 MORAI SensorData·실패/연구 캡처·모든 rosbag·전달용 Docker tar를 무조건 복사하지 않는다.
  GT 재가공에 필요한 원본과 metadata는 로컬에 보존하며 원격에서도 학습 정합 감사 자료를 유지한다.
- rsync 재개 또는 분할 archive+SHA256을 사용한다. JPEG/PNG를 불필요하게 decode해
  부풀리거나 lossily 재압축하지 않는다. archive와 해제본이 동시에 존재하는 최대 용량을 계산한다.
- 200GB는 첫5K 여유안이다. 이어서30K까지 한 인스턴스에서 쓰려면500GB 안팎을 검토하되
  archive를 원격에 모두 남기는 구성이라면 더 필요할 수 있다.

Vast container disk는 생성 시 크기가 고정되고 stop 후에도 요금이 발생한다. destroy하면
컨테이너 데이터는 삭제된다. 별도 volume도 특정 물리 host에 묶이므로 외부 백업을 대체하지 않는다.
[공식 저장공간 설명](https://docs.vast.ai/guides/instances/storage/types).

## SSH와 실행 환경

Vast는 Docker image와 SSH launch mode를 지정할 수 있다. 기본 인스턴스 자체가
컨테이너이므로 우리 host용 runner의 docker run을 그대로 중첩 실행하지 않는다.
팀 이미지를 registry에서 받을 수 있게 준비하고 SSH 환경에서 직접 train.py를 실행하는
remote runner를 만드는 쪽을 우선한다. 현재 로컬 이미지 tag만 있다고 원격에서 pull 가능한 것은 아니다.
이미지 게시·접근 범위는 별도 결정하고 실제 digest와 CUDA ops를 확인한다.
[공식 template 설정](https://docs.vast.ai/guides/instances/choosing/templates).

기존 [team runner](../../perception/detection3d/bevfusion/tools/run_team_experiment.py)는
source snapshot·checkpoint hash·resume 및 W&B 설정을 지원하지만 docker run 기반이다.
원격 direct runner, 주기적 Drive uploader와 원격 재개 smoke는 아직 준비하지 않았다.
원격에서도 동일 resolved config·run manifest·submodule revision·최종 전처리를 기록한다.

사용자는 계정에 공개 SSH key를 등록하고 임대 화면의 실제 host/port를 전달한다.
VS Code Remote-SSH는 선택 가능하며, 로컬 터미널의 SSH로 원격 작업을 수행하는 방식도 가능하다.
연결 후 원격 repo·데이터가 자동으로 생긴다고 가정하지 않는다. private key는 로컬에 두고,
W&B/Drive/API 자격증명을 Git·전달 ZIP·실행 로그에 넣지 않는다.
[공식 SSH와 VS Code 연결](https://docs.vast.ai/guides/instances/connect/ssh).

작업은 원격 tmux에서 실행하고 VS Code/SSH 연결 종료와 학습 종료를 분리한다.
편집할 때는 원격과 로컬 어느 checkout인지 확인하고 실행 중 code/config는 고정한다.
설치된 ROS 풀스택은 첫 학습의 필수조건이 아니다.

## 로컬에서 원격 학습까지

| 단계 | 실행 위치 | 수행과 완료 기준 |
|---|---|---|
| 1 | 로컬, 이번 수집/MORAI 종료 후 | 무결성·split·crop·투영·cache/pooling 출력/gradient 정합; 비용 후보 축소 |
| 2 | 로컬 | 전달5K·GT/info·코드·초기화·시험 raw manifest 확정, 원격 image/runner 준비 |
| 3 | 사용자 임대 후 원격4090 | SSH·CUDA/spconv/custom ops·loader·W&B·disk/공유메모리 smoke |
| 4 | 원격4090, 학습 전 | batch1 후보 추론 latency와VRAM, 동일 공간/depth·class/query 조건; 입력과fallback 선택 |
| 5 | 원격4090 | 실제 GT로 batch1/2 및 가능한4의 짧은 forward/backward/optimizer 시험; AMP skip·OOM·loader 비용 검사 |
| 6 | 원격4090 | 호환 nuScenes 초기화 로딩 감사, checkpoint 저장→외부 백업→중단/재개 확인 후Sunny13 5K 학습 |
| 7 | 원격 추론 + 로컬/원격CPU 평가 | checkpoint 후보의 mission dev-val detection/tracking/Map-CV 평가; 실패 거리·종류로 원인 정리 |
| 8 | 이후6환경30K | Sunny13 보존·나머지25K 새 geometry/split 감사; 공간 group 일관성과 초기화/finetune/resume 차이를 명시 |

4090 추론은 MORAI 없이 실제 raw의 근접·원거리·밀집 subset에서 cold/warm,
모델 단독/CPU 준비 포함 p50/p95/p99·peak VRAM을 기록한다. GPU 모델이 같아도
host CPU·전력·PCIe·디스크가 달라 최종 대회 풀스택 실시간성 인증과는 구분한다.
평균70ms가 아니라13Hz≈76.9ms의 전체 경로 여유와tail latency를 고려한다.
팀원4090 결과와도 [동일 조건](261009_RTX4090_CAPACITY_RESULTS.md)을 맞춰 대조한다.

기본은4-camera+LiDAR single-frame·7D·4-class, OD velocity/temporal OFF다.
물리 batch는 실측 후 결정하고 필요한 경우 accumulation을 쓴다. LR·warmup·scheduler는
최종 epoch/optimizer update 수에 맞춰 다시 검증한다. 짧은 smoke 스케줄을 본학습에
그대로 쓰거나, 추론의 GPU 속도 배수를 학습 ETA로 사용하지 않는다.
초기5K의 epoch와 본30K epoch는 아직 미확정이다. 짧은 측정에서
epoch 시간≈ceil(train 수/물리 batch)×iteration 시간+val/저장 시간으로 계산한다.

## Rosbag dev-val과 최종 평가

합성5K의train4255/val745를 유지하고 기존 반복 mission bag은 별도dev-val로 사용한다.
첫 Sunny13 학습은6환경 최종 성능을 대표하지 않는다. raw가 없는 bag은 새 detector를
재추론할 수 없으므로 sensor 포함 여부부터 확인한다.

bag→평가용 센서/GT info 가공은 가능하지만 전처리·timestamp·asset catalog/이름 없는
제원 복구·GT 속도 계약을 그대로 맞춘다. 인접 프레임을 무작위 train/val로 나누거나
사용한 bag을 최종 독립test로 다시 부르지 않는다. 평가 프레임·범위·threshold를 고정해
E25와 비교하고 threshold 튜닝은dev-val에서 한다. 필요하면원격은 detector 출력만
저장하고 기존CPU tracker/predictor·평가기는 로컬에서 실행한다. 새 checkpoint에 대한
추론 결과이지 rosbag에 과거 저장된 detection을 재사용하는 평가가 아니다.

장기 학습마다 모든 bag을 돌릴 필요는 없다. offline val로 후보를 좁힌 뒤 선택한
checkpoint에 같은 mission detection/tracking/Map-CV를 실행하고, 최종은 미사용 주행으로 확인한다.
[split·평가 계약](../../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md).

## 중단 복구와 종료 기준

- 최소 epoch마다 full checkpoint, 길어진epoch는15~30분 손실 예산에 맞춰 중간 저장도 검토한다.
  model·optimizer·scheduler·AMP scaler·epoch/global step·가능한 RNG 상태를 보존한다.
- checkpoint 완료 후만 비동기Drive 업로드하고 SHA256·완료 marker를 남긴다.
  latest resume본과best 후보·config/manifest/로그를 보관한다. 미완료파일을latest로 취급하지 않는다.
- 실제중단→재개에서 LR·update·optimizer/scaler가 이어지는지 확인한다. mid-epoch batch와
  sampler 순서까지 완전히 재현되는지는 별도로 검증하며 기본적으로 보장하지 않는다.
- 같은 실행의resume과 새학습/finetune의load_from을 구분한다. 5K→30K는dataset/schedule이
  달라지므로자동resume하지 않고 다음학습의초기화와스케줄을별도결정한다.
- 잔액·계약만료·disk·backup 실패를감시한다. instance destroy는사용자승인후,
  로컬/Drive에서weight·resume본·증거의무결성을확인한뒤한다. On-demand도외부백업은필수다.

임대 전 남은 준비는 최종 입력 후보, 실제 전달 용량, registry image와 direct runner,
4-class adapter, 주기적 uploader 및 재개 검증이다. 본학습 설정·예상시간·비용을 보고한 뒤
진행하며, 이 문서 작성만으로 임대나 업로드를 시작하지 않는다.
[상위 묶음 실험 계획](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md).

## 원격 MORAI는 학습 환경과 별도 검토

2026-10-09 기본 용도는 MORAI 없는 오프라인 학습과 추론 비용 시험이다. 원격 MORAI는
별도 소규모 실행 시험으로 분리하고 이를 위해 본학습을 늦추지 않는다. 실행 가능성은
대회용 MR에서 아직 검증하지 않았다. GPU 연산 능력 외에도 컨테이너의 GPU 그래픽/Vulkan,
원격 화면, 계정 인증·동시 실행 허용 범위 및 ROS/UDP 포트 매핑을 확인해야 한다.
[MORAI 공식 설치 안내](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/installation-and-setup),
[Vast 컨테이너·네트워크 안내](https://github.com/vast-ai/docs/blob/main/guides/instances/docker-environment.mdx).

원격 자동 수집을 시험한다면 MORAI와 캡처기를 같은 원격 환경에 두고 파일만 전달하는
구성을 우선 검토한다. 카메라·LiDAR를 인터넷으로 로컬에 보내는 방식의 대역폭·지연을
현재 localhost/LAN 계약과 같다고 가정하지 않는다. 동일 GPU의 MORAI/학습 병행은
경합과 latency 측정 왜곡 가능성이 있어 첫 구성에 포함하지 않는다. 인증 정보는 전달 이미지·Git에 넣지 않는다.
