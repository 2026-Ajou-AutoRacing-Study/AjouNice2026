# RTX 5060 Ti 검출 성능과 추론 속도 재평가

상태: 검증 완료. 2026-11-09 KST. 기존 production 설정은 변경하지 않았다.

E25와 E12의 검출 성능, E12 실행 최적화의 효과를 동일 RTX 5060 Ti 16GB에서
분리해 측정한다. 과거 4090 수치 또는 초기 비용 검사용 모델을 새로운 가속률의
분모로 합치지 않는다. 기존 validation 재평가이며 독립 test set은 아니다.

## 입력과 실행 환경

- 코드 기준: `archive/post-competition`, 시작 HEAD `5a2c780e74648d5c3dac5d4313c78849d4a04489`.
  기존 미커밋 작업은 보존하며 새 평가 코드는 별도 파일로 추가한다.
- GPU: RTX 5060 Ti, 표시 VRAM 16,311MiB. 드라이버 580.126.09.
- Docker: `ajounice/bevfusion:team-trt-cu128-261023-v1`, image ID
  `sha256:26880867295feaac329d5c9fdd20dcbe572677e5f0cf71e0515d82db8e2d1230`.
- Torch 2.7.1+cu128, TensorRT 10.13.3.9. 모든 GPU 작업은 직렬로 실행한다.
  MORAI 본체·ROS·추론 서버를 시작하지 않고 컨테이너 네트워크도 차단한다.
- native validation: 4,614프레임, 고유 token 4,614개, 센서 파일 23,070개.
  기존 파일을 읽기 전용으로 마운트하며 새 학습이나 원본 복제를 하지 않는다.
- infos SHA256: `b5196957992d1394f9d9917e2cc9481223aa524a578ef57fea5cf1c61d1f52c1`.
  원격의 과거 infos와 경로 문자열은 다르지만 GT identity를 별도로 비교한다.
- 고정 GT identity: `7f6b5b9ff7c3c981a4261bba035eae388c57a9827cefefa5685e02c084a1bbf2`.
- 모든 프레임의 카메라 보정 18,456개를 runtime 센서 설정과 대조했다.
  intrinsic 차이는 0, extrinsic 최대 절대 차이는 `3.33e-16`이다.
- E25 SHA256: `36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011`.
- E12 SHA256: `f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8`.
- 이 GPU에서 생성했던 E12 camera engine을 strict metadata·실제 추론으로 다시 검사한다.
  다른 GPU 엔진을 가져오거나 가속 실패를 eager 결과로 바꿔 표기하지 않는다.

## 검출 성능 비교

E25와 E12는 같은 프레임·GT·3클래스에서 FP32로 비교한다. 각 모델의 학습 당시
이미지 전처리·공간 범위를 유지하고, 주 지표는 두 모델의 공통 box와 LiDAR XY
반경 30m 미만의 교집합이다. E25에는 native intensity 0–1을 원래 학습 범위
0–255로 되돌리는 input-only adapter를 명시적으로 적용한다.

E12의 AMP와 TensorRT+SDPA 조합은 별도 전체 검증셋으로 비교해 최적화에 따른
AP 변화를 측정한다. 모든 출력은 raw native box이며 ROS class threshold·NMS를
정확도 평가에 추가하지 않는다. GT 0점 또는 빈 GT 프레임도 삭제하지 않는다.

여기서 AP는 중심점 XY 거리 0.5/1/2/4m threshold의 AP 평균이다.
공식 nuScenes mAP, 분류 정확도, 실제 주행 성공률과는 다르다.
전체 native GT는 차량 3,972·보행자 2,850·장애물 3,685개이며, 빈 GT 프레임은
232개다. LiDAR 0점 GT(차량 85·보행자 98·장애물 418개)도 보존한다.

성능 차이는 데이터·입력 표현·학습이 함께 달라진 모델 간 차이다.
해상도 또는 데이터 장수 하나의 인과효과로 해석하지 않는다.

### 완료된 FP32 공통 30m 결과

두 모델 모두 4,614프레임을 strict load·FP32로 다시 실행했다.
ordered GT identity는 동일하며 아래는 기존 validation 분포의 결과다.

| 모델 | Vehicle AP | Pedestrian AP | Obstacle AP | mAP |
|---|---:|---:|---:|---:|
| E25 | 93.0979% | 72.8773% | 63.8699% | 76.6151% |
| E12 | 98.8403% | 97.8790% | 92.4655% | 96.3949% |

mAP는 +19.7799%p, 장애물 AP는 +28.5955%p다. 최종 근거는 산출물의
`common30_metrics/comparison.json`이다. 20–30m 장애물 AP도 32.1027→86.8398%로
개선됐다. 데이터·입력·학습 전체의 차이이며 해상도만의 효과는 아니다.

## 실행 속도 비교

최종 E12 가중치·512×1056 입력·5cm voxel·20cm camera BEV·25cm depth 간격을
고정한다. 고정 seed로 validation에서 선택한 96개 고유 입력을 각 case에서
16회 warmup 후 2회 반복하고 순서를 뒤집어 다시 측정한다.

| 단계 | 변경 |
|---|---|
| original | 해당 인스턴스의 pooling hook을 해제해 원래 materialized 연산으로 복원 |
| cache | 원래 연산에 기하·정렬 cache만 추가 |
| separated | depth/feature 분리 CUDA pooling 적용 |
| parallel | camera/LiDAR 병렬화 추가 |
| trt | camera backbone·neck TensorRT 추가 |
| trt_head | head SDPA·BEV 좌표 cache 추가, proposal topk는 OFF |

서비스 구간은 메모리의 원본 JPEG에서 전처리·GPU 추론·CPU raw box 반환까지다.
파일 읽기·네트워크·ROS·tracking을 제외하며 모델 실행 구간도 별도로 기록한다.
평균·p50·p95·p99·최대와 allocator peak를 남긴다. allocator peak는 전체 GPU
사용량이 아니다. desktop GPU 프로세스와 온도·클럭 상태도 보존한다.
PyTorch AMP FP16·TF32 OFF·cuDNN benchmark OFF로 통제한다. 각 프로세스의
초기 준비와 16회 warmup은 제외하고, 단계별 총 384회(고유 입력 96개)를 측정한다.
GPU 클럭은 고정하지 않았으므로 두 실행 순서의 차이를 함께 해석한다.

원형이 OOM이면 실패로 남기고 해상도·범위·가중치를 줄여 대체하지 않는다.
원형과 후속 단계의 raw 및 ROS 전달 후 출력 차이를 별도로 계산한다.

### 완료된 속도 결과

| 누적 단계 | 평균 ms | p95 ms | 순방향/역방향 평균 ms | PyTorch peak allocated MiB |
|---|---:|---:|---:|---:|
| 원형 연산 복원 | 420.089 | 426.657 | 420.157 / 420.022 | 10998.3 |
| 기하·정렬 cache | 404.773 | 411.087 | 405.850 / 403.696 | 10635.7 |
| depth/feature 분리 CUDA pooling | 145.704 | 149.230 | 146.017 / 145.390 | 1600.0 |
| camera/LiDAR 병렬화 | 140.356 | 142.902 | 139.777 / 140.934 | 1609.6 |
| camera TensorRT | 106.755 | 109.700 | 106.343 / 107.166 | 1549.3 |
| head SDPA·좌표 cache | 99.865 | 101.743 | 99.941 / 99.789 | 960.5 |

평균 지연은 76.23% 감소했고 순차 처리율 환산은 2.38→10.01회/초, 4.21배다.
환산 처리율은 ROS 수신 Hz나 동시 실행 풀스택 처리율이 아니다.
최종 p99는 106.732ms, 최대는 245.309ms다. 이상치를 삭제하지 않았으므로
평균 100ms가 모든 프레임의 최대 지연을 보장하는 것은 아니다.
메모리는 PyTorch allocator만의 peak이며 TensorRT 외부 할당·화면 점유를 포함하지 않는다.

### 최적화에 따른 정확도 변화

| E12 실행 | 공통 30m mAP | 전체 ROI mAP |
|---|---:|---:|
| FP32 | 96.394947% | 94.068321% |
| AMP FP16 | 96.394978% | 94.067954% |
| TensorRT + head 최적화 | 96.396248% | 94.068818% |

전체 ROI의 AMP 대비 차이는 +0.000864%p로 사실상 유지됐다. 실제 정확도 향상으로
주장할 크기가 아니다. 공통 30m와 전체 ROI의 평가 범위가 달라 두 수치를 섞지 않는다.

고유 96프레임의 원형 대비 최종 출력 대조에서 ROS 전달 후 박스는 차량 86·보행자
55·장애물 50개로 서로 동일하고 미대응 박스는 없었다. XY 중심 최대 차이는
차량 0.605mm·보행자 0.290mm·장애물 0.238mm였다. 다만 보행자 score 최대 차이
0.004356이 기존 공학적 허용치 0.002를 넘으므로 엄격한 출력 동등성 검사는 실패다.
threshold 이전 raw top200에는 낮은 점수 박스의 대응 변화도 있다.
AP 보존을 근거로 기록하되 bitwise 동일·전 박스 동등성 통과로 보고하지 않는다.

## 재현과 결과

산출물: `artifacts/evaluation/bevfusion/portfolio_5060ti_261109_v1/`.
입력 SHA·manifest·명령·로그·압축 예측·통계는 이 경로에 보존한다.
4개 설정의 전체 validation 추론, 12개 속도 작업이 모두 exit 0으로 완료됐다.
자동 queue의 최초 CPU summary는 빈 GT 배열 shape 오류로 exit 1이었다.
빈 프레임을 삭제하지 않고 `(0, 7)`로 복원하도록 수정한 뒤 CPU 집계만 다시 실행해
exit 0을 확인했다(`summary.log`, `summary_retry_01.log`). GPU 결과는 재실행·덮어쓰기하지 않았다.
따라서 최초 queue의 실패 이력과 최종 `SUMMARY.json`의 `complete: true`를 함께 보존한다.
최종 결과·표·그림은 `SUMMARY.json`, `REPORT.md`, `runtime_ablation.png`,
`accuracy_common30.png`다. 전체 센서 파일 23,070개의 종료 SHA가 시작과 모두 같았다.

실행 도구는 `scripts/evaluation/*bevfusion_portfolio*.py`다. 같은 출력 경로에
새 queue를 중복 실행하지 않는다. 입력 계약·summary/metric CPU 검사 7개를 통과했다.
빈 GT/빈 예측 프레임의 집계 회귀도 해당 테스트에 포함했다. Python 문법 검사와
`git diff --check`를 통과했다. 전체 문맥 검사는 기존 `kcity_map/README.md`의 줄 수
136>65 문제만 실패했으며, 이 작업에서 지도 문서를 변경하지 않았다.

## 포트폴리오에 사용할 수 있는 요약

> 동일 RTX 5060 Ti 16GB와 기존 validation 4,614프레임에서 초기 E25 대비 최종 E12의
> 공통 30m 중심거리 mAP를 76.62→96.39%로 개선했다. 최종 E12를 고정한 별도 실험에서는
> 분리 CUDA pooling·병렬화·TensorRT·head 최적화를 누적해 JPEG 전처리 포함 평균
> 처리시간을 420.1→99.9ms로 76.2% 단축했고, 전체 ROI mAP는 94.068% 수준을 유지했다.

‘원 논문 모델보다 4.21배’가 아니라 **현재 최종 E12의 pooling 원형 복원 대비**다.
과거 약 200ms 기억이나 다른 GPU의 60ms 수치와 이 표를 연결하지 않는다.
독립 test·실차 일반화·폐루프 성공률은 이번 평가 범위가 아니다.
