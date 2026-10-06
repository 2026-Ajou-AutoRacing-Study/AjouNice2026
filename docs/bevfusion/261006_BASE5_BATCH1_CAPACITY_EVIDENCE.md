# Base5 batch1 / ROS 포화 처리량 — EVIDENCE

2026-10-06. RTX5060Ti16GB, MORAI simulator OFF(launcher/UI 등 소규모 display 프로세스는 남음),
RViz/Planning/Tracker OFF. Production E25·threshold·bridge·학습 설정은 변경하지 않았다.
4090 결과, 원거리 정확도, full-stack 속도, 장시간 안정성 인증이 아니다.

## 입력과 측정 계약

팀원 H01 development bag:
`rosbags/eval/perception/260923_teammate_manual/highway/260923_161315_H01_development/260923_161315_H01_ajounice_eval_highway_balanced_0.bag`.

batch1 AMP, deskew/temporal OFF. bag20/50/80% 시점의 원본9,435/10,636/9,468점 및4카메라
압축 payload를 선택했다. camera nearest 차이는 최대27.2ms로100ms 계약 이내다.
포인트 수를 줄이거나 임의 합성하지 않았다. 세 표본을 순환하며6회 워밍업 뒤60회 측정했다.

- `forward_ms`: GPU upload/정규화·voxelization·모델·decode까지 `test_step`을 CUDA synchronize 전후로 측정.
- `service_ms`: CPU압축 decode/resize/crop/pack + forward + 결과CPU복사. ROS/TCP/발행 제외.
- `serial_fps`: LiDAR cadence와 무관한 연속 batch1 완료수/실제 wall elapsed.
- 메모리는 워밍업 peak와 워밍업 후 steady peak를 구분한다. reserved/allocated는 GPU전체 사용량과 다르다.

## Offline 같은 실제 프레임 비교

| 구성 | 이미지 H×W | ROI 종/횡 | Service 평균 / p50 / p95 ms | Forward 평균 ms | 포화 serial FPS | Steady allocated peak MiB |
|---|---|---|---|---|---|---|
| Production E25 |256×704|±30/±30m|71.94 /71.90 /73.24|65.29|13.90|1304|
| E25 highres 비용 대조 |320×896|±30/±30m|96.77 /96.76 /98.59|89.56|10.33|1937|
| Base5 geometry 저해상도 |256×704|±100/±30m|71.59 /71.51 /73.97|65.95|13.97|1146|
| Base5 현재 geometry |320×896|±100/±30m|94.52 /94.39 /96.86|88.14|10.58|1684|

Base5두행은 fresh4-class architecture 비용 probe이며 학습되지 않은 가중치다.
E25 highres는 frustum shape가 달라 checkpoint geometry를 config 기준으로 복원했다.
세 경우 모두 정확도 개선을 의미하지 않는다.

Base5는 단순ROI만 확대하지 않았다. LiDAR voxel0.075→0.10m, camera BEV step0.3→0.4m,
depth bound 및rectangular head/4-class도 다르다. 따라서 이 표는 현재 후보 **전체 설정의 비용**이지
ROI 하나만의 인과 A/B가 아니다. wide와baseline의1ms 이하 차이는 유의미한 성능 우위로 해석하지 않는다.
현재 sampled cloud에서 wide ROI 보존점은8,644/10,306/8,651로 증가했다.
실제 높은 객체 밀도/다른 도로 입력에 대한 보장은 아니다.

카메라 해상도만 높인 E25 대조는 service약34.5% 증가, FPS약25.7% 감소다.
현5060Ti에서는320×896의약10–11FPS가 원래LiDAR약12Hz보다 낮다.
4090에서의 처리량은 같은 명령으로 별도 측정해야 한다.

초기 cuDNN/spconv 워밍업 allocated peak는 E25약10.6GiB,highres약10.6GiB였다.
wide도 실행 순서/커널 캐시에 따라약6.1–8.4GiB까지 올라갔다. steady1–2GiB만 보고
MORAI동시기동 시 VRAM 여유를 보장하지 않는다. 워밍업 peak는 알고리즘 탐색/cache 영향을 받는다.

## Production ROS bridge 원속도 / 포화

별도ROS master·TCP29501에서 최신 production bridge를 실행했다. 기존 master와 시뮬레이터는 건드리지 않았다.
20개의 실제 프레임 출력으로 워밍업한 뒤 각rate20초씩 측정했다. 원본header timestamp 유지,
sim-time delta를 latency로 계산하지 않는다.8.6배속은 bag 반복재생이며 temporal OFF다.

| 재생 | 관측 LiDAR / 출력 Hz | 입력 / 출력 개수 | 수신→응답 평균 / p50 / p95 ms | GPU forward 평균 ms | raw/prepared drop 증가 |
|---|---|---|---|---|---|
|1배|11.47 /11.47|234 /234|81.30 /80.40 /87.44|65.80|0 /0|
|8.6배|97.62 /12.97|1987 /264|88.89 /88.00 /97.76|69.30|32 /1691|

`latency_ms`는 bridge LiDAR callback의monotonic receipt→TCP 응답수신이다.
이 값은 detector message 발행 직전에 기록되므로 전체 Planning 수신까지의 지연이 아니다.
출력Hz는 observer가 detection topic에서 실제 수신한 개수/wall-time으로 측정했다.
관측입력·관측출력·내부drop counter는 boundary/ROS queue/센서매칭 차이 때문에 엄밀한 일대일 장부는 아니다.

포화 시 기존 latest-only drop으로 대기가 무한히 쌓이지 않는다. 약98Hz를 모두 처리한 것이 아니다.
앞뒤 처리가 overlap되므로 **1/평균 end-to-end latency**를 최대출력Hz로 그대로 쓰지 않는다.
현재 표본에서 CPU병목보다는GPU forward+request stage가 주로 처리량을 제한한다.
offline forward65.29ms와원속도ROS65.80ms가 근접함을 실제 확인했다.

처음 primer없이 측정한 `ros_e25`는 최초연결/real-frame warmup이 섞여원속도출력3.86Hz였다.
최종 steady 결과는 `ros_e25_warmed`를 사용하며, 초기 측정은 삭제하지 않는다.

## 산출물 / 재현

Git ignored root: `artifacts/evaluation/bevfusion/capacity_261006/`.
`samples/samples.json`, `offline_final.json`, `ros_e25_warmed/report.json`,
`ros_e25_warmed/status_*.jsonl` 및master/bridge/play 로그가 근거다.
초기실행에서wide_lowres augmentation/view shape 불일치를 잡아 benchmark의in-memory
test pipeline override만 수정했고최종4경우 모두 통과했다. Production config는 변경하지 않았다.

호스트 ROS 환경:

```bash
ajou
python3 perception/detection3d/bevfusion/scripts/extract_latency_samples.py \
  --bag /path/to/raw.bag --output /path/to/new_samples
```

팀Docker의 repository(`/workspace/AjouNice2026`)와최신app(`/opt/ajounice/app`) mount,
`PYTHONPATH=/opt/ajounice:/workspace/AjouNice2026:/opt/mmdetection3d` 환경에서:

```bash
python perception/detection3d/bevfusion/scripts/benchmark_base5_capacity.py \
  --samples /path/to/new_samples --output /path/to/offline.json --iterations 60
```

별도29501의AMP/E25 server를준비한뒤,호스트ROS 환경에서:

```bash
ajou
python3 perception/detection3d/bevfusion/scripts/benchmark_ros_capacity.py \
  --bag /path/to/raw.bag --output /path/to/new_ros_probe \
  --server-port 29501 --rates 1 8.6 --wall-seconds 20
```

이 harness는 own master/bridge/play만 종료하며 이미준비한server lifecycle은호출자가관리한다.
이번시험의 own Docker server/master/bridge/play는 종료했다.
현재bridge는256×704 고정이며 Base54-class runtime도미연결이다.
따라서확대후보 ROS처리량은측정하지 않았고그부분을 E25 ROS숫자로대체하지 않는다.

## 다음

### 4090 팀원 독립 측정 패키지

Git/ROS 없이팀Docker로같은고정간격4-case를실행하는handoff를만들었다.
빌더는`perception/detection3d/bevfusion/scripts/build_portable_capacity_bundle.py`,
팀원용instructions/runner는같은scripts의`portable_capacity/`다.
E25가중치·H01raw3장·Sensor교정·필요config/plugin을포함한다.
각60회측정/역순30회재확인, CPU전처리+모델처리평균/p50/p95,
단계별span·메모리·GPUtelemetry·image/driver정보를결과ZIP으로자동반환한다.
ROS/full-stack출력Hz·AP·4090실측을이미완료했다고해석하지않는다.

최종전달ZIP:
`artifacts/handoff/capacity_4090_261006_v2/ajounice_capacity_4090.zip`
(151,437,062bytes, SHA256 `1d695096388261ebd7df0e212a76ef9f24b5c31db231de5b21c75798d8aa149c`).
구v1은공통metric import누락으로FAILED. common모듈을포함한v2를패키지만mount한
Docker에서각10회/역순10회로검증해8/8case통과와최종PASS ZIP생성을확인했다.
전달ZIP내manifest의전체file해시와반환ZIP의해시도검사했다.
코드·README만tracked, 가중치/raw/delivery/result ZIP은ignored그대로다.

예상1.5–2배보다비용이낮았던원인은[단계별분해·finegrid/depth대조](261006_BASE5_LATENCY_STAGE_BREAKDOWN.md)에서실측했다.
격자/depth를기존처럼촘촘하게한100m highres 비용대조는GPU모델처리약129.21ms로E25의약1.98배다.

Base5 설계/수집 전 GPU wiring·용량 측정은 완료됐다.10K생성/캡처는별도승인후진행한다.
현320×896/±100m 후보로학습한 뒤 geometry/class/resize 계약을온라인에연결하고,
4090에서full-stack latency·출력률·정확도를함께검증한다. 원거리학습이나production 교체를
현재cost probe로선결정하지 않는다.
