# 팀원용 독립 GPU 속도 측정

Git clone, ROS 설치, 빌드, rosbag 재생, Google Drive 추가 다운로드는 필요 없다.
이 폴더에 실제 입력 3장과 E25 가중치가 포함되어 있다. 팀 내부에서만 공유한다.
기존 NVIDIA Docker 및 호환 BEVFusion 이미지, 호스트 Python3가 필요하다.

## 실행

MORAI, 학습, 다른 GPU 추론을 끈다. 폴더를 압축 해제한 뒤 터미널에서 실행한다.

```bash
cd ~/Downloads/ajounice_capacity_4090
bash run.sh
```

홈/다운로드 등 압축을 푼 위치에 맞춰 cd만 바꾼다. ajou/source/roscore는 사용하지 않는다.
기본 이미지 tag는 `ajounice/bevfusion:team-cu128-v1`이다.
이미 같은 호환 이미지를 다른 이름으로 가지고 있으면 다음과 같이 지정한다.

```bash
docker images
bash run.sh --image 실제_이미지_이름:태그
```

여러 GPU가 있으면 `bash run.sh --gpu 1`로 선택한다. 다른 프로세스를 자동 종료하지 않는다.
Docker daemon 권한이나 NVIDIA runtime 문제가 나오면 먼저 기존 Docker 실행 환경을 확인한다.
인터넷으로 이미지를 자동 다운로드하거나 Docker build하지 않는다.

## 무엇을 측정하나

batch1 AMP로 아래 네 조건을 각60회, 역순으로 각30회 재측정한다.
각 조건마다 실제 입력으로6회 워밍업한다. 보통 수분이며 첫 CUDA/autotune 준비에 따라 길어질 수 있다.

| 조건 | 이미지 | 입력 X/Y 범위 | depth bins |
|---|---|---|---:|
| E25 현재 |256×704|±30/±30m|118|
| 이미지 확대만 |320×896|±30/±30m|118|
| 범위 확대만 |256×704|±100/±30m|198|
| 둘 다 확대 |320×896|±100/±30m|198|

모두 voxel XYZ0.075/0.075/0.20m, camera BEV XY0.30m, depth간격0.50m,
3클래스/200query다. 범위 확대 시 내부 격자는 stride 때문에 ±100.8/±32.4m로 padding한다.
기존 E25 weights 중 호환되는 부분을 사용한다. 확대 depth 출력층은 채널수가 달라 새 초기화다.
이를 accuracy 평가나 학습 완료 모델로 해석하지 않는다.

측정값은 모델 처리 평균/p50/p95, CPU 전처리, 단계별 CUDA stream span,
CPU 전처리 포함 offline 처리 시간, 워밍업/측정 중 PyTorch 할당 메모리다.
GPU 사용률·메모리·전력·온도 로그, 드라이버/이미지/입력 해시도 보존한다.
단계 시간에는 host 대기가 포함될 수 있으며 순수 kernel 실행시간은 아니다.

ROS 전송/큐/최종 publish, 실제 full-stack 처리Hz·드롭, 탐지 AP·tracking 성능은 측정하지 않는다.
입력은 동일한 작은 고정 표본이다. 원거리 객체가 많은 가장 복잡한 장면의 최악 비용을 보장하지 않는다.
torch allocator 메모리는 nvidia-smi 전체 점유량과 다르므로 둘 다 보존한다.

## 보내줄 파일

끝나면 마지막 줄에 다음 형식의 경로가 출력된다.

```text
RESULT: PASS
SEND THIS ZIP: .../results/capacity_날짜_시간_PID.zip
```

**그 결과 zip 하나만 주상우에게 보내면 된다.** 원본 가중치·입력은 결과에 다시 넣지 않는다.
SUMMARY.md, forward/reverse JSON, 실행 로그, GPU telemetry, metadata, checksum이 들어 있다.
순서별 시간이10% 넘게 다르면 PASS_WITH_REPEAT_WARNING으로 표시한다.
FAILED라면 성공 결과가 아니다. 실패 로그 zip도 전달하면 원인 확인이 가능하다.
Ctrl+C는 해당 측정용 컨테이너만 종료한다. 기존 사용자 컨테이너는 건드리지 않는다.

정확한 기본 실험으로 비교하려면 iterations 값을 바꾸지 않는다.
간단한 실행 확인만 필요할 때는 `bash run.sh --iterations 10 --repeat-iterations 10`을 쓴다.
