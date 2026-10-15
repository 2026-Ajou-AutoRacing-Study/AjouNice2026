# Base5 입력·분리 pooling GPU 검증

2026-10-10. 실제 완료 5K에서 밀도/패밀리 기준으로 고른 14장과 최소 코드/GT만 포함한다.
학습된 checkpoint는 포함하지 않는다. 새 4-class 모델의 비용과 출력 동등성,
3-step loss/backward/AdamW 동작을 검증한다. 성능(AP) 실험이나 본학습이 아니다.

MORAI·다른 GPU 작업을 끈다. 호스트에 기존 팀 이미지가 있으면:

```bash
bash run.sh
```

Vast에서 같은 팀 이미지를 인스턴스 이미지로 사용한 경우 컨테이너 안에서:

```bash
AJOU_DIRECT_CONTAINER=1 bash run.sh
```

Docker 중첩 실행이나 ROS 설치가 필요 없다. 기존 이미지에 torch/cu128/spconv/
mmdetection3d 및 nvcc가 있어야 한다. 첫 CUDA 확장 빌드는 측정 전에 수행한다.
5060Ti용 컴파일 바이너리를 옮기지 않으며 해당 GPU에서 다시 빌드한다.

기본은 384×896/448×896/512×1056, 14장의 3회 순환, 8회 warmup이다.
baseline → cache A → cache+분리 B 비용, 모든 14장 출력 동등성,
각 shape의 baseline/B batch1/2 3-step 검증을 실행한다.
OOM도 결과로 보존한다. 실패하면 남은 단계는 중단하고 결과 ZIP을 남긴다.
일부 입력만 빠르게 재확인하려면:

```bash
bash run.sh --shapes 448x896 512x1056 --skip-training
```

완료 후 `SEND THIS ZIP:`의 ZIP 하나를 보내면 된다.
JPEG90은 CPU decode 비용 대조용이며 실제 MORAI 네트워크 JPEG 품질을 확인한 값은 아니다.
CUDA span은 호스트 idle을 포함한 critical-path 시간이다. parent와 substage를 더하지 않는다.
총 서비스 시간은 전처리/모델/출력 CPU 복사이며 ROS 전송·큐·tracker·planner를 포함하지 않는다.
구 4090 확대 비용은 직사각형 sparse 축 보정 이전이므로 새 비용으로 대체해야 한다.
