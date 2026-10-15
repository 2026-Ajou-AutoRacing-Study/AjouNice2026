# third_party
- 개발에 참고할 외부 코드들 -> 추후 우리 workspace에 통합할 대상

## Notes
- Submodule 방식으로 가져오는 것도 고려했으나, **이 경우 git 관리 난이도가 올라가므로** ~~그냥 통째로 원본 repo를 clone하기로 결정 -> 이를 **`vendoring`**이라고 부름.~~
  - 참고: https://blog.timhutt.co.uk/against-submodules/    
  - ~~원본 repo들을 그대로 두면 nested git이 되므로, repo 폴더 안의 `.git`을 모두 제거~~
- 위처럼 vendoring할 경우 나중에 git history가 너무 무거워짐...
  - -> 차라리 cloning scripts를 남기고 전부 제거해 버리자.
    - 이러면 추후 third party code 수정 시 버저닝 및 공유가 어렵긴 하지만, ... -> 나중에 생각해 보자. 
- **나중에 코드 수정이 필요하다면, fork 후 submodule 형태로 관리하면 됨. 언제든 방식 변경 가능. -> 하지만 이 경우 git 사용 난이도가 올라감 -> 고민**
- 나중에 이 코드들은 workspace 안의 `model_wrapper.py`와 같은 코드들이 호출해서 사용하게 될 듯.
- ~~새로 third_party code를 가져올 경우에는 `PATCH_NOTES.md`를 생성하기~~
  - ~~third_party code를 수정할 경우에는 `PATCH_NOTES.md`에 Local Changes를 간략하게라도 적기~~
- third_party repo들은 `.gitignore`에 추가 필요

## third_party list
- `FAST_LIO`: Localization / 원본 FAST-LIO 2.0 repo
- `FAST-LIO-Multi-Sensor-Fusion`: Localization / 위 stack에 gnss, wheel odem을 fusion한 repo -> 그대로 쓰지는 않을 수도 있음.
- `BEVFusion`: Perception
- `SimpleTrack`: Tracking
- `autoware_universe`: Autoware 전체 -> 여기서 prediction이나 planning 뽑아와서 우리 ws로 가져오기 -> 얘는 ros2 기반이기에 최종 실행 대상 코드는 아님.
- `lidar_ai_solution/CUDA-BEVFusion`: NVIDIA CUDA·TensorRT 배포 참조. 로컬 ignored clone이며 submodule이나 runtime dependency가 아니다.

## CUDA-BEVFusion 로컬 참조

2026-10-09에 공식 [NVIDIA Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)을
`third_party/lidar_ai_solution`에 shallow·partial clone했다. revision은
`7c1623f234d5a9794fd862c9456f68f87ae7b900`이며 sparse checkout으로
`CUDA-BEVFusion`과 `libraries/3DSparseConvolution`을 펼쳤다. 빌드·모델 다운로드·실행은 하지 않는다.
이 clone 자체는 Git ignore 상태로 유지하고, 이식 판단은
[CUDA-BEVFusion 검토](../docs/bevfusion/261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md)를 따른다.

새 checkout에서 참조 코드를 가져오는 명령은 아래와 같다. 기존 폴더가 있으면 재실행하지 않는다.

```bash
git clone --depth 1 --filter=blob:none --no-checkout \
  https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution.git \
  "$HOME/AjouNice2026/third_party/lidar_ai_solution"
git -C "$HOME/AjouNice2026/third_party/lidar_ai_solution" sparse-checkout init --cone
git -C "$HOME/AjouNice2026/third_party/lidar_ai_solution" sparse-checkout set \
  CUDA-BEVFusion libraries/3DSparseConvolution
git -C "$HOME/AjouNice2026/third_party/lidar_ai_solution" checkout master
```

위 명령은 실행 시점의 upstream을 받으므로 재현 시 `git rev-parse HEAD`를 기록한다.
과거 조사 revision과 달라졌다면 코드 차이를 확인한다.

## Quick Clone
```bash
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/zhh2005757/FAST-LIO-Multi-Sensor-Fusion.git
git clone https://github.com/mit-han-lab/bevfusion.git
git clone https://github.com/tusen-ai/SimpleTrack.git
git clone https://github.com/autowarefoundation/autoware_universe.git
```
