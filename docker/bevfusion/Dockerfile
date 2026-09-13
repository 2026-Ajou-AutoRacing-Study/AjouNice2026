FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-devel

ARG DEBIAN_FRONTEND=noninteractive
ARG MAX_JOBS=8
ARG MMCV_REF=v2.1.0

ENV PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1 \
    MAX_JOBS=${MAX_JOBS} \
    TORCH_CUDA_ARCH_LIST="8.6;8.9;12.0+PTX"

# CUDA extension 빌드와 OpenCV 실행에 필요한 OS 패키지 설치
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        git \
        libgl1 \
        libglib2.0-0 \
        wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ajounice

# 현재 검증된 Python 의존성 버전 설치
COPY docker/bevfusion/requirements/base.txt \
    /opt/requirements/base.txt
COPY docker/bevfusion/requirements/inference.txt \
    /opt/requirements/inference.txt

RUN python -m pip install --upgrade \
        pip==26.1.2 \
        setuptools==69.5.1 \
        wheel==0.47.0 \
    && python -m pip install \
        -r /opt/requirements/base.txt \
        -r /opt/requirements/inference.txt

# MMCV 2.1.0을 RTX 30·40·50용 CUDA ops와 함께 소스 빌드
RUN git clone \
        --branch "${MMCV_REF}" \
        --depth 1 \
        https://github.com/open-mmlab/mmcv.git \
        /opt/mmcv \
    && FORCE_CUDA=1 \
        MMCV_WITH_OPS=1 \
        python -m pip install \
            --verbose \
            --editable /opt/mmcv \
            --no-build-isolation

# 팀 MMDetection3D fork를 이미지에 복사
COPY third_party/mmdetection3d /opt/mmdetection3d

# 호스트 서브모듈 전용 Git 포인터를 제거하고 MMDetection3D 설치
RUN rm -f /opt/mmdetection3d/.git \
    && python -m pip install \
        --editable /opt/mmdetection3d \
        --no-deps \
        --no-build-isolation

# BEVFusion bev_pool·voxelization custom CUDA ops 빌드
RUN cd /opt/mmdetection3d \
    && FORCE_CUDA=1 \
        python projects/BEVFusion/setup.py develop

# Training-only observability is kept in a final layer so changing W&B does
# not invalidate the expensive MMCV/BEVFusion CUDA-extension build cache.
COPY docker/bevfusion/requirements/training.txt \
    /opt/requirements/training.txt
RUN python -m pip install -r /opt/requirements/training.txt

WORKDIR /opt/mmdetection3d

CMD ["bash"]
