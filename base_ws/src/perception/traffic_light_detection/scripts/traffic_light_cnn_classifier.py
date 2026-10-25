#!/usr/bin/env python3
"""연구 CNN의 원시 4-bit 관측 adapter. ROS·박스 선택·신호 정책과 분리한다.

선택한 원본 uint8 BGR crop만 받는다. 구간 ID·native GT·진행/정지 판단은
입력하지 않는다. 비어 있는 tuples라도 known=False(UNKNOWN)와
known=True/no_lamps는 다르다. 이 모듈은 어떤 경우에도 HSV를 실행하거나
빈 관측을 녹색/통과로 추정하지 않는다.

권장 시작점 initialize_classifier()는 파일/의존성/계약 실패를 ready=False로
반환한다. 운영 legacy/unknown fallback 선택은 호출자가 명시적으로 소유한다.
Torch·torchvision·Pillow·NumPy는 실제 모델 생성 때만 import한다.
"""

from dataclasses import dataclass
import hashlib
import math
from numbers import Real
from pathlib import Path
import re
import threading
import time
from typing import Optional, Tuple


CLASSES = ("red", "yellow", "green", "left_arrow")
CLASS_TUPLES = (("red", "circle", None), ("yellow", "circle", None),
                ("green", "circle", None), ("green", "arrow", True))
WIDTH, HEIGHT = 224, 128
MEAN, STD = (.485, .456, .406), (.229, .224, .225)
PREPROCESSING = "inference_fixed_approved_ROI_then_aspect_preserving_RGB_black_letterbox"
TRAINING_SCOPE = "full_backbone_finetune_BN_running_stats_frozen_affine_trainable_FP32"


class CnnClassifierError(RuntimeError):
    """호출자가 legacy/unknown fallback을 선택할 수 있는 CNN 준비 실패."""


@dataclass(frozen=True)
class CnnObservation:
    known: bool
    tuples: tuple
    probabilities: tuple
    bits: tuple
    bit_confidences: tuple
    confidence: float
    reason: str

    def to_dict(self):
        return {"source": "cnn", "known": self.known,
                "tuples": [list(value) for value in self.tuples],
                "probabilities": dict(zip(CLASSES, self.probabilities)),
                "bits": dict(zip(CLASSES, self.bits)),
                "bit_confidences": dict(zip(CLASSES, self.bit_confidences)),
                "confidence": self.confidence, "reason": self.reason,
                "empty_result_kind": (None if self.tuples else "no_lamps" if self.known else "unknown")}


@dataclass(frozen=True)
class ClassifierStartup:
    ready: bool
    classifier: object
    reason: str
    metadata: dict


def unknown_observation(reason):
    return CnnObservation(False, (), (None,) * 4, (None,) * 4, (None,) * 4, 0.0, str(reason))


def validate_min_confidence(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or not .5 <= value <= 1:
        raise ValueError("min_bit_confidence는 0.5~1의 유한한 수여야 합니다")
    return float(value)


def observation_from_probabilities(probabilities, *, min_bit_confidence=.6):
    """학습 threshold=.5를 유지하고 저확신 bit가 하나라도 있으면 abstain한다.

    confidence는 min(max(p,1-p))이며 calibrated probability라는 주장은 아니다.
    .6 기본값은 [.4,.6] 부근의 불확실 관측을 버리는 명시적 연구 옵션이다.
    성능 검증/승격 없이 운영 기본값으로 해석하면 안 된다.
    """
    minimum = validate_min_confidence(min_bit_confidence)
    try:
        if isinstance(probabilities, dict):
            if set(probabilities) != set(CLASSES):
                raise ValueError("4개 class 이름이 정확히 일치하지 않음")
            values = tuple(probabilities[key] for key in CLASSES)
        else:
            values = tuple(probabilities)
        if len(values) != 4 or any(isinstance(value, bool) or not isinstance(value, Real)
                                   or not math.isfinite(value) or not 0 <= value <= 1 for value in values):
            raise ValueError("유한한 [0,1] 확률 4개가 필요함")
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        return unknown_observation("invalid_probabilities:" + str(error))
    confidence = tuple(max(value, 1 - value) for value in values)
    uncertain = [key for key, value in zip(CLASSES, confidence) if value + 1e-12 < minimum]
    bits = tuple(None if key in uncertain else int(value >= .5) for key, value in zip(CLASSES, values))
    if uncertain:
        return CnnObservation(False, (), values, bits, confidence, min(confidence),
                              "low_confidence_bits:" + ",".join(uncertain))
    results = tuple(result for result, bit in zip(CLASS_TUPLES, bits) if bit)
    return CnnObservation(True, results, values, bits, confidence, min(confidence),
                          "observed_lamps" if results else "no_lamps")


def letterbox_geometry(width, height):
    """학습기의 fixed-ROI letterbox와 같은 반올림/중앙 black padding."""
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("crop 크기는 양의 정수여야 합니다")
    scale = min(WIDTH / width, HEIGHT / height)
    size = (max(1, min(WIDTH, round(width * scale))), max(1, min(HEIGHT, round(height * scale))))
    return size, ((WIDTH - size[0]) // 2, (HEIGHT - size[1]) // 2)


def validate_device(device):
    if device != "cpu" and (not isinstance(device, str) or not re.fullmatch(r"cuda:[0-9]+", device)):
        raise ValueError("device는 cpu 또는 명시적 cuda:N이어야 합니다")
    return device


def validate_checkpoint_contract(checkpoint):
    """primitive metadata와 핵심 tensor shape 검사. 전체 state는 strict load한다."""
    if not isinstance(checkpoint, dict):
        raise ValueError("checkpoint는 metadata+model_state_dict 객체여야 합니다")
    expected = {"architecture": "resnet18", "classes": list(CLASSES),
                "input_size_wh": [WIDTH, HEIGHT], "normalization": "ImageNet",
                "preprocessing": PREPROCESSING, "training_scope": TRAINING_SCOPE}
    for key, value in expected.items():
        if checkpoint.get(key) != value:
            raise ValueError("checkpoint 계약 불일치: " + key)
    if checkpoint.get("research_only") is not True or checkpoint.get("not_production") is not True:
        raise ValueError("명시적 연구 후보 metadata가 필요합니다")
    threshold, dropout = checkpoint.get("threshold"), checkpoint.get("dropout")
    if isinstance(threshold, bool) or not isinstance(threshold, Real) or threshold != .5:
        raise ValueError("checkpoint threshold는 고정 0.5여야 합니다")
    if isinstance(dropout, bool) or not isinstance(dropout, Real) or dropout not in (0., .1):
        raise ValueError("지원하는 dropout은 학습 계약의 0 또는 0.1뿐입니다")
    state = checkpoint.get("model_state_dict")
    if not isinstance(state, dict) or not state or any(not isinstance(key, str) for key in state):
        raise ValueError("model_state_dict가 필요합니다")
    final = "fc" if dropout == 0 else "fc.1"
    for key, shape in (("conv1.weight", (64, 3, 7, 7)),
                       (final + ".weight", (4, 512)), (final + ".bias", (4,))):
        if key not in state or tuple(getattr(state[key], "shape", ())) != shape:
            raise ValueError("checkpoint tensor shape 불일치: " + key)
    return {"architecture": "resnet18", "classes": list(CLASSES), "input_size_wh": [WIDTH, HEIGHT],
            "normalization": "ImageNet", "preprocessing": PREPROCESSING, "threshold": .5,
            "dropout": float(dropout), "research_only": True, "not_production": True,
            "training_scope": TRAINING_SCOPE, "best_validation": checkpoint.get("best_validation")}


def _digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


class TrafficLightCnnClassifier:
    """한 번 로드한 FP32 eval 모델. 입력/추론 실패 시 UNKNOWN만 반환한다."""

    def __init__(self, checkpoint_path, *, device="cpu", min_bit_confidence=.6,
                 expected_sha256=None, warmup=True):
        validate_device(device)
        self.min_bit_confidence = validate_min_confidence(min_bit_confidence)
        path = Path(checkpoint_path).expanduser().resolve(strict=True)
        if not path.is_file() or path.stat().st_size > 256 * 1024 * 1024:
            raise CnnClassifierError("256MiB 이하의 명시적 local checkpoint 파일이 필요합니다")
        if expected_sha256 is not None and (not isinstance(expected_sha256, str)
                                             or not re.fullmatch(r"[0-9a-f]{64}", expected_sha256)):
            raise ValueError("expected_sha256은 소문자 SHA256이어야 합니다")
        snapshot = path.stat()
        digest = _digest(path)
        if expected_sha256 is not None and digest != expected_sha256:
            raise CnnClassifierError("checkpoint SHA256 불일치")
        # 실제 생성 때만 무거운 라이브러리를 로드한다. 네트워크/torch.hub 호출 없음.
        import numpy as np
        from PIL import Image
        import torch
        import torchvision
        from torchvision.models import resnet18
        if device.startswith("cuda:"):
            index = int(device.split(":")[1])
            if not torch.cuda.is_available() or index >= torch.cuda.device_count():
                raise CnnClassifierError("요청한 CUDA 장치가 없어 사용하지 않습니다; 자동 CPU fallback 없음")
        started = time.perf_counter()
        # weights_only 미지원 환경에서 unsafe pickle fallback을 하지 않는다.
        checkpoint = torch.load(str(path), map_location="cpu", weights_only=True)
        contract = validate_checkpoint_contract(checkpoint)
        state = checkpoint["model_state_dict"]
        for key, value in state.items():
            if not isinstance(value, torch.Tensor):
                raise CnnClassifierError("state_dict에 tensor가 아닌 값이 있습니다: " + key)
            if value.is_floating_point() and not bool(torch.isfinite(value).all()):
                raise CnnClassifierError("state_dict에 비유한 tensor가 있습니다: " + key)
        # 모델 생성이 같은 프로세스의 YOLO CPU RNG 상태를 바꾸지 않도록 보존한다.
        with torch.random.fork_rng(devices=[]):
            model = resnet18(weights=None)
            model.fc = (torch.nn.Linear(512, 4) if contract["dropout"] == 0 else
                        torch.nn.Sequential(torch.nn.Dropout(contract["dropout"]), torch.nn.Linear(512, 4)))
        model.load_state_dict(state, strict=True)
        current = path.stat()
        if (snapshot.st_ino, snapshot.st_size, snapshot.st_mtime_ns) != (current.st_ino, current.st_size, current.st_mtime_ns):
            raise CnnClassifierError("로드 중 checkpoint 파일이 변경되었습니다")
        self._torch, self._np, self._image = torch, np, Image
        self._device, self._model = device, model.float().to(device).eval().requires_grad_(False)
        self._lock = threading.Lock()
        self._warmup_done = False
        self.metadata = {**contract, "checkpoint_path": str(path), "checkpoint_sha256": digest,
                         "device": device, "precision": "FP32", "autocast": False,
                         "global_torch_threads_and_tf32_unchanged": True,
                         "torch_version": str(torch.__version__), "torchvision_version": str(torchvision.__version__),
                         "load_count": 1, "warmup_count": 0, "load_ms": (time.perf_counter() - started) * 1000,
                         "min_bit_confidence": self.min_bit_confidence,
                         "fallback": "호출자가 legacy/unknown을 명시적으로 선택; adapter 내부 fallback 없음",
                         "decoder_limit": "입력은 호출자가 decode한 BGR ROI; Pillow JPEG 학습 decode와 픽셀 동등성은 별도 검증"}
        if warmup:
            self.warmup()

    def _forward(self, tensor):
        torch = self._torch
        with torch.inference_mode(), torch.autocast(device_type=self._device.split(":")[0], enabled=False):
            logits = self._model(tensor.float().to(self._device))
            if tuple(logits.shape) != (1, 4) or not bool(torch.isfinite(logits).all()):
                raise CnnClassifierError("CNN 출력 shape 또는 유한성 오류")
            return torch.sigmoid(logits).detach().cpu().tolist()[0]

    def warmup(self):
        """한 번만 수행한다. 전역 thread/CUDA backend 설정은 변경하지 않는다."""
        with self._lock:
            if not self._warmup_done:
                started = time.perf_counter()
                self._forward(self._torch.zeros((1, 3, HEIGHT, WIDTH), dtype=self._torch.float32))
                self._warmup_done = True
                self.metadata.update(warmup_count=1, warmup_ms=(time.perf_counter() - started) * 1000)
        return dict(self.metadata)

    def _input_tensor(self, crop_bgr):
        np, Image, torch = self._np, self._image, self._torch
        if (not isinstance(crop_bgr, np.ndarray) or crop_bgr.dtype != np.uint8
                or crop_bgr.ndim != 3 or crop_bgr.shape[2] != 3 or min(crop_bgr.shape[:2]) <= 0):
            raise ValueError("uint8 H×W×3 BGR 원본 crop이 필요합니다")
        rgb = np.ascontiguousarray(crop_bgr[:, :, ::-1])
        crop = Image.fromarray(rgb, mode="RGB")
        size, offset = letterbox_geometry(*crop.size)
        canvas = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
        canvas.paste(crop.resize(size, getattr(Image, "Resampling", Image).BILINEAR), offset)
        tensor = torch.from_numpy(np.asarray(canvas).copy()).permute(2, 0, 1).float() / 255.
        mean = torch.tensor(MEAN, dtype=torch.float32)[:, None, None]
        std = torch.tensor(STD, dtype=torch.float32)[:, None, None]
        return ((tensor - mean) / std).unsqueeze(0)

    def classify_bgr(self, crop_bgr):
        try:
            tensor = self._input_tensor(crop_bgr)
            with self._lock:
                probabilities = self._forward(tensor)
            return observation_from_probabilities(probabilities, min_bit_confidence=self.min_bit_confidence)
        except Exception as error:
            return unknown_observation("inference_error:" + type(error).__name__ + ":" + str(error))


def initialize_classifier(checkpoint_path=None, *, device="cpu", min_bit_confidence=.6,
                          expected_sha256=None, warmup=True, classifier_factory=None):
    """선택적 CNN 준비. 실패는 보고만 하며 launch/legacy 실행을 직접 거부하지 않는다."""
    fallback = {"implicit_fallback": False, "caller_must_choose_legacy_or_unknown": True}
    if checkpoint_path is None or str(checkpoint_path).strip() == "":
        return ClassifierStartup(False, None, "not_configured", fallback)
    try:
        factory = TrafficLightCnnClassifier if classifier_factory is None else classifier_factory
        classifier = factory(checkpoint_path, device=device, min_bit_confidence=min_bit_confidence,
                             expected_sha256=expected_sha256, warmup=warmup)
        return ClassifierStartup(True, classifier, "ready", dict(classifier.metadata))
    except Exception as error:
        return ClassifierStartup(False, None, "unavailable:" + type(error).__name__ + ":" + str(error),
                                 {**fallback, "checkpoint_path": str(checkpoint_path), "requested_device": device})
