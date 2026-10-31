"""Startup-only optional accelerator policy, never a driving-time fallback.

Only named accelerator availability failures qualify. The selected checkpoint,
config, input contract and model-loading errors are never replaced or waived.
The module has no eager GPU/framework imports.
"""
import gc
import logging
import sys


LOGGER = logging.getLogger("bevfusion_server")
POLICIES = ("off", "prefer", "require")


class AccelerationUnavailableError(RuntimeError, ValueError):
    """A named optional-accelerator failure before its first CUDA enqueue."""
    def __init__(self, reason_code, message):
        super().__init__(message)
        self.reason_code = str(reason_code)
        self.restart_required = False


class AccelerationExecutionError(RuntimeError):
    """No same-process recovery after an accelerator enqueue/CUDA failure."""
    def __init__(self, reason_code, message):
        super().__init__(message + "; restart a fresh process with --camera-tensorrt-policy off (or bevfusion --safe); no in-process eager fallback")
        self.reason_code = str(reason_code)
        self.restart_required = True


def is_nonrecoverable_cuda_error(error):
    """Recognize sticky CUDA faults, not ordinary bad inputs or allocation OOM."""
    if not isinstance(error, RuntimeError):
        return False
    message = str(error).lower()
    return any(marker in message for marker in (
        "illegal memory access", "device-side assert", "misaligned address",
        "unspecified launch failure", "device has been lost", "cuda context is destroyed"))


def resolve_camera_startup_policy(policy=None, engine=None, metadata=None):
    if policy is None or policy == "":
        return "prefer" if engine or metadata else "off"
    if policy not in POLICIES:
        raise ValueError("camera TensorRT policy must be off, prefer, or require")
    return policy


def _attempt(factory, kwargs):
    """Return text only on optional failure; do not retain a GPU traceback."""
    try:
        return factory(**kwargs), None
    except AccelerationUnavailableError as error:
        reason = dict(reason_code=error.reason_code, reason=str(error))
        # Constructor tracebacks can own the half-created CUDA model. Never
        # carry this exception (or chained exception) into the eager rebuild.
        error.__traceback__ = None
        error.__cause__ = None
        error.__context__ = None
    return None, reason


def _release_failed_startup():
    """Outside the exception scope: collect old refs, then check CUDA health."""
    gc.collect()
    torch = sys.modules.get("torch")
    cuda = getattr(torch, "cuda", None)
    if cuda is None or not getattr(cuda, 'is_initialized', lambda: False)():
        return
    try:
        cuda.synchronize()
        cuda.empty_cache()
    except Exception as error:
        # A sticky CUDA error cannot be hidden by reloading weights into the
        # same process. No factory retry occurs on this branch.
        raise AccelerationExecutionError("cuda_context_unhealthy",
            "CUDA health check failed after optional accelerator startup: " + str(error)) from error


def construct_camera_runtime(factory, factory_kwargs, policy=None, engine=None, metadata=None):
    """Construct one runtime, optionally retrying the same model eagerly once.

    `factory_kwargs` is copied, never mutated. Only engine/metadata options are
    removed for fallback. All model paths, AMP/parallel settings and thresholds
    remain the caller's original choices. Direct model/adapter APIs stay strict.
    """
    selected_policy = resolve_camera_startup_policy(policy, engine, metadata)
    kwargs = dict(factory_kwargs)
    kwargs.pop("camera_tensorrt_engine", None)
    kwargs.pop("camera_tensorrt_metadata", None)
    requested = bool(engine or metadata)
    report = dict(policy=selected_policy, requested=requested, active_backend="eager",
                  fallback=False, reason_code=None, reason=None,
                  startup_only=True, checkpoint_changed=False, config_changed=False)
    if selected_policy == "off":
        report["reason_code"] = "disabled_by_policy" if requested else "not_requested"
        return factory(**kwargs), report
    if not requested and selected_policy == "prefer":
        report["reason_code"] = "not_requested"
        return factory(**kwargs), report
    if not requested:
        raise AccelerationUnavailableError("engine_not_configured",
            "TensorRT require policy needs an explicit engine and metadata")
    kwargs.update(camera_tensorrt_engine=engine, camera_tensorrt_metadata=metadata)
    if selected_policy == "require":
        model = factory(**kwargs)
        report["active_backend"] = "camera_tensorrt"
        return model, report
    model, failure = _attempt(factory, kwargs)
    if failure is None:
        report["active_backend"] = "camera_tensorrt"
        return model, report
    _release_failed_startup()
    eager_kwargs = dict(factory_kwargs)
    eager_kwargs.pop("camera_tensorrt_engine", None)
    eager_kwargs.pop("camera_tensorrt_metadata", None)
    LOGGER.warning("Camera TensorRT unavailable at startup; using the SAME selected checkpoint/config eagerly. reason_code=%s; %s",
                   failure["reason_code"], failure["reason"])
    # This call is deliberately outside the except/_attempt scope. Essential
    # eager load/warmup failures propagate; never retry or change checkpoints.
    model = factory(**eager_kwargs)
    report.update(fallback=True, **failure)
    return model, report
