"""Keep pre-case failures in the execution folder, not only a vanished GUI log."""
import json
import traceback
from pathlib import Path
from .runtime import atomic_json, now


def mark_startup_phase(cfg, phase):
    cfg['_startup_phase'] = phase
    atomic_json(cfg['output'] / 'startup_status.json', {
        'schema_version': 1, 'status': 'RUNNING', 'phase': phase, 'updated_at': now()})


def complete_startup(cfg, exit_code):
    atomic_json(cfg['output'] / 'startup_status.json', {
        'schema_version': 1, 'status': 'COMPLETED', 'phase': 'CAMPAIGN_FINISHED',
        'exit_code': exit_code, 'updated_at': now()})
    state_path = cfg['output'] / 'campaign_state.json'
    if state_path.is_file():
        state = json.loads(state_path.read_text(encoding='utf-8'))
        # The previous failure file remains evidence; it is no longer the
        # current state after this invocation completes normally.
        state.pop('startup_error', None)
        state.pop('failure_stage', None)
        atomic_json(state_path, state)


def stream_console(process, on_line, log_path=None):
    """Tee runner stdout/stderr to local evidence without launching anything."""
    stream = None
    if log_path is not None:
        try:
            path = Path(log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            stream = path.open('a', encoding='utf-8')
        except OSError as error:
            on_line('실행 콘솔 파일 기록 실패: {}'.format(error))
    try:
        for line in process.stdout:
            if stream is not None:
                try:
                    stream.write(line)
                    stream.flush()
                except OSError as error:
                    try:
                        stream.close()
                    except OSError:
                        pass
                    stream = None
                    on_line('실행 콘솔 파일 기록 실패: {}'.format(error))
            on_line(line.rstrip('\n'))
    finally:
        if stream is not None:
            stream.close()


def record_startup_failure(cfg, error):
    report = {'schema_version': 1, 'status': 'ERROR',
              'phase': cfg.get('_startup_phase', 'UNKNOWN'), 'updated_at': now(),
              'error_type': type(error).__name__, 'error': str(error),
              'traceback': traceback.format_exc(),
              'conflicting_publishers': getattr(error, 'conflicting_publishers', {})}
    output = cfg['output']
    atomic_json(output / 'startup_error.json', report)
    atomic_json(output / 'startup_status.json', report)
    logs = output / 'stack_logs'
    logs.mkdir(parents=True, exist_ok=True)
    (logs / 'startup_error.log').write_text(
        '[{}] {}: {}\n{}'.format(report['phase'], report['error_type'],
                                report['error'], report['traceback']), encoding='utf-8')
    state_path = output / 'campaign_state.json'
    if state_path.is_file():
        state = json.loads(state_path.read_text(encoding='utf-8'))
        state.update(startup_error=report, failure_stage=report['phase'])
        atomic_json(state_path, state)
    return report
