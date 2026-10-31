#!/usr/bin/env python3
"""Read-only Sunny13 5K progress display; never starts ROS or collection."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

try:
    import live_v2 as live
except ModuleNotFoundError as error:
    if error.name != 'live_v2':
        raise
    import live


STAGES = {'running': '수집 중', 'paused': '일시정지', 'halted': '안전 중단',
          'complete': '완료', 'complete_with_deficit': '부족분이 있는 완료',
          'prepared_waiting_for_resume': '시작 대기', 'stopped': '중단'}


def record(path):
    value = live.read(live.owned(path, False))
    live.require(isinstance(value, dict), '상태 파일이 객체 형식이 아니다: ' + str(path))
    return value


def number(value, name, integer=False):
    live.require(type(value) is int if integer else type(value) in (int, float),
                 '잘못된 상태 값: ' + name)
    live.require(math.isfinite(value) and value >= 0, '잘못된 상태 값: ' + name)
    return value


def snapshot(ctx, now=None):
    """Only the approved authoritative full-batch output is observed."""
    now = time.time() if now is None else now
    root = live.no_symlinks(ctx['workspace'] / 'collections/sunny13_5k')
    base = dict(stage='waiting', accepted=0, review=0, target=5000, elapsed_sec=0,
                eta_sec=None, current=None, updated_age_sec=None, checkpoint_only=True,
                output=str(root), files_written=False, capture_started=False)
    if not root.exists():
        return base
    live.owned(root)
    status_path = live.no_symlinks(root / 'status.json')
    if not status_path.exists():
        return base
    state = record(status_path)
    live.require(state.get('requested') == 5000, '승인한 5,000장 본수집 상태가 아니다')
    accepted = number(state.get('accepted_candidate'), '승인 수', True)
    review = number(state.get('review'), '검토 수', True)
    live.require(accepted + review <= 5000, '상태의 장면 수가 5,000장을 넘는다')
    elapsed = number(state.get('elapsed_sec'), '경과 시간')
    eta = number(state['eta_sec'], '예상 잔여 시간') if state.get('eta_sec') is not None else None
    updated = number(state.get('updated_at'), '갱신 시간')
    live.require(isinstance(state.get('stage'), str), '상태 단계가 없다')
    quota_path = live.no_symlinks(root / 'quota.json')
    if quota_path.exists():
        quota = record(quota_path)
        live.require(quota.get('target') == 5000, '다른 수집의 할당량 파일이다')
        values = [number(quota.get(key), '할당량 ' + key, True)
                  for key in ('accepted', 'deficit', 'pending')]
        live.require(sum(values) == 5000, '할당량 합계가 5,000장과 다르다')
    base.update(stage=state['stage'], accepted=accepted, review=review,
                elapsed_sec=elapsed, eta_sec=eta, current=state.get('current'),
                updated_age_sec=max(0, now - updated),
                checkpoint_only=state.get('checkpoint_only') is True)
    return base


def duration(seconds):
    if seconds is None:
        return '아직 계산되지 않음'
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return (str(hours) + '시간 ' if hours else '') + str(minutes) + '분 ' + str(seconds) + '초'


def render(view):
    if view['stage'] == 'waiting':
        return '준비 대기 · 승인 0 / 5,000장 · 본수집을 아직 시작하지 않았거나 첫 상태 기록을 기다리는 중'
    stage = STAGES.get(view['stage'], '기록 단계: ' + view['stage'])
    eta = duration(view['eta_sec']) if view['stage'] == 'running' else '진행하지 않음'
    lines = [stage + ' · 승인 ' + format(view['accepted'], ',') + ' / 5,000장'
             + ' · 검토·부족 ' + format(view['review'], ',') + '장',
             '활성 누적 경과 ' + duration(view['elapsed_sec']) + ' · 예상 잔여 ' + eta,
             '현재 작업: ' + (str(view['current']) if view['current'] is not None else '없음'),
             '마지막 상태 갱신: ' + str(int(view['updated_age_sec'])) + '초 전'
             + ' · 부모 체크포인트 기준' * view['checkpoint_only']]
    if view['updated_age_sec'] > 30:
        lines.append('주의: 상태 기록이 오래됨. 현재 프로세스의 정상 동작을 보증하지 않는다.')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / live.WORKSPACE)
    parser.add_argument('--watch', action='store_true', help='5초마다 읽기 전용으로 관측')
    args = parser.parse_args(argv)
    ctx = live.load(args.workspace, verify=False)
    live.ready(ctx)
    errors = 0
    while True:
        try:
            print(render(snapshot(ctx)), flush=True)
        except (OSError, ValueError, TypeError, KeyError) as error:
            errors += 1
            print('읽기 오류 ' + str(errors) + '회 · 현재 상태 확인 불가. '
                  + '이전 값을 최신 정보로 표시하지 않음: ' + str(error), flush=True)
            if not args.watch:
                return 1
        if not args.watch:
            return 0
        print('—', flush=True)
        time.sleep(5)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('상태 관측만 종료했다. 수집기·ROS·브리지는 중단하지 않았다.')
        raise SystemExit(0)
    except (OSError, ValueError) as error:
        print('상태 관측을 시작하지 못함: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
