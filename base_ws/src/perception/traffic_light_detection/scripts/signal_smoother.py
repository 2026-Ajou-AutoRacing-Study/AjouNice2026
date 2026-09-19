#!/usr/bin/env python3
"""
signal_smoother.py

신호등 색상/모양 인식 결과에 큐 기반 과반수 투표(majority voting)를 적용해서,
단발성 오검출/미검출(예: 초록-초록-초록-미스-초록-초록의 "미스" 한 프레임)이
그대로 판단팀에 전달되는 것을 막는다.
"""
from collections import deque, Counter


def _make_key(color_results):
    return frozenset((color, shape, is_left) for color, shape, is_left in color_results)


class SignalSmoother:
    def __init__(self, window_size=5, majority_count=None):
        if window_size < 1:
            raise ValueError("window_size는 1 이상이어야 합니다")
        self.window_size = window_size
        self.majority_count = majority_count if majority_count is not None else (window_size // 2) + 1

        self._queue = deque(maxlen=window_size)
        self._latest_by_key = {}

    def update(self, color_results, payload=None):
        if payload is None:
            payload = color_results
        key = _make_key(color_results)
        self._queue.append(key)
        self._latest_by_key[key] = payload

        if len(self._queue) < self.window_size:
            return payload

        counts = Counter(self._queue)
        winner_key, winner_count = counts.most_common(1)[0]

        if winner_count >= self.majority_count:
            return self._latest_by_key.get(winner_key)

        return None

    def reset(self):
        self._queue.clear()
        self._latest_by_key.clear()
