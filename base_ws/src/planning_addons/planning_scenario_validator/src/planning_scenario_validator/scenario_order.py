"""Freeze execution order while preserving the filenames known to MORAI."""


def ordered_sources(sources, order=None):
    if order is None:
        return sources
    if not isinstance(order, list) or any(not isinstance(name, str) for name in order):
        raise ValueError('scenarios.order는 선택한 JSON 파일명 목록이어야 합니다')
    if len(order) != len(set(order)):
        raise ValueError('scenarios.order에 중복 파일명이 있습니다')
    by_name = {source.name: source for source in sources}
    if set(order) != set(by_name):
        raise ValueError('scenarios.order는 선택한 모든 JSON을 정확히 한 번씩 포함해야 합니다')
    return [by_name[name] for name in order]
