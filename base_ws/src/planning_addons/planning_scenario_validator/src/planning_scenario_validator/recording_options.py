"""Bag choices independent of screen video and live validation subscriptions."""
import json
import os
from pathlib import Path
from .topic_contract import PLANNER_TIME_PATTERNS, DOWNSTREAM_TIMING, MODEL_NAMESPACES

OFF = '끄기 (bag 없음)'
LIGHT = '경량 녹화'
BASIC = '기본 녹화'
ANALYSIS = '인지 분석 (센서+영상)'
CUSTOM = '직접 선택'
LATENCY = '지연 분석 (센서 제외)'
LOCALIZATION = '측위 분석 (센서 원본 제외)'
MODES = (OFF, LIGHT, BASIC, LATENCY, LOCALIZATION, ANALYSIS, CUSTOM)
PROFILE_ROOT = 'base_ws/src/planning_addons/planning_scenario_validator/config/internal/topic_profiles'
SENSORS = ['/sensor/lidar1'] + [
    '/sensor/camera/{}/compressed'.format(camera)
    for camera in ('front', 'front_left', 'front_right', 'back')]
RECORDING_PATHS = [('recording', key) for key in
                   ('bag_enabled', 'analysis_mode', 'topic_profile', 'include_sensors', 'selected_topics', 'profile_preset')]


def ensure_video_display(recording, environ=None):
    """Resolve a missing X11 address only for enabled screen recording.

    Preserve explicit user addresses and minimal video-OFF documents. New GUI
    saves/execution snapshots persist the resolved address for later resume.
    """
    video = recording.get('video', {})
    if not isinstance(video, dict):
        raise ValueError('recording.video는 YAML mapping이어야 합니다')
    if not video.get('enabled', False):
        return
    display = video.get('display')
    if display is None or (isinstance(display, str) and not display.strip()):
        environment = os.environ if environ is None else environ
        video['display'] = str(environment.get('DISPLAY') or '').strip() or ':0'


def mode_label(recording):
    if not recording.get('bag_enabled', True):
        return OFF
    if recording.get('analysis_mode', False):
        return ANALYSIS
    if 'selected_topics' in recording or recording.get('include_sensors', False):
        return CUSTOM
    if recording.get('profile_preset') == 'lightweight':
        return LIGHT
    if recording.get('profile_preset') == 'basic':
        return BASIC
    stem = Path(str(recording.get('topic_profile', ''))).stem
    if recording.get('profile_preset') == 'latency_analysis' or stem == 'latency_analysis':
        return LATENCY
    if recording.get('profile_preset') == 'localization_analysis' or stem == 'localization_analysis':
        return LOCALIZATION
    return LIGHT if stem == 'lightweight' else BASIC if stem in ('basic', 'topic_profile') else CUSTOM


def apply_mode(document, label):
    if label not in MODES:
        raise ValueError('rosbag 녹화 모드를 선택하세요')
    recording = document.setdefault('recording', {})
    recording['bag_enabled'] = label != OFF
    recording['analysis_mode'] = label == ANALYSIS
    if label in (LIGHT, BASIC, ANALYSIS, LATENCY, LOCALIZATION):
        name = {LIGHT: 'lightweight', BASIC: 'basic', ANALYSIS: 'perception_analysis',
                LATENCY: 'latency_analysis', LOCALIZATION: 'localization_analysis'}[label]
        recording['topic_profile'] = PROFILE_ROOT + '/' + name + '.json'
        recording['profile_preset'] = name
        recording.pop('selected_topics', None)
        recording['include_sensors'] = label == ANALYSIS
    if label == ANALYSIS:
        # Preserve the existing raw-analysis contract. Other modes never change video.
        recording.setdefault('video', {})['enabled'] = True
        ensure_video_display(recording)


def checked_topics(topics):
    if not isinstance(topics, list) or not topics or any(
            not isinstance(t, str) or not t.startswith('/') or any(c.isspace() for c in t)
            or any(c in t for c in '*?[]()|^$\\') for t in topics):
        raise ValueError('녹화 토픽은 비어 있지 않은 절대 토픽명 목록이어야 합니다 (정규식 불가)')
    return list(dict.fromkeys(topics))


def topic_patterns(recording):
    if diagnostics_enabled(recording):
        return list(PLANNER_TIME_PATTERNS)
    return []


def diagnostics_enabled(recording):
    return recording.get('bag_enabled', True) and mode_label(recording) in (ANALYSIS, LATENCY)


def localization_analysis_enabled(recording):
    """Explicit localization evidence profile, without perception timing overhead."""
    return recording.get('bag_enabled', True) and mode_label(recording) == LOCALIZATION


def sharing_enabled(recording):
    """Analysis modes that need a self-contained manual or automatic share bundle."""
    return diagnostics_enabled(recording) or localization_analysis_enabled(recording)


def sharing_warnings(recording, upload):
    if sharing_enabled(recording) and upload.get('profile', 'basic') not in ('bag', 'full'):
        analysis = '측위 입력·드리프트' if localization_analysis_enabled(recording) else '프레임·지연'
        return ['분석 녹화지만 공유에서 bag 제외: 공유 자료만으로 {} 분석 불가. bag/원본 공유를 선택하세요.'.format(analysis)]
    return []


def effective_topics(recording, resolve_path):
    if type(recording.get('bag_enabled', True)) is not bool:
        raise ValueError('recording.bag_enabled는 true/false여야 합니다')
    if not recording.get('bag_enabled', True):
        return []
    if 'selected_topics' in recording:
        topics = checked_topics(recording['selected_topics'])
    else:
        profile = resolve_path(recording.get('topic_profile'))
        try:
            topics = checked_topics(json.loads(profile.read_text(encoding='utf-8')))
        except (OSError, ValueError) as error:
            raise ValueError('토픽 프로파일을 읽을 수 없습니다: {} ({})'.format(profile, error)) from error
    if recording.get('include_sensors', False):
        topics += SENSORS
    if diagnostics_enabled(recording):
        topics += DOWNSTREAM_TIMING + [namespace + '/frame_timing' for namespace in set(MODEL_NAMESPACES.values())]
    return list(dict.fromkeys(topics))


def topic_catalog(root, current=()):
    topics = set(current) | set(SENSORS)
    for name in ('lightweight', 'basic', 'perception_analysis', 'latency_analysis', 'localization_analysis', 'full'):
        path = Path(root) / PROFILE_ROOT / (name + '.json')
        if path.is_file():
            topics.update(checked_topics(json.loads(path.read_text(encoding='utf-8'))))
    return sorted(topics)


def topic_group(topic):
    if topic.startswith('/sensor/camera/') or topic.startswith('/sensor/lidar'):
        return '원본 센서 (고용량)'
    if topic.startswith('/perception/') or topic.startswith('/evaluation/'):
        return '인지·시각화'
    if topic.startswith('/planning/') or topic.startswith('/purepursuit') or topic.startswith('/control/') or topic == '/ctrl_cmd':
        return '판단·제어'
    if topic.startswith('/localization/') or topic in ('/tf', '/tf_static'):
        return '측위·TF'
    return '차량·기타'


def choose_topics(parent, topics, selected):
    """Local catalog only: no ROS subscription or master lookup in the GUI."""
    import tkinter as tk
    from tkinter import ttk, messagebox
    window = tk.Toplevel(parent)
    window.title('rosbag 토픽 선택')
    window.geometry('860x570')
    chosen = set(selected)
    result = [None]
    search = tk.StringVar()
    group = tk.StringVar(value='전체')
    count = tk.StringVar()
    groups = ['전체'] + list(dict.fromkeys(topic_group(t) for t in topics))
    ttk.Label(window, text='검색·그룹으로 좁힌 뒤 한 번에 선택하세요. 클릭 또는 Space로 체크. 영상 녹화는 별도입니다.').pack(anchor='w', padx=12, pady=8)
    bar = ttk.Frame(window)
    bar.pack(fill='x', padx=12)
    ttk.Label(bar, text='검색').pack(side='left')
    ttk.Entry(bar, textvariable=search).pack(side='left', fill='x', expand=True, padx=6)
    ttk.Combobox(bar, textvariable=group, values=groups, state='readonly', width=22).pack(side='left')
    body = ttk.Frame(window)
    body.pack(fill='both', expand=True, padx=12, pady=8)
    tree = ttk.Treeview(body, columns=('check', 'topic', 'group'), show='headings', selectmode='extended')
    for key, label, width in [('check', '선택', 50), ('topic', '토픽', 540), ('group', '그룹', 180)]:
        tree.heading(key, text=label)
        tree.column(key, width=width, stretch=key == 'topic')
    scroll = ttk.Scrollbar(body, command=tree.yview)
    tree.configure(yscrollcommand=scroll.set)
    tree.pack(side='left', fill='both', expand=True)
    scroll.pack(side='right', fill='y')
    visible = []

    def refresh(*_):
        visible[:] = [t for t in topics if search.get().casefold() in t.casefold()
                      and (group.get() == '전체' or group.get() == topic_group(t))]
        tree.delete(*tree.get_children())
        for index, topic in enumerate(visible):
            tree.insert('', 'end', iid=str(index), values=('✓' if topic in chosen else '', topic, topic_group(topic)))
        count.set('선택 {}개 / 표시 {}개 · 원본 센서 {}/5개'.format(len(chosen), len(visible), len(chosen & set(SENSORS))))

    def toggle(event):
        rows = tree.selection() if event.type == tk.EventType.KeyPress else (tree.identify_row(event.y),)
        for row in rows:
            if row:
                topic = visible[int(row)]
                chosen.symmetric_difference_update({topic})
                tree.set(row, 'check', '✓' if topic in chosen else '')
        count.set('선택 {}개 / 표시 {}개 · 원본 센서 {}/5개'.format(len(chosen), len(visible), len(chosen & set(SENSORS))))
        return 'break' if event.type == tk.EventType.KeyPress else None

    def bulk(enable):
        chosen.update(visible) if enable else chosen.difference_update(visible)
        refresh()

    def apply():
        if not chosen:
            messagebox.showwarning('빈 선택', '토픽을 선택하거나, 메인 화면에서 bag 끄기를 선택하세요.', parent=window)
            return
        result[0] = sorted(chosen)
        window.destroy()

    tree.bind('<ButtonRelease-1>', toggle)
    tree.bind('<space>', toggle)
    search.trace_add('write', refresh)
    group.trace_add('write', refresh)
    buttons = ttk.Frame(window)
    buttons.pack(fill='x', padx=12, pady=4)
    ttk.Button(buttons, text='표시된 토픽 모두 선택', command=lambda: bulk(True)).pack(side='left', padx=3)
    ttk.Button(buttons, text='표시된 토픽 모두 해제', command=lambda: bulk(False)).pack(side='left', padx=3)
    ttk.Button(buttons, text='적용', command=apply).pack(side='right', padx=3)
    ttk.Button(buttons, text='취소', command=window.destroy).pack(side='right', padx=3)
    ttk.Label(window, textvariable=count).pack(anchor='w', padx=12, pady=4)
    ttk.Label(window, text='카메라 4개 + LiDAR가 있어야 사후 재추론 가능. 목록은 후보이며 실제 발행 여부는 별도 확인. YAML 저장 후 다음 실행에 적용.', foreground='#856333', wraplength=820).pack(anchor='w', padx=12, pady=6)
    refresh()
    window.transient(parent)
    window.grab_set()
    parent.wait_window(window)
    return result[0]
