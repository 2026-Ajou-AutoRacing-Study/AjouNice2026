"""Readable YAMLs shared by mission examples, workspace copies and quick tests."""
import copy
import re
from pathlib import Path

import yaml


COMMENTS = {
    "campaign_name": "결과 폴더의 이름 앞부분. 영문/숫자/_/- 사용. 날짜는 실행할 때 자동으로 붙습니다.",
    "scenarios": "실행할 JSON 묶음. 시작 위치·객체 배치는 JSON에, 공통 목적지는 아래 고정 설정에 있습니다.",
    "scenarios.directory": "저장소 루트 기준 상대경로 또는 ~/ 경로. 보통 같은 미션 workspace/scenarios 사용.",
    "scenarios.glob": "include가 없으면 이 패턴의 JSON을 모두 실행합니다. '*.json' = 전체.",
    "scenarios.include": "이 목록에 적은 파일만 실행합니다. 전체 실행하려면 include 항목을 삭제하세요.",
    "scenarios.repeat": "JSON 한 개당 실행 횟수. 2개 JSON × repeat: 10 = 총 20회. 중단 후 남은 회차부터 재개.",
    "scenarios.randomization": "고주로 NPC 파라미터 범위. 객체 배치·경로·차종은 바꾸지 않습니다.",
    "scenarios.randomization.enabled": "true: 아래 범위에서 회차마다 추출. false: JSON 값 그대로 반복.",
    "scenarios.randomization.seed": "같은 seed/JSON/회차는 같은 파라미터. 다른 추출 조합을 보려면 seed만 바꾸세요.",
    "scenarios.randomization.vehicle_speed_kmh": "NPC 목표 속도 [최소, 최대], km/h. Ego 속도 설정이 아닙니다.",
    "scenarios.randomization.path_offset_percent": "NPC 차로 내 path offset [최소, 최대], %. 처음엔 좁게 유지하세요.",
    "recording": "권장값: basic 토픽 + 센서 원본 제외 + 화면 영상. 센서는 MORAI에서 계속 켜둡니다.",
    "recording.include_sensors": "false 권장. true는 bag에 카메라/LiDAR를 추가하므로 용량이 크게 늘어납니다.",
    "recording.topic_profile": "basic.json 유지. 판단·제어·인지 결과 위주로 녹화합니다.",
    "recording.video.enabled": "화면 영상 녹화 여부. true일 때 다른 창으로 가리거나 화면을 잠그지 마세요.",
    "recording.video.window_layout": "split: RViz 왼쪽, MORAI 오른쪽, GUI 왼쪽 아래. manual: 수동 배치.",
    "recording.video.bottom_bar_px": "auto 유지. 모니터 바가 잘릴 때만 '18%' 또는 180처럼 높이를 지정.",
    "recording.minimum_free_disk_gib": "이보다 여유 공간이 적으면 다음 회차를 시작하지 않고 멈춥니다.",
    "upload.enabled": "Drive 자동 업로드. 처음에는 false. 업로드를 쓸 때만 rclone 본인 인증 필요.",
    "upload.profile": "basic=CSV/JSON, bag=기본+bag, video=기본+영상, full=모두. 로컬 원본은 보존.",
    "upload.rclone_remote": "공유 AjouNice2026 폴더 아래 planning_validator. 본인 rclone remote 확인.",
    "stack": "측위·탐지·신호등 on/off는 GUI에서도 수정·저장 가능. Python 실행 환경은 YAML에서만 수정.",
    "default_case": "아래 목적지/시간은 이 미션의 모든 JSON에 공통. 시작점이 다른 미션을 섞지 마세요.",
    "default_case.goal": "planning map 좌표(m), yaw_deg는 도 단위. MORAI 화면 좌표와 다릅니다.",
    "default_case.timeout_sec": "제어기 준비 후 주행 제한(초). 로드/스택 준비 대기와 별개. 늦게 도착하면 FAIL.",
    "default_case.arrival_mode": "stop: 목적지에서 정지까지 확인. pass_through: 목적지 통과 확인.",
    "stack.pose_source": "localization: 실제 측위, maptf_morai: Ego GT 위치 대조. 빠른 대조는 planning 탭.",
    "stack.detector": "bevfusion: 실제 인지, od_gt: GT 객체 대조. 빠른 대조는 planning 탭.",
    'stack.bevfusion_model': 'base5_e12=production(기본, TRT+SDPA/cache) 0.40/0.10/0.10, base5_e24/e25=명시적 복귀, base5_e9=val best 비교 0.30/0.20/0.12.',
    'recording.analysis_mode': 'true=인지 분석. 센서 원본+영상 필수, 마감 후 실제 bag 메시지 수 검사. 기본 basic 모드는 유지.',
    'recording.bag_enabled': 'false=rosbag 프로세스 자체를 실행하지 않음. 실시간 판정·JSON·CSV와 별도 영상 설정은 유지. 생략 시 기존처럼 true.',
    'recording.selected_topics': '직접 선택한 절대 토픽 목록. topic_profile 대신 사용. include_sensors가 true면 센서 5개를 추가.',
    "stack.traffic_light": "이 미션에서 신호등 판단/인지 사용 여부. true는 Python 환경 설치가 필요합니다.",
    "stack.traffic_light_runtime": "system=기존 /usr/bin/python3, virtualenv=아래 Python. 팀원이 설치한 환경에 맞춥니다.",
    "stack.traffic_light_python": "가상환경 bin/python 경로. conda activate는 필요 없습니다.",
    "stack.traffic_light_device": "GPU 한 개면 '0'. CPU 시험은 cpu. 신호등 off면 사용하지 않습니다.",
    "network": "MORAI GUI에 실제 Load/Connect한 Sensor/Ego Network와 같아야 합니다.",
    "morai.scenario_install_dir": "auto: 각 PC의 ~/MoraiLauncher_Lin을 찾습니다. 설치 위치가 다를 때만 수정.",
    "validation": "충돌은 1회라도 FAIL. 목적지 도착 반경/정지 기준은 기본값 유지.",
    "output_root": "저장소 기준 결과 경로. 실행마다 별도 timestamp 폴더, 과거 기록은 덮어쓰지 않습니다.",
    "resume": "true 유지. 이어가기는 GUI 기록 탭 또는 CLI --execution 결과폴더.",
    "continue_on_system_error": "false 유지. 주행 FAIL은 계속, 수신/스택/녹화 같은 시스템 오류는 중단.",
    "readiness_timeout_sec": "ROS 입력·스택 준비 대기의 상한. 실제 주행 제한시간과 별개.",
    "scenario_ready_timeout_sec": "ScenarioLoad 후 Ego 시작 위치 확인의 상한. 정상 연결이면 일찍 진행.",
    "route_ready_timeout_sec": "목적지 경로·첫 판단 궤적 준비의 상한. 준비 실패는 별도 결과로 기록.",
}


class _ReadableDumper(yaml.SafeDumper):
    pass


def _sequence(dumper, values):
    pair = len(values) == 2 and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                                  for value in values)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', values, flow_style=pair)


_ReadableDumper.add_representer(list, _sequence)


def annotated_yaml(document):
    """Give new examples/copies explanations; existing saves patch values only."""
    order = ("campaign_name", "scenarios", "stack", "recording", "upload", "schema_version",
             "default_case", "network", "morai", "validation", "output_root",
             "resume", "continue_on_system_error", "readiness_timeout_sec",
             "scenario_ready_timeout_sec", "route_ready_timeout_sec")
    ordered = {key: document[key] for key in order if key in document}
    ordered.update({key: value for key, value in document.items() if key not in ordered})
    lines = ["# 일상 옵션은 GUI에서 수정 → YAML 저장. 고급 설정은 VSCode 저장 → GUI 불러오기.",
             "# 공유 base는 수정하지 말고 '베이스 → 내 작업 폴더 복사'로 시작하세요.",
             "# === 자주 바꾸는 부분: 이름 / JSON 선택 / 반복 / 측위·탐지 / 녹화·공유 여부 ==="]
    trail = []
    for line in yaml.dump(ordered, Dumper=_ReadableDumper, allow_unicode=True, sort_keys=False).splitlines():
        match = re.match(r"^( *)([A-Za-z_][A-Za-z_0-9]*):", line)
        if match:
            indent, key = len(match[1]), match[2]
            while trail and trail[-1][0] >= indent:
                trail.pop()
            path = ".".join([part[1] for part in trail] + [key])
            trail.append((indent, key))
            if indent == 0:
                lines.append("")
                if key == "schema_version":
                    lines += ["# === 여기부터 수정 X: 미션 고정 설정 (필요할 때만 담당자와 확인) ===",
                              "# Ego 시작점은 시나리오 JSON에 있습니다. 여기서 시작점/목적지를 임의 변경하지 마세요."]
            if path in COMMENTS:
                lines.append(" " * indent + "# " + COMMENTS[path])
        lines.append(line)
    return "\n".join(lines) + "\n"


def pasted_yaml_path(value):
    """Accept VSCode Copy Path, quoted paths, relative paths and ~/ paths."""
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        value = value[1:-1].strip()
    if not value or "\n" in value or "\r" in value:
        raise ValueError("YAML 파일 경로 한 개를 붙여넣으세요")
    if Path(value).suffix.lower() not in (".yaml", ".yml"):
        raise ValueError(".yaml 또는 .yml 파일 경로를 넣으세요")
    return value


def updated_yaml(text, document):
    """Patch changed YAML values, leaving untouched fields and comments in place.

    This is deliberately not a general YAML editor. Ordinary block/flow mappings
    work; aliases are refused when editing so one control cannot modify another.
    """
    original = yaml.safe_load(text)
    if original == document:
        return text
    if any(isinstance(token, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken))
           for token in yaml.scan(text)):
        raise ValueError('앵커/별칭이 있는 YAML은 VSCode에서 수정하세요. 원본은 변경하지 않았습니다.')
    edits = []

    def flow(value):
        rendered = yaml.safe_dump(value, allow_unicode=True, default_flow_style=True, width=100000)
        return (rendered[:-4] if rendered.endswith('...\n') else rendered).rstrip('\n')

    def last_value(node):
        if isinstance(node, yaml.MappingNode) and node.value:
            return last_value(node.value[-1][1])
        if isinstance(node, yaml.SequenceNode) and node.value and not node.flow_style:
            return last_value(node.value[-1])
        return node

    def line_end(node):
        index = last_value(node).end_mark.index
        newline = text.find('\n', index)
        return len(text) if newline < 0 else newline + 1

    def patch_node(node, before, after, path):
        if before == after:
            return
        if not (isinstance(node, yaml.MappingNode) and isinstance(after, dict)):
            end = last_value(node).end_mark.index
            if isinstance(node, yaml.MappingNode) or (isinstance(node, yaml.SequenceNode) and node.flow_style):
                end = node.end_mark.index
            edits.append((node.start_mark.index, end, flow(after)))
            return
        if node.flow_style and set(before) != set(after):
            edits.append((node.start_mark.index, node.end_mark.index, flow(after)))
            return
        nodes = {key.value: (key, value) for key, value in node.value}
        for key, (key_node, value_node) in nodes.items():
            if key not in after:
                start = text.rfind('\n', 0, key_node.start_mark.index) + 1
                edits.append((start, line_end(value_node), ''))
            else:
                patch_node(value_node, before[key], after[key], path + (key,))
        additions = {key: value for key, value in after.items() if key not in before}
        if additions:
            if not node.value:
                edits.append((node.start_mark.index, node.end_mark.index, flow(after)))
                return
            index = line_end(node.value[-1][1])
            block = yaml.safe_dump(additions, allow_unicode=True, sort_keys=False)
            indent = ' ' * node.start_mark.column
            insertion = ('' if index and text[index - 1] == '\n' else '\n')
            insertion += ''.join(indent + line + '\n' for line in block.splitlines())
            edits.append((index, index, insertion))

    patch_node(yaml.compose(text), original, document, ())
    for start, end, replacement in sorted(edits, reverse=True):
        text = text[:start] + replacement + text[end:]
    if yaml.safe_load(text) != document:
        raise ValueError('이 YAML 서식은 GUI 저장을 지원하지 않습니다. VSCode에서 수정하세요. 원본은 변경하지 않았습니다.')
    return text


def quick_test_document(template, names, *, detector, pose_source, traffic_light):
    if not 1 <= len(names) <= 2 or len(set(names)) != len(names):
        raise ValueError("planning 빠른 테스트는 JSON 1~2개를 선택하세요")
    if detector not in ("bevfusion", "od_gt") or pose_source not in ("localization", "maptf_morai"):
        raise ValueError("지원하지 않는 planning 대조 스택입니다")
    result = copy.deepcopy(template)
    result["campaign_name"] = "planning_quick"
    result["scenarios"]["include"] = sorted(names)
    result["scenarios"]["repeat"] = 1
    # Quick tests should not silently randomize the scenario under diagnosis.
    if "randomization" in result["scenarios"]:
        result["scenarios"]["randomization"]["enabled"] = False
    result["stack"].update(detector=detector, pose_source=pose_source,
                           traffic_light=bool(traffic_light))
    result.setdefault("upload", {})["enabled"] = False
    return result
