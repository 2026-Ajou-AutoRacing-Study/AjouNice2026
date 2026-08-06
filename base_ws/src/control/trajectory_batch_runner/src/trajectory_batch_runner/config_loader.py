# 목적: batch YAML을 읽고 저장소 기준 상대경로를 안전하게 해석한다.
# 역할: config 검증과 경로 해석을 한 곳에서 제공해 다른 모듈의 중복을 없앤다.
# 주요 흐름: 진입 YAML과 include 병합 -> 필수 키 검증 -> 저장소 루트 확인 -> 경로 확인.

"""trajectory_batch_runner 설정 파일 로더."""

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


class ConfigError(ValueError):
    """설정 파일의 값이나 경로가 올바르지 않을 때 발생한다."""


@dataclass(frozen=True)
class BatchConfig:
    """검증된 batch 설정과 경로 기준을 함께 보관한다."""

    path: Path
    repository_root: Path
    package_root: Path
    manifest_path: Path
    document: Dict[str, Any]

    def get(self, dotted_key: str) -> Any:
        """점으로 구분된 config 키의 값을 반환한다."""

        value: Any = self.document
        for key in dotted_key.split("."):
            if not isinstance(value, dict) or key not in value:
                raise ConfigError("config에 '{}' 값이 없습니다".format(dotted_key))
            value = value[key]
        return value

    def resolve_repo_path(
        self,
        value: str,
        label: str,
        *,
        must_exist: bool = True,
        expect_directory: Optional[bool] = None,
    ) -> Path:
        """저장소 상대경로를 저장소 밖으로 벗어나지 않게 해석한다."""

        if not isinstance(value, str) or not value.strip():
            raise ConfigError("{}은 비어 있지 않은 문자열이어야 합니다".format(label))
        relative = Path(value)
        if relative.is_absolute():
            raise ConfigError("{}은 저장소 상대경로여야 합니다: {}".format(label, value))
        resolved = (self.repository_root / relative).resolve()
        _require_inside(resolved, self.repository_root, label)
        if must_exist and not resolved.exists():
            raise ConfigError("{} 경로가 없습니다: {}".format(label, resolved))
        if must_exist and expect_directory is True and not resolved.is_dir():
            raise ConfigError("{}은 폴더여야 합니다: {}".format(label, resolved))
        if must_exist and expect_directory is False and not resolved.is_file():
            raise ConfigError("{}은 파일이어야 합니다: {}".format(label, resolved))
        return resolved

    def resolve_config_path(
        self,
        dotted_key: str,
        *,
        must_exist: bool = True,
        expect_directory: Optional[bool] = None,
    ) -> Path:
        """config의 경로 값을 저장소 절대경로로 변환한다."""

        return self.resolve_repo_path(
            self.get(dotted_key),
            dotted_key,
            must_exist=must_exist,
            expect_directory=expect_directory,
        )

    def resolve_child(
        self,
        directory_key: str,
        child_value: str,
        label: str,
        *,
        must_exist: bool,
    ) -> Path:
        """config 기준 폴더 아래의 manifest 파일명을 안전하게 결합한다."""

        base = self.resolve_config_path(
            directory_key, must_exist=True, expect_directory=True
        )
        child = Path(child_value)
        if child.is_absolute() or not child_value.strip():
            raise ConfigError("{}은 기준 폴더 상대경로여야 합니다".format(label))
        resolved = (base / child).resolve()
        _require_inside(resolved, base, label)
        if must_exist and not resolved.is_file():
            raise ConfigError("{} 파일이 없습니다: {}".format(label, resolved))
        return resolved


def _require_inside(path: Path, parent: Path, label: str) -> None:
    try:
        path.relative_to(parent)
    except ValueError as error:
        raise ConfigError(
            "{} 경로가 허용된 폴더 밖을 가리킵니다: {}".format(label, path)
        ) from error


def _require_mapping(document: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ConfigError("config의 '{}'는 mapping이어야 합니다".format(key))
    return value


def _load_yaml_mapping(path: Path, label: str) -> Dict[str, Any]:
    """YAML 파일 하나를 mapping으로 읽는다."""

    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError("{}을 읽을 수 없습니다: {}".format(label, path)) from error
    if not isinstance(document, dict):
        raise ConfigError("{} 최상위 값은 mapping이어야 합니다: {}".format(label, path))
    return document


def _merge_without_overwrite(
    destination: Dict[str, Any],
    source: Dict[str, Any],
    source_label: str,
    prefix: str = "",
) -> None:
    """설정 조각을 병합하되 같은 최종 키의 중복 정의는 거부한다."""

    for key, value in source.items():
        dotted_key = "{}.{}".format(prefix, key) if prefix else key
        if key not in destination:
            destination[key] = value
            continue
        current = destination[key]
        if isinstance(current, dict) and isinstance(value, dict):
            _merge_without_overwrite(current, value, source_label, dotted_key)
            continue
        raise ConfigError(
            "{}에서 config 키 '{}'를 중복 정의했습니다".format(
                source_label, dotted_key
            )
        )


def _load_composed_document(path: Path) -> Dict[str, Any]:
    """짧은 진입 config와 역할별 include 파일을 하나의 설정으로 합친다."""

    entrypoint = _load_yaml_mapping(path, "config")
    if entrypoint.get("config_version") != 4:
        raise ConfigError("지원하는 config_version은 4입니다")
    includes = entrypoint.get("includes")
    if not isinstance(includes, dict) or not includes:
        raise ConfigError("config의 'includes'는 비어 있지 않은 mapping이어야 합니다")

    config_directory = path.parent.resolve()
    document: Dict[str, Any] = {}
    for role, relative_value in includes.items():
        if not isinstance(role, str) or not role.strip():
            raise ConfigError("include 역할 이름은 비어 있지 않은 문자열이어야 합니다")
        if not isinstance(relative_value, str) or not relative_value.strip():
            raise ConfigError("include '{}' 경로가 비어 있습니다".format(role))
        relative_path = Path(relative_value)
        if relative_path.is_absolute():
            raise ConfigError("include '{}'는 config 상대경로여야 합니다".format(role))
        include_path = (config_directory / relative_path).resolve()
        _require_inside(include_path, config_directory, "include '{}'".format(role))
        if not include_path.is_file():
            raise ConfigError(
                "include '{}' 파일이 없습니다: {}".format(role, include_path)
            )
        fragment = _load_yaml_mapping(
            include_path, "include '{}'".format(role)
        )
        reserved = {"config_version", "includes", "path_resolution"} & set(fragment)
        if reserved:
            raise ConfigError(
                "include '{}'에는 진입 config 전용 키를 둘 수 없습니다: {}".format(
                    role, ", ".join(sorted(reserved))
                )
            )
        _merge_without_overwrite(document, fragment, "include '{}'".format(role))

    entry_values = {
        key: value
        for key, value in entrypoint.items()
        if key not in ("config_version", "includes")
    }
    _merge_without_overwrite(document, entry_values, "진입 config")
    document["config_version"] = 4
    return document


def _resolve_manifest(
    package_root: Path,
    repository_root: Path,
    configured_value: str,
    override: Optional[str],
) -> Path:
    value = override if override is not None else configured_value
    if not isinstance(value, str) or not value.strip():
        raise ConfigError("manifest 파일명이 비어 있습니다")
    candidate = Path(value)
    if candidate.is_absolute():
        resolved = candidate.resolve()
    elif len(candidate.parts) == 1:
        resolved = (package_root / "manifests" / candidate).resolve()
    else:
        resolved = (repository_root / candidate).resolve()
    _require_inside(resolved, repository_root, "manifest")
    if not resolved.is_file():
        raise ConfigError("manifest 파일이 없습니다: {}".format(resolved))
    return resolved


def _select_latest_lanelet_map(
    document: Dict[str, Any], repository_root: Path
) -> None:
    """버전 파일 목록에서 숫자가 가장 큰 Lanelet map을 선택한다."""

    try:
        lanelet = document["map"]["lanelet"]
        directory_value = lanelet["directory"]
        filename_pattern = lanelet["filename_pattern"]
        selection = lanelet["selection"]
    except (KeyError, TypeError) as error:
        raise ConfigError("map.lanelet 선택 설정이 부족합니다") from error
    if selection != "latest_numeric_version":
        raise ConfigError(
            "지원하는 map.lanelet.selection은 latest_numeric_version입니다"
        )
    if not isinstance(directory_value, str) or not directory_value.strip():
        raise ConfigError("map.lanelet.directory가 비어 있습니다")
    directory_relative = Path(directory_value)
    if directory_relative.is_absolute():
        raise ConfigError("map.lanelet.directory는 저장소 상대경로여야 합니다")
    directory = (repository_root / directory_relative).resolve()
    _require_inside(directory, repository_root, "map.lanelet.directory")
    if not directory.is_dir():
        raise ConfigError("Lanelet map 폴더가 없습니다: {}".format(directory))
    if (
        not isinstance(filename_pattern, str)
        or not filename_pattern
        or Path(filename_pattern).name != filename_pattern
    ):
        raise ConfigError("map.lanelet.filename_pattern은 단일 파일 glob이어야 합니다")

    versioned_maps = []
    for candidate in directory.glob(filename_pattern):
        match = re.fullmatch(r"lanelet2_map_v([0-9]+)\.osm", candidate.name)
        if match and candidate.is_file():
            versioned_maps.append((int(match.group(1)), candidate.resolve()))
    if not versioned_maps:
        raise ConfigError(
            "버전이 있는 Lanelet map을 찾지 못했습니다: {}/{}".format(
                directory, filename_pattern
            )
        )
    selected_version, selected_path = max(versioned_maps, key=lambda item: item[0])
    selected_relative = selected_path.relative_to(repository_root).as_posix()

    paths = document.setdefault("paths", {})
    files = paths.setdefault("files", {})
    if not isinstance(paths, dict) or not isinstance(files, dict):
        raise ConfigError("config의 paths.files는 mapping이어야 합니다")
    if "lanelet_map" in files:
        raise ConfigError(
            "paths.files.lanelet_map을 직접 지정하지 말고 map.lanelet 선택 규칙을 사용하세요"
        )
    files["lanelet_map"] = selected_relative
    lanelet["selected_version"] = selected_version
    lanelet["selected_file"] = selected_relative


def load_batch_config(
    config_path: str, manifest_override: Optional[str] = None
) -> BatchConfig:
    """YAML을 읽고 실행에 필요한 공통 구조와 경로를 검증한다."""

    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise ConfigError("config 파일이 없습니다: {}".format(path))
    document = _load_composed_document(path)

    for key in (
        "batch",
        "map",
        "path_resolution",
        "paths",
        "manifest",
        "scenario_generation",
        "morai",
        "persistent_stack",
        "iteration_stack",
        "monitoring",
    ):
        _require_mapping(document, key)

    relative_root = document["path_resolution"].get("repository_root_from_config")
    if not isinstance(relative_root, str) or not relative_root:
        raise ConfigError("repository_root_from_config가 필요합니다")
    repository_root = (path.parent / relative_root).resolve()
    if not (repository_root / "base_ws" / "src").is_dir():
        raise ConfigError(
            "저장소 루트를 찾을 수 없습니다: {}".format(repository_root)
        )

    _select_latest_lanelet_map(document, repository_root)

    package_root = path.parent.parent.resolve()
    _require_inside(package_root, repository_root, "trajectory_batch_runner package")
    configured_manifest = document["batch"].get("manifest_csv")
    manifest_path = _resolve_manifest(
        package_root,
        repository_root,
        configured_manifest,
        manifest_override,
    )
    config = BatchConfig(
        path=path,
        repository_root=repository_root,
        package_root=package_root,
        manifest_path=manifest_path,
        document=document,
    )

    for key in (
        "paths.directories.trajectory_directory",
        "paths.directories.scenario_directory",
        "paths.directories.validator_history_root",
    ):
        config.resolve_config_path(key, expect_directory=True)
    for key in (
        "paths.files.scenario_template_json",
        "paths.files.ego_network_json",
        "paths.files.sensor_json",
        "paths.files.morai_static_tf_yaml",
        "paths.files.lanelet_map",
        "paths.files.map_projector",
        "paths.files.localization_manager_config",
        "paths.files.vehicle_config",
        "paths.files.follower_common_config",
        "paths.files.follower_lateral_config",
        "paths.files.follower_longitudinal_config",
    ):
        config.resolve_config_path(key, expect_directory=False)
    return config
