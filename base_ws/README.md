# base_ws
- base workspace

## Notes
- 2025년도의 naming을 따랐음.
- sub workspace로 구성하기보단 하나의 mono workspace로 일단 구성 -> 추후 중첩 형태로 변경??
- 현재는 틀만 만들어 놓음. 나중에 직접 생성한 workspace로 대체하기

## Packages list(예시)
- `bringup/`
- `msgs/`
- `sensor_interface/`
- `localization/`
- `perception/`
- `tracking/`
- `prediction/`
- `planning/`
- `control/`
- `diagnostics/`
- `third_party/`

## Package layout(예시)
```text
base_ws/src/<package_name>/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── <module_name>.launch
├── config/
│   └── <module_name>.yaml
├── scripts/
│   └── <node_name>.py
├── src/
│   └── <package_name>/
│       ├── __init__.py
│       ├── core.py
│       ├── msg_converter.py
│       └── utils.py
└── test/
    └── test_<module_name>.py
```

```text
# perception package 예시
perception/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── perception.launch
├── config/
│   └── perception.yaml
├── scripts/
│   └── bevfusion_node.py
├── src/
│   └── ajn_perception/
│       ├── __init__.py
│       ├── bevfusion_wrapper.py
│       ├── msg_converter.py
│       └── utils.py
└── test/
```