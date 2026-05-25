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
- `morai_ros_bridge/` -> 이거 나중에 어디에 위치시킬지나 어디서 launch할지 등등 고민하기

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


## 초기 패키지들
- msgs
    - 우리 custom msg 정의
- bringup
    - launch 파일 모음. 여러 노드를 한 번에 켜는 진입점
- sensor_interface
    - MORAI 센서 토픽을 우리 표준 토픽 이름으로 변환/relay
- morai_ros_bridge
    - MORAI와 연결되는 launch/config 정리