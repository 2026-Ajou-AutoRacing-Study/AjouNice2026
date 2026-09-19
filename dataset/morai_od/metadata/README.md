# Metadata

각 생성 실행의 seed, route anchor, 날씨, 시간, 입력 템플릿과 MGeo 정보를
기록하는 로컬 산출물 위치다. 실제 센서 데이터는 포함하지 않는다.

`prototypes/`와 `ros_capture_poc/`의 최소 fixture만 Git으로 추적한다. Production,
pilot과 trial의 per-scene metadata·route sidecar는 generator와 config로 재현되므로
Git으로 추적하지 않는다.
