# rosbag legacy experiments

이 디렉터리는 2026-06 FAST-LIO offline 검증 때 만든 일회성 스크립트를 보존한다.
기본 rosbag 보정 흐름에는 사용하지 않는다.

- `lidar_loop_back_test.py`, `valid_rosbag_check.py`, `valid_rosbag_maker.py`는
  `/home/ubuntu2004/AjouNice2026/...` 입력 경로를 코드에 고정하고 있다.
- 새 bag에는 `../../rosbag/repair/`와 `../../rosbag/inspect/`의 인자형 도구를 사용한다.
