cmake_minimum_required(VERSION 3.0.2)
project(trajectory_batch_runner)

find_package(catkin REQUIRED)

catkin_package()

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 실행 로직을 구현하기 전까지 Python 문서 골격은 실행 파일로 설치하지 않는다.
install(FILES scripts/run_trajectory_batch.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
