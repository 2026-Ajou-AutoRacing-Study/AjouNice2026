# Bringup RViz rosbag recorder

PlanViz (`aw149_planning_rviz.launch`)의 Rosbag START는 `rosbag record --all`로
전체 토픽을 기록한다. 녹화 도중 새로 생긴 토픽도 자동으로 구독한다.
저장 위치는 `rosbag_output_directory`이며 기본값은 `planner_logs/bags`다.
LZ4 압축과 1024 MiB 녹화 버퍼를 사용한다. STOP으로 bag 색인을 마감한다.

영상 payload는 제외한다. START 시 ROS master에서 `sensor_msgs/Image`,
`sensor_msgs/CompressedImage`, `theora_image_transport/Packet` 타입의 토픽을
찾아 제외하므로 센서 영상과 BEVFusion 투영 영상도 빠진다. 녹화 도중
추가되는 표준 image_transport 이름(`image`, `image_raw`, `image_rect`,
`image_rect_color`, `image_color`, `image_mono`, `compressed`, `compressedDepth`,
`theora`) 및 그 하위 토픽도 제외한다. 비표준 이름의 영상 publisher를
나중에 추가한다면 녹화를 다시 시작해 타입 목록을 갱신해야 한다.
master 조회 실패 시에는 녹화를 시작하지 않는다.

LiDAR 점군(`sensor_msgs/PointCloud`, `sensor_msgs/PointCloud2`)과 RViz 표시용
`visualization_msgs/*`, `rviz_2d_overlay_msgs/*`, `jsk_rviz_plugins/*`도
START 시 타입으로 제외한다. `/sensor/lidar1` 점군은 제외하고,
`/sensor/lidar1/packets` (`velodyne_msgs/VelodyneScan`)는 보존한다.
시각화용 `/app/perc/vis_*`, `/app/perc/jsk_all_track`,
`/planning/debug/final_trajectory/path`도 제외한다.
녹화 중 추가되는 `points`, `points_raw`, `points_filtered`, `pointcloud`,
`pointcloud_raw` 이름의 토픽과 `visualization`, `markers`, `marker`,
`virtual_wall`, `virtual_walls` 경로도 제외한다. 다른 이름의 점군·시각화
publisher를 나중에 추가한다면 START 시 타입 조회를 위해 녹화를 다시 시작한다.
`nav_msgs/Path`와 Autoware 경로·궤적·인지 메시지는 분석 데이터로 유지한다.

`CameraInfo`, 카메라 인식 결과, LiDAR 원본 패킷,
GT·충돌·localization·planning·control·진단 토픽은 포함한다.
기존 `~rosbag_topics` 고정 목록은 사용하지 않는다.

영상·점군·시각화 제외로 용량은 줄지만 LiDAR 원본 패킷과 고주기 디버그 토픽의
네트워크·CPU·디스크 부하는 남는다. 실제 주행 중 저장률과 메시지 유실을
확인해야 하며, 정차 상태의 짧은 녹화만으로 주행 성능을 보증하지 않는다.
C++ 플러그인을 재빌드·설치한 뒤 PlanViz를 새로 실행하면 적용된다.

`show_morai_gt:=false` 실행에서는 녹화기의 `record_morai_gt`도 false가 되어
`/Ego_topic`, `/Object_topic`을 제외한다. `/Competition_topic`은 계속 기록한다.
