# 추적(또는 dummy 배정)된 단일 객체
uint32 track_id
string class_label          # "vehicle" / "pedestrian" / "obstacle"

geometry_msgs/Point position        # x, y, z
float64 heading              # radian, 표준 ROS yaw 컨벤션 (반시계 +)
geometry_msgs/Vector3 velocity      # m/s, world/ego 기준 (입력 프레임 그대로 통과)
geometry_msgs/Vector3 acceleration  # m/s^2
geometry_msgs/Vector3 size          # length(x), width(y), height(z)
