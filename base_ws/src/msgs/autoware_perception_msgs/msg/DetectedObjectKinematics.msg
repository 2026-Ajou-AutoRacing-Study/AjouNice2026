# Only position is available; object-facing orientation is unknown.
uint8 UNAVAILABLE=0
# Orientation is known only up to a 180-degree sign flip.
uint8 SIGN_UNKNOWN=1
# The full front-facing orientation is available.
uint8 AVAILABLE=2

geometry_msgs/PoseWithCovariance pose_with_covariance

bool has_position_covariance
uint8 orientation_availability

geometry_msgs/TwistWithCovariance twist_with_covariance

bool has_twist
bool has_twist_covariance
