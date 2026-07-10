uint16 UNKNOWN=0
uint16 POINTCLOUD=1
uint16 OBJECT=2

uint16 type
unique_identifier_msgs/UUID object_id
autoware_perception_msgs/PredictedPath predicted_path
float32 ttc_begin
float32 ttc_end
geometry_msgs/Point[] points
bool is_safe
string detail
