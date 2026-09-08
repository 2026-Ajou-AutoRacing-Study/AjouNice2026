#include "autoware/path_smoother/elastic_band_smoother.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_path_smoother");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  autoware::path_smoother::ElasticBandSmoother smoother(node, private_node);
  ros::spin();
  return 0;
}
