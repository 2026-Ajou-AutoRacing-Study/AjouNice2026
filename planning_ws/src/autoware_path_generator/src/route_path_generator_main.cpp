#include <autoware/path_generator/route_path_generator_node.hpp>
int main(int argc, char ** argv) { ros::init(argc, argv, "path_generator"); ros::NodeHandle node("~"); autoware::path_generator::RoutePathGeneratorNode app(node); ros::spin(); }
