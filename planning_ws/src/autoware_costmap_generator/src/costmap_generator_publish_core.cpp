#include <autoware/costmap_generator/costmap_core.hpp>
#include <autoware/costmap_generator/utils/points_to_costmap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
int main(int argc,char ** argv){ros::init(argc,argv,"costmap_generator");ros::NodeHandle p("~");double lx=20,ly=20,res=1;bool got_points=false;p.param("grid_length_x",lx,lx);p.param("grid_length_y",ly,ly);p.param("grid_resolution",res,res);auto map=autoware::costmap_generator::CostmapCore::initialize("map",lx,ly,res,0,0);map.add("points",0.F);map.add("combined",0.F);autoware::costmap_generator::PointsToCostmap raster;tf2_ros::Buffer tf;tf2_ros::TransformListener listener(tf);auto sub=p.subscribe<sensor_msgs::PointCloud2>("input/points_no_ground",1,[&](const sensor_msgs::PointCloud2::ConstPtr & msg){try{pcl::PointCloud<pcl::PointXYZ> input,cloud;pcl::fromROSMsg(*msg,input);const auto points_to_map=tf.lookupTransform("map",msg->header.frame_id,ros::Time(0),ros::Duration(0.05));pcl_ros::transformPointCloud(input,cloud,points_to_map.transform);map["points"]=raster.makeCostmapFromPoints(5.,0.,0.,1.,map,"points",cloud);map["combined"]=map["points"];got_points=true;}catch(const tf2::TransformException &){got_points=false;}});auto pub=p.advertise<nav_msgs::OccupancyGrid>("output/occupancy_grid",1,true);ros::Rate rate(10);while(ros::ok()){ros::spinOnce();try{const auto t=tf.lookupTransform("map","base_link",ros::Time(0),ros::Duration(0.05));if(got_points){autoware::costmap_generator::CostmapCore::snapCenterToPose(map,t.transform.translation.x,t.transform.translation.y);nav_msgs::OccupancyGrid out;grid_map::GridMapRosConverter::toOccupancyGrid(map,"combined",0,1,out);out.header.frame_id="map";out.header.stamp=ros::Time::now();pub.publish(out);}}catch(const tf2::TransformException &){}rate.sleep();}}
