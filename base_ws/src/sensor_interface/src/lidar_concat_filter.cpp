/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted from Autoware.AI points_preprocessor/points_concat_filter for
 * the AjouNice2026 ROS1 sensor_interface package.
 */

#include <algorithm>
#include <cassert>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

class LidarConcatFilter
{
public:
  LidarConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  std::unique_ptr<message_filters::Subscriber<PointCloudMsgT> > cloud_subscribers_[8];
  std::unique_ptr<message_filters::Synchronizer<SyncPolicyT> > cloud_synchronizer_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_frame_id_;
  std::string output_topic_;
  double wait_for_transform_sec_;
  int subscriber_queue_size_;
  int sync_queue_size_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                           const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                           const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

LidarConcatFilter::LidarConcatFilter()
  : node_handle_()
  , private_node_handle_("~")
  , input_topics_size_(0)
  , wait_for_transform_sec_(1.0)
  , subscriber_queue_size_(1)
  , sync_queue_size_(10)
{
  private_node_handle_.param("input_topics", input_topics_, std::string("[/sensor/lidar1, /sensor/lidar2]"));
  private_node_handle_.param("output_frame_id", output_frame_id_, std::string("base_link"));
  private_node_handle_.param("output_topic", output_topic_, std::string("/sensor/lidar_concat"));
  private_node_handle_.param("wait_for_transform_sec", wait_for_transform_sec_, 1.0);
  private_node_handle_.param("subscriber_queue_size", subscriber_queue_size_, 1);
  private_node_handle_.param("sync_queue_size", sync_queue_size_, 10);

  YAML::Node topics;
  try
  {
    topics = YAML::Load(input_topics_);
  }
  catch (const YAML::Exception &ex)
  {
    ROS_FATAL_STREAM("Failed to parse input_topics YAML list: " << ex.what());
    ros::shutdown();
    return;
  }

  input_topics_size_ = topics.size();
  if (input_topics_size_ < 2 || 8 < input_topics_size_)
  {
    ROS_FATAL("The size of input_topics must be between 2 and 8");
    ros::shutdown();
    return;
  }

  const int subscriber_queue_size = std::max(1, subscriber_queue_size_);
  for (size_t i = 0; i < 8; ++i)
  {
    const std::string topic = i < input_topics_size_ ? topics[i].as<std::string>() : topics[0].as<std::string>();
    cloud_subscribers_[i].reset(new message_filters::Subscriber<PointCloudMsgT>(
        node_handle_, topic, static_cast<uint32_t>(subscriber_queue_size)));
  }

  cloud_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(std::max(1, sync_queue_size_)), *cloud_subscribers_[0], *cloud_subscribers_[1],
      *cloud_subscribers_[2], *cloud_subscribers_[3], *cloud_subscribers_[4], *cloud_subscribers_[5],
      *cloud_subscribers_[6], *cloud_subscribers_[7]));
  cloud_synchronizer_->registerCallback(
      boost::bind(&LidarConcatFilter::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>(output_topic_, 1);

  ROS_INFO_STREAM("lidar_concat_filter publishing " << output_topic_ << " in frame " << output_frame_id_
                                                    << " from " << input_topics_);
}

void LidarConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1,
                                            const PointCloudMsgT::ConstPtr &msg2,
                                            const PointCloudMsgT::ConstPtr &msg3,
                                            const PointCloudMsgT::ConstPtr &msg4,
                                            const PointCloudMsgT::ConstPtr &msg5,
                                            const PointCloudMsgT::ConstPtr &msg6,
                                            const PointCloudMsgT::ConstPtr &msg7,
                                            const PointCloudMsgT::ConstPtr &msg8)
{
  assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

  PointCloudMsgT::ConstPtr msgs[8] = { msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8 };
  PointCloudT::Ptr cloud_sources[8];
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  try
  {
    for (size_t i = 0; i < input_topics_size_; ++i)
    {
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0),
                                    ros::Duration(wait_for_transform_sec_));
      pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id,
                                   *cloud_sources[i], tf_listener_);
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }

  for (size_t i = 0; i < input_topics_size_; ++i)
  {
    *cloud_concatenated += *cloud_sources[i];
  }

  PointCloudMsgT output_msg;
  pcl::toROSMsg(*cloud_concatenated, output_msg);
  output_msg.header = msgs[0]->header;
  output_msg.header.frame_id = output_frame_id_;
  cloud_publisher_.publish(output_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_concat_filter");
  LidarConcatFilter node;
  ros::spin();
  return 0;
}
