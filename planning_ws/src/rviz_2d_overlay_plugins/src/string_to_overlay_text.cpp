// ROS1 adapter: std_msgs/String -> rviz_2d_overlay_msgs/OverlayText.
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz_2d_overlay_msgs/OverlayText.h>

class Rviz2dString
{
public:
  Rviz2dString()
  {
    ros::NodeHandle private_node("~");
    private_node.param<std::string>("string_topic", string_topic_, "/chatter");
    private_node.param<std::string>("fg_color", fg_color_, "");
    overlay_text_topic_ = string_topic_ + "_overlay_text";
    publisher_ = node_.advertise<rviz_2d_overlay_msgs::OverlayText>(overlay_text_topic_, 1);
    subscriber_ = node_.subscribe(string_topic_, 10, &Rviz2dString::strCallback, this);
    ROS_INFO_STREAM("subscribed: " << string_topic_ << " publishing: " << overlay_text_topic_);
  }

private:
  void strCallback(const std_msgs::String::ConstPtr &input)
  {
    rviz_2d_overlay_msgs::OverlayText output;
    output.text = input->data;
    output.fg_color.a = 1.0;
    if (fg_color_ == "r") { output.fg_color.r = .96F; output.fg_color.g = .22F; output.fg_color.b = .06F; }
    else if (fg_color_ == "g") { output.fg_color.r = .30F; output.fg_color.g = .69F; output.fg_color.b = .31F; }
    else if (fg_color_ == "b") { output.fg_color.r = .02F; output.fg_color.g = .50F; output.fg_color.b = .70F; }
    else if (fg_color_ == "k") { output.fg_color.r = .19F; output.fg_color.g = .19F; output.fg_color.b = .23F; }
    else if (fg_color_ == "w") { output.fg_color.r = .89F; output.fg_color.g = .89F; output.fg_color.b = .93F; }
    else if (fg_color_ == "p") { output.fg_color.r = .91F; output.fg_color.g = .12F; output.fg_color.b = .39F; }
    else { output.fg_color.r = .94F; output.fg_color.g = .83F; output.fg_color.b = .07F; }
    output.width = 200;
    output.height = 40;
    publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string string_topic_, overlay_text_topic_, fg_color_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rviz2d_from_string_node");
  Rviz2dString node;
  ros::spin();
  return 0;
}
