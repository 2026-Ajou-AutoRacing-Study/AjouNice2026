#ifndef BRINGUP__SCOPED_IMAGE_DISPLAY_HPP_
#define BRINGUP__SCOPED_IMAGE_DISPLAY_HPP_

#include <rviz/default_plugin/image_display.h>

namespace bringup
{
// Keep RViz's renderer/decoder and wire topics; scope only transport parameters.
class ScopedImageDisplay : public rviz::ImageDisplay
{
public:
  ScopedImageDisplay();

protected:
  void subscribe() override;

private:
  ros::NodeHandle transport_parameters_;
};
}  // namespace bringup
#endif
