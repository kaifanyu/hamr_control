#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace hamr_control_cpp
{

class ImageToGridmap : public rclcpp::Node
{
public:
  ImageToGridmap();
  ~ImageToGridmap();

private:
  bool readParameters();
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

  grid_map::GridMap map_;
  bool mapInitialized_;

  std::string imageTopic_;
  double resolution_;
  double minHeight_;
  double maxHeight_;
  std::string mapFrameId_;
};

}  // namespace hamr_control_cpp
