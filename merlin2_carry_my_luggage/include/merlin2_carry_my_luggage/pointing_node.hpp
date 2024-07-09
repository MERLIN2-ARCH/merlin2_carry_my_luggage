#ifndef POINTING_NODE_HPP
#define POINTING_NODE_HPP

#include <memory>
#include <unordered_map>
#include <vector>

#include "merlin2_carry_my_luggage_msgs/msg/pointing.hpp"
#include "merlin2_carry_my_luggage_msgs/msg/pointing_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

using namespace yolov8_msgs::msg;
using namespace merlin2_carry_my_luggage_msgs::msg;

class PointingNode : public rclcpp::Node {
public:
  PointingNode();

private:
  void detection_cb(const DetectionArray::SharedPtr msg);
  int pointing_direction(
      const std::unordered_map<int, std::pair<KeyPoint3D, KeyPoint2D>>
          &keypoints);

  rclcpp::Publisher<PointingArray>::SharedPtr pub_;
  rclcpp::Subscription<DetectionArray>::SharedPtr sub_;
};

#endif // POINTING_NODE_HPP
