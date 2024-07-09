#include "merlin2_carry_my_luggage/pointing_node.hpp"

using std::placeholders::_1;

PointingNode::PointingNode() : Node("pointing_node") {
  pub_ = this->create_publisher<PointingArray>("pointing", 10);
  sub_ = this->create_subscription<DetectionArray>(
      "/yolo/detections_3d", 10,
      std::bind(&PointingNode::detection_cb, this, _1));
  RCLCPP_INFO(this->get_logger(), "Pointing node started");
}

void PointingNode::detection_cb(const DetectionArray::SharedPtr msg) {
  PointingArray pointing_array;

  for (const auto &d : msg->detections) {
    std::unordered_map<int, std::pair<KeyPoint3D, KeyPoint2D>> keypoints;
    for (size_t i = 0; i < d.keypoints3d.data.size(); ++i) {
      keypoints[d.keypoints3d.data[i].id] =
          std::make_pair(d.keypoints3d.data[i], d.keypoints.data[i]);
    }

    Pointing p_msg;
    p_msg.detection = d;
    p_msg.direction = pointing_direction(keypoints);
    pointing_array.pointings.push_back(p_msg);
  }

  pub_->publish(pointing_array);
}

int PointingNode::pointing_direction(
    const std::unordered_map<int, std::pair<KeyPoint3D, KeyPoint2D>>
        &keypoints) {
  auto are_collinear =
      [](const std::array<double, 3> &A, const std::array<double, 3> &B,
         const std::array<double, 3> &C, double threshold = 0.01) {
        std::array<double, 3> AB = {B[0] - A[0], B[1] - A[1], B[2] - A[2]};
        std::array<double, 3> AC = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};

        std::array<double, 3> cross_product = {AB[1] * AC[2] - AB[2] * AC[1],
                                               AB[2] * AC[0] - AB[0] * AC[2],
                                               AB[0] * AC[1] - AB[1] * AC[0]};

        return (std::abs(cross_product[0]) < threshold &&
                std::abs(cross_product[1]) < threshold &&
                std::abs(cross_product[2]) < threshold);
      };

  auto calculate_direction =
      [&](const std::pair<KeyPoint3D, KeyPoint2D> &shoulder,
          const std::pair<KeyPoint3D, KeyPoint2D> &elbow,
          const std::pair<KeyPoint3D, KeyPoint2D> &wrist,
          const std::pair<KeyPoint3D, KeyPoint2D> &hip) {
        std::array<double, 3> shoulder_point = {shoulder.first.point.x,
                                                shoulder.first.point.y,
                                                shoulder.first.point.z};
        std::array<double, 3> elbow_point = {
            elbow.first.point.x, elbow.first.point.y, elbow.first.point.z};
        std::array<double, 3> wrist_point = {
            wrist.first.point.x, wrist.first.point.y, wrist.first.point.z};

        if (are_collinear(shoulder_point, elbow_point, wrist_point)) {
          return (wrist.second.point.x > hip.second.point.x) ? 0 : 1;
        }
        return -1;
      };

  int left_arm_direction = -1;
  try {
    left_arm_direction = calculate_direction(
        keypoints.at(6), keypoints.at(8), keypoints.at(10), keypoints.at(12));
  } catch (...) {
    left_arm_direction = -1;
  }

  int right_arm_direction = -1;
  try {
    right_arm_direction = calculate_direction(
        keypoints.at(7), keypoints.at(9), keypoints.at(11), keypoints.at(13));
  } catch (...) {
    right_arm_direction = -1;
  }

  return (left_arm_direction != -1) ? left_arm_direction : right_arm_direction;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
