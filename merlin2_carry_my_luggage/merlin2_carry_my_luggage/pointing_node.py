#!/usr/bin/env python3

# Copyright (C) 2024  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import List, Dict

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray, Detection, KeyPoint3D, KeyPoint2D
from merlin2_carry_my_luggage_msgs.msg import Pointing, PointingArray


class PointingNode(Node):

    def __init__(self) -> None:
        super().__init__("pointing_node")
        self.pub = self.create_publisher(PointingArray, "pointing", 10)
        self.sub = self.create_subscription(
            DetectionArray, "/yolo/detections_3d", self.detection_cb, 10)
        self.get_logger().info("Pointing node started")

    def detection_cb(self, msg: DetectionArray) -> None:

        pointing_array = PointingArray()

        d: Detection
        for d in msg.detections:

            keypoints = {}
            for i in range(len(d.keypoints3d.data)):
                kp = d.keypoints3d.data[i]
                keypoints[kp.id] = (kp, d.keypoints.data[i])

            p_msg = Pointing()
            p_msg.detection = d
            p_msg.direction = self.pointing_direction(keypoints)
            pointing_array.pointings.append(p_msg)

        self.pub.publish(pointing_array)

    @staticmethod
    def pointing_direction(keypoints: Dict[int, List[KeyPoint3D | KeyPoint2D]]) -> int:

        def are_collinear(A, B, C, threshold=0.02):
            # Vector AB
            AB = (B[0] - A[0], B[1] - A[1], B[2] - A[2])

            # Vector AC
            AC = (C[0] - A[0], C[1] - A[1], C[2] - A[2])

            # Cross product AB x AC
            cross_product = (
                AB[1] * AC[2] - AB[2] * AC[1],
                AB[2] * AC[0] - AB[0] * AC[2],
                AB[0] * AC[1] - AB[1] * AC[0]
            )

            # Check if cross product is close to zero vector within the threshold
            if (
                abs(cross_product[0]) < threshold and
                abs(cross_product[1]) < threshold and
                abs(cross_product[2]) < threshold
            ):
                return True
            else:
                return False

        def calculate_direction(
            shoulder: List[KeyPoint3D | KeyPoint2D],
            elbow: List[KeyPoint3D | KeyPoint2D],
            wrist: List[KeyPoint3D | KeyPoint2D],
            hip: List[KeyPoint3D | KeyPoint2D]
        ) -> int:

            if are_collinear(
                (shoulder[0].point.x, shoulder[0].point.y,
                 shoulder[0].point.z),
                (elbow[0].point.x, elbow[0].point.y, elbow[0].point.z),
                (wrist[0].point.x, wrist[0].point.y, wrist[0].point.z)
            ):

                if wrist[1].point.x > hip[1].point.x:
                    return 0
                else:
                    return 1
            else:
                return -1

        try:
            left_shoulder = keypoints[6]
            left_elbow = keypoints[8]
            left_wrist = keypoints[10]
            left_hip = keypoints[12]
            left_arm_direction = calculate_direction(
                left_shoulder, left_elbow, left_wrist, left_hip)
        except:
            left_arm_direction = -1

        try:
            right_shoulder = keypoints[7]
            right_elbow = keypoints[9]
            right_wrist = keypoints[11]
            right_hip = keypoints[13]
            right_arm_direction = calculate_direction(
                right_shoulder, right_elbow, right_wrist, right_hip)
        except:
            right_arm_direction = -1

        if left_arm_direction != -1:
            return left_arm_direction
        else:
            return right_arm_direction


def main():
    rclpy.init()
    node = PointingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
