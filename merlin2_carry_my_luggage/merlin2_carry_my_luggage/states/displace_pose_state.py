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


import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin.blackboard import Blackboard


class DisplacePoseState(State):

    def __init__(self, node: Node, distance: float = 0.7) -> None:

        super().__init__([SUCCEED, ABORT])

        self.node = node
        self.distance = distance
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def execute(self, blackboard: Blackboard) -> str:
        # get person position
        P = [blackboard.pose.position.x,
             blackboard.pose.position.y]

        # get tf
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link",
                "base_link",
                rclpy.time.Time())
        except TransformException as ex:
            self.node.get_logger().info(
                f"Could not transform base_link to map: {ex}")
            return ABORT

        Q = [t.transform.translation.x,
             t.transform.translation.y]

        new_pose = self.displace_point(P, Q, self.distance)
        blackboard.displace_pose = Pose()
        blackboard.displace_pose.position.x = new_pose[0]
        blackboard.displace_pose.position.y = new_pose[1]
        blackboard.displace_pose.position.z = 0.0
        blackboard.displace_pose.orientation.z = t.transform.rotation.z
        blackboard.displace_pose.orientation.w = t.transform.rotation.w
        return SUCCEED

    @staticmethod
    def displace_point(P: List[float], Q: List[float], distance: float) -> List[float]:
        # Convert points to numpy arrays
        P = np.array(P)
        Q = np.array(Q)

        # Calculate the direction vector from P to Q
        direction_vector = Q - P

        # Normalize the direction vector
        norm = np.linalg.norm(direction_vector)
        if norm == 0:
            raise ValueError("P and Q cannot be the same point")
        normalized_direction = direction_vector / norm

        # Scale the normalized direction vector by the displacement distance
        displacement_vector = normalized_direction * distance

        # Add the displacement vector to the original point P
        new_point = P + displacement_vector

        return new_point.tolist()
