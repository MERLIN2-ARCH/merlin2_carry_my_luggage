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


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from kant_dao.dao_factory import DaoFamilies


def generate_launch_description():

    #
    # ARGS
    #
    dao_family = LaunchConfiguration("dao_family")
    dao_family_cmd = DeclareLaunchArgument(
        "dao_family",
        default_value=str(int(DaoFamilies.ROS2)),
        description="DAO family")

    mongo_uri = LaunchConfiguration("mongo_uri")
    mongo_uri_cmd = DeclareLaunchArgument(
        "mongo_uri",
        default_value="mongodb://localhost:27017/merlin2",
        description="MongoDB URI")

    planner = LaunchConfiguration("planner")
    planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value="2",
        description="PDDL planner")

    #
    # NODES
    #
    merlin2_navigation_action_cmd = Node(
        package="merlin2_demo",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_detect_bag_action_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="merlin2_detect_bag_action",
        name="detect_bag",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_pick_bag_action_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="merlin2_pick_bag_action",
        name="pick_bag",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_offer_bag_action_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="merlin2_offer_bag_action",
        name="offer_bag",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_follow_person_action_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="merlin2_follow_person_action",
        name="follow_person",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    merlin2_mission_node_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="merlin2_mission_node",
        name="merlin2_mission_node",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    pointing_node_cmd = Node(
        package="merlin2_carry_my_luggage",
        executable="pointing_node",
        name="pointing_node"
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
        parameters=[{
            "host": "0.0.0.0"
        }],
    )

    #
    # LAUNCHES
    #
    waypoint_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("waypoint_navigation"),
            "waypoint_navigation.launch.py"
        )),
        launch_arguments={"wps": os.path.join(
            get_package_share_directory(
                "merlin2_carry_my_luggage_bringup"),
            "/params/waypoints.yaml"
        )}.items()
    )

    stt_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("speech_to_text"),
            "speech_to_text.launch.py"
        )),
        launch_arguments={
            "stt_grammar": os.path.join(
                get_package_share_directory(
                    "merlin2_carry_my_luggage_bringup"),
                "params/carry.gram"),
            "parser_grammar": os.path.join(
                get_package_share_directory(
                    "merlin2_carry_my_luggage_bringup"),
                "params/carry.gram"),
            "stt_service": "google"
        }.items()
    )

    tts_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("text_to_speech"),
            "text_to_speech.launch.py"
        ))
    )

    yolov8_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yolov8_bringup"),
            "launch/yolov8_3d.launch.py"
        )),
        launch_arguments={
            "model": "yolov8m-pose.pt",
            "image_reliability": "1",
            "depth_image_reliability": "1",
            "depth_info_reliability": "1",
            "input_image_topic": "/head_front_camera/rgb/image_raw",
            "input_depth_topic": "/head_front_camera/depth/image_raw",
            "input_depth_info_topic": "/head_front_camera/depth/camera_info",
        }.items()
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("merlin2_planning_layer"),
            "merlin2_planning_layer.launch.py"
        )),
        launch_arguments={
            "dao_family": dao_family,
            "mongo_uri": mongo_uri,
            "planner": planner
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(dao_family_cmd)
    ld.add_action(mongo_uri_cmd)
    ld.add_action(planner_cmd)

    ld.add_action(waypoint_navigation_cmd)
    ld.add_action(stt_cmd)
    ld.add_action(tts_cmd)
    ld.add_action(yolov8_cmd)

    ld.add_action(merlin2_navigation_action_cmd)
    ld.add_action(merlin2_detect_bag_action_cmd)
    ld.add_action(merlin2_pick_bag_action_cmd)
    ld.add_action(merlin2_offer_bag_action_cmd)
    ld.add_action(merlin2_follow_person_action_cmd)

    ld.add_action(merlin2_planning_layer_cmd)
    ld.add_action(merlin2_mission_node_cmd)
    ld.add_action(pointing_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
