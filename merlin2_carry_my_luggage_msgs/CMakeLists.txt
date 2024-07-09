cmake_minimum_required(VERSION 3.8)
project(merlin2_carry_my_luggage_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yolov8_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pointing.msg"
  "msg/PointingArray.msg"
  DEPENDENCIES yolov8_msgs
)

ament_package()
