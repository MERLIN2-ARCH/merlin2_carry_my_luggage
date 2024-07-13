cmake_minimum_required(VERSION 3.8)
project(merlin2_carry_my_luggage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(merlin2_carry_my_luggage_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  include
)

add_executable(pointing_node src/pointing_node.cpp)
ament_target_dependencies(pointing_node PUBLIC rclcpp yolov8_msgs merlin2_carry_my_luggage_msgs)
install(TARGETS
  pointing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_detect_bag_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_detect_bag_action
)

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_pick_bag_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_pick_bag_action
)

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_offer_bag_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_offer_bag_action
)

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_follow_person_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_follow_person_action
)

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_mission_node
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
