cmake_minimum_required(VERSION 3.8)
project(scoutfetch_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(scoutfetch_interfaces REQUIRED)

add_executable(pick_node src/pick_node.cpp)

target_include_directories(pick_node PRIVATE
  include
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
)

ament_target_dependencies(pick_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_ros_planning
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  scoutfetch_interfaces
)

install(TARGETS pick_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contract test/test_contract.py)
endif()

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
