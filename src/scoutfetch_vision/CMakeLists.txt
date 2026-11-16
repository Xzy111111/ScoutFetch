cmake_minimum_required(VERSION 3.8)
project(scoutfetch_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# OpenCV 系统库 (非 ament 包, 仅 CMake 链接)
find_package(OpenCV REQUIRED)

add_executable(localization_node src/localization_node.cpp)

target_include_directories(localization_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(localization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(localization_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# yolo_detector: pixel_source:=yolo 时的检测来源 (Python + onnxruntime, 见 README)
install(PROGRAMS scripts/yolo_detector.py scripts/detection_overlay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contract test/test_contract.py)
  ament_add_pytest_test(detector_frames test/test_detector_frames.py)
  ament_add_pytest_test(rgb_depth_identity test/test_rgb_depth_identity.py)
endif()

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
