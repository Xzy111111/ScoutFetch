cmake_minimum_required(VERSION 3.8)
project(scoutfetch_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(moveit_contract test/test_moveit_contract.py TIMEOUT 60)
endif()

ament_package()

