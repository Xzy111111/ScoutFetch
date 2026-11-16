cmake_minimum_required(VERSION 3.8)
project(scoutfetch_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_contract test/test_navigation_contract.py)
endif()

ament_package()
