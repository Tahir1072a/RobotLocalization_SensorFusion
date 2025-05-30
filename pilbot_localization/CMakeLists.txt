cmake_minimum_required(VERSION 3.8)
project(pilbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/imu_logger.py
  ${PROJECT_NAME}/imu_republisher.py
  ${PROJECT_NAME}/pose_prediction.py
  ${PROJECT_NAME}/pose_logger.py
  ${PROJECT_NAME}/draw_graphs.py
  ${PROJECT_NAME}/test_timer.py
  ${PROJECT_NAME}/virtual_imu_publisher.py
  ${PROJECT_NAME}/draw_graphs_2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
