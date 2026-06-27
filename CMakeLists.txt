cmake_minimum_required(VERSION 3.8)
project(arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(odrive_can REQUIRED)

add_executable(arm_teleop_node src/arm_teleop_node.cpp)

ament_target_dependencies(arm_teleop_node
  rclcpp
  sensor_msgs
  odrive_can
)

install(TARGETS arm_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()