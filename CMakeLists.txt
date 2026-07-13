cmake_minimum_required(VERSION 3.8)
project(arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(odrive_can REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(arm_teleop_messages REQUIRED)

add_executable(arm_teleop_node src/arm_teleop_node.cpp)
target_include_directories(arm_teleop_node PRIVATE include)
add_executable(arm_joint_sim_node src/arm_joint_sim_node.cpp)
add_executable(arm_safety_node src/arm_safety_node.cpp)
target_include_directories(arm_safety_node PRIVATE include)


ament_target_dependencies(arm_joint_sim_node
  rclcpp
  sensor_msgs
  odrive_can
)

ament_target_dependencies(arm_teleop_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  odrive_can
  arm_teleop_messages
)

ament_target_dependencies(arm_safety_node
  rclcpp
  std_msgs
  std_srvs
  odrive_can
  controller_manager_msgs
  arm_teleop_messages
)

target_link_libraries(arm_teleop_node "${cpp_typesupport_target}")
target_link_libraries(arm_safety_node "${cpp_typesupport_target}")

install(TARGETS arm_teleop_node arm_joint_sim_node arm_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()