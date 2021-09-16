cmake_minimum_required(VERSION 3.8)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lgsvl_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp joy sensor_msgs lgsvl_msgs)

install(TARGETS
  teleop_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
