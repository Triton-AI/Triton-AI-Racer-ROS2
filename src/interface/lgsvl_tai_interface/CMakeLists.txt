cmake_minimum_required(VERSION 3.5)
project(lgsvl_tai_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lgsvl_sender.cpp
  src/lgsvl_receiver.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN lgsvl_tai::LGSVLSender
  EXECUTABLE lgsvl_sender_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    param
)
