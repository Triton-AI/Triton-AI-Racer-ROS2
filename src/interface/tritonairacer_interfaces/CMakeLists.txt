cmake_minimum_required(VERSION 3.8)
project(tritonairacer_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Brake.msg"
  "msg/RaceFlag.msg"
  "msg/RaceStage.msg"
  "msg/RaceState.msg"
  "msg/Steer.msg"
  "msg/Throttle.msg"
  "msg/TritonAIRacerControl.msg"
  "msg/VehicleMode.msg"
  "srv/EStop.srv"
  DEPENDENCIES std_msgs
 )


ament_auto_package()
