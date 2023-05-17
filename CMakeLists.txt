cmake_minimum_required(VERSION 3.8)
project(franka_streaming_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(franka_streaming_driver REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrankaJointState.msg"
  "msg/FrankaJointCmd.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC src/examples_common.cpp)
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen)
target_include_directories(examples_common PRIVATE include/franka_streaming_driver)

add_executable(franka_interface src/franka_interface.cpp include/franka_streaming_driver/franka_interface.h)
target_link_libraries(franka_interface Franka::Franka examples_common Eigen3::Eigen Threads::Threads "${cpp_typesupport_target}")
target_include_directories(franka_interface PRIVATE include/franka_streaming_driver)

ament_target_dependencies(franka_interface
  rclcpp 
  std_msgs 
  sensor_msgs
  franka_streaming_driver)

add_executable(joint_teleop src/joint_teleop.cpp)
target_link_libraries(joint_teleop "${cpp_typesupport_target}")
  
ament_target_dependencies(joint_teleop
  rclcpp 
  std_msgs 
  sensor_msgs
  franka_streaming_driver)

install(TARGETS
  franka_interface
  joint_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
