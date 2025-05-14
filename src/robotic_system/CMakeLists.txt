cmake_minimum_required(VERSION 3.8)
project(robotic_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED) 
find_package(rosbag2_compression REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(interface_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_core REQUIRED)


add_executable(mediated_pose_planner src/mediated_pose_planner.cpp)
ament_target_dependencies(
  mediated_pose_planner
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclpy"
  "moveit_visual_tools"
  tf2
  tf2_ros
  geometry_msgs
  interface_msgs
  tf2_geometry_msgs
  builtin_interfaces
  moveit_core
)
target_compile_features(mediated_pose_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(moveit_controller src/moveit_controller.cpp)
ament_target_dependencies(
  moveit_controller
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclpy"
  "moveit_visual_tools"
)
target_compile_features(moveit_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(no_pose_calculator src/no_pose_calculator.cpp)
target_include_directories(moveit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  no_pose_calculator
  interface_msgs
  rclcpp
  trajectory_msgs
  builtin_interfaces
  sensor_msgs
)
target_compile_features(no_pose_calculator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(store_datapoint src/store_datapoint.cpp)
target_include_directories(moveit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  store_datapoint
  rclcpp
  interface_msgs
  visualization_msgs
  builtin_interfaces
  rosbag2_cpp
  rosbag2_compression
)
target_compile_features(store_datapoint PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


add_executable(produce_sound src/produce_sound.cpp)
target_include_directories(moveit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  produce_sound
  rclcpp
  std_msgs
)
target_compile_features(produce_sound PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(skeleton_converter src/skeleton_converter.cpp)
target_include_directories(skeleton_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  skeleton_converter
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  interface_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)
target_compile_features(skeleton_converter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(interaction_data_converter src/interaction_data_converter.cpp)
target_include_directories(interaction_data_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  interaction_data_converter
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  interface_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)
target_compile_features(interaction_data_converter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(test_transitions src/test_transitions.cpp)
ament_target_dependencies(
  test_transitions
  rclcpp
  trajectory_msgs
  builtin_interfaces
  sensor_msgs
)
target_compile_features(test_transitions PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS moveit_controller no_pose_calculator test_transitions
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS store_datapoint
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS produce_sound
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS skeleton_converter
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS interaction_data_converter
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mediated_pose_planner
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


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
