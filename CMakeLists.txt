cmake_minimum_required(VERSION 3.8)
project(arctos_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(arctos_description REQUIRED)
find_package(arctos_moveit_config REQUIRED)
find_package(moveit_common REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
)

add_executable(grasp_block_demo src/grasp_block_demo.cpp)
ament_target_dependencies(grasp_block_demo
  rclcpp
  moveit_common
  moveit_ros_planning_interface
  interactive_markers
  visualization_msgs
  std_msgs
  geometry_msgs
)

add_executable(move_demo src/move_demo.cpp)
ament_target_dependencies(move_demo
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(capture_cube_pose src/capture_cube_pose.cpp)
ament_target_dependencies(capture_cube_pose
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(realsense_to_moveit_node src/realsense_to_moveit_node.cpp)
ament_target_dependencies(realsense_to_moveit_node
  rclcpp
  sensor_msgs
  octomap_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  octomap_ros
  PCL
  message_filters
  pcl_msgs
  urdf
  visualization_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(realsense_to_moveit_local_updates_only_node src/realsense_to_moveit_local_updates_only_node.cpp)
ament_target_dependencies(realsense_to_moveit_local_updates_only_node
  rclcpp
  sensor_msgs
  octomap_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  octomap_ros
  PCL
  message_filters
  pcl_msgs
  urdf
  visualization_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(serial_joint_state_publisher src/serial_joint_state_publisher.cpp)
ament_target_dependencies(serial_joint_state_publisher
  rclcpp
  sensor_msgs
)

install(
  TARGETS 
  grasp_block_demo
  move_demo
  capture_cube_pose
  realsense_to_moveit_node
  realsense_to_moveit_local_updates_only_node
  serial_joint_state_publisher
  DESTINATION 
    lib/${PROJECT_NAME}
)

target_link_libraries(
  realsense_to_moveit_local_updates_only_node
  ${PCL_LIBRARIES}
)

target_link_libraries(
  realsense_to_moveit_node
  ${PCL_LIBRARIES}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
