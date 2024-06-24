#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("grasp_block_demo", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arctos_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  for (const auto& group_name : move_group.getJointModelGroupNames()) {
    RCLCPP_INFO(LOGGER, "Available group: %s", group_name.c_str());
  }

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.0;
  target_pose1.position.y = -0.41;
  target_pose1.position.z = 0.65;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(LOGGER, "Starting planning to pose goal");
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(success) move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}

// Move using joint states

/*
int main(int argc, char** argv) {

    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_arm");

    // Create a ROS spinner to handle callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup Move group (replace "arm" with your move group name)
    static const std::string PLANNING_GROUP = "arctos_arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Set target joint values (example values, replace with your specific joint names and target values)
    std::map<std::string, double> target_joints = {
        {"joint_1", 0.5},
        {"joint_2", 1.0},
        {"joint_3", -0.5},
        {"joint_4", 0.0}
    };
    move_group.setJointValueTarget(target_joints);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.move();
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
*/
