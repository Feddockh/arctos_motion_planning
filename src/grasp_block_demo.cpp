#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>


std::string initialize_input_path() {
    std::string package_name = "arctos_motion_planning";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    std::string file_path = package_share_directory + "/config/cube_pose.txt";
    return file_path;
}

// Define a structure to hold position data
struct Position {
    double x, y, z;
};

// Function to parse a line containing the position
Position parsePosition(const std::string& line) {
    std::istringstream iss(line.substr(10)); // Remove "Position: "
    Position pos;
    char comma; // To consume the commas in the input
    iss >> pos.x >> comma >> pos.y >> comma >> pos.z;
    return pos;
}

// Function to read positions from a file and return a vector of Position structures
std::vector<Position> readPositions(const std::string& filename) {
    std::vector<Position> positions;
    std::ifstream file(filename);
    std::string line;

    while (getline(file, line)) {
        if (line.find("Position:") != std::string::npos) {
            positions.push_back(parsePosition(line)); // Corrected to push_back
        }
    }
    return positions;
}

std::string initialize_joint_config_path() {
    std::string package_name = "arctos_motion_planning";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    std::string file_path = package_share_directory + "/config/joint_config_path.txt";
    return file_path;
}

// Dimensions of the 2D array
// 71.12cm by 50.8

struct JointSet {
    double joint_1;
    double joint_2;
    double joint_3;
    double joint_4;
    JointSet(double j1, double j2, double j3, double j4)
        : joint_1(j1), joint_2(j2), joint_3(j3), joint_4(j4) {
    }
};

JointSet zero_position(0, 0, 0, 0);
JointSet view_position(0, 0, 0, 0);
JointSet left_position(0.21, 0.452, 0.188, 0);
JointSet right_position(-0.518, 0.509, 0.135, 0);

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_arm");

    // Create a ROS spinner for asynchronous callback handling
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup Move group (replace "arm" with your move group name)
    static const std::string PLANNING_GROUP = "arctos_arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setEndEffector("tool0");

    // Read positions from a text file
    std::string filename = initialize_input_path();
    std::vector<Position> positions = readPositions(filename);

    // Setup Move group (replace "arm" with your move group name)
    static const std::string GRIPPER_GROUP = "arctos_gripper";
    moveit::planning_interface::MoveGroupInterface gripper_move_group(node, GRIPPER_GROUP);
    std::map<std::string, double> gripper_position;

    // // Output the positions to verify they are correct
    // for (const auto& pos : positions) {
    //     std::cout << "Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
    // }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(2);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    bool success = false;

    bool left = true;

    while(1) {

    // Check if the cube is in the left or right position
    if (left) {

        // Move to left pre-grasp
        std::map<std::string, double> target_joints = {
            {"joint_1", left_position.joint_1},
            {"joint_2", left_position.joint_2},
            {"joint_3", 0},
            {"joint_4", left_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

        // Move to left grasp
        target_joints = {
            {"joint_1", left_position.joint_1},
            {"joint_2", left_position.joint_2},
            {"joint_3", left_position.joint_3},
            {"joint_4", left_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

    } else {

        // Move to right pre-grasp
        std::map<std::string, double> target_joints = {
            {"joint_1", right_position.joint_1},
            {"joint_2", right_position.joint_2},
            {"joint_3", 0},
            {"joint_4", right_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

        // Move to right grasp
        target_joints = {
            {"joint_1", right_position.joint_1},
            {"joint_2", right_position.joint_2},
            {"joint_3", right_position.joint_3},
            {"joint_4", right_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();
    }

    // Set the gripper joint's grasp position
    gripper_position["left_jaw"] = 0.015;
    gripper_move_group.setJointValueTarget(gripper_position);

    // Move the gripper to grasp
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) gripper_move_group.move();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Move back to zero
    std::map<std::string, double> target_joints = {
        {"joint_1", 0.0},
        {"joint_2", 0.0},
        {"joint_3", 0.0},
        {"joint_4", 0.0}
    };
    move_group.setJointValueTarget(target_joints);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) move_group.move();


    // Check if the cube is in the left or right position
    if (!left) {

        // Move to left pre-drop
        std::map<std::string, double> target_joints = {
            {"joint_1", left_position.joint_1},
            {"joint_2", left_position.joint_2},
            {"joint_3", 0},
            {"joint_4", left_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

        // Move to left drop
        target_joints = {
            {"joint_1", left_position.joint_1},
            {"joint_2", left_position.joint_2},
            {"joint_3", left_position.joint_3},
            {"joint_4", left_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

    } else {

        // Move to right pre-drop
        std::map<std::string, double> target_joints = {
            {"joint_1", right_position.joint_1},
            {"joint_2", right_position.joint_2},
            {"joint_3", 0},
            {"joint_4", right_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

        // Move to right drop
        target_joints = {
            {"joint_1", right_position.joint_1},
            {"joint_2", right_position.joint_2},
            {"joint_3", right_position.joint_3},
            {"joint_4", right_position.joint_4}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();
    }

    // Set the gripper joint's release position
    gripper_position["left_jaw"] = 0.0;
    gripper_move_group.setJointValueTarget(gripper_position);

    // Move the gripper to drop
    success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) gripper_move_group.move();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Move to post-drop
    // Check if the cube is in the left or right position
    if (!left) {

        // Move to left post-drop
        std::map<std::string, double> target_joints = {
            {"joint_1", left_position.joint_1},
            {"joint_2", 0},
            {"joint_3", 0},
            {"joint_4", 0}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();

    } else {

        // Move to right post-drop
        std::map<std::string, double> target_joints = {
            {"joint_1", right_position.joint_1},
            {"joint_2", 0},
            {"joint_3", 0},
            {"joint_4", 0}
        };

        move_group.setJointValueTarget(target_joints);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) move_group.move();
    }

    // Move back to zero
    target_joints = {
        {"joint_1", 0.0},
        {"joint_2", 0.0},
        {"joint_3", 0.0},
        {"joint_4", 0.0}
    };
    move_group.setJointValueTarget(target_joints);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) move_group.move();

    std::this_thread::sleep_for(std::chrono::seconds(10));
    left = !left;
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
