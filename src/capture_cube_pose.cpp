#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PoseTransformer : public rclcpp::Node {
public:
    PoseTransformer() : Node("pose_transformer"), output_file_path_(initialize_output_path()) {
        // Initialize transform listener and buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for visualization marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Subscriber to the Pose topic
        subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/cube/pose", 10, std::bind(&PoseTransformer::pose_callback, this, std::placeholders::_1));
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::string output_file_path_;

    std::string initialize_output_path() {
        std::string package_name = "arctos_motion_planning";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string file_path = package_share_directory + "/config/cube_pose.txt";
        std::filesystem::create_directories(package_share_directory + "/config"); // Ensure directory exists
        return file_path;
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        try {
            // Prepare the target pose in the new frame
            geometry_msgs::msg::PoseStamped current_pose, transformed_pose;
            current_pose.header.frame_id = "camera_color_optical_frame";
            // current_pose.header.frame_id = "camera_color_frame";
            current_pose.header.stamp = this->get_clock()->now();
            current_pose.pose = *msg;

            // Transform the pose to the base_link frame
            // transformed_pose = tf_buffer_->transform(current_pose, "base_link", tf2::durationFromSec(1.0));
            transformed_pose = current_pose;
            transformed_pose.pose.position.y = -transformed_pose.pose.position.y;
            transformed_pose.pose.position.x = -transformed_pose.pose.position.x;
            RCLCPP_INFO(this->get_logger(), "Transformed Data: X=%f m, Y=%f m, Z=%f m", 
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z);

            // Output the transformed pose to a file
            write_pose_to_file(transformed_pose.pose);

            // Publish the marker for visualization
            publish_marker(transformed_pose);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failure to transform pose: %s", ex.what());
        }
    }

    void write_pose_to_file(const geometry_msgs::msg::Pose& pose) {
        std::ofstream outfile(output_file_path_);
        if (!outfile) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file to write pose at path: %s", output_file_path_.c_str());
            return;
        }
        outfile << "Position: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "\n"
                << "Orientation: " << pose.orientation.x << ", " << pose.orientation.y << ", "
                << pose.orientation.z << ", " << pose.orientation.w << "\n\n";
        outfile.close();
    }

    void publish_marker(const geometry_msgs::msg::PoseStamped& pose) {
        visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "base_link";
        marker.header.frame_id = "camera_color_optical_frame";
        marker.header.stamp = pose.header.stamp;
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose.pose;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub_->publish(marker);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
