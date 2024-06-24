#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class RealSenseToMoveItLocalUpdatesOnlyNode : public rclcpp::Node {
public:
    RealSenseToMoveItLocalUpdatesOnlyNode() : Node("realsense_to_moveit_local_updates_only_node"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
        
        camera_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 1, std::bind(&RealSenseToMoveItLocalUpdatesOnlyNode::pointCloudCallback, this, std::placeholders::_1));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    sensor_msgs::msg::PointCloud2 last_cloud;
    double maxRange = 1.0;
    double resolution = 0.05;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tfBuffer.lookupTransform("base_link", cloud_msg->header.frame_id, cloud_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            tf2::doTransform(*cloud_msg, transformed_cloud, transform);
        } catch (const tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud: %s", e.what());
            return;
        }

        // Update the left point cloud
        last_cloud = transformed_cloud;
        processPointClouds(transformed_cloud);
    }

    void processPointClouds(const sensor_msgs::msg::PointCloud2 cloud_msg) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // Convert ROS PointCloud2 to PCL PointCloud for left and right clouds
        pcl::fromROSMsg(cloud_msg, *pcl_cloud);

        // Insert point cloud into the OctoMap
        // for (const auto& point : *pcl_cloud) {
        //     tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        // }

        // Start timing
        auto start = std::chrono::high_resolution_clock::now();

        // Parallel processing
        // const size_t numThreads = 1;
        const size_t numThreads = std::thread::hardware_concurrency();
        RCLCPP_INFO(this->get_logger(), "numThreads: %ld", numThreads);

        // Create a vector to store thread-local octree pointers
        std::vector<std::unique_ptr<octomap::OcTree>> localTrees(numThreads);
        for (auto& tree : localTrees) {
            tree = std::make_unique<octomap::OcTree>(resolution);
        }

        size_t chunkSize = pcl_cloud->size() / numThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < numThreads; ++i) {
            size_t startIdx = i * chunkSize;
            size_t endIdx = (i + 1 == numThreads) ? pcl_cloud->size() : (i + 1) * chunkSize;

            threads.emplace_back([&, startIdx, endIdx, i] {
                for (size_t j = startIdx; j < endIdx; ++j) {
                    const pcl::PointXYZ& point = (*pcl_cloud)[j];
                        processPoint(*localTrees[i], point);
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }

        // Merge thread-local trees into a main tree
        octomap::OcTree tree(resolution);
        for (auto& localTree : localTrees) {
            for (octomap::OcTree::leaf_iterator it = localTree->begin_leafs(), end = localTree->end_leafs(); it != end; ++it) {
                if (localTree->isNodeOccupied(*it)) {
                    tree.updateNode(it.getKey(), true);
                }
            }
        }

        // Update to reflect changes
        tree.updateInnerOccupancy();

        // Stop timing and calculate the elapsed time
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO(this->get_logger(), "Processing time: %ld ms", duration.count());

        // Convert the OctoMap to a ROS message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = cloud_msg.header.frame_id;
        octomap_msg.header.stamp = this->get_clock()->now();
        octomap_msgs::fullMapToMsg(tree, octomap_msg);

        // Update the /move_group topic using the planning scene interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.octomap.octomap = octomap_msg;
        planning_scene_msg.world.octomap.header.frame_id = "base_link";
        planning_scene_msg.is_diff = true;
        planning_scene_interface.applyPlanningScene(planning_scene_msg);
    }

    void processPoint(octomap::OcTree &tree, const pcl::PointXYZ &point) {
        
        // Check if the point's coordinates are within the specified range
        if (point.x >= -1.0 && point.x <= 1.0 &&
            point.y >= -1.0 && point.y <= 1.0 &&
            point.z >= -1.0 && point.z <= 1.0) {

            // Add the point to the OctoMap tree
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseToMoveItLocalUpdatesOnlyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
