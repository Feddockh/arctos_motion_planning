#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <string>
#include <chrono>

class serial_joint_state_publisher : public rclcpp::Node
{
public:
    serial_joint_state_publisher()
    : Node("serial_joint_state_publisher")
    //   io(),
    //   serial(io, "/dev/ttyACM0"), // If there is a boost error it is because this port is not available
    //   response_buffer()
    {
        // Set the serial port settings
        // serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        // serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

        // Subscribe to JointState topic
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->joint_state_callback(msg);
            });

        stepdownfreq = 0;
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        stepdownfreq++;
        if (stepdownfreq < 10) {
            return;
        }
        stepdownfreq = 0;

        std::ostringstream oss;
        oss << "A";

        double positionInDegrees;

        std::map<std::string, double> joint_positions;

        // Map the positions to the joint names
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_positions[msg->name[i]] = msg->position[i];
        }

        // Invert X
        if (joint_positions.count("joint_1") > 0) {
            positionInDegrees = std::round(joint_positions["joint_1"] * 180.0 / M_PI);
            oss << -positionInDegrees << ",";
        }

        // Y
        if (joint_positions.count("joint_2") > 0) {
            positionInDegrees = std::round(joint_positions["joint_2"] * 180.0 / M_PI);
            oss << positionInDegrees << ",";
        }

        // Invert Z
        if (joint_positions.count("joint_3") > 0) {
            positionInDegrees = std::round(joint_positions["joint_3"] * 180.0 / M_PI);
            oss << -positionInDegrees << ",";
        }

        // A
        if (joint_positions.count("joint_4") > 0) {
            positionInDegrees = std::round(joint_positions["joint_4"] * 180.0 / M_PI);
            oss << -positionInDegrees << ",";
        }

        // B
        if (joint_positions.count("joint_5") > 0) {
            positionInDegrees = std::round(joint_positions["joint_5"] * 180.0 / M_PI);
            oss << positionInDegrees << ",";
        }

        // C
        if (joint_positions.count("joint_6") > 0) {
            positionInDegrees = std::round(joint_positions["joint_6"] * 180.0 / M_PI);
            oss << -positionInDegrees << ",";
        }

        // Gripper
        if (joint_positions.count("left_jaw") > 0) {
            if (joint_positions["left_jaw"] > 0.007) {
                oss << "1";
            } else {
                oss << "0";
            }
        }

        oss << "B";

        std::string data = oss.str();

        // Debugging Data Collection
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
        RCLCPP_INFO(this->get_logger(), "[%ld] Sending: '%s'", seconds, data.c_str());
    

        // RCLCPP_INFO(this->get_logger(), "Sending: '%s'", data.c_str());
        
        // Write data to serial port
        // boost::asio::write(serial, boost::asio::buffer(data + '\n')); // Add newline for separation
        // boost::asio::write(serial, boost::asio::buffer(data));

        // Read response from serial port
        // boost::asio::read_until(serial, response_buffer, '\n'); // Read until newline character
        // std::istream response_stream(&response_buffer);
        // std::string response;
        // std::getline(response_stream, response);

        // RCLCPP_INFO(this->get_logger(), "Received: '%s'", response.c_str());

    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    // boost::asio::io_service io;
    // boost::asio::serial_port serial;
    // boost::asio::streambuf response_buffer;
    int stepdownfreq;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_joint_state_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}