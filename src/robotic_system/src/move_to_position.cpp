/*
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

class JointTrajectoryControllerNode : public rclcpp::Node
{
public:
    JointTrajectoryControllerNode() : Node("publisher_scaled_joint_trajectory_controller")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {});

        // Create a publisher for the joint trajectory topic
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 1);

        // Create a timer to trigger the callback function every 10 seconds
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
        [this]() {
            // Construct a JointTrajectory message containing the joint configuration
            auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
            msg->joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
            point->positions = {double(1.45), double(-1.57), double(1.75), double(1.57), double(0.0), double(0.0)}; // Set the desired joint configuration here
            auto time_from_start = std::make_shared<builtin_interfaces::msg::Duration>();
            time_from_start->sec = int(10);
            msg->points.push_back(*point);

            // Publish the joint configuration
            pub_->publish(*msg);
        });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create the JointTrajectoryControllerNode
    auto node = std::make_shared<JointTrajectoryControllerNode>();

    // Spin the node until it is shutdown
    rclcpp::spin(node);

    // Shutdown the node
    rclcpp::shutdown();

    return 0;
}
*/

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <vector>

class JointTrajectoryControllerNode : public rclcpp::Node
{
public:
    JointTrajectoryControllerNode(const std::vector<double>& jointValues)
        : Node("publisher_scaled_joint_trajectory_controller")
    {
        // Create a publisher for the joint trajectory topic
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 1);

        // Create a timer to trigger the callback function every 12 seconds
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
            [this, jointValues]() {
                // Construct a JointTrajectory message containing the joint configuration
                auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
                msg->joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
                auto point = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
                point->positions = jointValues; // Set the desired joint configuration here
                auto time_from_start = std::make_shared<builtin_interfaces::msg::Duration>();
                time_from_start->sec = int(4);
                point->time_from_start = *time_from_start; // Set the duration for this configuration
                msg->points.push_back(*point);

                // Publish the joint configuration
                pub_->publish(*msg);

                rclcpp::shutdown();

            });
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Check if the joint values are provided as command-line arguments
    std::vector<double> jointValues;
    if (argc == 7)
    {
        for (int i = 1; i < argc; ++i)
        {
            jointValues.push_back(std::stod(argv[i]));
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid number of command-line arguments!");
        return 1;
    }

    // Create the JointTrajectoryControllerNode
    auto node = std::make_shared<JointTrajectoryControllerNode>(jointValues);

    // Spin the node until it is shutdown
    rclcpp::spin(node);

    // Shutdown the node
    rclcpp::shutdown();

    return 0;
}


