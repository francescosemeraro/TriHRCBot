#include "rclcpp/rclcpp.hpp"
#include "interface_msgs/msg/classification_data.hpp"
#include "interface_msgs/msg/robot_performance.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rosbag2_cpp/writer.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DataRecorderNode : public rclcpp::Node
{
public:
    DataRecorderNode()
        : Node("data_recorder_node") 
    {
        this->declare_parameter("bag_file_name", "sample");
        std::string bag_file_name = this->get_parameter("bag_file_name").as_string();
        this->declare_parameter("selected_topic", "sample");
        std::string selected_topic_ = this->get_parameter("selected_topic").as_string();

        // Create the bag writer
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        

        // Create the subscriber based on the selected topic
        if (selected_topic_ == "/classification_data") {
            writer_->open(bag_file_name + "_classification_data");
            classification_data_subscriber_ = create_subscription<interface_msgs::msg::ClassificationData>(
                selected_topic_, 10, std::bind(&DataRecorderNode::handleClassificationData, this, _1));
            RCLCPP_INFO(get_logger(), "Correctly subscribed to %s", selected_topic_.c_str());
        } else if (selected_topic_ == "/robot_performance") {
            writer_->open(bag_file_name + "_robot_performance");
            robot_performance_subscriber_ = create_subscription<interface_msgs::msg::RobotPerformance>(
                selected_topic_, 10, std::bind(&DataRecorderNode::handleRobotPerformance, this, _1));
            RCLCPP_INFO(get_logger(), "Correctly subscribed to %s", selected_topic_.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Invalid topic selected, received name %s", selected_topic_.c_str());
            return;
        }

    }

private:
    void handleRobotPerformance(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();
        writer_->write(msg, "robot_performance", "interface_msgs/msg/RobotPerformance", time_stamp);
    }

    void handleClassificationData(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();
        writer_->write(msg, "classification_data", "interface_msgs/msg/ClassificationData", time_stamp);
    }

    rclcpp::Subscription<interface_msgs::msg::ClassificationData>::SharedPtr classification_data_subscriber_;
    rclcpp::Subscription<interface_msgs::msg::RobotPerformance>::SharedPtr robot_performance_subscriber_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::string selected_topic_;
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("DataRecorderNode"), "Topic argument missing. Usage: ./your_executable <topic>");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorderNode>());
    rclcpp::shutdown();

    return 0;
}
