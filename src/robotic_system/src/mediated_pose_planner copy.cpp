#include "rclcpp/rclcpp.hpp"
#include "manos_openpose/msg/pose.hpp"  // Assuming manos_openpose package and Pose message exist
#include "moveit/move_group_interface/move_group_interface.h"

class MoveEnable : public rclcpp::Node {
public:
    MoveEnable() : Node("move_enable") {
        // Initialize MoveIt
        move_group_ = new moveit::planning_interface::MoveGroupInterface("arm");

        // Subscribe to the coordinates topic
        subscription_ = this->create_subscription<manos_openpose::msg::Pose>(
            "coords_topic", 10, std::bind(&MoveEnable::coordsCallback, this, std::placeholders::_1)
        );
    }

    ~MoveEnable() {
        delete move_group_;
    }

    void coordsCallback(const manos_openpose::msg::Pose::SharedPtr waypoints) {
        // Insert code here
    }

private:
    moveit::planning_interface::MoveGroupInterface* move_group_;
    rclcpp::Subscription<manos_openpose::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveEnable>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
