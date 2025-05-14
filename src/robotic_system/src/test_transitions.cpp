#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class JointTrajectoryControllerNode : public rclcpp::Node
{
public:
    JointTrajectoryControllerNode()
        : Node("joint_trajectory_controller_node")
    {
        this->declare_parameter("condition", "not set"); //CHANGE BACK
        condition_ = this->get_parameter("condition").as_string();
        std::string message = "Condition is ";
        RCLCPP_INFO(this->get_logger(), (message + condition_).c_str());

        // Create a subscriber for the label topic
        sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/label", 1, [this](const std_msgs::msg::Int32::SharedPtr label) {
                // Callback function to handle the received label value
                // Define the pre-recorded joint positions based on the label value
                switch (label->data) {
                    case 0: // R-R
                        //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -1.40};
                        //jointValues = {3.81, -0.631, 1.48, -2.48, 1.47, -1.39};
                        jointValues = {3.96, -0.499, 1.18, -2.29, 1.46, -1.54};
                        RCLCPP_INFO(this->get_logger(), "Command to R-R pose sent");
                        publishJointConfiguration(jointValues, label->data);
                        break;
                    case 2: // W-P
                        if (case1Count == 0) {
                            //jointValues = {4.01, -0.169, 1.12, -2.53, 1.57, -1.66};
                            //jointValues = {3.81, -0.646, 1.50, -2.44, 1.42, -1.55};
                            jointValues = {3.94, -0.598, 1.38, -2.35, 1.41, -1.68};
                            RCLCPP_INFO(this->get_logger(), "Command to first W-P pose sent");
                            publishJointConfiguration(jointValues, label->data);
                        }
                        else if (case1Count == 1) {
                            //jointValues = {4.09, -0.494, 1.13, -2.21, 1.57, -1.72};
                            jointValues = {3.87, -0.405, 0.978, -2.28, 1.55, -1.40}; //MEASURE A NEW ONE IF YOU HAVE THE TIME
                            RCLCPP_INFO(this->get_logger(), "Command to second W-P pose sent");
                            publishJointConfiguration(jointValues, label->data);
                        }
                        case1Count++;
                        break;
                    case 3: // W-R reactive
                        if (condition_ == "adaptive") {
                            std_msgs::msg::Bool signal;
                            signal.data = true;
                            pub_2->publish(signal);
                            if (!timer_) {
                                // Set timer for 5 seconds
                                timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = label->data]() {
                                    // After 5 seconds, set the joint configuration
                                    //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -3.14};
                                    //jointValues = {3.83, -0.547, 1.29, -2.39, 1.49, -2.32};
                                    jointValues = {3.91, -0.53, 1.25, -2.36, 1.48, -2.4};
                                    RCLCPP_INFO(this->get_logger(), "Command to W-R pose sent");
                                    publishJointConfiguration(jointValues, data);
                                    timer_ = nullptr;
                                });
                            }
                        }
                        else {
                            //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -3.14};
                            //jointValues = {3.83, -0.547, 1.29, -2.39, 1.49, -2.32};
                            jointValues = {3.91, -0.53, 1.25, -2.36, 1.48, -2.4};
                            RCLCPP_INFO(this->get_logger(), "Command to W-R pose sent");
                            publishJointConfiguration(jointValues, label->data);
                        }
                        break;
                    case 4: // P-W 
                        //jointValues = {3.91, -0.447, 1.13, -2.26, 1.55, -3.14};
                        //jointValues = {3.83, -0.553, 1.29, -2.37, 1.51, -2.59};
                        jointValues = {3.96, -0.589, 1.37, -2.4, 1.5, -2.72};
                        RCLCPP_INFO(this->get_logger(), "Command to P-W pose sent");
                        publishJointConfiguration(jointValues, label->data);
                        break;
                    case 5: // R-W
                        if (condition_ == "adaptive") {
                            if (!timer_) {
                                std_msgs::msg::Bool signal;
                                signal.data = true;
                                pub_2->publish(signal);
                                // Set timer for 5 seconds
                                timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = label->data]() {
                                    // After 5 seconds, set the joint configuration
                                    //jointValues = {3.92, -0.440, 1.13, -2.26, 1.55, -2.36};
                                    //jointValues = {3.86, -0.406, 0.978, -2.28, 1.56, -2.39};
                                    jointValues = {3.98, -0.507, 1.2, -2.4, 1.54, -2.5};
                                    RCLCPP_INFO(this->get_logger(), "Command to R-W pose sent");
                                    publishJointConfiguration(jointValues, data);
                                    timer_ = nullptr;
                                });
                            }
                        }
                        else {
                            //jointValues = {3.92, -0.440, 1.13, -2.26, 1.55, -2.36};
                            //jointValues = {3.86, -0.406, 0.978, -2.28, 1.56, -2.39};
                            jointValues = {3.98, -0.507, 1.2, -2.4, 1.54, -2.5};
                            RCLCPP_INFO(this->get_logger(), "Command to R-W pose sent");
                            publishJointConfiguration(jointValues, label->data);
                        }
                        break;
                    case 6: // Default starting position
                        //jointValues = {3.93, -0.942, 2.18, -2.84, 1.57, -1.52};
                        jointValues = {3.99, -0.777, 1.73, -2.54, 1.57, -1.65};
                        case1Count = 0;
                        RCLCPP_INFO(this->get_logger(), "Command to P-P pose sent");
                        publishJointConfiguration(jointValues, label->data);
                        break;
                }
            });

        // Create a publisher for the joint trajectory topic
        pub_1 = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 1);
            //"/joint_trajectory_controller/joint_trajectory", 1);
        pub_2 = this->create_publisher<std_msgs::msg::Bool>(
            "/utter_message", 1);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_1;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_2;
    int case1Count = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string condition_;
    std::vector<double> jointValues;


    void publishJointConfiguration(const std::vector<double> jointValues, int32_t data)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names =  {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = jointValues;
        builtin_interfaces::msg::Duration time_from_start;
        time_from_start.sec = 4;
        point.time_from_start = time_from_start;
        msg.points.push_back(point);
        if (data != 1){
            pub_1->publish(msg);
        }
    }
};

int main(int argc, char* argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create the JointTrajectoryControllerNode
    auto node = std::make_shared<JointTrajectoryControllerNode>();

    // Spin the node until it is shutdown
    rclcpp::spin(node);

    return 0;
}


