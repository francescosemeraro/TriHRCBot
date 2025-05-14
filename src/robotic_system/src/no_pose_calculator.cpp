#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "interface_msgs/msg/interaction_data.hpp"

class JointTrajectoryControllerNode : public rclcpp::Node
{
public:
    JointTrajectoryControllerNode()
        : Node("no_pose_calculator")
    {
        // Create a subscriber for the label topic
        sub_ = this->create_subscription<interface_msgs::msg::InteractionData>(
            "/converted_interaction_data", 1, [this](interface_msgs::msg::InteractionData::SharedPtr msg) {
                // Callback function to handle the received label value
                // Define the pre-recorded joint positions based on the label value

                if (msg->state == 7) {
                    RCLCPP_INFO(this->get_logger(), "Interaction data not received yet, keep listening...");
                }
                else if (msg->state == 1){
                    RCLCPP_INFO(this->get_logger(), "Both users currently working, no need to intervene");
                }
                else if ((msg->state == 0 && previous_state == 7) || (msg->state == 0 && previous_state == 6)){
                    //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -1.40};
                    jointValues = {3.81, -0.631, 1.48, -2.48, 1.47, -1.39};
                    RCLCPP_INFO(this->get_logger(), "Command to R-R pose sent");
                    publishJointConfiguration(jointValues, msg->state);
                }
                else if (msg->state == 2 && previous_state == 1){
                    //jointValues = {4.01, -0.169, 1.12, -2.53, 1.57, -1.66};
                    jointValues = {3.81, -0.646, 1.50, -2.44, 1.42, -1.55};
                    RCLCPP_INFO(this->get_logger(), "Command to first W-P pose sent");
                    publishJointConfiguration(jointValues, msg->state);
                    detected_2_first_time = true;
                }
                else if (msg->state == 3 && previous_state == 2) {
                    std_msgs::msg::Bool signal;
                    signal.data = true;
                    pub_2->publish(signal);
                    // Set timer for 5 seconds
                    timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = msg->state]() {
                        // After 5 seconds, set the joint configuration
                        //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -3.14};
                        jointValues = {3.83, -0.547, 1.29, -2.39, 1.49, -2.32};
                        RCLCPP_INFO(this->get_logger(), "Command to W-R pose sent");
                        publishJointConfiguration(jointValues, data);
                        timer_ = nullptr;
                    });
                }
                else if (msg->state == 4  && previous_state == 1) {
                    //jointValues = {3.91, -0.447, 1.13, -2.26, 1.55, -3.14};
                    jointValues = {3.83, -0.553, 1.29, -2.37, 1.51, -2.59};
                    RCLCPP_INFO(this->get_logger(), "Command to P-W pose sent");
                    publishJointConfiguration(jointValues, msg->state);
                }
                else if (msg->state == 5 && previous_state == 4){
                    std_msgs::msg::Bool signal;
                    signal.data = true;
                    pub_2->publish(signal);
                    // Set timer for 5 seconds
                    timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = msg->state]() {
                        // After 5 seconds, set the joint configuration
                        //jointValues = {3.92, -0.440, 1.13, -2.26, 1.55, -2.36};
                        jointValues = {3.86, -0.406, 0.978, -2.28, 1.56, -2.39};
                        RCLCPP_INFO(this->get_logger(), "Command to R-W pose sent");
                        publishJointConfiguration(jointValues, data);
                        timer_ = nullptr;
                    });
                }
                else if (msg->state == 2 && (previous_state == 1 || previous_state ==5) && detected_2_first_time){
                    //jointValues = {3.83, -0.494, 1.13, -2.21, 1.57, -1.72};
                    jointValues = {3.87, -0.405, 0.978, -2.28, 1.55, -1.40};
                    RCLCPP_INFO(this->get_logger(), "Command to second W-P pose sent");
                    publishJointConfiguration(jointValues, msg->state);
                }
                else if ((msg->state == 6 && (previous_state == 1 || previous_state == 2)) || (msg->state == 6 && previous_state == 4)){
                    RCLCPP_INFO(this->get_logger(), "Assembly completed, please revert to starting position");    
                }    
                else{
                    RCLCPP_INFO(this->get_logger(), "Received message not relatable to any interaction state known, please check for errors!");
                }
                previous_state =msg->state;
                /*
                switch (msg->state) {
                    case 0: // R-R
                        //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -1.40};
                        jointValues = {3.81, -0.631, 1.48, -2.48, 1.47, -1.39};
                        RCLCPP_INFO(this->get_logger(), "Command to R-R pose sent");
                        publishJointConfiguration(jointValues, msg->state);
                        break;
                    case 2: // P-W
                        if (case1Count == 0) {
                            //jointValues = {4.01, -0.169, 1.12, -2.53, 1.57, -1.66};
                            jointValues = {3.81, -0.646, 1.50, -2.44, 1.42, -1.55};
                            RCLCPP_INFO(this->get_logger(), "Command to first W-P pose sent");
                            publishJointConfiguration(jointValues, msg->state);
                        }
                        else if (case1Count == 1) {
                            //jointValues = {3.83, -0.494, 1.13, -2.21, 1.57, -1.72};
                            jointValues = {3.87, -0.405, 0.978, -2.28, 1.55, -1.40};
                            RCLCPP_INFO(this->get_logger(), "Command to second W-P pose sent");
                            publishJointConfiguration(jointValues, msg->state);
                        }
                        case1Count++;
                        break;
                    case 3: // R-W reactive
                        if (!timer_) {
                            std_msgs::msg::Bool signal;
                            signal.data = true;
                            pub_2->publish(signal);
                            // Set timer for 5 seconds
                            timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = msg->state]() {
                                // After 5 seconds, set the joint configuration
                                //jointValues = {3.75, -0.445, 1.13, -2.27, 1.17, -3.14};
                                jointValues = {3.83, -0.547, 1.29, -2.39, 1.49, -2.32};
                                RCLCPP_INFO(this->get_logger(), "Command to W-R pose sent");
                                publishJointConfiguration(jointValues, data);
                                timer_ = nullptr;
                            });
                        }
                        break;
                    case 4: // W-P 
                        //jointValues = {3.91, -0.447, 1.13, -2.26, 1.55, -3.14};
                        jointValues = {3.83, -0.553, 1.29, -2.37, 1.51, -2.59};
                        RCLCPP_INFO(this->get_logger(), "Command to P-W pose sent");
                        publishJointConfiguration(jointValues, msg->state);
                        break;
                    case 5: // W-R
                        if (!timer_) {
                            std_msgs::msg::Bool signal;
                            signal.data = true;
                            pub_2->publish(signal);
                            // Set timer for 5 seconds
                            timer_ = this->create_wall_timer(std::chrono::seconds(5), [this, data = msg->state]() {
                                // After 5 seconds, set the joint configuration
                                //jointValues = {3.92, -0.440, 1.13, -2.26, 1.55, -2.36};
                                jointValues = {3.86, -0.406, 0.978, -2.28, 1.56, -2.39};
                                RCLCPP_INFO(this->get_logger(), "Command to R-W pose sent");
                                publishJointConfiguration(jointValues, data);
                                timer_ = nullptr;
                            });
                        }
                        break;
                    case 6: // Default starting position
                        //jointValues = {3.93, -0.942, 2.18, -2.84, 1.57, -1.52};
                        //jointValues = {3.93, -0.942, 2.18, -2.84, 1.57, -1.52};
                        jointValues = {3.99, -0.777, 1.73, -2.54, 1.57, -1.65};
                        case1Count = 0;
                        RCLCPP_INFO(this->get_logger(), "Command to P-P pose sent");
                        publishJointConfiguration(jointValues, msg->state);
                        break;
                }*/
            });

        // Create a publisher for the joint trajectory topic
        pub_1 = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            //"/joint_trajectory_controller/joint_trajectory", 1);
            "/scaled_joint_trajectory_controller/joint_trajectory", 1);
        pub_2 = this->create_publisher<std_msgs::msg::Bool>(
            "/utter_message", 1);
        RCLCPP_INFO(this->get_logger(), "Waiting for converted interaction data...");
        
    }

private:
    rclcpp::Subscription<interface_msgs::msg::InteractionData>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_1;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_2;
    int case1Count = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> jointValues;
    int previous_state = 7;
    bool detected_2_first_time = false;


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


