#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstdlib>

class SpeechNode : public rclcpp::Node
{
public:
    SpeechNode() : Node("speech_node")
    {
        // Create a subscriber for the /utter_message topic
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/utter_message", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                // When a true message is received, call the speak() function
                if (msg->data) {
                    RCLCPP_INFO(this->get_logger(), "Voice message sent");
                    speak("About to move");
                }
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

    void speak(const std::string &message)
    {
        // Use the espeak command to speak the given message
        std::string command = "espeak \"" + message + "\"";
        std::system(command.c_str());
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create the SpeechNode
    auto node = std::make_shared<SpeechNode>();

    // Spin the node until it is shutdown
    rclcpp::spin(node);

    // Clean up and shutdown
    rclcpp::shutdown();

    return 0;
}
