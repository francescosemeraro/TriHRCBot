#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "interface_msgs/msg/converted_datapoint.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

class BodyTrackingConverterNode : public rclcpp::Node
{
public:
    BodyTrackingConverterNode()
        : Node("body_tracking_converter_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        
        //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Create a subscriber for the original body tracking data
        body_tracking_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/body_tracking_data", 10,
            [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                // Callback function to handle the received body tracking data
                convertAndPublish(msg);
            });

        // Create a publisher for the converted body tracking data
        converted_body_tracking_pub_ = create_publisher<interface_msgs::msg::ConvertedDatapoint>(
            "/converted_body_tracking_data", 10);
    }

private:
    void convertAndPublish(const visualization_msgs::msg::MarkerArray::SharedPtr original_msg)
    {
        // Replace with the target frame you want to convert to
        std::string target_frame = "base_link";

        // Transform each marker in the array
        interface_msgs::msg::ConvertedDatapoint converted_msg;
        for (const auto &marker : original_msg->markers)
        {
            visualization_msgs::msg::Marker converted_marker = marker;
            transformedPose(marker, target_frame, converted_marker.pose);
            converted_marker.header.frame_id = target_frame;

            converted_msg.marker_array.markers.push_back(converted_marker);
        }

        if (original_msg->markers[0].pose.position.x >=0){
            converted_msg.side = "Right";
        }
        else{
            converted_msg.side = "Left";
        }
        // Publish the converted body tracking data
        converted_body_tracking_pub_->publish(converted_msg);
    }

    void transformedPose(const visualization_msgs::msg::Marker &input_pose,
                         const std::string &target_frame,
                         geometry_msgs::msg::Pose &output_pose)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(target_frame, input_pose.header.frame_id, rclcpp::Time(0));

            // Transform the pose using tf2
            tf2::doTransform(input_pose.pose, output_pose, transform_stamped);

        }
        catch (tf2::TransformException &ex)
        {
            // Handle exceptions (e.g., transform not available)
            RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
        }
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr body_tracking_sub_;
    rclcpp::Publisher<interface_msgs::msg::ConvertedDatapoint>::SharedPtr converted_body_tracking_pub_;

    //tf2_ros::Buffer tf_buffer_;
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyTrackingConverterNode>());
    rclcpp::shutdown();
    return 0;
}
