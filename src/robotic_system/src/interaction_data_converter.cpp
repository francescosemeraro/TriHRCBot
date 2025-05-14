#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include "interface_msgs/msg/interaction_data.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class InteractionDataConverterNode : public rclcpp::Node
{
public:
    InteractionDataConverterNode()
        : Node("interaction_data_converter_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        
        //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Create a subscriber for the original body tracking data
        interaction_data_sub_ = create_subscription<interface_msgs::msg::InteractionData>(
            "/interaction_data", 10,
            [this](const interface_msgs::msg::InteractionData::SharedPtr msg) {
                // Callback function to handle the received body tracking data
                convertAndPublish(msg);
            });
        
        interaction_data_pub_ = create_publisher<interface_msgs::msg::InteractionData>(
            "/interaction_data_to_save", 10);

        // Create a publisher for the converted body tracking data
        converted_interaction_data_pub_ = create_publisher<interface_msgs::msg::InteractionData>(
            "/converted_interaction_data", 10);        // Create a publisher for the converted body tracking data
        marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/converted_interaction_markers", 10);
        // Create a publisher for the converted body tracking data
        //converted_pose_data_pub_ = create_publisher<geometry_msgs::msg::Pose>(
        //    "/converted_pose_data", 10);

    }

private:

    void initialise_pose(geometry_msgs::msg::Pose &pose, geometry_msgs::msg::Point p, geometry_msgs::msg::Quaternion q)
    {
        //RCLCPP_INFO(this->get_logger(), "In the callback: (x: {%f}, y: {%f}, z: {%f})", original_msg->p_right_user.x, original_msg->p_right_user.y, original_msg->p_right_user.z);
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        pose.orientation.x = q.x;
        pose.orientation.y = q.y;
        pose.orientation.z = q.z;
        pose.orientation.w = q.w;
    }

    void convertAndPublish(const interface_msgs::msg::InteractionData::SharedPtr original_msg)
    {

        // Replace with the target frame you want to convert to
        std::string target_frame = "base_link";

        // Transform each marker in the array
        interface_msgs::msg::InteractionData converted_msg;
        geometry_msgs::msg::Point dummy;

        converted_msg.state = original_msg->state;

        //RCLCPP_INFO(this->get_logger(), "Right Position Received: (x: {%f}, y: {%f}, z: {%f})", original_msg->spine_naval_right_user.x, original_msg->spine_naval_right_user.y, original_msg->spine_naval_right_user.z);
        //RCLCPP_INFO(this->get_logger(), "Right Orientation Received: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", original_msg->orientation_right_user.x, original_msg->orientation_right_user.y, original_msg->orientation_right_user.z, original_msg->orientation_right_user.w);
        //RCLCPP_INFO(this->get_logger(), "Left Position Received: (x: {%f}, y: {%f}, z: {%f})", original_msg->spine_naval_left_user.x, original_msg->spine_naval_left_user.y, original_msg->spine_naval_left_user.z);
        //RCLCPP_INFO(this->get_logger(), "Left Orientation Received: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", original_msg->orientation_left_user.x, original_msg->orientation_left_user.y, original_msg->orientation_left_user.z, original_msg->orientation_left_user.w);

        geometry_msgs::msg::Pose right_converted_point;
        geometry_msgs::msg::Pose left_converted_point;
        geometry_msgs::msg::Pose right_dummy_pose;
        geometry_msgs::msg::Pose left_dummy_pose;
        

        
        
        initialise_pose(right_dummy_pose,  original_msg->neck_right_user, original_msg->orientation_right_user); //We are passing a pseudo-pose for efficiency

        //RCLCPP_INFO(this->get_logger(), "Right Position Assigned: (x: {%f}, y: {%f}, z: {%f})", dummy_pose.position.x, dummy_pose.position.y, dummy_pose.position.z);
        //RCLCPP_INFO(this->get_logger(), "Right Orientation Assigned: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", dummy_pose.orientation.x, dummy_pose.orientation.y, dummy_pose.orientation.z, dummy_pose.orientation.w);
        
        transformedPose(right_dummy_pose, original_msg->header.frame_id, target_frame, right_converted_point);
        
        converted_msg.neck_right_user.x = right_converted_point.position.x;
        converted_msg.neck_right_user.y = right_converted_point.position.y;
        converted_msg.neck_right_user.z = right_converted_point.position.z;
        
        converted_msg.orientation_right_user.x = right_converted_point.orientation.x;
        converted_msg.orientation_right_user.y = right_converted_point.orientation.y;
        converted_msg.orientation_right_user.z = right_converted_point.orientation.z;
        converted_msg.orientation_right_user.w = right_converted_point.orientation.w;
        
        initialise_pose(right_dummy_pose,  original_msg->spine_naval_right_user, original_msg->orientation_right_user); //We are passing a pseudo-pose for efficiency

        //RCLCPP_INFO(this->get_logger(), "Right Position Assigned: (x: {%f}, y: {%f}, z: {%f})", dummy_pose.position.x, dummy_pose.position.y, dummy_pose.position.z);
        //RCLCPP_INFO(this->get_logger(), "Right Orientation Assigned: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", dummy_pose.orientation.x, dummy_pose.orientation.y, dummy_pose.orientation.z, dummy_pose.orientation.w);
        
        transformedPose(right_dummy_pose, original_msg->header.frame_id, target_frame, right_converted_point);
        
        converted_msg.spine_naval_right_user.x = right_converted_point.position.x;
        converted_msg.spine_naval_right_user.y = right_converted_point.position.y;
        converted_msg.spine_naval_right_user.z = right_converted_point.position.z;
        
        
        initialise_pose(left_dummy_pose,  original_msg->neck_left_user, original_msg->orientation_left_user); //We are passing a pseudo-pose for efficiency
        
        //RCLCPP_INFO(this->get_logger(), "Left Position Assigned: (x: {%f}, y: {%f}, z: {%f})", dummy_pose.position.x, dummy_pose.position.y, dummy_pose.position.z);
        //RCLCPP_INFO(this->get_logger(), "Left Orientation Assigned: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", dummy_pose.orientation.x, dummy_pose.orientation.y, dummy_pose.orientation.z, dummy_pose.orientation.w);
        
        transformedPose(left_dummy_pose, original_msg->header.frame_id, target_frame, left_converted_point);
        
        converted_msg.neck_left_user.x = left_converted_point.position.x;
        converted_msg.neck_left_user.y = left_converted_point.position.y;
        converted_msg.neck_left_user.z = left_converted_point.position.z;

        converted_msg.orientation_left_user.x = left_converted_point.orientation.x;
        converted_msg.orientation_left_user.y = left_converted_point.orientation.y;
        converted_msg.orientation_left_user.z = left_converted_point.orientation.z;
        converted_msg.orientation_left_user.w = left_converted_point.orientation.w;

        initialise_pose(left_dummy_pose,  original_msg->spine_naval_left_user, original_msg->orientation_left_user); //We are passing a pseudo-pose for efficiency
        
        //RCLCPP_INFO(this->get_logger(), "Left Position Assigned: (x: {%f}, y: {%f}, z: {%f})", dummy_pose.position.x, dummy_pose.position.y, dummy_pose.position.z);
        //RCLCPP_INFO(this->get_logger(), "Left Orientation Assigned: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", dummy_pose.orientation.x, dummy_pose.orientation.y, dummy_pose.orientation.z, dummy_pose.orientation.w);
        
        transformedPose(left_dummy_pose, original_msg->header.frame_id, target_frame, left_converted_point);
        
        converted_msg.spine_naval_left_user.x = left_converted_point.position.x;
        converted_msg.spine_naval_left_user.y = left_converted_point.position.y;
        converted_msg.spine_naval_left_user.z = left_converted_point.position.z;

        
        converted_msg.shoulder_left_user.x = original_msg->shoulder_left_user.x;
        converted_msg.shoulder_left_user.y = original_msg->shoulder_left_user.y;
        converted_msg.shoulder_left_user.z = original_msg->shoulder_left_user.z;

        converted_msg.elbow_left_user.x = original_msg->elbow_left_user.x;
        converted_msg.elbow_left_user.y = original_msg->elbow_left_user.y;
        converted_msg.elbow_left_user.z = original_msg->elbow_left_user.z;

        converted_msg.wrist_left_user.x = original_msg->wrist_left_user.x;
        converted_msg.wrist_left_user.y = original_msg->wrist_left_user.y;
        converted_msg.wrist_left_user.z = original_msg->wrist_left_user.z;

        converted_msg.hand_left_user.x = original_msg->hand_left_user.x;
        converted_msg.hand_left_user.y = original_msg->hand_left_user.y;
        converted_msg.hand_left_user.z = original_msg->hand_left_user.z;

        converted_msg.handtip_left_user.x = original_msg->handtip_left_user.x;
        converted_msg.handtip_left_user.y = original_msg->handtip_left_user.y;
        converted_msg.handtip_left_user.z = original_msg->handtip_left_user.z;


        converted_msg.shoulder_right_user.x = original_msg->shoulder_right_user.x;
        converted_msg.shoulder_right_user.y = original_msg->shoulder_right_user.y;
        converted_msg.shoulder_right_user.z = original_msg->shoulder_right_user.z;

        converted_msg.elbow_right_user.x = original_msg->elbow_right_user.x;
        converted_msg.elbow_right_user.y = original_msg->elbow_right_user.y;
        converted_msg.elbow_right_user.z = original_msg->elbow_right_user.z;

        converted_msg.wrist_right_user.x = original_msg->wrist_right_user.x;
        converted_msg.wrist_right_user.y = original_msg->wrist_right_user.y;
        converted_msg.wrist_right_user.z = original_msg->wrist_right_user.z;

        converted_msg.hand_right_user.x = original_msg->hand_right_user.x;
        converted_msg.hand_right_user.y = original_msg->hand_right_user.y;
        converted_msg.hand_right_user.z = original_msg->hand_right_user.z;

        converted_msg.handtip_right_user.x = original_msg->handtip_right_user.x;
        converted_msg.handtip_right_user.y = original_msg->handtip_right_user.y;
        converted_msg.handtip_right_user.z = original_msg->handtip_right_user.z;
        
        converted_msg.header.frame_id = target_frame;
        
        RCLCPP_INFO(this->get_logger(), "Publishing converted interaction data"); 
        //RCLCPP_INFO(this->get_logger(), "Right Position Converted: (x: {%f}, y: {%f}, z: {%f})", converted_msg.spine_naval_right_user.x, converted_msg.spine_naval_right_user.y, converted_msg.spine_naval_right_user.z);
        //RCLCPP_INFO(this->get_logger(), "Right Orientation Converted: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", converted_msg.orientation_right_user.x, converted_msg.orientation_right_user.y, converted_msg.orientation_right_user.z, converted_msg.orientation_right_user.w);
        //RCLCPP_INFO(this->get_logger(), "Left Position Converted: (x: {%f}, y: {%f}, z: {%f})", converted_msg.spine_naval_left_user.x, converted_msg.spine_naval_left_user.y, converted_msg.spine_naval_left_user.z);
        //RCLCPP_INFO(this->get_logger(), "Left Orientation Converted: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", converted_msg.orientation_left_user.x, converted_msg.orientation_left_user.y, converted_msg.orientation_left_user.z, converted_msg.orientation_left_user.w);

        auto now = this->get_clock()->now();
        converted_msg.header.stamp = now;
        converted_interaction_data_pub_->publish(converted_msg);

        // Create a MarkerArray to store markers
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker right_marker;
        right_marker.header = converted_msg.header;
        right_marker.header.frame_id = target_frame;
        right_marker.ns = "right_user";
        right_marker.id = 0;
        right_marker.type = visualization_msgs::msg::Marker::SPHERE;
        right_marker.action = visualization_msgs::msg::Marker::ADD;
        right_marker.pose.position.x = right_converted_point.position.x; // Replace with the actual pose for the right user
        right_marker.pose.position.y = right_converted_point.position.y; // Replace with the actual pose for the right user
        right_marker.pose.position.z = right_converted_point.position.z; // Replace with the actual pose for the right user
        //right_marker.pose.orientation = converted_point.orientation; // Replace with the actual pose for the right user
        right_marker.scale.x = 0.07; // Adjust the scale as needed
        right_marker.scale.y = 0.07;
        right_marker.scale.z = 0.07;
        right_marker.color.r = 1.0; // Color for the right user (red)
        right_marker.color.a = 1.0; // Fully opaque
        right_marker.lifetime = rclcpp::Duration::from_seconds(7.0); // Set the lifetime of the marker (optional)
        marker_array.markers.push_back(right_marker);


        visualization_msgs::msg::Marker left_marker;
        left_marker.header = converted_msg.header;
        left_marker.header.frame_id = target_frame;
        left_marker.ns = "left_user";
        left_marker.id = 1;
        left_marker.type = visualization_msgs::msg::Marker::SPHERE;
        left_marker.action = visualization_msgs::msg::Marker::ADD;
        left_marker.pose.position.x = left_converted_point.position.x; // Replace with the actual pose for the left user
        left_marker.pose.position.y = left_converted_point.position.y; // Replace with the actual pose for the left user
        left_marker.pose.position.z = left_converted_point.position.z; // Replace with the actual pose for the left user
        left_marker.scale.x = 0.07; // Adjust the scale as needed
        left_marker.scale.y = 0.07;
        left_marker.scale.z = 0.07;
        left_marker.color.r = 1.0; // Color for the left user (green)
        left_marker.color.a = 1.0; // Fully opaque
        left_marker.lifetime = rclcpp::Duration::from_seconds(7.0); // Set the lifetime of the marker (optional)

        marker_array.markers.push_back(left_marker);
        
        // Publish the MarkerArray
        marker_array_pub_->publish(marker_array);
        //}
        

    }


    geometry_msgs::msg::Quaternion rotate_frame_axis(geometry_msgs::msg::Quaternion& original_quaternion) {
        // Extract components from original quaternion
        double w = original_quaternion.w;
        double x = original_quaternion.x;
        double y = original_quaternion.y;
        double z = original_quaternion.z;

        // Create quaternion for desired rotation
        double x_rot =0.0;
        double y_rot =0.0;
        double z_rot =sin(M_PI / 4);
        double w_rot =cos(M_PI / 4);

        // Perform quaternion multiplication using component-wise operations
        double wr = w_rot * w - x_rot * x - y_rot * y - z_rot * z;
        double xr = w_rot * x + x_rot * w + y_rot * z - z_rot * y;
        double yr = w_rot * y - x_rot * z + y_rot * w + z_rot * x;
        double zr = w_rot * z + x_rot * y - y_rot * x + z_rot * w;
        // Return the rotated quaternion

        geometry_msgs::msg::Quaternion rotated_quaternion;

        rotated_quaternion.x = xr;
        rotated_quaternion.y = yr;
        rotated_quaternion.z = zr;
        rotated_quaternion.w = wr;

        return rotated_quaternion;
    }

    geometry_msgs::msg::Quaternion rotateQuaternionToAxis(const geometry_msgs::msg::Quaternion& input_quaternion, const std::string& axis) {
        // Convert input quaternion to tf2 quaternion
        tf2::Quaternion tf2_quaternion;
        tf2::fromMsg(input_quaternion, tf2_quaternion);

        // Extract roll, pitch, and yaw from the input quaternion
        tf2::Matrix3x3 matrix(tf2_quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        // Create a quaternion to represent the desired rotation
        tf2::Quaternion rotation_quaternion;

        // Choose the desired axis
        if (axis == "x") {
            roll = 0.0;
        } else if (axis == "y") {
            pitch = 0.0;
        } else if (axis == "z") {
            // No need to rotate for Z-axis
            return input_quaternion;
        } else {
            // Invalid axis, return the original quaternion
            return input_quaternion;
        }

        // Set the new roll, pitch, and yaw
        rotation_quaternion.setRPY(roll, pitch, yaw);

        // Rotate the original quaternion
        tf2_quaternion *= rotation_quaternion;

        // Convert back to geometry_msgs Quaternion
        geometry_msgs::msg::Quaternion rotated_quaternion;
        rotated_quaternion.x = tf2_quaternion.x();
        rotated_quaternion.y = tf2_quaternion.y();
        rotated_quaternion.z = tf2_quaternion.z();
        rotated_quaternion.w = tf2_quaternion.w();

        return rotated_quaternion;
    }

    void transformedPose(geometry_msgs::msg::Pose &input_pose,
                         std::string &original_frame,
                         std::string &target_frame,
                         geometry_msgs::msg::Pose &output_pose)
        {
            try
            {
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(target_frame, original_frame, this->now() - rclcpp::Duration::from_seconds(0.1));

                // Transform the pose using tf2
                tf2::doTransform(input_pose, output_pose, transform_stamped); 

            }
            catch (tf2::TransformException &ex)
            {
                // Handle exceptions (e.g., transform not available)
                RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
            }
        }

    
    
    
    rclcpp::Subscription<interface_msgs::msg::InteractionData>::SharedPtr interaction_data_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<interface_msgs::msg::InteractionData>::SharedPtr converted_interaction_data_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr converted_pose_data_pub_;
    rclcpp::Publisher<interface_msgs::msg::InteractionData>::SharedPtr interaction_data_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InteractionDataConverterNode>());
    rclcpp::shutdown();
    return 0;
}
