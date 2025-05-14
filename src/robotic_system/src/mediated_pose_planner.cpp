#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <builtin_interfaces/msg/time.hpp>
#include "interface_msgs/msg/robot_performance.hpp"


using std::string;


class MoveItInitialiser : public rclcpp::Node {

public:    
    /// Constructor
    explicit MoveItInitialiser(const rclcpp::NodeOptions &options);
    

private:

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void load_planning_scene();
    void setPathConstraints();
    int checkAndAdjustPathConstraints(moveit_msgs::msg::RobotTrajectory& trajectory);
    bool adjustWaypoint(trajectory_msgs::msg::JointTrajectoryPoint& point);

    geometry_msgs::msg::PoseStamped getPrecedingFramePose(const std::string target_frame, 
                                            const geometry_msgs::msg::PoseStamped::SharedPtr target_pose);
    visualization_msgs::msg::Marker waypoints_to_markers(geometry_msgs::msg::PoseStamped& waypoints);
    
    /// Move group interface for the robot
    /// Subscriber for target pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    /// Target pose that is used to detect changes
    geometry_msgs::msg::Pose previous_target_pose_;
    rclcpp::Publisher<interface_msgs::msg::RobotPerformance>::SharedPtr publisher_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
    moveit_msgs::msg::Constraints path_constraints_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

};

void MoveItInitialiser::load_planning_scene() {
    
    auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Create collision object for the robot to avoid
    auto const scene = [frame_id =
                                    "base_link"] {
        
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive0;


        // Define the size of the box in meters
        primitive0.type = primitive0.BOX;
        primitive0.dimensions.resize(3);
        primitive0.dimensions[primitive0.BOX_X] = 1.0;
        primitive0.dimensions[primitive0.BOX_Y] = 1.7;
        primitive0.dimensions[primitive0.BOX_Z] = 0.92;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose base_pose;
        base_pose.orientation.w = 0.924;
        base_pose.orientation.z = 0.383;
        base_pose.position.x = 0.015;
        base_pose.position.y = 0.572;
        base_pose.position.z = -0.465;

        collision_object.primitives.push_back(primitive0);
        collision_object.primitive_poses.push_back(base_pose);
        collision_object.operation = collision_object.ADD;

        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX; 
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.01;
        primitive.dimensions[primitive.BOX_Y] = 1.0;
        primitive.dimensions[primitive.BOX_Z] = 0.8;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose camera_barrier;
        camera_barrier.orientation.w = 0.924;
        camera_barrier.orientation.z = 0.383;
        camera_barrier.position.x = 0.3;
        camera_barrier.position.y = 0.1;
        camera_barrier.position.z = 0.4;

        //collision_object.primitives.push_back(primitive);
        //collision_object.primitive_poses.push_back(camera_barrier);
        collision_object.operation = collision_object.ADD;

        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive2;
    
        primitive2.type = primitive2.BOX; 
        primitive2.dimensions.resize(3);
        primitive2.dimensions[primitive2.BOX_X] = 1.7;
        primitive2.dimensions[primitive2.BOX_Y] = 0.01;
        primitive2.dimensions[primitive2.BOX_Z] = 1.8;


        geometry_msgs::msg::Pose left_wall;
        left_wall.orientation.w = 0.924;
        left_wall.orientation.z = 0.383;
        left_wall.position.x = 0.0;
        //left_wall.position.y = -0.65;
        left_wall.position.y = -0.8;
        left_wall.position.z = 0.0;

        collision_object.primitives.push_back(primitive2);
        collision_object.primitive_poses.push_back(left_wall);

        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive3;
    
        primitive3.type = primitive3.BOX; 
        primitive3.dimensions.resize(3);
        primitive3.dimensions[primitive3.BOX_X] = 1.7;
        primitive3.dimensions[primitive3.BOX_Y] = 0.01;
        primitive3.dimensions[primitive3.BOX_Z] = 1.8;


        geometry_msgs::msg::Pose right_wall;
        right_wall.orientation.w = 0.924;
        right_wall.orientation.z = 0.383;
        //right_wall.position.x = -0.55;
        right_wall.position.x = -0.561;
        //right_wall.position.y = -0.05;
        right_wall.position.y = -0.25;
        right_wall.position.z = 0.0;

        collision_object.primitives.push_back(primitive3);
        collision_object.primitive_poses.push_back(right_wall);
        
        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive4;
    
        primitive4.type = primitive4.BOX; 
        primitive4.dimensions.resize(3);
        primitive4.dimensions[primitive4.BOX_X] = 1.3;
        primitive4.dimensions[primitive4.BOX_Y] = 0.8;
        primitive4.dimensions[primitive4.BOX_Z] = 0.01;


        geometry_msgs::msg::Pose upper_wall;
        upper_wall.orientation.w = 0.924;
        upper_wall.orientation.z = 0.383;
        upper_wall.position.x = -0.17;
        upper_wall.position.y = -0.267;
        upper_wall.position.z = 0.9;

        //collision_object.primitives.push_back(primitive4);
        //collision_object.primitive_poses.push_back(upper_wall);
        
        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive5;
    
        primitive5.type = primitive5.BOX; 
        primitive5.dimensions.resize(3);
        primitive5.dimensions[primitive5.BOX_X] = 0.8;
        primitive5.dimensions[primitive5.BOX_Y] = 0.8;
        primitive5.dimensions[primitive5.BOX_Z] = 0.08;


        geometry_msgs::msg::Pose table;
        table.orientation.w = 0.924;
        table.orientation.z = 0.383;
        table.position.x = -0.3;
        table.position.y = -0.4;
        table.position.z = -0.19;

        //collision_object.primitives.push_back(primitive5);
        //collision_object.primitive_poses.push_back(table);
        
        collision_object.operation = collision_object.ADD;

        
        return collision_object;
    }();

    // Create collision object to attach to the robot to consider the target object 
    auto const target_object = [frame_id =
                                    "tool0"] {
        moveit_msgs::msg::AttachedCollisionObject attached_collision_object;

        attached_collision_object.link_name = frame_id;

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "target_object";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the object of interest in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.3;

        // Define the pose of the object of interest (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.315;//0.276;

        // Define the subframe of the box
        collision_object.subframe_names.resize(1);
        collision_object.subframe_poses.resize(1);
        
        geometry_msgs::msg::Pose subpose;
        subpose.position.x = 0.0;
        subpose.position.y = 0.0;
        subpose.position.z = 0.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        // Define the size of the gripper in meters
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.16;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.035;

        // Define the pose of the gripper (relative to the frame_id)
        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.orientation.w = 0.0;//0.707;
        cylinder_pose.orientation.x = 0.0;
        cylinder_pose.orientation.y = 0.0;//0.707;
        cylinder_pose.orientation.z = 0.0;
        cylinder_pose.position.x = 0.0;
        cylinder_pose.position.y = 0.0;
        cylinder_pose.position.z = 0.085;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cylinder_pose);

        attached_collision_object.object = collision_object;

        attached_collision_object.weight = 2.0;
        
        return attached_collision_object;
    }();


    // Apply objects to the planning scene
    //planning_scene_interface->applyCollisionObject(scene);
    planning_scene_interface->applyAttachedCollisionObject(target_object);
    RCLCPP_INFO(this->get_logger(), "Initialised scene!");
}

void MoveItInitialiser::setPathConstraints() {
    
    // Define joint constraints
    moveit_msgs::msg::JointConstraint joint_constraint;

    joint_constraint.joint_name = "shoulder_pan_joint";
    joint_constraint.position = 3.925;
    joint_constraint.tolerance_above = 0.52;
    joint_constraint.tolerance_below = 0.52;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "shoulder_lift_joint";
    joint_constraint.position = -0.804;
    joint_constraint.tolerance_above = 1.6;
    joint_constraint.tolerance_below = 0.174;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);
    
    joint_constraint.joint_name = "elbow_joint";
    joint_constraint.position = 1.81;
    joint_constraint.tolerance_above = 0.785;
    joint_constraint.tolerance_below = 2;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "wrist_1_joint";
    joint_constraint.position = -2.63;
    joint_constraint.tolerance_above = 0.785;
    joint_constraint.tolerance_below = 1.57;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "wrist_2_joint";
    joint_constraint.position = 1.57;
    joint_constraint.tolerance_above = 1.57;
    joint_constraint.tolerance_below = 1.57;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "wrist_3_joint";
    joint_constraint.position = -1.57;
    joint_constraint.tolerance_above = 2.355;
    joint_constraint.tolerance_below = 2.355;
    joint_constraint.weight = 1.0;
    path_constraints_.joint_constraints.push_back(joint_constraint);



    //move_group_interface_->setPathConstraints(path_constraints_);
}

int MoveItInitialiser::checkAndAdjustPathConstraints(moveit_msgs::msg::RobotTrajectory& trajectory){

    int outcome = 1;

    //RCLCPP_INFO(this->get_logger(), "Joint limits: %f %f %f %f %f %f", path_constraints_.joint_constraints[0].position, path_constraints_.joint_constraints[1].position, path_constraints_.joint_constraints[2].position, 
    //path_constraints_.joint_constraints[3].position, path_constraints_.joint_constraints[4].position, path_constraints_.joint_constraints[5].position);

    for (int i = 0; i < int(trajectory.joint_trajectory.points.size()); ++i)
    {
      auto& point = trajectory.joint_trajectory.points[i];
      //RCLCPP_INFO(this->get_logger(), "Joint values: %f %f %f %f %f %f", trajectory.joint_trajectory.points[i].positions[0], trajectory.joint_trajectory.points[i].positions[1], 
      //trajectory.joint_trajectory.points[i].positions[2], trajectory.joint_trajectory.points[i].positions[3], trajectory.joint_trajectory.points[i].positions[4], trajectory.joint_trajectory.points[i].positions[5]);

      for (int j = 0; j < 6; j++)// : joint_positions)
      {
          if (point.positions[j] > path_constraints_.joint_constraints[j].position + path_constraints_.joint_constraints[j].tolerance_above){
            point.positions[j] = path_constraints_.joint_constraints[j].position + path_constraints_.joint_constraints[j].tolerance_above - 0.1;
            //RCLCPP_WARN(this->get_logger(), "Corrected %i-th joint of %i-th waypoint", j+1, i+1);
            outcome = 2;
          }
          else if (point.positions[j] < path_constraints_.joint_constraints[j].position - path_constraints_.joint_constraints[j].tolerance_below){
            point.positions[j] = path_constraints_.joint_constraints[j].position - path_constraints_.joint_constraints[j].tolerance_below + 0.1;
            //RCLCPP_WARN(this->get_logger(), "Corrected %i-th joint of %i-th waypoint", j+1, i+1);
            outcome = 2;
          }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Checked %i waypoints", int(trajectory.joint_trajectory.points.size()));
    return outcome;
}

bool MoveItInitialiser::adjustWaypoint(trajectory_msgs::msg::JointTrajectoryPoint& point) {

  for (int j = 0; j < 6; j++)// : joint_positions)
  {
      if (point.positions[j] > path_constraints_.joint_constraints[j].position + path_constraints_.joint_constraints[j].tolerance_above){
        point.positions[j] = path_constraints_.joint_constraints[j].position + path_constraints_.joint_constraints[j].tolerance_above - 0.1;
        RCLCPP_WARN(this->get_logger(), "Corrected %i-th joint", j);

      }
      else if (point.positions[j] < path_constraints_.joint_constraints[j].position - path_constraints_.joint_constraints[j].tolerance_below){
        point.positions[j] = path_constraints_.joint_constraints[j].position - path_constraints_.joint_constraints[j].tolerance_below + 0.1;
        RCLCPP_WARN(this->get_logger(), "Corrected %i-th joint", j);
      }
  }
  
  return true;
}

geometry_msgs::msg::PoseStamped MoveItInitialiser::getPrecedingFramePose(const std::string target_frame, 
                                            const geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
    // Ensure target pose has a valid timestamp
    if (target_pose->header.stamp == rclcpp::Time()) {
        throw std::runtime_error("Target pose must have a valid timestamp");
    }

    // Get the transform from the preceding frame to the target frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            "target_object", target_frame, this->now() - rclcpp::Duration::from_seconds(0.1));//, target_pose->header.stamp);
    } 
    catch (const tf2::TransformException& ex) {
        throw std::runtime_error(ex.what());
    }

    // Transform the target pose to the preceding frame
    geometry_msgs::msg::PoseStamped preceding_frame_pose;
    tf2::doTransform(*target_pose, preceding_frame_pose, transform_stamped);

    return preceding_frame_pose;

}

MoveItInitialiser::MoveItInitialiser(const rclcpp::NodeOptions &options) : Node("mediated_pose_commander", options),
                                        node_(std::make_shared<rclcpp::Node>("example_group_node")),
                                        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mediated_pose", rclcpp::QoS(1), std::bind(&MoveItInitialiser::pose_callback, this, std::placeholders::_1));
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visual_tool0", rclcpp::QoS(1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");
    
    publisher_ = this->create_publisher<interface_msgs::msg::RobotPerformance>(
            "/robot_performance", 10);

    //move_group_interface_->setPlannerId("PRMkConfigDefault");
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

    load_planning_scene();
    RCLCPP_INFO(this->get_logger(), "Scene loaded");
    setPathConstraints();
    RCLCPP_INFO(this->get_logger(), "Path constraints loaded");
}

visualization_msgs::msg::Marker MoveItInitialiser::waypoints_to_markers(geometry_msgs::msg::PoseStamped& waypoints) {

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "tool0";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 0.33;
    marker.color.g = 0.34;
    marker.color.b = 0.33;

    marker.pose = waypoints.pose;
    marker.pose.orientation = waypoints.pose.orientation;
    marker.id++;
    
    // Set the duration (e.g., 5 seconds)
    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 10;
    marker.lifetime = lifetime;  // Assign the Duration to lifetime
    
    return marker;
}

void MoveItInitialiser::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {

  if (pose_msg->pose == previous_target_pose_)
  {

      RCLCPP_INFO(this->get_logger(), "Target pose has not changed. Waiting...");

      return;
  }

  RCLCPP_INFO(this->get_logger(), "Received position: (x: {%f}, y: {%f}, z: {%f})", pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Received orientation: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);

  RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning...");

  // Plan and execute trajectory based on received pose

  geometry_msgs::msg::PoseStamped preceding_frame_pose = getPrecedingFramePose("tool0", pose_msg);

  RCLCPP_INFO(this->get_logger(), "Planning for position: (x: {%f}, y: {%f}, z: {%f})", preceding_frame_pose.pose.position.x, preceding_frame_pose.pose.position.y, preceding_frame_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Planning for orientation: (x: {%f}, y: {%f}, z: {%f}, w: {%f})", preceding_frame_pose.pose.orientation.x, preceding_frame_pose.pose.orientation.y, preceding_frame_pose.pose.orientation.z, preceding_frame_pose.pose.orientation.w);


  // Assuming you have a function to convert your waypoints to markers, e.g., waypoints_to_markers
  visualization_msgs::msg::Marker marker = waypoints_to_markers(preceding_frame_pose);
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(preceding_frame_pose.pose);

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  move_group_interface_->clearPoseTargets();
  move_group_interface_->setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  
  RCLCPP_INFO(this->get_logger(), "Computed trajectory with fraction: %2f / 100, validating...", fraction * 100);
  // Validate and, if needed, adjust the trajectory 
  int valid = checkAndAdjustPathConstraints(trajectory);
  //int valid = 1;
  
  if (valid == 1) { 
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_interface_->getRobotModel(), "ur_manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*(move_group_interface_->getCurrentState()), trajectory);
    // Third create an IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps

    bool success = iptp.computeTimeStamps(rt, 0.1);// Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    if (success){
      move_group_interface_->execute(trajectory);
      
      interface_msgs::msg::RobotPerformance robot_performance;
      robot_performance.fraction = fraction;
      robot_performance.object_requested_pose = *pose_msg;
      robot_performance.tool0_requested_pose = preceding_frame_pose;
      // Create PoseStamped message
      geometry_msgs::msg::PoseStamped end_effector_pose_stamped;
      end_effector_pose_stamped.header.frame_id = move_group_interface_->getPlanningFrame();
      end_effector_pose_stamped.header.stamp = this->now();
      end_effector_pose_stamped.pose = move_group_interface_->getCurrentPose().pose;
      robot_performance.tool0_achieved_pose = end_effector_pose_stamped;
      publisher_->publish(robot_performance);

      previous_target_pose_ = pose_msg->pose;
    }
    else{
      RCLCPP_WARN(this->get_logger(), "Timestamps computation failed, not executing");
    }
  }
  else if (valid==2){
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_interface_->getRobotModel(), "ur_manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*(move_group_interface_->getCurrentState()), trajectory);
    // Third create an IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps

    bool success = iptp.computeTimeStamps(rt, 0.1);// Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    if (success){
      move_group_interface_->execute(trajectory);
      RCLCPP_WARN(this->get_logger(), "Executing, after adjustments to avoid singularities...");

      interface_msgs::msg::RobotPerformance robot_performance;
      robot_performance.fraction = fraction;
      robot_performance.object_requested_pose = *pose_msg;
      robot_performance.tool0_requested_pose = preceding_frame_pose;
      // Create PoseStamped message
      geometry_msgs::msg::PoseStamped end_effector_pose_stamped;
      end_effector_pose_stamped.header.frame_id = move_group_interface_->getPlanningFrame();
      end_effector_pose_stamped.header.stamp = this->now();
      end_effector_pose_stamped.pose = move_group_interface_->getCurrentPose().pose;
      robot_performance.tool0_achieved_pose = end_effector_pose_stamped;
      publisher_->publish(robot_performance);

      previous_target_pose_ = pose_msg->pose;
    }
    else{
      RCLCPP_WARN(this->get_logger(), "Timestamps computation failed after adjustments, not executing");
    }
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Something else went wrong, please check for errors!");
  }

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto target_follower = std::make_shared<MoveItInitialiser>(node_options);

    rclcpp::spin(target_follower);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}