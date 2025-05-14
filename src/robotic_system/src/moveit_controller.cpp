#include <memory>
//#include <iosrteam>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

using std::string;

struct MyStruct {
    bool success;
    robot_trajectory::RobotTrajectory rt;
    double fraction;
};

MyStruct plan_trajectory(std::vector<geometry_msgs::msg::Pose> &waypoints, moveit_msgs::msg::RobotTrajectory &trajectory, moveit::planning_interface::MoveGroupInterface &move_group_interface){

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "ur_manipulator");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt, 0.1);
  // Get RobotTrajectory_msg from RobotTrajectory
  //rt.getRobotTrajectoryMsg(trajectory);

  return {success, rt, fraction};
}

int main(int argc, char * argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_commander");
  
  // Check if the parameter is provided, otherwise default to some value
  //node.declare_parameter("execution","value not set");
  auto param_value = node->get_parameter("execution").as_string();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  
  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); }); 
  

  move_group_interface.setEndEffectorLink("tool0");
  move_group_interface.clearPoseTargets();
  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setPlannerId("RRTkConfigDefault");
  move_group_interface.setPlanningTime(15.0);
/*  
  // Set a target Pose
  auto const target_pose0 = []{
    geometry_msgs::msg::Pose msg;

    msg.position.x = -0.239;
    msg.position.y = -0.431;
    msg.position.z = 0.238;
    msg.orientation.x = 0.0146;
    msg.orientation.y = 0.00566;
    msg.orientation.z = 0.9124;
    msg.orientation.w = -0.409;
    
    return msg;
  }();
*/
  // Set a target Pose
  auto const target_pose0 = []{
    geometry_msgs::msg::Pose msg;

    msg.position.x = -0.316;
    msg.position.y = -0.511;
    msg.position.z = 0.477;
    msg.orientation.x = 0.0258;
    msg.orientation.y = 0.01002;
    msg.orientation.z = 0.933;
    msg.orientation.w = -0.358;
    
    return msg;
  }();

  if (param_value == "start_over") {

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
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
        primitive.dimensions[primitive.BOX_X] = 0.15;
        primitive.dimensions[primitive.BOX_Y] = 1.7;
        primitive.dimensions[primitive.BOX_Z] = 0.8;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose wall_pose;
        wall_pose.orientation.w = 0.924;
        wall_pose.orientation.z = 0.383;
        wall_pose.position.x = 0.0;
        wall_pose.position.y = 0.5;
        wall_pose.position.z = 0.4;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(wall_pose);
        collision_object.operation = collision_object.ADD;

        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive2;
    
        primitive2.type = primitive2.BOX; 
        primitive2.dimensions.resize(3);
        primitive2.dimensions[primitive2.BOX_X] = 1.3;
        primitive2.dimensions[primitive2.BOX_Y] = 0.01;
        primitive2.dimensions[primitive2.BOX_Z] = 1.8;


        geometry_msgs::msg::Pose wall2;
        wall2.orientation.w = 0.924;
        wall2.orientation.z = 0.383;
        wall2.position.x = 0.05;
        wall2.position.y = -0.65;
        wall2.position.z = 0.0;

        collision_object.primitives.push_back(primitive2);
        collision_object.primitive_poses.push_back(wall2);

        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive3;
    
        primitive3.type = primitive3.BOX; 
        primitive3.dimensions.resize(3);
        primitive3.dimensions[primitive3.BOX_X] = 1.3;
        primitive3.dimensions[primitive3.BOX_Y] = 0.01;
        primitive3.dimensions[primitive3.BOX_Z] = 1.8;


        geometry_msgs::msg::Pose wall3;
        wall3.orientation.w = 0.924;
        wall3.orientation.z = 0.383;
        wall3.position.x = -0.55;
        wall3.position.y = -0.05;
        wall3.position.z = 0.0;

        collision_object.primitives.push_back(primitive3);
        collision_object.primitive_poses.push_back(wall3);
        
        // Define the pose of the box (relative to the frame_id)
        shape_msgs::msg::SolidPrimitive primitive4;
    
        primitive4.type = primitive4.BOX; 
        primitive4.dimensions.resize(3);
        primitive4.dimensions[primitive4.BOX_X] = 0.8;
        primitive4.dimensions[primitive4.BOX_Y] = 0.8;
        primitive4.dimensions[primitive4.BOX_Z] = 0.01;


        geometry_msgs::msg::Pose upper_wall;
        upper_wall.orientation.w = 0.924;
        upper_wall.orientation.z = 0.383;
        upper_wall.position.x = -0.3;
        upper_wall.position.y = -0.4;
        upper_wall.position.z = 0.9;

        collision_object.primitives.push_back(primitive4);
        collision_object.primitive_poses.push_back(upper_wall);
        
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
        table.position.z = -0.27;

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
        primitive.dimensions[primitive.BOX_X] = 0.15;
        primitive.dimensions[primitive.BOX_Y] = 0.15;
        primitive.dimensions[primitive.BOX_Z] = 0.29;

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

    planning_scene_interface.applyAttachedCollisionObject(target_object);
    planning_scene_interface.applyCollisionObject(scene);

    std::vector<geometry_msgs::msg::Pose> waypoints0;
    waypoints0.push_back(target_pose0);
    
    moveit_msgs::msg::RobotTrajectory trajectory0;
    
    MyStruct planned_trajectory = plan_trajectory(waypoints0, trajectory0, move_group_interface);
    
    if (planned_trajectory.success){
      // Get RobotTrajectory_msg from RobotTrajectory
      planned_trajectory.rt.getRobotTrajectoryMsg(trajectory0);
      RCLCPP_INFO(node->get_logger(), "Executing trajectory with fraction: %f", planned_trajectory.fraction);

      move_group_interface.execute(trajectory0);
      RCLCPP_INFO(node->get_logger(), "Trajectory executed");
    }
    else{
      RCLCPP_ERROR(logger, "Planning failed!");
    } 
    
  } 
  
  else if (param_value == "first_start") {

    auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();  

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
  
    // Create collision object for the robot to avoid
    auto const scene = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id;
      collision_object.id = "table";
      shape_msgs::msg::SolidPrimitive primitive;
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
      table.position.z = -0.27;
      //table.position.z = -0.1;

      //collision_object.primitives.push_back(primitive5);
      //collision_object.primitive_poses.push_back(table);
      
      collision_object.operation = collision_object.ADD;

      
      return collision_object;
    }();

    // Apply objects to the planning scene
    planning_scene_interface->applyCollisionObject(scene);
    planning_scene_interface->applyAttachedCollisionObject(target_object);
    RCLCPP_INFO(logger, "Initialised scene!");

    //move_group_interface.setPoseTarget(target_pose0);

    //const std::vector<double> joint_values = {3.93, -0.942, 2.18, 3.35, 1.57, -1.52};
    //const std::vector<double> joint_values = {3.93, -0.804, 1.81, -2.63, 1.56, -1.63}; //OLD ONES
    //const std::vector<double> joint_values = {1.27, -2.17, -2.14, -0.337, -1.59, 1.12};
    const std::vector<double> joint_values = {3.99, -0.777, 1.73, -2.54, 1.57, -1.65}; //NEW ONES

    move_group_interface.setJointValueTarget(joint_values);
   
    auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
    
    std::vector<std::string> names = {"table"};
    planning_scene_interface->removeCollisionObjects(names);
  } 
  
  else {
    // Default case or handle invalid parameter value
    RCLCPP_WARN(logger, "Invalid or unspecified parameter value. Defaulting to some behavior.");
  }
  /*// Set a target Pose
  auto const target_pose00  = []{
    geometry_msgs::msg::Pose msg;

    msg.position.x = -0.5296;
    msg.position.y = -0.641206;
    msg.position.z = 0.064395;
    msg.orientation.x = 0.024036;
    msg.orientation.y = 0.182602;
    msg.orientation.z = 0.968574;
    msg.orientation.w = -0.167162;
    
    return msg;
  }();
  */

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
