from scipy.optimize import minimize
import rclpy
import tf2_ros
import transforms3d.euler as euler
from rclpy.node import Node
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from interface_msgs.msg import InteractionData
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
from builtin_interfaces.msg import Duration
import numpy as np
import sys
import threading


class PoseOptimizer(Node):
    def __init__(self):
        super().__init__('mediated_pose_calculator')

        # Declare parameters with default values
        self.declare_parameter('body_joints', False)
        # Retrieve parameter value
        self.joints_use = self.get_parameter('body_joints').value
        self.get_logger().info("body_joints value set to: " + str(self.joints_use))
        # Create the subscriber based on the selected topic
        self.subscription_interaction_data = self.create_subscription(
                InteractionData,
                'converted_interaction_data',
                self.callback_interaction_data,
                10
            )

        self.publisher = self.create_publisher(PoseStamped, 'mediated_pose', 10)
        self.publisher2 = self.create_publisher(MarkerArray, 'visual_mediated_pose', 10)
        self.publisher3 = self.create_publisher(Bool, 'utter_message', 10)

        #Variables to interpret received interaction state
        self.detected_2_first_time = False
        self.previous_state = 7 #This variable is introduced to handle misclassification and to understand the step of the process
        self.current_state = 7 
        self.already_optimised = False


        #Variables for orientation problem
        self.previous_mediated_position = Point()
        self.previous_mediated_orientation = Quaternion()
        self.header = Header()
        self.header.frame_id = "base_link"
        
        # Variables for frame broadcasting
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tiles_frame = "tiles_face"
        self.beads_frame = "beads_face"
        self.shapes_frame = "shapes_face"
        self.gears_frame = "gears_face"
        self.object_frame = "target_object"
        self.marker_duration_time = 7
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.wait_on_publishing = False
    
    def euclidean_distance_3d(self, point1, point2):
        """
        Calculate the Euclidean distance between two 3D points.

        Parameters:
            point1 (geometry_msgs.msg.Point): First 3D point.
            point2 (geometry_msgs.msg.Point): Second 3D point.

        Returns:
            float: Euclidean distance between the two points.
        """
        # Extract coordinates from Point messages
        x1, y1, z1 = point1.x, point1.y, point1.z
        x2, y2, z2 = point2.x, point2.y, point2.z
        
        # Calculate Euclidean distance using numpy
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance 
  
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a 3x3 rotation matrix to a quaternion.

        :param R: 3x3 rotation matrix
        :return: quaternion [x, y, z, w]
        """
        # Ensure the input is a 3x3 matrix
        assert R.shape == (3, 3), "Rotation matrix must be 3x3"

        # Compute the trace of the matrix
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        # Normalize the quaternion
        quaternion = np.array([x, y, z, w])
        quaternion /= np.linalg.norm(quaternion)
        
        return quaternion

    def compute_rotation_to_align(self, q_source, q_target):
        # Convert quaternions to rotation matrices
        R_source = quaternion_matrix([q_source.x, q_source.y, q_source.z, q_source.w])[:3, :3]
        R_target = quaternion_matrix([q_target.x, q_target.y, q_target.z, q_target.w])[:3, :3]
        
        # Compute the x-axis of the source frame and the negative y-axis of the target frame
        x_axis_source = R_source[:, 0]
        neg_y_axis_target = -R_target[:, 1]
        
        # Compute the rotation axis (cross product of the vectors)
        rotation_axis = np.cross(x_axis_source, neg_y_axis_target)
        
        # Normalize the rotation axis
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        
        # Compute the angle between the vectors (dot product and arccos)
        dot_product = np.dot(x_axis_source, neg_y_axis_target)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        # Compute the rotation matrix using the axis-angle representation
        K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                    [rotation_axis[2], 0, -rotation_axis[0]],
                    [-rotation_axis[1], rotation_axis[0], 0]])
        R_align = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        
        # Convert the rotation matrix to a quaternion
        rotation_quat = quaternion_from_matrix(np.vstack((np.hstack((R_align, np.zeros((3, 1)))), np.array([0, 0, 0, 1]))))
        
        # Convert the quaternion to RPY angles
        roll, pitch, yaw = euler_from_quaternion(rotation_quat)
        
        return [roll, pitch, yaw]
        
    def compute_mediated_rotation(self, quats_list):

        rpys = []

        n_users = len(quats_list)
        for quats in quats_list:
            rpy = self.compute_rotation_to_align(quats[0], quats[1])
            rpys.append(rpy)

        rotations = []
        matrix = np.eye(3)

        for rpy in rpys:
            r_m = euler.euler2mat(rpy[0]/n_users,rpy[1]/n_users,rpy[2]/n_users, axes = 'sxyz')
            rotations.append(r_m)

        for rots in rotations:
            matrix = np.matmul(rots, matrix)
        
        return matrix

    def compute_limb(self, joints_list):

        user_upper_arm = self.euclidean_distance_3d(joints_list[0], joints_list[1])
        user_lower_arm = self.euclidean_distance_3d(joints_list[1], joints_list[2])
        user_upper_hand = self.euclidean_distance_3d(joints_list[2], joints_list[3])
        user_lower_hand = self.euclidean_distance_3d(joints_list[3], joints_list[4])

        user_limb = user_upper_arm + user_lower_arm + user_upper_hand + user_lower_hand

        return user_limb

    def compute_mediated_position(self, points_list):

        if self.joints_use:
                
            limbs = []

            for info in points_list:
                limbs.append(self.compute_limb(info[1:-2]))
            
            limbs = np.array(limbs)
            
            norm_factor = np.sum(1./limbs, axis=0)
            weights = (1./limbs)/norm_factor 
             

            weighted_x = []
            weighted_y = []
            weighted_z = []

            for i in range(len(points_list)):
                weighted_x.append([points_list[i][0].x*weights[i]])
                weighted_y.append([points_list[i][0].y*weights[i]])
                weighted_z.append([points_list[i][0].z*weights[i]])

            m_x = np.sum(weighted_x)
            m_y = np.sum(weighted_y)
            m_z = np.sum(weighted_z)
            return Point(x=m_x,y=m_y,z=m_z)

        else:

            x_av = []
            y_av = []
            z_av = []

            for info in points_list:
                x_av.append(info[0].x)
                y_av.append(info[0].y)
                z_av.append(info[0].z)
            
            m_x = np.sum(x_av)/len(points_list)
            m_y = np.sum(y_av)/len(points_list)
            m_z = np.sum(z_av)/len(points_list)

            return Point(x=m_x,y=m_y,z=m_z)

    def quaternion_to_list(self, quaternion):
        return [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
    
    def list_to_quaternion(self, quat_list):
        return Quaternion(x=quat_list[0], y=quat_list[1], z=quat_list[2], w=quat_list[3])

    def create_marker(self, id, header, position, orientation, color):
        marker = Marker()
        marker.header = header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = id
        marker.ns = f"frame_{id}"

        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = orientation.x
        marker.pose.orientation.y = orientation.y
        marker.pose.orientation.z = orientation.z
        marker.pose.orientation.w = orientation.w

        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Set the duration (e.g., 5 seconds)
        marker.lifetime = Duration()
        marker.lifetime.sec = self.marker_duration_time

        return marker

    def assign_quaternions(self, target_frame):
        
        # Get the latest transform from the source frame to the target frame
        transform = self.tf_buffer.lookup_transform("base_link", target_frame, rclpy.time.Time())
        
        return transform.transform.rotation
    
    def broadcast_frame(self, position, orientation, name):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = name

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t) 

    def compute_optimal_position(self, marker):
        
        U_x = marker.x
        U_y = marker.y
        O_x = self.previous_mediated_position.x
        O_y = self.previous_mediated_position.y

        U_x_p = -0.7071 * (U_x + U_y)

        O_y_p = 0.7071 * (O_y - O_x)

        N_x = -0.7071 * (U_x_p + O_y_p)
        N_y = -0.7071 * (U_x_p - O_y_p)
        self.get_logger().info("New point: " + str(N_x) + str(N_y))


        return (N_x, N_y)

    def callback_interaction_data(self, msg):

        if msg.state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = msg.state
            #self.get_logger().info('State: ' + str(msg.state))
            self.already_optimised = False

            q_left_face = Quaternion()
            q_right_face = Quaternion()

            right_info = [msg.spine_naval_right_user,msg.shoulder_right_user,msg.elbow_right_user,msg.wrist_right_user,msg.hand_right_user,msg.handtip_right_user,msg.neck_right_user,msg.orientation_right_user]
            left_info = [msg.spine_naval_left_user,msg.shoulder_left_user,msg.elbow_left_user,msg.wrist_left_user,msg.hand_left_user,msg.handtip_left_user,msg.neck_left_user,msg.orientation_left_user]
            #optimized_quaternion_transform = -100

            self.get_logger().info("Previous state: " + str(self.previous_state))
            self.get_logger().info("Current state: " + str(self.current_state))

            q_object = self.assign_quaternions(self.object_frame)
            self.previous_orientation = q_object


            if self.current_state == 7:
                self.get_logger().info("Interaction data not received yet, keep listening...")
                return
            if self.current_state == 1:
                self.get_logger().info("Both users currently working, no need to intervene")
                return
            elif (self.current_state == 0 and self.previous_state == 7) or (self.current_state == 0 and self.previous_state == 6): #or (self.current_state == 1 and self.previous_state == 0):
                self.get_logger().info("Optimising for beads and gears (first W-W)")
                q_left_face = self.assign_quaternions(self.beads_frame)
                q_right_face = self.assign_quaternions(self.gears_frame)
                self.broadcast_frame(left_info[-2],left_info[-1], "left_user")
                self.broadcast_frame(right_info[-2],right_info[-1], "right_user")
                mediated_rotation = self.compute_mediated_rotation([[q_left_face, left_info[-1]],[q_right_face, right_info[-1]]])#,[q_right_face,self.q_right_user]])
                mediated_position = self.compute_mediated_position([left_info,right_info])
            elif self.current_state == 2 and self.previous_state == 1:# and not self.detected_2_first_time:
                self.get_logger().info("Optimising for beads only (first W-P)")
                q_left_face = self.assign_quaternions(self.beads_frame)
                self.broadcast_frame(left_info[-2],left_info[-1], "left_user")
                mediated_rotation = self.compute_mediated_rotation([[q_left_face, left_info[-1]]])
                p = self.compute_optimal_position(left_info[0])
                mediated_position = Point(x = p[0], y = p[1], z = left_info[0].z)
                #mediated_position = self.previous_mediated_position
                self.detected_2_first_time = True       
            elif (self.current_state == 3 and self.previous_state == 2):# and not self.already_optimised):# or (self.current_state == 1 and self.previous_state == 3):
                self.get_logger().info("Optimising for beads and tiles (second W-W)")
                q_left_face = self.assign_quaternions(self.beads_frame)
                q_right_face = self.assign_quaternions(self.tiles_frame)
                self.broadcast_frame(left_info[-2],left_info[-1], "left_user")
                self.broadcast_frame(right_info[-2],right_info[-1], "right_user")
                mediated_rotation = self.compute_mediated_rotation([[q_left_face, left_info[-1]],[q_right_face, right_info[-1]]])#,[q_right_face,self.q_right_user]])
                mediated_position = self.compute_mediated_position([left_info,right_info])
                self.wait_on_publishing = True
            elif self.current_state == 4  and self.previous_state == 1:
                self.get_logger().info("Optimising for tiles only (P-W)")
                q_right_face = self.assign_quaternions(self.tiles_frame)
                self.broadcast_frame(right_info[-2],right_info[-1], "right_user")#average_angle, average_axis = self.compute_mediated_rotation([[q_right_face,self.q_right_user]])
                mediated_rotation = self.compute_mediated_rotation([[q_right_face,right_info[-1]]])
                p = self.compute_optimal_position(right_info[0])
                mediated_position = Point(x = p[0], y = p[1], z = right_info[0].z)
                #mediated_position = self.previous_mediated_position
            elif (self.current_state == 5 and self.previous_state == 4):# and not self.already_optimised):# or (self.current_state == 1 and self.previous_state == 5):
                self.get_logger().info("Optimising for beads and shapes (third W-W)")
                q_left_face = self.assign_quaternions(self.shapes_frame)
                q_right_face = self.assign_quaternions(self.tiles_frame)
                self.broadcast_frame(left_info[-2],left_info[-1], "left_user")
                self.broadcast_frame(right_info[-2],right_info[-1], "right_user")
                mediated_rotation = self.compute_mediated_rotation([[q_left_face, left_info[-1]],[q_right_face, right_info[-1]]])#,[q_right_face,self.q_right_user]])
                mediated_position = self.compute_mediated_position([left_info,right_info])
                self.wait_on_publishing = True
            elif self.current_state == 2 and (self.previous_state == 1 or self.previous_state ==5) and self.detected_2_first_time:
                self.get_logger().info("Optimising for shapes only (second W-P)")
                q_left_face = self.assign_quaternions(self.shapes_frame)
                self.broadcast_frame(left_info[-2],left_info[-1], "left_user")
                mediated_rotation = self.compute_mediated_rotation([[q_left_face, left_info[-1]]])
                p = self.compute_optimal_position(left_info[0])
                mediated_position = Point(x = p[0], y = p[1], z = left_info[0].z)
                #mediated_position = self.previous_mediated_position
            #elif (self.current_state == 6 and (self.previous_state == 1 or self.previous_state == 2) and not self.already_optimised) or (self.current_state == 6 and self.previous_state == 4 and not self.already_optimised):
            elif (self.current_state == 6 and (self.previous_state == 1 or self.previous_state == 2)) or (self.current_state == 6 and self.previous_state == 4):
                self.get_logger().info ("Assembly completed, please revert to starting position")   
                return     
            else:
                self.get_logger().error ("Received message not relatable to any interaction state known, please check for errors!")
                return

            face_R = euler.quat2mat(np.array(self.quaternion_to_list(q_left_face)))
            self.get_logger().info("Face orientation before (rpy): " + str(euler.quat2euler(self.quaternion_to_list(q_left_face))))
            object_R = euler.quat2mat(np.array(self.quaternion_to_list(q_object)))
            self.get_logger().info("Object orientation before (rpy): " + str(euler.quat2euler(self.quaternion_to_list(q_object))))
            self.get_logger().info("Average rotation Euler angles (rpy): " + str(euler.mat2euler(mediated_rotation)))

            
            mediated_orientation_R = np.matmul(mediated_rotation, object_R)

            new_face_orientation_R = np.matmul(mediated_rotation, face_R)
            self.get_logger().info("Face orientation after (rpy): " + str(euler.mat2euler(new_face_orientation_R)))
            self.get_logger().info("Object orientation after (rpy): " + str(euler.mat2euler(mediated_orientation_R)))
            mediated_orientation =self.rotation_matrix_to_quaternion(mediated_orientation_R)
            
            self.get_logger().info("Mediated position: " + str(mediated_position))
            self.get_logger().info("Mediated orientation: " + str(mediated_orientation))

            self.header.stamp = self.get_clock().now().to_msg() 
            mediated_orientation_q = self.list_to_quaternion(mediated_orientation)
            stamped_mediated_pose = PoseStamped(pose = Pose(position=mediated_position, orientation=mediated_orientation_q), header = self.header)
            if self.previous_mediated_orientation != stamped_mediated_pose.pose.orientation:
                if self.wait_on_publishing:
                    # Delayed publishing
                    signal = Bool()
                    signal.data = True
                    self.publisher3.publish(signal)
                    threading.Timer(5.0, self.publisher.publish, args=[stamped_mediated_pose]).start()
                    self.wait_on_publishing = False
                else:
                    self.publisher.publish(stamped_mediated_pose)
                #self.broadcast_frame(mediated_position,mediated_orientation_q, "mediated_orientation")

                self.previous_mediated_position = stamped_mediated_pose.pose.position
                self.previous_mediated_orientation = stamped_mediated_pose.pose.orientation
                marker_array = MarkerArray()
                marker_array.markers.append(self.create_marker(0, self.header, mediated_position, mediated_orientation_q, [0.0, 1.0, 1.0, 1.0]))
                #self.publisher2.publish(marker_array)

    

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = PoseOptimizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

