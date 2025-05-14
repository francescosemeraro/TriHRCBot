from scipy.optimize import minimize
import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from interface_msgs.msg import InteractionData
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
#from tf2_ros.transformations import euler_from_quaternion, quaternion_from_euler
import transformations as tf
import math


class MyQuaternion:
    def __init__(self, x, y, z, w):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

def multiply_quaternions(q1, q2):
    w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
    z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x

    return MyQuaternion(x=x, y=y, z=z, w=w)


class PoseOptimizer(Node):
    def __init__(self):
        super().__init__('mediated_pose_calculator')

        # Create the subscriber based on the selected topic
        self.subscription_interaction_data = self.create_subscription(
                InteractionData,
                'converted_interaction_data',
                self.callback_interaction_data,
                10
            )

        self.publisher = self.create_publisher(PoseStamped, 'mediated_pose', 10)
        self.publisher2 = self.create_publisher(MarkerArray, 'visual_mediated_pose', 10)

        #Variables to interpret received interaction state
        self.detected_2_first_time = False
        self.previous_state = None #This variable is introduced to handle misclassification and to understand the step of the process
        self.current_state = None 
        self.already_optimised = False


        #Variables for optimisation problem
        self.p_left_user = Pose()
        self.p_right_user = Pose()
        self.q_left_user = Quaternion()
        self.q_right_user = Quaternion()
        self.previous_orientation = [0,0,0,1]
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
        self.marker_duration_time = 30

    def callback_interaction_data(self, msg):

        self.p_left_user = msg.p_left_user
        self.q_left_user = msg.q_left_user
        self.p_right_user = msg.p_right_user
        self.q_right_user = msg.q_right_user

        
        if msg.state != self.current_state and msg.state != 1:
            self.previous_state = self.current_state
            self.current_state = msg.state
            self.get_logger().info('State: ' + str(msg.state))
            self.already_optimised = False

        q_left_face = Quaternion()
        q_right_face = Quaternion()
        optimized_quaternion_transform = None

        print("Previous state: ", self.previous_state)
        print("Current state: ", self.current_state)

        q_object = self.assign_quaternions(self.object_frame)
        self.previous_orientation = q_object


        if self.current_state == None:
            print("Interaction data not received yet, keep listening...")
            return
        elif (self.current_state == 0 and self.previous_state == None and not self.already_optimised) or (self.current_state == 0 and self.previous_state == 6 and not self.already_optimised): #or (self.current_state == 1 and self.previous_state == 0):
            print("Optimising for beads and gears (first W-W)")
            self.already_optimised = True
            q_left_face = self.assign_quaternions(self.beads_frame)
            q_right_face = self.assign_quaternions(self.gears_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_both(q_left_face, self.q_left_user, q_right_face, self.q_right_user)
        elif self.current_state == 2 and self.previous_state == 1 and not self.detected_2_first_time:
            print("Optimising for beads only (first W-P)")
            return #Remove after
            q_left_face = self.assign_quaternions(self.beads_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_one(q_left_face, self.q_left_user)
        elif (self.current_state == 3 and self.previous_state == 2 and not self.already_optimised):# or (self.current_state == 1 and self.previous_state == 3):
            print("Optimising for beads and tiles (second W-W)")
            self.already_optimised = True
            q_left_face = self.assign_quaternions(self.beads_frame)
            q_right_face = self.assign_quaternions(self.tiles_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_both(q_left_face, self.q_left_user, q_right_face, self.q_right_user)
        elif self.current_state == 4  and self.previous_state == 1:
            print("Optimising for tiles only (P-W)")
            return #Remove after
            q_right_face = self.assign_quaternions(self.tiles_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_one(q_right_face, self.q_right_user)
        elif (self.current_state == 5 and self.previous_state == 4 and not self.already_optimised):# or (self.current_state == 1 and self.previous_state == 5):
            print("Optimising for beads and shapes (third W-W)")
            self.already_optimised = True
            q_left_face = self.assign_quaternions(self.shapes_frame)
            q_right_face = self.assign_quaternions(self.tiles_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_both(q_left_face, self.q_left_user, q_right_face, self.q_right_user)
        elif self.current_state == 2 and self.previous_state == 1 and self.detected_2_first_time:
            print("Optimising for shapes only (second W-P)")
            return #Remove after
            q_left_face = self.assign_quaternions(self.shapes_frame)
            optimized_quaternion_transform = self.optimize_quaternion_transform_for_one(q_left_face, self.q_left_user)
        elif (self.current_state == 6 and self.previous_state == 2 and not self.already_optimised) or (self.current_state == 6 and self.previous_state == 4 and not self.already_optimised):
            print ("Assembly completed, reverting to starting position")   
            self.already_optimised = True
            #self.detected_2_first_time = True
            #self.header.stamp = self.get_clock().now().to_msg()
            #self.publisher.publish(PoseStamped(pose = Pose(), header = self.header)) #To measure with RViz 2
            return     
        else:
            print ("Received message not relatable to any interaction state known, please check for errors!")
            return

        transform = self.tf_buffer.lookup_transform("base_link", self.object_frame, rclpy.time.Time())
        previous_t = transform.transform.translation
        previous_pose = Pose()
        previous_pose.position.x = previous_t.x
        previous_pose.position.y = previous_t.y
        previous_pose.position.z = previous_t.z

        print(optimized_quaternion_transform) 
        #m_o = np.dot(np.array([[optimized_quaternion_transform.x],[optimized_quaternion_transform.y],[optimized_quaternion_transform.z],[optimized_quaternion_transform.w]]),
        #                           np.array([[q_object.x],[q_object.y],[q_object.z],[q_object.w]]).transpose()).astype(float)
        
        object_rot = MyQuaternion(q_object.x,q_object.y,q_object.z,q_object.w)
        opt_rot = MyQuaternion(optimized_quaternion_transform[0],optimized_quaternion_transform[1],optimized_quaternion_transform[2],optimized_quaternion_transform[3])
        q_rotated_object = multiply_quaternions(opt_rot, object_rot)
        print(q_rotated_object)
        mediated_position = Point(x=(self.p_left_user.x + self.p_right_user.x)/2,y=(self.p_left_user.y + self.p_right_user.y)/2,z=(self.p_left_user.z + self.p_right_user.z)/2)

        mediated_orientation = Quaternion(x=q_rotated_object.x,y=q_rotated_object.y,z=q_rotated_object.z,w=q_rotated_object.w)
        self.header.stamp = self.get_clock().now().to_msg() 

        stamped_mediated_pose = PoseStamped(pose = Pose(position=mediated_position, orientation=mediated_orientation), header = self.header)
        self.publisher.publish(stamped_mediated_pose)

        marker_array = MarkerArray()
        #q_m = MyQuaternion(x=mediated_orientation.x,y=mediated_orientation.y,z=mediated_orientation.z,w=mediated_orientation.w)
        #marker = self.create_arrow_marker(0, self.header, previous_pose, self.previous_orientation, [0.5, 0.5, 0.0, 1.0])
        #marker_array.markers.append(marker)
        
        q_left = MyQuaternion(x=q_left_face.x,y=q_left_face.y,z=q_left_face.z,w=q_left_face.w)
        marker_left = self.create_arrow_marker(1, self.header, previous_pose, q_left, [0.0, 1.0, 0.0, 1.0])
        marker_array.markers.append(marker_left)

        q_right = MyQuaternion(x=q_right_face.x,y=q_right_face.y,z=q_right_face.z,w=q_right_face.w)
        marker_right = self.create_arrow_marker(2, self.header, previous_pose, q_right, [0.0, 0.0, 1.0, 1.0])
        marker_array.markers.append(marker_right)

        #q_m = MyQuaternion(x=mediated_orientation.x,y=mediated_orientation.y,z=mediated_orientation.z,w=mediated_orientation.w)
        #q_m = multiply_quaternions(opt_rot,q_m) #Because x axis is already oriented in an infromative way
        #r_marker = self.create_arrow_marker(3, self.header, stamped_mediated_pose.pose, q_m, [0.5, 0.5, 0.0, 1.0])
        #marker_array.markers.append(r_marker)
        
        q_left = MyQuaternion(x=q_left_face.x,y=q_left_face.y,z=q_left_face.z,w=q_left_face.w)
        #q_left = self.rotate_frame_axis(multiply_quaternions(opt_rot,q_left)) 
        q_left_r = multiply_quaternions(opt_rot,q_left) 
        print(q_left_r.x,q_left_r.y,q_left_r.z,q_left_r.w)
        r_marker_left = self.create_arrow_marker(4, self.header, stamped_mediated_pose.pose, q_left_r, [0.0, 1.0, 0.0, 1.0])
        marker_array.markers.append(r_marker_left)

        q_right = MyQuaternion(x=q_right_face.x,y=q_right_face.y,z=q_right_face.z,w=q_right_face.w)
        #q_right = self.rotate_frame_axis(multiply_quaternions(opt_rot,q_right)) 
        q_right_r = multiply_quaternions(opt_rot,q_right) 
        print(q_right_r.x,q_right_r.y,q_right_r.z,q_right_r.w)
        r_marker_right = self.create_arrow_marker(5, self.header, stamped_mediated_pose.pose, q_right_r, [0.0, 0.0, 1.0, 1.0])
        marker_array.markers.append(r_marker_right)

        # Publish the MarkerArray
        self.publisher2.publish(marker_array)


    def create_arrow_marker(self, id, header, pose, orientation, color):
        marker = Marker()
        marker.header = header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = id
        marker.ns = f"frame_{id}"

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation.x = orientation.x
        marker.pose.orientation.y = orientation.y
        marker.pose.orientation.z = orientation.z
        marker.pose.orientation.w = orientation.w

        marker.scale.x = 0.3
        marker.scale.y = 0.03
        marker.scale.z = 0.03

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
        
    def optimize_quaternion_transform_for_both(self, q_left_user, q_left_face, q_right_user, q_right_face):

        # Function to minimize
        def objective_function(q_t, q_left_user, q_left_face, q_right_user, q_right_face, users_axis, object_axis):
            q_transform = MyQuaternion(q_t[0],q_t[1],q_t[2],q_t[3])

            # Axes used for alignment
            users_axis = users_axis

            # Objective function
            rotated_left = multiply_quaternions(q_transform, q_left_face)

            left_t = R.from_quat([rotated_left.x,rotated_left.y,rotated_left.z,rotated_left.w])
            q_lu = R.from_quat(q_left_user)
            term1 = np.linalg.norm(left_t.apply(object_axis) + q_lu.apply(users_axis))
            
            rotated_right = multiply_quaternions(q_transform, q_right_face)

            right_t = R.from_quat([rotated_right.x,rotated_right.y,rotated_right.z,rotated_right.w])
            q_lu = R.from_quat(q_right_user)
            term2 = np.linalg.norm(right_t.apply(object_axis) + q_lu.apply(users_axis))

            objective = term1 + term2
            
            return objective

        # Constraints
        def norm_constraint(q_t):
            # Norm constraint
            return np.linalg.norm(q_t) - 1
        

        q_left_user = np.array([q_left_user.x,q_left_user.y,q_left_user.z,q_left_user.w])
        q_right_user = np.array([q_right_user.x,q_right_user.y,q_right_user.z,q_right_user.w])
        q_left_face = MyQuaternion(q_left_face.x,q_left_face.y,q_left_face.z,q_left_face.w)
        q_right_face = MyQuaternion(q_right_face.x,q_right_face.y,q_right_face.z,q_right_face.w)
        users_axis = np.array([0, 1, 0])
        object_axis = np.array([0, 0, 1])

        # Initial guess for quaternion transformation
        initial_guess = [self.previous_orientation.x,self.previous_orientation.y,self.previous_orientation.z,self.previous_orientation.w]

        # Optimization
        constraints = [{'type': 'eq', 'fun': norm_constraint}]
        result = minimize(objective_function, initial_guess, args = (q_left_user, q_left_face, q_right_user, q_right_face, users_axis, object_axis), constraints=constraints)
        
        # Extract optimized quaternion transformation
        return result.x

    def optimize_quaternion_transform_for_one(self, q_user, q_face):
        
        q_user = np.array([[q_user.x],[q_user.y],[q_user.z],[q_user.w]])
        q_face = np.array([[q_face.x],[q_face.y],[q_face.z],[q_face.w]])
        
        # Function to minimize
        def objective_function(quaternion_transform):
            q_transform = R.from_quat(quaternion_transform)

            # Z-axis vector
            Z_axis = np.array([0, 0, 1])

            # Objective function
            objective = np.linalg.norm(q_transform.apply(q_face * Z_axis) + q_user * Z_axis)               

            return objective

        # Constraints
        def norm_constraint(quaternion_transform):
            # Norm constraint
            return np.linalg.norm(quaternion_transform) - 1
        
        # Initial guess for quaternion transformation
        initial_guess = np.random.rand(4)

        # Optimization
        constraints = [{'type': 'eq', 'fun': norm_constraint}]
        result = minimize(objective_function, initial_guess, constraints=constraints)

        orientation_result = Quaternion()
        orientation_result.x = result.x[0].astype(float)
        orientation_result.y = result.x[1].astype(float)
        orientation_result.z = result.x[2].astype(float)
        orientation_result.w = result.x[3].astype(float)
        # Extract optimized quaternion transformation
        return orientation_result

    def rotate_quaternion_to_y_axis(input_quaternion):
        # Normalize input quaternion
        input_quaternion /= np.linalg.norm(input_quaternion)

        # Find the rotation axis that aligns y-axis with x-axis
        target_axis = [1, 0, 0]
        rotation_quaternion = quaternion_about_axis(math.atan2(target_axis[1], target_axis[0]), target_axis)

        # Apply the rotation to the input quaternion
        rotated_quaternion = multiply_quaternions(rotation_quaternion, input_quaternion)

        return rotated_quaternion  
    
    def rotate_quaternion_to_axis(self, input_quaternion, axis):
        # Extract roll, pitch, and yaw from the input quaternion
        roll, pitch, yaw = tf.euler_from_quaternion([input_quaternion.x, input_quaternion.y, input_quaternion.z, input_quaternion.w])

        # Create a quaternion to represent the desired rotation
        rotation_quaternion = Quaternion()

        # Choose the desired axis
        if axis.lower() == 'x':
            print("Entered here")
            roll = 0.0
        elif axis.lower() == 'y':
            print("Entered here")
            pitch = 0.0
        elif axis.lower() == 'z':
            print("Entered here")
            # No need to rotate for Z-axis
            return Quaternion(x=input_quaternion.x, y=input_quaternion.y, z=input_quaternion.z, w=input_quaternion.w)
        else:
            # Invalid axis, return the original quaternion
            return Quaternion(x=input_quaternion.x, y=input_quaternion.y, z=input_quaternion.z, w=input_quaternion.w)

        # Convert back to quaternion
        rotated_quaternion = Quaternion()
        rotated_quaternion.x, rotated_quaternion.y, rotated_quaternion.z, rotated_quaternion.w = tf.quaternion_from_euler(roll, pitch, yaw)

        return rotated_quaternion

    def rotate_frame_axis(self, original_quaternion):
    # Convert the original quaternion to individual quaternion components
        w, x, y, z = original_quaternion.w, original_quaternion.x, original_quaternion.y, original_quaternion.z
        o_q = MyQuaternion(x=x,y=y,z=z,w=w)
        # Create a quaternion to represent the desired rotation
        rotation_quaternion = []

        #if source_axis == 'x' and target_axis == 'y':
        #    # Rotate around the z-axis to align x with y
        #    rotation_quaternion = MyQuaternion(x = 0.0, y = 0.0, w=math.cos(math.radians(45)), z = math.sin(math.radians(45)))
        #elif source_axis == 'x' and target_axis == 'z':
            # Rotate around the y-axis to align x with z
        rotation_quaternion = MyQuaternion(x = 0.0, y = -math.sin(math.radians(45)), z= 0.0, w=math.cos(math.radians(45)))
        # Add more cases for different source and target axes

        # Rotate the original quaternion
        rotated_quaternion = multiply_quaternions(rotation_quaternion,o_q)

        return Quaternion(x=rotated_quaternion.x,y=rotated_quaternion.y,z=rotated_quaternion.z,w=rotated_quaternion.w)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = PoseOptimizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
def optimize_quaternion_transform_for_both(self, q_left_user, q_left_face, q_right_user, q_right_face):
        
        q_left_user = np.array([[q_left_user.x],[q_left_user.y],[q_left_user.z],[q_left_user.w]])
        q_right_user = np.array([[q_right_user.x],[q_right_user.y],[q_right_user.z],[q_right_user.w]])
        q_left_face = np.array([[q_left_face.x],[q_left_face.y],[q_left_face.z],[q_left_face.w]])
        q_right_face = np.array([[q_right_face.x],[q_right_face.y],[q_right_face.z],[q_right_face.w]])
        
        # Function to minimize
        def objective_function(quaternion_transform):
            q_transform = R.from_quat(quaternion_transform)

            # Z-axis vector
            Z_axis = np.array([0, 0, 1])

            # Objective function
            objective = np.linalg.norm(q_transform.apply(q_left_face * Z_axis) + q_left_user * Z_axis) + \
                np.linalg.norm(q_transform.apply(q_right_face * Z_axis) + q_right_user * Z_axis)

            return objective

        # Constraints
        def norm_constraint(quaternion_transform):
            # Norm constraint
            return np.linalg.norm(quaternion_transform) - 1
        
        # Initial guess for quaternion transformation
        initial_guess = np.random.rand(4)

        # Optimization
        constraints = [{'type': 'eq', 'fun': norm_constraint}]
        result = minimize(objective_function, initial_guess, constraints=constraints)
        print(result.x[0])
        orientation_result = Quaternion()
        orientation_result.x = result.x[0].astype(float)
        orientation_result.y = result.x[1].astype(float)
        orientation_result.z = result.x[2].astype(float)
        orientation_result.w = result.x[3].astype(float)
        # Extract optimized quaternion transformation
        return orientation_result

    def optimize_quaternion_transform_for_one(self, q_user, q_face):
        
        q_user = np.array([[q_user.x],[q_user.y],[q_user.z],[q_user.w]])
        q_face = np.array([[q_face.x],[q_face.y],[q_face.z],[q_face.w]])
        
        # Function to minimize
        def objective_function(quaternion_transform):
            q_transform = R.from_quat(quaternion_transform)

            # Z-axis vector
            Z_axis = np.array([0, 0, 1])

            # Objective function
            objective = np.linalg.norm(q_transform.apply(q_face * Z_axis) + q_user * Z_axis)               

            return objective

        # Constraints
        def norm_constraint(quaternion_transform):
            # Norm constraint
            return np.linalg.norm(quaternion_transform) - 1
        
        # Initial guess for quaternion transformation
        initial_guess = np.random.rand(4)

        # Optimization
        constraints = [{'type': 'eq', 'fun': norm_constraint}]
        result = minimize(objective_function, initial_guess, constraints=constraints)

        orientation_result = Quaternion()
        orientation_result.x = result.x[0].astype(float)
        orientation_result.y = result.x[1].astype(float)
        orientation_result.z = result.x[2].astype(float)
        orientation_result.w = result.x[3].astype(float)
        # Extract optimized quaternion transformation
        return orientation_result


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = PoseOptimizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''