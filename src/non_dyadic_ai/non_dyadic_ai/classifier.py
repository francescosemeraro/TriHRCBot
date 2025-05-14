import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Int32
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion, Point
from interface_msgs.msg import InteractionData, RawDatapoint, ClassificationData
import tensorflow as tf
from tensorflow.keras.models import model_from_json
import numpy as np
import time

        
class SkeletonProcessor(Node):
    def __init__(self):
        super().__init__('skeleton_processor')
        
        self.declare_parameter('topic_to_subscribe', 'no topic name given')
        self.declare_parameter('to_save', False)
        # Retrieve parameter value
        self.topic_to_subscribe = self.get_parameter('topic_to_subscribe').value
        self.to_save = self.get_parameter('to_save').value
        self.subscriber = []
        self.publisher3 = []

        if self.topic_to_subscribe == '/raw_datapoint':
            self.subscriber = self.create_subscription(
            RawDatapoint,
            '/raw_datapoint',
            self.raw_datapoint_callback,
            10
            )
            self.get_logger().info('Subscribed to bag topic')
        elif self.topic_to_subscribe == '/body_tracking_data':
            # Get the topic name from a ROS2 parameter
            self.subscriber = self.create_subscription(
                MarkerArray,
                '/body_tracking_data',
                self.skeleton_callback,
                10
            )
            self.get_logger().info('Subscribed to Kinect topic')
        elif self.topic_to_subscribe == '/fake_body_tracking_data':
            # Get the topic name from a ROS2 parameter
            self.subscriber = self.create_subscription(
                RawDatapoint,
                '/raw_datapoint',
                self.fake_skeleton_callback,
                10
            )
            self.get_logger().info('Subscribed to fake Kinect topic')
        else:
            self.get_logger().info('Not given topic name or topic not available, check for errors!')

        self.subscriber2 = self.create_subscription(Int32, '/ground_truth', self.store_ground_truth, 10)
        
        self.publisher = self.create_publisher(InteractionData, 'interaction_data', 10)
        self.publisher2 = self.create_publisher(MarkerArray, 'detected_skeleton', 10) # Only for test purposes
        if self.to_save:
            self.publisher3 = self.create_publisher(ClassificationData, 'classification_data', 10)
            self.get_logger().info('Publisher for bag files generated!')
        else:
            self.get_logger().info('Not saving data')
            



        # Variables for processing the raw skeletons
        self.frames_of_interest = [0, 2, 3, 5, 6, 7, 12, 13, 14, 26]
        self.skeleton_size = len(self.frames_of_interest)
        self.skeleton_pair = np.full((2 * self.skeleton_size, 3), np.nan)
        self.window_size = 22
        self.window_overlap = 11 
        self.stamps_to_wait = 0
        self.stamp_to_fill = 0
        self.window = np.full((1, self.window_size, 2 * self.skeleton_size, 3), np.nan)
        self.label = None
        self.dnn_processed = False
        

        self.model_file = []
        self.model_weights = []
        self.model = []
        
        if self.topic_to_subscribe == '/body_tracking_data' or self.topic_to_subscribe == '/fake_body_tracking_data':
            self.model_file = "/home/francesco/final_system_ws/src/networks/network_weights/rnn250rnn150e200dr25_3_5_out_no_RR_2_right_weights_097.tflite"
            self.get_logger().info('Getting model from ' + self.model_file)
            self.model = tf.lite.Interpreter(model_path = self.model_file)
            self.model.allocate_tensors()
            self.get_logger().info('Model successfully loaded')


        # Variables for optimisation problem
        self.neck_left_user = Point()
        self.neck_right_user = Point()
        self.orientation_left_user = Quaternion()
        self.orientation_right_user = Quaternion()

        self.shoulder_left_user = Point()
        self.elbow_left_user = Point()
        self.wrist_left_user = Point()
        self.hand_left_user = Point()
        self.handtip_left_user = Point()
        self.spine_naval_left_user = Point()
        self.shoulder_right_user = Point()
        self.elbow_right_user = Point()
        self.wrist_right_user = Point()
        self.hand_right_user = Point()
        self.handtip_right_user = Point()
        self.spine_naval_right_user = Point()
        self.header = Header()
        self.header.frame_id = "depth_camera_link"

        #Variables for validation
        self.previous_state = 7
        self.last_received_state = None
        self.WW1 = False
        self.WW2 = False
        self.WW3 = False
        self.WP1 = False
        self.WP2 = False
        self.states = ['RR','WW','WP','WR','PW','RW','PP','None']

        #Variables to store performance
        self.ground_truth = None
        self.output_to_save = None
        self.skeleton_to_save = None

    def store_ground_truth(self, msg):
        if msg.data != self.ground_truth:
            self.ground_truth = msg.data
            self.get_logger().info('Updated ground truth to ' + str(self.ground_truth))
            #if self.ground_truth == 0: #Uncomment if you are testing with someone else
            #    time.sleep(5)


    def fill_right_interaction_data(self, msg):
        self.skeleton_pair[:self.skeleton_size, :] = self.normalise_skeleton(msg)
        self.neck_right_user = msg.markers[26].pose.position
        self.orientation_right_user = msg.markers[26].pose.orientation
        #self.neck_right_user = msg.markers[3].pose.position
        #self.orientation_right_user = msg.markers[3].pose.orientation
        self.shoulder_right_user = msg.markers[12].pose.position
        self.elbow_right_user = msg.markers[13].pose.position
        self.wrist_right_user = msg.markers[14].pose.position
        self.hand_right_user = msg.markers[15].pose.position
        self.handtip_right_user = msg.markers[16].pose.position
        #self.spine_naval_right_user = msg.markers[1].pose.position
        self.spine_naval_right_user = msg.markers[2].pose.position

    def fill_left_interaction_data(self, msg):
        self.skeleton_pair[self.skeleton_size:, :] = self.normalise_skeleton(msg)
        self.neck_left_user = msg.markers[26].pose.position
        self.orientation_left_user = msg.markers[26].pose.orientation
        #self.neck_left_user = msg.markers[3].pose.position
        #self.orientation_left_user = msg.markers[3].pose.orientation
        self.shoulder_left_user = msg.markers[12].pose.position
        self.elbow_left_user = msg.markers[13].pose.position
        self.wrist_left_user = msg.markers[14].pose.position
        self.hand_left_user = msg.markers[15].pose.position
        self.handtip_left_user = msg.markers[16].pose.position
        #self.spine_naval_left_user = msg.markers[1].pose.position
        self.spine_naval_left_user = msg.markers[2].pose.position

    def prepare_interaction_data_to_send(self, label):
        interaction_data = InteractionData()
        interaction_data.state = label #self.label CONSIDER REMOVING RR AS CLASS, SO MAYBE YOU WON'T NEED THIS
        interaction_data.header = self.header
        interaction_data.neck_right_user = self.neck_right_user
        interaction_data.neck_left_user = self.neck_left_user
        interaction_data.orientation_right_user = self.orientation_right_user
        interaction_data.orientation_left_user = self.orientation_left_user

        interaction_data.shoulder_left_user = self.shoulder_left_user
        interaction_data.elbow_left_user = self.elbow_left_user
        interaction_data.wrist_left_user = self.wrist_left_user
        interaction_data.hand_left_user = self.hand_left_user
        interaction_data.handtip_left_user = self.handtip_left_user
        interaction_data.spine_naval_left_user = self.spine_naval_left_user
        interaction_data.shoulder_right_user = self.shoulder_right_user
        interaction_data.elbow_right_user = self.elbow_right_user
        interaction_data.wrist_right_user = self.wrist_right_user
        interaction_data.hand_right_user = self.hand_right_user
        interaction_data.handtip_right_user = self.handtip_right_user
        interaction_data.spine_naval_right_user = self.spine_naval_right_user

        return interaction_data

    def validate_classification(self, label):
        if label == self.previous_state or label == self.last_received_state:
            #self.get_logger().info('Interaction not changed, nothing to update')
            return (False, None)
        elif self.previous_state == 7:# and label == 0:
            label = 0
            self.previous_state = label
            self.last_received_state = label
            #self.get_logger().info("Transitioning from default state to RR")
            self.get_logger().info("Default transition from starting state to RR")
            return (True, label)
        elif self.previous_state == 0 and label == 1:
            self.previous_state = label
            self.last_received_state = label
            self.WW1 = True
            self.get_logger().info("Transitioning from RR to WW")
            return (True, label)
        elif  self.previous_state == 1 and label == 2 and self.WW1 and not self.WW2 and not self.WW3:
            self.previous_state = label
            self.last_received_state = label
            self.WP1 = True
            self.get_logger().info("Transitioning from WW to WP the first time")
            return (True, label)
        elif self.previous_state == 2 and label == 3 and self.WP1 and not self.WP2:
            self.previous_state = label
            self.last_received_state = label
            self.get_logger().info("Transitioning from WP to WR")
            return (True, label)
        elif self.previous_state == 2 and label == 0 and self.WP1 and not self.WP2:
            label = 3
            self.previous_state = label
            self.last_received_state = label
            self.get_logger().warn("Transitioning from WP to WR (cheater case 1)")
            return (True, label)
        elif  self.previous_state == 3 and label == 1:
            self.previous_state = label
            self.last_received_state = label
            self.WW2 = True
            self.get_logger().info("Transitioning from WR to WW")
            return (True, label)
        elif self.previous_state == 1 and label == 4 and self.WW1 and self.WW2 and not self.WW3:
            self.previous_state = label
            self.last_received_state = label
            self.get_logger().info("Transitioning from WW to PW")
            return (True, label)
        elif self.previous_state == 4 and label == 5: #and self.PW:
            self.previous_state = label
            self.last_received_state = label
            self.get_logger().info("Transitioning from PW to RW")
            return (True, label)
        elif self.previous_state == 5 and label == 1:
            self.previous_state = label
            self.last_received_state = label
            self.WW3 = True
            self.WP2 = True
            self.get_logger().info("Transitioning from RW to WW")
            return (True, label)
        elif self.previous_state == 5 and label == 0 and self.WP1 and not self.WP2:
            label = 1
            self.previous_state = label
            self.last_received_state = label
            self.WW3 = True
            self.WP2 = True
            self.get_logger().warn("Transitioning from RW to WW (cheater case 2)")
            return (True, label)
        elif self.previous_state == 5 and label == 2:
            self.previous_state = label
            self.last_received_state = label
            self.WW3 = True
            self.WP2 = True
            self.get_logger().warn("Transitioning from RW to WP (tracker too slow or users very fast)")
            return (True, label)
        elif  self.previous_state == 1 and label == 4 and self.WP1 and self.WP2 and self.WW3:
            self.previous_state = label
            self.last_received_state = label
            self.get_logger().warn("Uncommon transitioning from WW to PW for the second time")
            return (True, label)
        elif  self.previous_state == 1 and label == 2 and self.WP1 and self.WP2 and self.WW3:
            self.previous_state = label
            self.last_received_state = label
            self.WW3 = True
            self.WP2 = True
            self.get_logger().info("Transitioning from WW to WP for the second time")
            return (True, label)
        elif  (self.previous_state == 1 or self.previous_state == 2 or self.previous_state == 4) and label == 6 and self.WP1 and self.WP2 and self.WW3:
            self.get_logger().info("Transitioning from WW to PP")
            self.get_logger().info("Successful completion of task, resetting variables")
            self.WW1 = False
            self.WW2 = False
            self.WW3 = False
            self.WP1 = False
            self.WP2 = False
            #self.previous_state = 7
            self.last_received_state = label
            return (True, label)
        else:
            self.get_logger().warn("Attempted wrong transition from " + self.states[self.previous_state] + " to " + self.states[int(label)] + ", label not validated!")
            self.last_received_state = label
            return (False, None)

    def normalise_skeleton(self, skeleton):
        J0 = skeleton.markers[1].pose.position
        J1 = skeleton.markers[3].pose.position

        diffs = np.full((3, 1), np.nan)
        diffs[0] = J1.x - J0.x
        diffs[1] = J1.y - J0.y
        diffs[2] = J1.z - J0.z
        norms = np.linalg.norm(diffs)

        normalised_skeleton = np.empty((self.skeleton_size, 3))

        for i in range(self.skeleton_size):
            normalised_skeleton[i, 0] = (skeleton.markers[self.frames_of_interest[i]].pose.position.x - J0.x) / norms
            normalised_skeleton[i, 1] = (skeleton.markers[self.frames_of_interest[i]].pose.position.y - J0.y) / norms
            normalised_skeleton[i, 2] = (skeleton.markers[self.frames_of_interest[i]].pose.position.z - J0.z) / norms

        return normalised_skeleton
    
    def skeleton_callback(self, msg):
        if len(msg.markers) == 0:
            self.get_logger().info('No skeletons detected yet')
            return
        elif (msg.markers[20].pose.position.z <= 0.9 or 
              msg.markers[21].pose.position.z <= 0.9 or 
              msg.markers[24].pose.position.z <= 0.9 or 
              msg.markers[25].pose.position.z <= 0.9 or 
              msg.markers[20].pose.position.z >= 2.5 or 
              msg.markers[21].pose.position.z >= 2.5 or 
              msg.markers[24].pose.position.z >= 2.5 or 
              msg.markers[25].pose.position.z >= 2.5 or
              np.abs(msg.markers[26].pose.position.x) <= 0.25):
            self.get_logger().warn('Anomaly in the detected skeleton, skipping')
            return

        self.publisher2.publish(msg) # For test purposes only

        if msg.markers[0].pose.position.x > 0:
            self.fill_right_interaction_data(msg)
        else:
            self.fill_left_interaction_data(msg)

        if not np.isnan(self.skeleton_pair).any():
            if self.stamp_to_fill < self.window_size:
                #self.get_logger().info('Window not full yet')
                self.window[0,self.stamp_to_fill,:,:] = self.skeleton_pair
                self.stamp_to_fill = self.stamp_to_fill + 1
            else:
                #self.get_logger().info('Window full, so shifting')
                self.window[0,:-1,:,:] = self.window[0,1:,:,:]
                self.window[0,-1,:,:] = self.skeleton_pair
                self.stamps_to_wait = self.stamps_to_wait + 1

        if self.label is not None and self.ground_truth is not None:
            classification_data = ClassificationData()
            classification_data.ground_truth = self.ground_truth
            classification_data.classifier_output = int(self.label)
            classification_data.skeleton = msg
            self.publisher3.publish(classification_data)
            #self.get_logger().info('Sending classification data to be stored')
        
        if not np.isnan(self.window).any() and self.stamps_to_wait == self.window_overlap and self.ground_truth is not None:
            #self.get_logger().info('Feeding window to neural network')
            self.stamps_to_wait = 0

            # Get input and output tensors details
            input_details = self.model.get_input_details()
            output_details = self.model.get_output_details()
            
            # Flatten the window for input to the model
            input_tensor = tf.convert_to_tensor(self.window, dtype=tf.float32)
            input_tensor = tf.transpose(input_tensor, [0,3,1,2])
            self.model.set_tensor(input_details[0]['index'], input_tensor)
            
            # Run the inference
            self.model.invoke()

            # Get the output tensor
            output = self.model.get_tensor(output_details[0]['index'])
            output = tf.squeeze(output)
            self.label = int(tf.argmax(output).numpy().item() + 1)
            #self.get_logger().info('Label: ' + str(self.label))
            self.dnn_processed = True

            label_to_send = self.validate_classification(self.label)

            if label_to_send[0]:
                interaction_data = self.prepare_interaction_data_to_send(label_to_send[1])
                
                
                self.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('Publishing, with interaction state: "%i"' % interaction_data.state)
                self.publisher.publish(interaction_data)

        if self.stamps_to_wait == self.window_overlap:
            self.stamps_to_wait = 0

        if self.label is not None and self.ground_truth is not None:
            classification_data = ClassificationData()
            classification_data.ground_truth = self.ground_truth
            classification_data.classifier_output = int(self.label)
            classification_data.skeleton = msg
            classification_data.dnn_processed.data = self.dnn_processed
            self.publisher3.publish(classification_data)
            self.dnn_processed = False
            #self.get_logger().info('Sending classification data to be stored')

    def fake_skeleton_callback(self, msg):

        if len(msg.marker_array.markers) == 0:
            self.get_logger().info('No skeletons detected yet')
            return
        elif msg.label == -1:
            self.get_logger().info('Task not started yet')
            self.publisher2.publish(msg.marker_array) # For test purposes only
            return
        elif (msg.marker_array.markers[20].pose.position.z <= 0.9 or 
              msg.marker_array.markers[21].pose.position.z <= 0.9 or 
              msg.marker_array.markers[24].pose.position.z <= 0.9 or 
              msg.marker_array.markers[25].pose.position.z <= 0.9 or 
              msg.marker_array.markers[20].pose.position.z >= 2.5 or 
              msg.marker_array.markers[21].pose.position.z >= 2.5 or 
              msg.marker_array.markers[24].pose.position.z >= 2.5 or 
              msg.marker_array.markers[25].pose.position.z >= 2.5 or
              np.abs(msg.marker_array.markers[26].pose.position.x) <= 0.25):
            self.get_logger().warn('Anomaly in the detected skeleton, skipping')
            return

        self.publisher2.publish(msg.marker_array) # For test purposes only

        if msg.marker_array.markers[0].pose.position.x > 0:
            self.fill_right_interaction_data(msg.marker_array)            
        else:
            self.fill_left_interaction_data(msg.marker_array)

        if not np.isnan(self.skeleton_pair).any():
            if self.stamp_to_fill < self.window_size:
                #self.get_logger().info('Window not full yet')
                self.window[0,self.stamp_to_fill,:,:] = self.skeleton_pair
                self.stamp_to_fill = self.stamp_to_fill + 1
            else:
                #self.get_logger().info('Window full, so shifting')
                self.window[0,:-1,:,:] = self.window[0,1:,:,:]
                self.window[0,-1,:,:] = self.skeleton_pair
                self.stamps_to_wait = self.stamps_to_wait + 1
        
        if not np.isnan(self.window).any() and self.stamps_to_wait == self.window_overlap and self.ground_truth is not None:
            #self.get_logger().info('Feeding window to neural network')
            self.stamps_to_wait = 0

            # Get input and output tensors details
            input_details = self.model.get_input_details()
            output_details = self.model.get_output_details()
            
            # Flatten the window for input to the model
            #input_data = self.window.reshape((1, self.window_size, 2*len(self.frames_of_interest), 3))
            input_tensor = tf.convert_to_tensor(self.window, dtype=tf.float32)
            input_tensor = tf.transpose(input_tensor, [0,3,1,2])
            self.model.set_tensor(input_details[0]['index'], input_tensor)
            
            # Run the inference
            self.model.invoke()

            # Get the output tensor
            output = self.model.get_tensor(output_details[0]['index'])
            output = tf.squeeze(output)
            self.label = int(tf.argmax(output).numpy().item() + 1)
            #self.get_logger().info('Label: ' + str(self.label))
            self.dnn_processed = True

            label_to_send = self.validate_classification(self.label)
            if label_to_send[0]:
                interaction_data = self.prepare_interaction_data_to_send(label_to_send[1])
                                
                self.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('Publishing, with interaction state: "%i"' % interaction_data.state)
                self.publisher.publish(interaction_data)

        if self.stamps_to_wait == self.window_overlap:
            self.stamps_to_wait = 0
        
        if self.label is not None and self.ground_truth is not None:
            classification_data = ClassificationData()
            classification_data.ground_truth = self.ground_truth
            classification_data.classifier_output = int(self.label)
            classification_data.skeleton = msg.marker_array
            classification_data.dnn_processed.data = self.dnn_processed
            self.publisher3.publish(classification_data)
            self.dnn_processed = False
            #self.get_logger().info('Sending classification data to be stored')

    def raw_datapoint_callback(self, msg):
        
        if len(msg.marker_array.markers) == 0:
            self.get_logger().info('No skeletons detected yet')
            return
        elif msg.label == -1:
            self.get_logger().info('Task not started yet')
            self.publisher2.publish(msg.marker_array) # For test purposes only
            return
        elif (msg.marker_array.markers[20].pose.position.z <= 0.9 or 
              msg.marker_array.markers[21].pose.position.z <= 0.9 or 
              msg.marker_array.markers[24].pose.position.z <= 0.9 or 
              msg.marker_array.markers[25].pose.position.z <= 0.9 or 
              msg.marker_array.markers[20].pose.position.z >= 2.5 or 
              msg.marker_array.markers[21].pose.position.z >= 2.5 or 
              msg.marker_array.markers[24].pose.position.z >= 2.5 or 
              msg.marker_array.markers[25].pose.position.z >= 2.5 or
              np.abs(msg.marker_array.markers[26].pose.position.x) <= 0.25):
            self.get_logger().info('Anomaly in the detected skeleton, skipping')
            return
        
        self.publisher2.publish(msg.marker_array) # For test purposes only

        if msg.label == -1:
            self.get_logger().info('Data is not usable, waiting for more...')
            return
        if msg.label == self.label:
            pass #You can enable the logger here, but it is going to spam a lot of messages
            #self.get_logger().info("Interaction not changed, nothing to update")
        else:
            if msg.marker_array.markers[0].pose.position.x > 0:
                self.fill_right_interaction_data(msg.marker_array)
            else:
                self.fill_left_interaction_data(msg.marker_array)

            interaction_data = self.prepare_interaction_data_to_send(msg.label)
            
            if self.neck_right_user == Point() or self.neck_left_user == Point():
                return
            else:
                # Add timestamp to InteractionData message
                self.label = msg.label
                self.header.stamp = self.get_clock().now().to_msg()
                interaction_data.header = self.header
                self.get_logger().info('Publishing, with interaction state: "%i"' % interaction_data.state)
                self.publisher.publish(interaction_data)


def main(args=None):
    rclpy.init(args=args)
    skeleton_processor = SkeletonProcessor()
    rclpy.spin(skeleton_processor)
    skeleton_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
