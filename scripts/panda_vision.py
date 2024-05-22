import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from panda_pick_place_msgs.msg import DetectionResults, Box, StatusHeader, RealWorldCoord, RealWorldSize

class PandaVision:
    def __init__(self) -> None:
        self.init_trigger = False
        self.detected_pub = None
        self.object_count = 0
        self.ack_sent = False
        self.K_inv = None

        # Listen for the status messages for handshaking and start the init process
        self.status_pub = rospy.Publisher('/panda_vision/status', StatusHeader, queue_size=10)
        self.status_sub = rospy.Subscriber('/panda_vision/status', StatusHeader, self.status_callback, queue_size=10)
        while not self.init_trigger and not rospy.is_shutdown():
            rospy.sleep(0.2)

        # Get the camera intrinsics
        # Create a publisher for the status on the topic /panda_vision/status
        self.status_pub.publish(status = "waiting", message = "Retrieving camera info...      ")
        self.get_camera_intrinsics()

        # Create a publisher for the detected objects
        self.detected_pub = rospy.Publisher('/panda_vision/detected_objects', DetectionResults, queue_size=10)

        # Create a publisher for the status on the topic /panda_vision/status
        self.status_pub.publish(status = "waiting", message = "Loading the model...           ")


        # Get the model path from the parameter server
        self.model_path = rospy.get_param('/panda_pick_place/model/path')

        # Check if the path exists
        if not os.path.exists(self.model_path):
            self.status_pub.publish(status = "error", message = "Model path does not exist!")
            return

        # Initialize the YOLO model
        self.model = YOLO(self.model_path)

        self.status_pub.publish(status = "waiting", message = "Model loaded successfully...   ")
        rospy.sleep(1)
        self.status_pub.publish(status = "ready", message = "Ready to detect objects!       ")

    def status_callback(self, msg: StatusHeader) -> None:
        """
        Callback function for the synchronization message

        Parameters:
        msg (StatusHeader): The status message

        Returns:
        None
        """

        # Check if the handshaking is done
        if msg.status == "syncdone" and msg.message == "Handshaking done!":
            self.init_trigger = True
        
        # Send the ack if the syn message is received and the ack is not sent
        if not self.ack_sent and  msg.status == "syn": # Send the ack if the syn message is received and the ack is not sent
            self.status_pub.publish(StatusHeader(status = "synack", message = "ACK"))
            self.ack_sent = True

        # If is a shutdown message, shutdown the node
        if msg.status == "shutdown":
            rospy.signal_shutdown(msg.message)

    def get_camera_intrinsics(self):
        """
        Retrieves the camera intrinsics from the camera_info topic

        Returns:
        None
        """
        try:
            # Listen to /camera/aligned_depth_to_color/camera_info topic to get the camera intrinsics
            camera_info = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo, timeout=5)

            # Get the camera intrinsics
            P = camera_info.P    

            # Reshape the P matrix
            P = np.array(P).reshape(3, 4)

            # K = The 3x3 left upper matrix of P
            K = camera_info.K
            K = np.array(K).reshape(3, 3)
            self.K_inv = np.linalg.inv(K)

        except rospy.ROSException:
            self.status_pub.publish(status = "error", message = "Timeout! Make sure the camera node is running!")
            return

    def start_detection(self, data: UInt8) -> None:
        """
        Starts the object detection
        It tries to detect as many objects as data says and then publishes the results on the /detected_object topic

        Parameters:
        data (UInt8): The message data

        Returns:
        None
        """

        print("OK! Let me see...")
        self.object_count = data.data
        
        if(self.object_count <= 0):
            self.status_pub.publish(status = "error", message = "Invalid data! Please enter a number greater than 0.")
            return

        print(f"You want to detect {self.object_count} objects. Let's start!")
        self.status_pub.publish(status = "info", message = "Remember to listen to /detected_object topic to get the results!")

        # Wait for a message on the camera topic and on the pointcloud topic
        try:
            # Listen for one message on the camera topic and on the pointcloud topic
            image = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5)
            pointcloud = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=5)

            # Call the detect function
            self.detect(image, pointcloud)

        except rospy.ROSException:
            self.status_pub.publish(status = "error", message = "Timeout! Make sure the camera node is running!")
            return

    def detect(self, image: Image, pointcloud: Image) -> None:
        """
        Detects objects in an image and publishes the detected object

        Parameters:
        image (Image): The image message
        pointcloud (Image): The pointcloud message

        Returns:
        None 
        """
        
        try:
            # Create the directory if it doesn't exist
            if not os.path.exists('/tmp/panda_vision'):
                os.makedirs('/tmp/panda_vision')

            bridge = CvBridge()

            # Convert the ROS Image message to a CV Image
            cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

            # Save the CV Image to a file
            cv2.imwrite("/tmp/panda_vision/current_image.jpg", cv_image)

            # Run the object detection on the saved image
            results = self.model("/tmp/panda_vision/current_image.jpg", save=True, project="/tmp/panda_vision")

            # Log the results
            self.status_pub.publish(status = "info", message = f"Detected {len(results[0].boxes.xyxy)} objects!")
            self.status_pub.publish(status = "info", message = f"Results in /tmp/panda_vision/. Confidence results: {results[0].boxes.conf}")

            # Create a DetectionResults message to publish 
            detection_msg = DetectionResults()
            detection_msg.num_objects = 0
            detection_msg.confidence = []
            detection_msg.boxes = []
            detection_msg.center_coords = []
            detection_msg.sizes = []

            # I take the box with the highest confidence and should be also higher than 0.7
            if len(results[0].boxes.xyxy) > 0 and results[0].boxes.conf[0] > 0.7:


                for i in range(len(results[0].boxes.conf)):
                    # If i is equal to the object count, break the loop
                    if i == self.object_count:
                        break

                    # If the confidence is higher than 0.7 and the object count is less than the requested object count
                    if i <= self.object_count and results[0].boxes.conf[i] > 0.7:  
                    
                        # Get the center of the bounding box
                        x1, y1, x2, y2 = results[0].boxes.xyxy[i]
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2

                        # Get the depth value of the point
                        depth_value_meters = self.get_depth_value(bridge, pointcloud, center_x, center_y)

                        detection_msg.num_objects += 1
                        detection_msg.confidence.append(float(results[0].boxes.conf[i]))
                        detection_msg.boxes.append(Box(x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2), height=pointcloud.height, width=pointcloud.width))
                        
                        center_coords, sizes = self.get_real_world_coordinates(x1, y1, x2, y2, depth_value_meters)

                        detection_msg.center_coords.append(center_coords)
                        detection_msg.sizes.append(sizes)

                # Set the status to (Success)
                detection_msg.status = StatusHeader(status = "success", message = f"Detected {detection_msg.num_objects} out of {self.object_count} requested!") 

                # Publish the detected object and the depth value
                self.detected_pub.publish(detection_msg)
                self.status_pub.publish(status = "info", message = f"Detection results: {detection_msg}")

            elif len(results[0].boxes.xyxy) > 0:
                # Set the status to (No Content)
                detection_msg.status = StatusHeader(status = "empty", message = f"Not enough confidence in the detection!") 

                self.detected_pub.publish(detection_msg)
                self.status_pub.publish(status = "warning", message = f"Not enough confidence in the detection!")
            else:
                # Set the status to (Not Found)
                detection_msg.status = StatusHeader(status = "empty", message = f"No objects detected!") 

                self.detected_pub.publish(detection_msg)
                self.status_pub.publish(status = "warning", message = f"No objects detected!")

        except Exception as e:
            self.status_pub.publish(status = "error", message = f"Error in object detection: {e}")
        
    def get_depth_value(self, bridge: CvBridge, pointcloud: Image, center_x: float, center_y: float) -> float:
        """
        Returns the depth value of a point in the pointcloud image

        Parameters:
        pointcloud (Image): The pointcloud image
        center_x (float): The normalized x coordinate of the point
        center_y (float): The normalized y coordinate of the point

        Returns:
        float: The depth value of the point in meters
        """

        # Convert the pointcloud image to a CV image
        cv_image = bridge.imgmsg_to_cv2(pointcloud, "16UC1")

        # Get the center of the bounding box
        center_x = int(center_x)
        center_y = int(center_y)
        
        # Get the depth value of the point
        depth_value = cv_image[center_y, center_x] * 0.001

        # Convert the depth value to meters
        return depth_value
    
    def get_real_world_coordinates(self, u1: int, v1: int, u2: int, v2: int, depth: float) -> tuple:
        """
        Returns the real world coordinates of a point in the image
        
        Parameters:
        u (int): The x coordinate of the point in the pixel space
        v (int): The y coordinate of the point in the pixel space
        depth (float): The depth value of the point (in meters)

        Returns:
        RealWorldCoord: The real world coordinates of the point
        """

        # Calculate [X, Y, Z]' = K^-1 * ([u*depth, v*depth, depth] - [Tx Ty 1])'
        uv1 = np.array([u1*depth, v1*depth, depth]).reshape(3, 1)
        uv2 = np.array([u2*depth, v2*depth, depth]).reshape(3, 1)

        # Calculate the real world coordinates
        xyz1 = np.matmul(self.K_inv, uv1)
        xyz2 = np.matmul(self.K_inv, uv2)

        # Center coordinates
        center_x = ((xyz1[0] + xyz2[0]) / 2)[0]
        center_y = ((xyz1[1] + xyz2[1]) / 2)[0]

        # Return the real world coordinates of the center and the size of the object
        return RealWorldCoord(X=center_x, Y=center_y, Z=depth), RealWorldSize(width=xyz2[0][0] - xyz1[0][0], height=xyz2[1][0] - xyz1[1][0])

def main():
    # Initialize the ROS node
    rospy.init_node('panda_vision')

    try:
        # Create the PandaVision object
        panda_vision = PandaVision()

        # Create a subscriber to the camera topic
        rospy.Subscriber('/panda_vision/start_detection', UInt8, panda_vision.start_detection, queue_size=10)

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

