import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy
import moveit_commander
import moveit_msgs.msg
from panda_pick_place_msgs.msg import StatusHeader, DetectionResults, DetectionRequest
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose, PoseStamped
import actionlib
import franka_gripper.msg
import math
import numpy as np
import threading

class PandaPickPlace:
    def __init__(self) -> None:
        self.init_trigger = False
        self.ack_sent = False
        self.move_arm_group = None
        self.move_hand_group = None 
        self.scene = None 
        self.display_trajectory_publisher = None
        self.pick_pose = None
        self.gripper_closed_epsilon = rospy.get_param("/panda_pick_place/gripper_closed_epsilon", 0.005)
        self.tf_matrix = rospy.get_param("/panda_pick_place/tf/tf_matrix")
        self.tf_matrix = np.array(self.tf_matrix).reshape(4, 4)
        self.init_thread = None

        # Setting pick and place poses
        pick_pose_param = rospy.get_param("/panda_pick_place/default_pick_pose")
        self.pick_pose = Pose()
        self.pick_pose.position.x = pick_pose_param[0]
        self.pick_pose.position.y = pick_pose_param[1]
        self.pick_pose.position.z = pick_pose_param[2]
        self.pick_pose.orientation.x = pick_pose_param[3]
        self.pick_pose.orientation.y = pick_pose_param[4]
        self.pick_pose.orientation.z = pick_pose_param[5]
        self.pick_pose.orientation.w = pick_pose_param[6]

        place_pose_param = rospy.get_param("/panda_pick_place/default_place_poses")
        self.place_poses_count = rospy.get_param("/panda_pick_place/place_poses_count", 2)
        self.place_poses = []
        for i in range(self.place_poses_count):
            self.place_poses.append(Pose())
            self.place_poses[i].position.x = place_pose_param[i][0]
            self.place_poses[i].position.y = place_pose_param[i][1]
            self.place_poses[i].position.z = place_pose_param[i][2]
            self.place_poses[i].orientation.x = place_pose_param[i][3]
            self.place_poses[i].orientation.y = place_pose_param[i][4]
            self.place_poses[i].orientation.z = place_pose_param[i][5]
            self.place_poses[i].orientation.w = place_pose_param[i][6]

        # Register the pub and sub for the vision
        self.start_detection_pub = rospy.Publisher("/panda_vision/start_detection", DetectionRequest, queue_size=10)
        
        self.debug_pub = rospy.Publisher("/panda_pick_place/debug", StatusHeader, queue_size=10)

        # Listen for the status messages for the handshaking and then start the init process
        self.status_pub = rospy.Publisher('/panda_pick_place/status', StatusHeader, queue_size=10)
        rospy.Subscriber('/panda_pick_place/status', StatusHeader, self.status_callback, queue_size=10)
        while not self.init_trigger and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.initController()
    
    def initController(self):
        self.status_pub.publish(StatusHeader(status = "waiting", message = "Initializing the controller... "))

        self.scene = None
        self.move_arm_group = None
        self.move_hand_group = None
        self.display_trajectory_publisher = None

        # Init the controller and get the robot commander, move group and scene
        self.init_trigger = False
        self.init_error = False
        self.init_thread = threading.Thread(target=self.initMoveGroups, args=(sys.argv,))
        self.init_thread.start()
        while not self.init_trigger and not rospy.is_shutdown():
            rospy.sleep(0.2)

        if not self.init_error:
            self.status_pub.publish(StatusHeader(status = "ready", message = "Ready to move the arm!         "))

    def initMoveGroups(self, args: list) -> None:
        """
        Initialize the moveit commander, robot commander, move group, planning scene interface and display trajectory publisher

        Parameters:
        args: the command line arguments

        Returns:
        None
        """

        try:
            rospy.wait_for_service('/get_planning_scene', timeout=5.0)
        except rospy.ROSException:
            self.status_pub.publish(StatusHeader(status = "error", message = "Check if the franka control is live!"))
            self.init_error = True
            self.init_trigger = True
            return

        try:
            # Init the moveit commander
            moveit_commander.roscpp_initialize(args)
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Moveit commander initialized..."))

            # Create a planning scene interface object
            self.scene = moveit_commander.PlanningSceneInterface()
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Planning scene initialized...  "))

            # Create a move group_arm object
            group_name = "panda_arm"
            self.move_arm_group = moveit_commander.MoveGroupCommander(group_name)
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Move group arm initialized...  "))

            self.display_trajectory_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20,
            )

            # Create a move group gripper object
            group_name = "panda_hand"
            self.move_hand_group = moveit_commander.MoveGroupCommander(group_name)
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Move group hand initialized... "))

            # Setting velocity and acceleration scaling factors
            self.move_arm_group.set_max_acceleration_scaling_factor(rospy.get_param("/panda_pick_place/acceleration_scaling_factor", 0.3))
            self.move_arm_group.set_max_velocity_scaling_factor(rospy.get_param("/panda_pick_place/acceleration_scaling_factor", 0.5))

            self.init_trigger = True
        except Exception as e:
            self.status_pub.publish(StatusHeader(status = "error", message = f"Error during the initialization: {str(e)}"))
            rospy.signal_shutdown(f"Error during the initialization: {str(e)}")

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
        if not self.ack_sent and  msg.status == "syn": 
            self.status_pub.publish(StatusHeader(status = "synack", message = "ACK"))
            self.ack_sent = True

        # If is a shutdown message, shutdown the node
        if msg.status == "shutdown":
            if self.init_thread is not None:
                self.init_thread.join(0.1)
            rospy.signal_shutdown(msg.message)

        if msg.status == "reset" and self.init_trigger:
            self.initController()

    def homing(self, msg=None, homing_pub=None) -> bool:
        """
        Move the robot to the home position
        """
        # Avoiding the loog
        if msg != None and msg.status != "homing":
            return

        try:
            # The home position of the robot is the "ready" goal state
            self.move_arm_group.set_named_target("ready")

            # Execute the trajectory
            success = self.move_arm_group.go(wait=True)

            if homing_pub != None and success:
                homing_pub.publish(StatusHeader(status = "info", message = "Homing done!"))
            elif homing_pub != None:
                homing_pub.publish(StatusHeader(status = "error", message = "Error during the homing!"))

            # Return the result
            return success
        except Exception as e:
            print(f"Error during the homing: {str(e)}")
            if homing_pub != None:
                rospy.sleep(0.4)
                homing_pub.publish(StatusHeader(status = "error", message = f"Error: {str(e)}"))
            return False

    def open_gripper(self, msg=None, pub=None) -> bool:
        """
        Open the gripper
        """

        # Avoiding the loog
        if msg != None and msg.status != "open_gripper":
            return False

        try:
            # The open position of the gripper is the "open" goal state
            self.move_hand_group.set_named_target("open")

            # Execute the trajectory
            success = self.move_hand_group.go(wait=True)

            # Calling `stop()` ensures that there is no residual movement
            self.move_hand_group.stop()

            if pub != None and success:
                pub.publish(StatusHeader(status = "info", message = "Gripper opened!"))
            elif pub != None:
                pub.publish(StatusHeader(status = "error", message = "Error opening the gripper!"))

            # Return the result
            return success
        except Exception as e:
            print(f"Error opening the gripper: {str(e)}")
            if pub != None:
                rospy.sleep(0.4)
                pub.publish(StatusHeader(status = "error", message = f"Error: {str(e)}"))
            return False
    
    def close_gripper(self, msg=None, pub=None) -> bool:
        """
        Closes the gripper
        """

        # Avoiding the loog
        if msg != None and msg.status != "close_gripper":
            return False

        try:
            # The close position of the gripper is the "close" goal state
            self.move_hand_group.set_named_target("close")

            # Execute the trajectory
            success = self.move_hand_group.go(wait=True)

            # Calling `stop()` ensures that there is no residual movement
            self.move_hand_group.stop()

            if pub != None and success:
                pub.publish(StatusHeader(status = "info", message = "Gripper closed!"))
            elif pub != None:
                pub.publish(StatusHeader(status = "error", message = "Error closing the gripper!"))

            # Return the result
            return success
        except Exception as e:
            print(f"Error closing the gripper: {str(e)}")
            if pub != None:
                rospy.sleep(0.4)
                pub.publish(StatusHeader(status = "error", message = f"Error: {str(e)}"))
            return False

    def go_to_pick_pose(self, msg=None, pub=None) -> bool:
        """
        Move the robot to the pick position
        """

        # Avoiding the loog
        if msg != None and msg.status != "go_to_pick_pose":
            return False

        try:
            # Stop the trajectory execution
            self.move_arm_group.clear_pose_targets()

            # Go to the pick pose
            success = self.move_arm_group.go(self.pick_pose, wait=True)

            if pub != None and success:
                print(pub)
                pub.publish(StatusHeader(status = "info", message = "I'm there!"))
            elif pub != None:
                pub.publish(StatusHeader(status = "error", message = "Something went wrong!"))

            # Return the result
            return success
        except Exception as e:
            print(f"Error setting the pose: {str(e)}")
            if pub != None:
                rospy.sleep(0.4)
                pub.publish(StatusHeader(status = "error", message = f"Error: {str(e)}"))
            return False

    def go_to_place_pose(self, msg=None, pub=None, pose_index:int=0) -> bool:
        """
        Move the robot to the place position
        """

        # Avoiding the loog
        if msg != None and msg.status != "go_to_place_pose":
            return False

        try:
            # Stop the trajectory execution
            self.move_arm_group.clear_pose_targets()

            # Go to the place pose
            success = self.move_arm_group.go(self.place_poses[pose_index], wait=True)

            if pub != None and success:
                pub.publish(StatusHeader(status = "info", message = "I'm there!"))
            elif pub != None:
                pub.publish(StatusHeader(status = "error", message = "Something went wrong!"))

            # Return the result
            return success
        except Exception as e:
            print(f"Error setting the pose: {str(e)}")
            if pub != None:
                rospy.sleep(0.4)
                pub.publish(StatusHeader(status = "error", message = f"Error: {str(e)}"))
            return False

    def set_pick_pose(self, msg, pub) -> None:
        """
        Set the pick pose
        """

        # Avoiding the loog
        if msg.status != "set_pick_pose":
            return

        try:
            # Set the pick pose in the parameter server 
            # Convert the current pose in a list of 7 floats
            new_pick_pose = self.move_arm_group.get_current_pose().pose
            self.pick_pose = new_pick_pose
            new_pick_pose = [new_pick_pose.position.x, new_pick_pose.position.y, new_pick_pose.position.z, new_pick_pose.orientation.x, new_pick_pose.orientation.y, new_pick_pose.orientation.z, new_pick_pose.orientation.w]
            rospy.set_param("/pick_pose", new_pick_pose)

            # Publish the status
            rospy.loginfo(f"Pick pose: {rospy.get_param('/pick_pose')}")

            pub.publish(StatusHeader(status = "success", message = f"Pick pose: {rospy.get_param('/pick_pose')}"))
        except Exception as e:
            rospy.sleep(0.4)
            pub.publish(StatusHeader(status = "error", message = f"Error setting the pose: {str(e)}"))

    def set_place_pose(self, msg, pub) -> None:
        """
        Set the place pose
        """

        # Avoiding the loog
        if msg.status != "set_place_pose":
            return

        try:
            for i in range(self.place_poses_count):
                # Set the place pose in the parameter server 
                # Convert the current pose in a list of 7 floats
                new_place_pose = self.move_arm_group.get_current_pose().pose
                self.place_poses[i] = new_place_pose
                new_place_pose = [new_place_pose.position.x, new_place_pose.position.y, new_place_pose.position.z, new_place_pose.orientation.x, new_place_pose.orientation.y, new_place_pose.orientation.z, new_place_pose.orientation.w]
                rospy.set_param(f"/place_pose_{i}", new_place_pose)    

                # Publish the status
                rospy.loginfo(f"Place pose: {rospy.get_param('/place_pose')}")
                rospy.wait_for_message("/panda_pick_place/set_place_pose", StatusHeader, timeout=30)

            pub.publish(StatusHeader(status = "success", message = f"Place pose: {rospy.get_param('/place_pose')}"))
        except Exception as e:
            rospy.sleep(0.4)
            pub.publish(StatusHeader(status = "error", message = f"Error setting the pose: {str(e)}"))

    def grasp(self):
        """
        Grasp the object
        """

        # Trying to grasp the object
        # To grasp the object I try to call franka_gripper::GraspAction(width, epsilon_inner, epsilon_outer, speed, force)
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

        # Wait for the action server to start
        rospy.logdebug("Waiting the server to start...")
        client.wait_for_server()

        # Create a goal
        goal = franka_gripper.msg.GraspGoal()
        goal.width = 0.0 # m
        goal.speed = 0.05 # m/s
        goal.force = 10 # N
        goal.epsilon.inner = 0.04
        goal.epsilon.outer = 0.08

        # Send the goal to the action server
        client.send_goal(goal)

        # Wait for the server to finish performing the action
        rospy.loginfo("Grasping the object...")
        client.wait_for_result()

        # Get the result
        result = client.get_result()
        rospy.loginfo(f"Grasp result: {result}")

        return result

    def get_initial_gripper_angle(self, gx, gy) -> float:
        """
        Return the link8 joint value based on the end effector position
        The formula is 
        phi = tan^-1|gy/gx|
        if gx * gy > 0 => return -phi
        else => return phi

        Parameters:
        gx (float): The x coordinate of the end effector
        gy (float): The y coordinate of the end effector

        Returns:
        float: The angle of the gripper
        """

        # Calculate the angle
        angle = math.atan(abs(gy/gx))

        # Check the sign
        if gx * gy < 0:
            return -angle

        return angle
    
    def rotate_gripper(self, angle) -> bool:
        """
        Rotate the gripper vertically
        """

        # Setting the joint values for the gripper rotated by 90 degrees
        current_joint_values = self.move_arm_group.get_current_joint_values()
        current_joint_values[6] = angle
        self.move_arm_group.set_joint_value_target(current_joint_values)

        # Rotate the gripper
        success = self.move_arm_group.go(wait=True)

        return success

    def find_optimal_gripper_angle(self, current_width, obj_id) -> tuple:
        """
        """

        rotation_step = rospy.get_param("/panda_pick_place/rotation_step", math.pi/16)

        # Get the current joint values
        current_joint_values = self.move_arm_group.get_current_joint_values()

        # Rotate rotation_step, detect and check if i increased or decreased the width
        self.rotate_gripper(current_joint_values[6] + rotation_step)
        obj_detected = self.detect_object(obj_id=obj_id)

        try:
            # If for this angle the object is already graspable return the angle
            if obj_detected.num_objects == 1 and obj_detected.sizes[0].width < 0.07:
                return current_joint_values[6] + rotation_step, obj_detected.sizes[0].width
            
            # If i minimized the width of the object, continue on this path
            if obj_detected.num_objects == 1 and (obj_detected.sizes[0].width > self.gripper_closed_epsilon and obj_detected.sizes[0].height > self.gripper_closed_epsilon) and obj_detected.sizes[0].width < current_width:
                min_width = obj_detected.sizes[0].width + 1 # So i can enter the while loop
                min_angle = current_joint_values[6] + rotation_step
                while obj_detected.num_objects == 1 and obj_detected.sizes[0].width > self.gripper_closed_epsilon and obj_detected.sizes[0].width < min_width:
                    min_width = obj_detected.sizes[0].width
                    min_angle += rotation_step
                    print(min_angle)
                    self.rotate_gripper(min_angle + rotation_step)
                    obj_detected = self.detect_object(obj_id=obj_id, old_box=obj_detected.boxes[0])

                print(f"Min angle found: {min_angle}")
                print(f"Min width found: {min_width}")

                return min_angle, min_width
            
            # If i increased the width of the object, continue on this path and then give the angle found rotated by -math.pi/2
            if obj_detected.num_objects == 1 and obj_detected.sizes[0].width > current_width:
                max_angle = current_joint_values[6] + rotation_step
                min_height = obj_detected.sizes[0].height + 1 # So i can enter the while loop
                while obj_detected.num_objects == 1 and (obj_detected.sizes[0].width > self.gripper_closed_epsilon and obj_detected.sizes[0].height > self.gripper_closed_epsilon) and obj_detected.sizes[0].height < min_height:
                    min_height = obj_detected.sizes[0].height
                    max_angle += rotation_step

                    current_joint_values = self.move_arm_group.get_current_joint_values()
                    self.rotate_gripper(max_angle + rotation_step)
                    obj_detected = self.detect_object(obj_id=obj_id, old_box=obj_detected.boxes[0])

                print(f"Max angle found: {max_angle}")
                print(f"Min height found: {min_height}")
                print(f"Rotating by -math.pi/2: {max_angle - math.pi/2}")

                return max_angle - math.pi/2, min_height
            
            return current_joint_values[6], current_width
        except Exception as e:
            print(f"Error finding the optimal gripper angle: {str(e)}")
            return current_joint_values[6], current_width

    def _find_optimal_gripper_angle(self, max_angle, current_width) -> tuple:
        """
        Find the optimal gripper angle based on the object's dimensions

        Parameters:
        object (DetectionResults): The detected object

        Returns:
        tuple: The optimal gripper angle and a boolean indicating if the angle was found
        """


        current_joint_values = self.move_arm_group.get_current_joint_values()
        if max_angle < math.pi/16:
            print("Max angle reached")
            return current_joint_values[6], current_width
        try:
            # I detect the object from two angles: current - max_angle/2 and current + max_angle/2
            print(f"Going to {current_joint_values[6] + max_angle/2}")
            self.rotate_gripper(current_joint_values[6] + max_angle/2)
            object1 = self.detect_object()

            # If for this angle the object is already graspable return the angle
            print("------------------------462")
            if object1.num_objects == 1 and object1.sizes[0].width < 0.07:
                print("------------------------464")
                return current_joint_values[6] + max_angle/2, object1.sizes[0].width
            # If i minimized the width of the object, continue on this path
            print("------------------------466")
            if object1.num_objects == 1 and object1.sizes[0].width > self.gripper_closed_epsilon and object1.sizes[0].width < current_width:
                print("------------------------469")
                return self.find_optimal_gripper_angle(max_angle/2, object1.sizes[0].width)

            print(f"Going to {current_joint_values[6] - max_angle/2}")
            self.rotate_gripper(current_joint_values[6] - max_angle)
            object2 = self.detect_object()

            # If for this angle the object is already graspable return the angle
            print("------------------------477")
            if object2.num_objects == 1 and object2.sizes[0].width < 0.07:
                print("------------------------479")
                return current_joint_values[6] - max_angle/2, object2.sizes[0].width
            # If i minimized the width of the object, continue on this path
            print("------------------------482")
            print(current_width)
            if object2.num_objects == 1 and object2.sizes[0].width > self.gripper_closed_epsilon and object2.sizes[0].width < current_width:
                print("------------------------484")
                return self.find_optimal_gripper_angle(max_angle/2, object2.sizes[0].width)

            print("Stopped the search")
            return current_joint_values[6], current_width
        except Exception as e:
            print(f"Error finding the optimal gripper angle: {str(e)}")
            return current_joint_values[6], current_width

        

    def detect_object(self, obj_count=1, obj_id=-1, confidence=0.7, old_box=None) -> DetectionResults:
        """
        Detect the object
        """
        
        try:
            # Try to detect the object
            self.start_detection_pub.publish(DetectionRequest(obj_count=obj_count, obj_id=obj_id, confidence=confidence, old_box=old_box))

            # Wait for the detection to be done or timeout after 10 seconds
            object: DetectionResults = rospy.wait_for_message("/panda_vision/detected_objects", DetectionResults, timeout=10)

            return object
        except Exception as e:
            print(f"Error detecting the object: {str(e)}")
            return DetectionResults()

    def pick_place_routine(self, msg: StatusHeader, pub, retry=False) -> bool:
        """
        Pick the object
        """

        if not (msg.status == "pick_place_routine"):
            return
        
        # Homing
        homing_result = self.homing()
        
        # Go to the pick pose
        pick_pose_result = self.go_to_pick_pose()

        # Open the gripper
        gripper_result = self.open_gripper()

        if pick_pose_result and gripper_result:
            # Try to detect an object
            object = self.detect_object()

            # If an object is detected, try to grasp it
            if object.num_objects == 1:
                
                # Grasp the object:
                # 1. Rotate the gripper along the shortest dimension of the object
                # 2. Move the gripper object_center_x meters with cartesian path planning
                # 3. Move the gripper object_center_y meters with cartesian path planning
                # 4. Move the gripper object_center_z meters with cartesian path planning
                # 5. Grasp!
                try:
                    coords = np.array([[object.center_coords[0].X - 0.009, object.center_coords[0].Y + 0.02, object.center_coords[0].Z, 1]]).T
                    coords = np.dot(self.tf_matrix, coords)

                    # Move the gripper with cartesian path planning
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "panda_hand"
                    
                    # PRE GRASP
                    # Check if the shortest dimension of the object is less than the gripper width
                    gripper_axis_angle = 0
                    if(object.sizes[0].width < object.sizes[0].height):
                        shortest_dimension = object.sizes[0].width
                        gripper_axis_angle = math.pi/4
                    else:
                        shortest_dimension = object.sizes[0].height
                        gripper_axis_angle = -math.pi/4

                    # Transform from the camera frame to hand frame    
                    offset = 0.3           
                    camera_from_gripper_z = 0.09

                    target_pose.pose.position.x = coords[0]
                    target_pose.pose.position.y = coords[1]
                    target_pose.pose.position.z = coords[2] - offset - camera_from_gripper_z

                    success = self.move_arm_group.go(target_pose, wait=True)

                    if not success:
                        pub.publish(StatusHeader(status = "error", message = "Error moving the gripper"))
                        return False
                    
                    # If the object is outside the center area of the camera, try to detect it again when the gripper is above the object
                    # Then calculate the position again and move the gripper
                    while abs(object.center_coords[0].X) > 0.09 or abs(object.center_coords[0].Y) > 0.09:
                        print(f"Object outside the center area: {object.center_coords[0].X}, {object.center_coords[0].Y}")
                        # Detect the object again
                        object = self.detect_object(obj_id=object.ids[0])

                        # Calculate the new position
                        coords = np.array([[object.center_coords[0].X - 0.009, object.center_coords[0].Y + 0.02, object.center_coords[0].Z, 1]]).T
                        coords = np.dot(self.tf_matrix, coords)

                        # Move the gripper with cartesian path planning
                        target_pose = PoseStamped()
                        target_pose.header.frame_id = "panda_hand"

                        target_pose.pose.position.x = coords[0]
                        target_pose.pose.position.y = coords[1]
                        target_pose.pose.position.z = coords[2] - offset - camera_from_gripper_z

                        success = self.move_arm_group.go(target_pose, wait=True)

                        if not success:
                            pub.publish(StatusHeader(status = "error", message = "Error moving the gripper"))
                            return False

                    # Get the initial gripper angle
                    # Rotate the gripper
                    current_pose = self.move_arm_group.get_current_pose().pose
                    initial_gripper_angle = self.get_initial_gripper_angle(current_pose.position.x, current_pose.position.y)
                    print(f"Initial gripper angle: {initial_gripper_angle}")

                    success = self.rotate_gripper(initial_gripper_angle + gripper_axis_angle)
                    
                    # Check if the object is too big to be grasped and try to find a better angle
                    if shortest_dimension >= 0.077:
                        # To start the algorithm, I need to be on the object rotated by gripper_axis_angle
                        gripper_axis_angle, optimal_width = self.find_optimal_gripper_angle(shortest_dimension, object.ids[0])
                        # current_joint_values = self.move_arm_group.get_current_joint_values()
                        # initial_gripper_angle = current_joint_values[6]
                        initial_gripper_angle = 0
                        print(f"Optimal gripper angle: {gripper_axis_angle}")
                        print(f"Optimal width: {optimal_width}")
                        if optimal_width > 0.077 or optimal_width < self.gripper_closed_epsilon:
                            if retry:
                                return self.pick_place_routine(StatusHeader(status = "pick_place_routine"), pub, retry=False)
                            pub.publish(StatusHeader(status = "error", message = "The object is too big to be grasped"))
                            return False
                        
                    pub.publish(StatusHeader(status = "debug", message = "Check if is everything ok"))

                    # Rotate the gripper vertically
                    success = self.rotate_gripper(gripper_axis_angle + initial_gripper_angle)

                    if not success:
                        print("Error rotating the gripper")
                        pub.publish(StatusHeader(status = "error", message = "Error rotating the gripper"))
                        return False
                    
                    print("Rotated the gripper")

                    # GRASP                    
                    # Moving cartesian
                    grasp_pose = self.move_arm_group.get_current_pose().pose
                    grasp_pose.position.z -= offset
                    path, _ = self.move_arm_group.compute_cartesian_path([grasp_pose], 0.01, 0.0, avoid_collisions=True)

                    success = self.move_arm_group.execute(path, wait=True)

                    if not success:
                        pub.publish(StatusHeader(status = "error", message = "Error moving the gripper"))
                        return False           
                    
                    # Ask if debug is enabled
                    try:
                        # Publish the debug message
                        self.debug_pub.publish(StatusHeader(status = "ask", message = "Waiting for the debug message"))
                        debug_msg = rospy.wait_for_message("/panda_pick_place/debug", StatusHeader, timeout=10)

                        if debug_msg.status == "wait":
                            go_msg = rospy.wait_for_message("/panda_pick_place/debug", StatusHeader, timeout=10)
                    except Exception as e:
                        print(f"Error waiting for the debug message: {str(e)}")

                    # Grasp the object
                    self.grasp()

                    # Check if the object is grasped
                    finger1_width = self.move_hand_group.get_current_joint_values()[0]
                    finger2_width = self.move_hand_group.get_current_joint_values()[1]
                    print(f"Finger 1 width: {finger1_width}")
                    print(f"Finger 2 width: {finger2_width}")
                    if finger1_width+finger2_width < self.gripper_closed_epsilon:
                        pub.publish(StatusHeader(status = "error", message = "Failed to grasp!"))
                        if retry:
                            return self.pick_place_routine(StatusHeader(status = "pick_place_routine"), pub, retry=False)
                        return False
                    
                    # Return to the pick pose
                    result_pick_pose = self.go_to_pick_pose()
                    if not result_pick_pose:
                        pub.publish(StatusHeader(status = "error", message = "Error moving to pick pose"))
                        return False

                    # PICKED!
                    print("Object picked!")
                    pub.publish(StatusHeader(status = "done", message = "Object picked!"))

                    # Go to the place pose
                    result = self.go_to_place_pose(pose_index=object.classes[0])
                    if not result:
                        pub.publish(StatusHeader(status = "error", message = "Error moving to place pose"))
                        return False
                    
                    # Place the object
                    self.open_gripper()
                    return True
                except Exception as e:
                    pub.publish(StatusHeader(status = "error", message = f"{str(e)}"))
                    return False
                
                    
            else:
                pub.publish(StatusHeader(status = "error", message = "No object detected"))
                return False
            
        else:
            pub.publish(StatusHeader(status = "error", message = "Error moving to pick pose"))
            return False
        
    def clean_table_routine(self, msg: StatusHeader, pub) -> None:
        """
        Clean the table
        """

        if not (msg.status == "clean_table_routine"):
            return

        success = True
        while success:
            success = self.pick_place_routine(StatusHeader(status = "pick_place_routine"), pub, retry=True)

        print("Table cleaned!")

        pub.publish(StatusHeader(status = "done", message = "Table cleaned!"))

    def place_object(self) -> None:
        """
        Place the object
        """

        # Go to the place pose
        self.go_to_place_pose(move_arm_group=self.move_arm_group)

        # Open the gripper
        self.open_gripper(move_hand_group=self.move_hand_group)

    def pick(self) -> None:
        """
        Pick and place routine
        """

        # # Go to the pick pose
        # print("Press any key to go to the pick pose")
        # input()
        # success = self.go_to_pick_pose()
        # print(f"Pick pose result: {success}")

        # # Go to the place pose
        # print("Press any key to go to the place pose")
        # input()
        # success = self.go_to_place_pose()
        # print(f"Place pose result: {success}")

        # # Open the gripper
        # self.open_gripper()

def main():
    # Init the node
    rospy.init_node('panda_pick_place')

    try:
        panda_pick_place = PandaPickPlace()

        # Homing topic pub
        homing_pub = rospy.Publisher('/panda_pick_place/homing', StatusHeader, queue_size=10)
        # Subscribe to the homing topic
        rospy.Subscriber('/panda_pick_place/homing', StatusHeader, lambda msg: panda_pick_place.homing(msg, homing_pub), queue_size=10)

        # Open gripper pub
        open_gripper_pub = rospy.Publisher('/panda_pick_place/open_gripper', StatusHeader, queue_size=10)
        # Subscribe to the open gripper topic
        rospy.Subscriber('/panda_pick_place/open_gripper', StatusHeader, lambda msg: panda_pick_place.open_gripper(msg, open_gripper_pub), queue_size=10)

        # Close gripper pub
        close_gripper_pub = rospy.Publisher('/panda_pick_place/close_gripper', StatusHeader, queue_size=10)
        # Subscribe to the close gripper topic
        rospy.Subscriber('/panda_pick_place/close_gripper', StatusHeader, lambda msg: panda_pick_place.close_gripper(msg, close_gripper_pub), queue_size=10)

        # Set pick pose pub
        set_pick_pose_pub = rospy.Publisher('/panda_pick_place/set_pick_pose', StatusHeader, queue_size=10)
        # Subscribe to the set pick pose topic
        rospy.Subscriber('/panda_pick_place/set_pick_pose', StatusHeader, lambda msg: panda_pick_place.set_pick_pose(msg, set_pick_pose_pub), queue_size=10)

        # Set place pose pub
        set_place_pose_pub = rospy.Publisher('/panda_place_place/set_place_pose', StatusHeader, queue_size=10)
        # Subscribe to the set place pose topic
        rospy.Subscriber('/panda_pick_place/set_place_pose', StatusHeader, lambda msg: panda_pick_place.set_place_pose(msg, set_place_pose_pub), queue_size=10)

        # Pick and place routine pub
        pick_place_routine_pub = rospy.Publisher('/panda_pick_place/pick_place_routine', StatusHeader, queue_size=10)
        # Subscribe to the pick and place routine topic
        rospy.Subscriber('/panda_pick_place/pick_place_routine', StatusHeader, lambda msg: panda_pick_place.pick_place_routine(msg, pick_place_routine_pub), queue_size=10)
        
        # Clean the table routine pub
        clean_table_routine_pub = rospy.Publisher('/panda_pick_place/clean_table_routine', StatusHeader, queue_size=10)
        # Subscribe to the clean table routine topic
        rospy.Subscriber('/panda_pick_place/clean_table_routine', StatusHeader, lambda msg: panda_pick_place.clean_table_routine(msg, clean_table_routine_pub), queue_size=10)

        # Go to pick pose pub
        go_to_pick_pose_pub = rospy.Publisher('/panda_place_place/go_to_pick_pose', StatusHeader, queue_size=10)
        # Subscribe to go to pick pose topic
        rospy.Subscriber('/panda_pick_place/go_to_pick_pose', StatusHeader, lambda msg: panda_pick_place.go_to_pick_pose(msg, go_to_pick_pose_pub), queue_size=10)

        # Go to place pose pub
        go_to_place_pose_pub = rospy.Publisher('/panda_place_place/go_to_place_pose', StatusHeader, queue_size=10)
        # Subscribe to go to place pose topic
        rospy.Subscriber('/panda_pick_place/go_to_place_pose', StatusHeader, lambda msg: panda_pick_place.go_to_place_pose(msg, go_to_place_pose_pub), queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()