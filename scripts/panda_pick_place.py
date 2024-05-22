import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy
import moveit_commander
import moveit_msgs.msg
from panda_pick_place_msgs.msg import StatusHeader, DetectionResults
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose, PoseStamped
import actionlib
import franka_gripper.msg
import math

class PandaPickPlace:
    def __init__(self) -> None:
        self.init_trigger = False
        self.ack_sent = False
        self.robot = None 
        self.move_arm_group = None
        self.move_hand_group = None 
        self.scene = None 
        self.display_trajectory_publisher = None
        self.pick_pose = None
        self.tf_matrix = rospy.get_param("/panda_pick_place/tf/tf_matrix")

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

        place_pose_param = rospy.get_param("/panda_pick_place/default_place_pose")
        self.place_pose = Pose()
        self.place_pose.position.x = place_pose_param[0]
        self.place_pose.position.y = place_pose_param[1]
        self.place_pose.position.z = place_pose_param[2]
        self.place_pose.orientation.x = place_pose_param[3]
        self.place_pose.orientation.y = place_pose_param[4]
        self.place_pose.orientation.z = place_pose_param[5]
        self.place_pose.orientation.w = place_pose_param[6]


        # Register the pub and sub for the vision
        self.start_detection_pub = rospy.Publisher("/panda_vision/start_detection", UInt8, queue_size=10)
        self.pick_pub = rospy.Publisher("/panda_pick_place/pick", StatusHeader, queue_size=10)

        # Listen for the status messages for the handshaking and then start the init process
        self.status_pub = rospy.Publisher('/panda_pick_place/status', StatusHeader, queue_size=10)
        rospy.Subscriber('/panda_pick_place/status', StatusHeader, self.status_callback, queue_size=10)
        while not self.init_trigger and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.status_pub.publish(StatusHeader(status = "waiting", message = "Initializing the controller... "))

        # Init the controller and get the robot commander, move group and scene
        self.initController(sys.argv)
        self.status_pub.publish(StatusHeader(status = "ready", message = "Ready to move the arm!         "))

    def initController(self, args: list) -> None:
        """
        Initialize the moveit commander, robot commander, move group, planning scene interface and display trajectory publisher

        Parameters:
        args: the command line arguments

        Returns:
        None
        """

        try:
            # Init the moveit commander
            moveit_commander.roscpp_initialize(args)
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Moveit commander initialized..."))

            # Create a robot commander object
            self.robot = moveit_commander.RobotCommander()
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Robot commander initialized... "))

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

            # Set the planning time
            self.move_arm_group.set_planning_time(30)

            # Set the reference frame
            reference_frame = 'panda_link0'
            self.move_arm_group.set_pose_reference_frame(reference_frame)
            self.move_arm_group.set_planning_pipeline_id("ompl")

            # Set the end effector link
            end_effector_link = 'panda_link8'
            self.move_arm_group.set_end_effector_link(end_effector_link)

            # Create a move group gripper object
            group_name = "panda_hand"
            self.move_hand_group = moveit_commander.MoveGroupCommander(group_name)
            self.status_pub.publish(StatusHeader(status = "waiting", message = "Move group hand initialized... "))

            # Setting velocity and acceleration scaling factors
            self.move_arm_group.set_max_acceleration_scaling_factor(rospy.get_param("/panda_pick_place/acceleration_scaling_factor", 0.7))
            self.move_arm_group.set_max_velocity_scaling_factor(rospy.get_param("/panda_pick_place/acceleration_scaling_factor", 0.7))
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
            rospy.signal_shutdown(msg.message)

    def homing(self) -> bool:
        """
        Move the robot to the home position
        """

        # The home position of the robot is the "ready" goal state
        self.move_arm_group.set_named_target("ready")

        # Execute the trajectory
        success = self.move_arm_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_arm_group.stop()

        # Return the result
        return success

    def open_gripper(self) -> bool:
        """
        Open the gripper
        """

        # The open position of the gripper is the "open" goal state
        self.move_hand_group.set_named_target("open")

        # Execute the trajectory
        success = self.move_hand_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_hand_group.stop()

        # Return the result
        return success
    
    def close_gripper(self) -> bool:
        """
        Closes the gripper
        """

        # The close position of the gripper is the "close" goal state
        self.move_hand_group.set_named_target("close")

        # Execute the trajectory
        success = self.move_hand_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_hand_group.stop()

        # Return the result
        return success

    def go_to_place_pose(self) -> bool:
        """
        Move the robot to the place position
        """

        try:
            # Stop the trajectory execution
            self.move_arm_group.clear_pose_targets()

            # Set plan pose
            # success, traj, _, __ = self.move_arm_group.plan(self.place_pose)
            # success = self.move_arm_group.execute(traj, wait=True)
            path, _ = self.move_arm_group.compute_cartesian_path([self.place_pose], 0.01, 0.0)

            # Execute the trajectory with execute()
            success = self.move_arm_group.execute(path, wait=True)

            # Calling `stop()` ensures that there is no residual movement
            self.move_arm_group.stop()

            # Opening the gripper
            open_hand_success = self.open_gripper()

            # Return the result
            return success and open_hand_success
        except Exception as e:
            self.status_pub.publish(StatusHeader(status = "error", message = f"Error opening the gripper: {str(e)}"))
            return False

    def go_to_pick_pose(self) -> bool:
        """
        Move the robot to the pick position
        """

        try:
            # Stop the trajectory execution
            self.move_arm_group.clear_pose_targets()

            # Open the gripper
            open_hand_success = self.open_gripper()
            if not open_hand_success:
                return False

            # Set plan pose
            path, _ = self.move_arm_group.compute_cartesian_path([self.pick_pose], 0.01, 0.0)

            # Execute the trajectory with execute()
            success = self.move_arm_group.execute(path, wait=True)

            # Calling `stop()` ensures that there is no residual movement
            self.move_arm_group.stop()

            # Return the result
            return success
        except Exception as e:
            self.status_pub.publish(StatusHeader(status = "error", message = f"Error opening the gripper: {str(e)}"))
            return False    

    def pick_place_routine(self) -> bool:
        """
        Pick and place routine
        """

        # Go to the pick pose
        # print("Press any key to go to the pick pose")
        # input()
        self.status_pub.publish(StatusHeader(status = "info", message = "Going to the pick pose...    "))
        success = self.go_to_pick_pose(move_arm_group=self.move_arm_group)
        self.status_pub.publish(StatusHeader(status = "info", message = f"Pick pose result: {success}"))

        # Go to the place pose
        # print("Press any key to go to the place pose")
        # input()
        self.status_pub.publish(StatusHeader(status = "info", message = "Going to the place pose...   "))
        success = self.go_to_place_pose(move_arm_group=self.move_arm_group, move_hand_group=self.move_hand_group)
        self.status_pub.publish(StatusHeader(status = "info", message = f"Place pose result: {success}"))

        # Open the gripper
        self.open_gripper(move_hand_group=self.move_hand_group)

        # Return the result
        return success

    def set_pick_pose(self) -> None:
        """
        Set the pick pose
        """

        try:
            # Set the pick pose in the parameter server 
            # Convert the current pose in a list of 7 floats
            new_pick_pose = self.move_arm_group.get_current_pose().pose
            self.pick_pose = new_pick_pose
            new_pick_pose = [new_pick_pose.position.x, new_pick_pose.position.y, new_pick_pose.position.z, new_pick_pose.orientation.x, new_pick_pose.orientation.y, new_pick_pose.orientation.z, new_pick_pose.orientation.w]
            rospy.set_param("/pick_pose", new_pick_pose)

            # Print the pick pose
            # print("Current pose: ", self.move_arm_group.get_current_pose())
            print("Pick pose: ", rospy.get_param("/pick_pose"))
        except Exception as e:
            self.status_pub.publish(StatusHeader(status = "error", message = f"Error setting the pick pose: {str(e)}"))

    def set_place_pose(self) -> None:
        """
        Set the place pose
        """

        # Set the place pose in the parameter server 
        # Convert the current pose in a list of 7 floats
        new_place_pose = self.move_arm_group.get_current_pose().pose
        self.place_pose = new_place_pose
        new_place_pose = [new_place_pose.position.x, new_place_pose.position.y, new_place_pose.position.z, new_place_pose.orientation.x, new_place_pose.orientation.y, new_place_pose.orientation.z, new_place_pose.orientation.w]
        rospy.set_param("/place_pose", new_place_pose)        

        # Print the place pose
        # print("Current pose: ", self.move_arm_group.get_current_pose())
        print("Place pose: ", rospy.get_param("/place_pose"))

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
        goal.speed = 0.1 # m/s
        goal.force = 2 # N
        goal.epsilon.inner = 0.04
        goal.epsilon.outer = 0.04

        # Send the goal to the action server
        client.send_goal(goal)

        # Wait for the server to finish performing the action
        rospy.logdebug("Grasping the object...")
        client.wait_for_result()

        # Get the result
        result = client.get_result()
        rospy.loginfo(f"Grasp result: {result}")

        return result

    def pick_object(self, msg: StatusHeader) -> None:
        """
        Pick the object
        """

        if not (msg.status == "pick"):
            return

        # Go to the pick pose
        result = self.go_to_pick_pose()
        if result:
            # Try to detect an object
            self.start_detection_pub.publish(1)

            # Wait for the detection to be done or timeout after 10 seconds
            object: DetectionResults = rospy.wait_for_message("/panda_vision/detected_objects", DetectionResults, timeout=10)

            # If an object is detected, try to grasp it
            if object.num_objects == 1:
                # Check if the shortest dimension of the object is less than the gripper width
                if(object.sizes[0].width < object.sizes[0].height):
                    shortest_dimension = object.sizes[0].width
                else:
                    shortest_dimension = object.sizes[0].height
                if shortest_dimension >= 0.08:
                    self.status_pub.publish(StatusHeader(status = "warn", message = "The object is too big to be grasped"))
                    self.pick_pub.publish(StatusHeader(status = "warn", message = "The object is too big to be grasped"))
                    return
                
                # Grasp the object:
                # 1. Rotate the gripper along the shortest dimension of the object
                # 2. Move the gripper object_center_x meters with cartesian path planning
                # 3. Move the gripper object_center_y meters with cartesian path planning
                # 4. Move the gripper object_center_z meters with cartesian path planning
                # 5. Grasp!
                if(object.sizes[0].width > object.sizes[0].height):
                    try:
                        # Setting the joint values for the gripper rotated by 90 degrees
                        print(self.move_arm_group.get_current_joint_values())
                        current_joint_values = self.move_arm_group.get_current_joint_values()
                        current_joint_values[6] = math.pi/4
                        self.move_arm_group.set_joint_value_target(current_joint_values)

                        # Rotate the gripper
                        success = self.move_arm_group.go(wait=True)

                        if not success:
                            self.status_pub.publish(StatusHeader(status = "error", message = "Error rotating the gripper"))
                            return

                        self.move_arm_group.stop()

                        # Move the gripper with cartesian path planning
                        target_pose = PoseStamped()
                        target_pose.header.frame_id = "camera_link"
                        target_pose.pose.position.x = object.center_coords[0].X
                        waypoints = [target_pose.pose]
                        target_pose.pose.position.y = object.center_coords[0].Y
                        waypoints.append(target_pose.pose)
                        target_pose.pose.position.z = object.center_coords[0].Z
                        waypoints.append(target_pose.pose)

                        (plan, _) = self.move_arm_group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step in meters
                            0.0)         # jump_threshold
                        
                        success = self.move_arm_group.execute(plan, wait=True)

                        if not success:
                            self.status_pub.publish(StatusHeader(status = "error", message = "Error moving the gripper"))
                            return

                        self.move_arm_group.stop()

                        # Grasp the object
                        self.grasp()

                        # Check if the object is grasped
                        finger1_width = self.move_hand_group.get_current_joint_values()[0]
                        if finger1_width == 0:
                            self.status_pub.publish(StatusHeader(status = "warn", message = "Failed to grasp!"))
                            self.pick_pub.publish(StatusHeader(status = "warn", message = "The object is too big to be grasped"))
                            return
                        
                        # Return to the pick pose
                        result = self.go_to_pick_pose()
                        if not result:
                            self.status_pub.publish(StatusHeader(status = "error", message = "Error moving to pick pose"))
                            return

                        # PICKED!
                        self.status_pub.publish(StatusHeader(status = "info", message = "Object picked!"))
                        self.pick_pub.publish(StatusHeader(status = "picked", message = "Object picked!"))
                        return
                    except Exception as e:
                        self.status_pub.publish(StatusHeader(status = "error", message = f"Error rotating the gripper: {str(e)}"))
                        return
                    
                else:
                    rospy.loginfo(f"-----------------------------------378")
                    try:
                        # Move the gripper with cartesian path planning
                        target_pose = PoseStamped()
                        target_pose.header.frame_id = "camera_link"
                        target_pose.pose.position.x = object.center_coords[0].X
                        waypoints = [target_pose.pose]
                        target_pose.pose.position.y = object.center_coords[0].Y
                        waypoints.append(target_pose.pose)
                        target_pose.pose.position.z = object.center_coords[0].Z
                        waypoints.append(target_pose.pose)

                        (plan, fraction) = self.move_arm_group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step in meters
                            0.0)         # jump_threshold
                        
                        self.move_arm_group.execute(plan, wait=True)
                        self.move_arm_group.stop()

                        # Grasp the object
                        self.grasp()

                        # Check if the object is grasped
                        finger1_width = self.move_hand_group.get_current_joint_values()[0]
                        if finger1_width == 0:
                            self.status_pub.publish(StatusHeader(status = "warn", message = "Failed to grasp!"))
                            self.pick_pub.publish(StatusHeader(status = "warn", message = "The object is too big to be grasped"))
                            return
                        
                        # Return to the pick pose
                        result = self.go_to_pick_pose()
                        if not result:
                            self.status_pub.publish(StatusHeader(status = "error", message = "Error moving to pick pose"))
                            return

                        # PICKED!
                        self.status_pub.publish(StatusHeader(status = "info", message = "Object picked!"))
                        self.pick_pub.publish(StatusHeader(status = "picked", message = "Object picked!"))
                        return
                    except Exception as e:
                        self.status_pub.publish(StatusHeader(status = "error", message = f"Error rotating the gripper: {str(e)}"))
                        return
                    
            else:
                self.pick_pub.publish(StatusHeader(status = "warn", message = "No object detected"))
                return
            
        else:
            self.status_pub.publish(StatusHeader(status = "error", message = "Error moving to pick pose"))
            return
        

    def place_object(self) -> None:
        """
        Place the object
        """

        # Go to the place pose
        self.go_to_place_pose(move_arm_group=self.move_arm_group)

        # Open the gripper
        self.open_gripper(move_hand_group=self.move_hand_group)

    def pick_place_routine(self) -> None:
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

        # Subscribe to the homing topic
        rospy.Subscriber('/panda_pick_place/homing', StatusHeader, lambda _: panda_pick_place.homing(), queue_size=10)

        # Subscribe to the open gripper topic
        rospy.Subscriber('/panda_pick_place/open_gripper', StatusHeader, lambda _: panda_pick_place.open_gripper(), queue_size=10)

        # Subscribe to the close gripper topic
        rospy.Subscriber('/panda_pick_place/close_gripper', StatusHeader, lambda _: panda_pick_place.close_gripper(), queue_size=10)

        # Subscribe to the set pick pose topic
        rospy.Subscriber('/panda_pick_place/set_pick_pose', StatusHeader, lambda _: panda_pick_place.set_pick_pose(), queue_size=10)

        # Subscribe to the set place pose topic
        rospy.Subscriber('/panda_pick_place/set_place_pose', StatusHeader, lambda _: panda_pick_place.set_place_pose(), queue_size=10)

        # Subscribe to the pick topic
        rospy.Subscriber('/panda_pick_place/pick', StatusHeader, lambda msg: panda_pick_place.pick_object(msg), queue_size=10)

        # Subscribe to the place topic
        rospy.Subscriber('/panda_pick_place/place', StatusHeader, lambda _: panda_pick_place.place_object(), queue_size=10)

        # Subscribe to the pick and place routine topic
        rospy.Subscriber('/panda_pick_place/pick_place_routine', StatusHeader, lambda _: panda_pick_place.pick_place_routine(), queue_size=10)

        # Subscribe to go to pick pose topic
        rospy.Subscriber('/panda_pick_place/go_to_pick_pose', StatusHeader, lambda _: panda_pick_place.go_to_pick_pose(), queue_size=10)

        # Subscribe to go to place pose topic
        rospy.Subscriber('/panda_pick_place/go_to_place_pose', StatusHeader, lambda _: panda_pick_place.go_to_place_pose(), queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()