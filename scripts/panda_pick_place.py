import rospy
import sys
import moveit_commander
import actionlib
import moveit_msgs.msg
from actionlib_msgs.msg import GoalStatus

def initController(args: list) -> tuple:
    """
    Initialize the moveit commander, robot commander, move group, planning scene interface and display trajectory publisher

    Args:
    - args: the command line arguments

    Returns:
    - robot: the robot commander object
    - move_arm_group: the move group commander object
    - scene: the planning scene interface object
    - display_trajectory_publisher: the display trajectory publisher object
    """

    # Init the moveit commander
    moveit_commander.roscpp_initialize(args)

    # Create a robot commander object
    robot = moveit_commander.RobotCommander()

    # Create a planning scene interface object
    scene = moveit_commander.PlanningSceneInterface()

    # Create a move group_arm object
    group_name = "panda_arm"
    move_arm_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
    )

    # Set the planning time
    move_arm_group.set_planning_time(30)

    # Set the reference frame
    reference_frame = 'panda_link0'
    move_arm_group.set_pose_reference_frame(reference_frame)

    # Set the end effector link
    end_effector_link = 'panda_link8'
    move_arm_group.set_end_effector_link(end_effector_link)

    # Create a move group gripper object
    group_name = "panda_hand"
    move_hand_group = moveit_commander.MoveGroupCommander(group_name)

    # Setting velocity and acceleration scaling factors
    move_arm_group.set_max_acceleration_scaling_factor(rospy.get_param("/acceleration_scaling_factor", 0.7))
    move_arm_group.set_max_velocity_scaling_factor(rospy.get_param("/acceleration_scaling_factor", 0.7))

    # return the robot commander, move group, scene and display trajectory publisher
    return robot, move_arm_group, move_hand_group, scene, display_trajectory_publisher

def homing(move_arm_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Move the robot to the home position

    Args:
    - move_arm_group: the move arm group commander object
    """

    # The home position of the robot is the "ready" goal state
    move_arm_group.set_named_target("ready")

    # Execute the trajectory
    rospy.logdebug("Executing the trajectory to the goal...")
    success = move_arm_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_arm_group.stop()

    # Return the result
    return success

def open_gripper(move_hand_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Open the gripper

    Args:
    - move_hand_group: the move group commander object
    """

    # The open position of the gripper is the "open" goal state
    move_hand_group.set_named_target("open")

    # Execute the trajectory
    rospy.logdebug("Executing the trajectory to the goal...")
    success = move_hand_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_hand_group.stop()

    # Return the result
    return success

def close_gripper(move_hand_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Close the gripper

    Args:
    - move_hand_group: the move group commander object
    """

    # The close position of the gripper is the "close" goal state
    move_hand_group.set_named_target("close")

    # Execute the trajectory
    rospy.logdebug("Executing the trajectory to the goal...")
    success = move_hand_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_hand_group.stop()

    # Return the result
    return success

def go_to_place_pose(move_arm_group: moveit_commander.MoveGroupCommander, move_hand_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Move the robot to the place position

    Args:
    - move_arm_group: the move group commander object
    """

    # Execute the trajectory
    rospy.logdebug("Executing the trajectory to the goal...")
    success = move_arm_group.go(rospy.get_param("/place_pose", rospy.get_param("/default_place_pose")), wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_arm_group.stop()

    # Opening the gripper
    open_hand_success = open_gripper(move_hand_group=move_hand_group)

    # Return the result
    return success and open_hand_success

def go_to_pick_pose(move_arm_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Move the robot to the pick position

    Args:
    - move_arm_group: the move group commander object
    """

    # Execute the trajectory
    rospy.logdebug("Executing the trajectory to the goal...")
    success = move_arm_group.go(rospy.get_param("/pick_pose", rospy.get_param("/default_pick_pose")), wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_arm_group.stop()

    # Return the result
    return success

def pick_place_routine(move_arm_group: moveit_commander.MoveGroupCommander, move_hand_group: moveit_commander.MoveGroupCommander) -> bool:
    """
    Pick and place routine

    Args:
    - move_arm_group: the move arm group commander object
    - move_hand_group: the move hand group commander object
    """

    # Go to the pick pose
    # print("Press any key to go to the pick pose")
    # input()
    success = go_to_pick_pose(move_arm_group=move_arm_group)
    rospy.loginfo(f"Go to pick pose result: {success}")

    # Go to the place pose
    # print("Press any key to go to the place pose")
    # input()
    success = go_to_place_pose(move_arm_group=move_arm_group, move_hand_group=move_hand_group)
    rospy.loginfo(f"Go to place pose result: {success}")

    # Open the gripper
    open_gripper(move_hand_group=move_hand_group)

    # Return the result
    return success

def main():
    # Init the node
    rospy.init_node('panda_pick_place', anonymous=True)

    # Init the controller and get the robot commander, move group and scene
    robot, move_arm_group, move_hand_group, scene, display_trajectory_publisher = initController(sys.argv)
    rospy.logdebug("Controller initialized")

    # Homing
    print("Press any key to home the robot arm")
    input()
    success = homing(move_arm_group=move_arm_group)
    rospy.loginfo(f"Homing result: {success}")

    # Pick and place routine
    repeat = 'y'
    while repeat == 'y':
        success = pick_place_routine(move_arm_group=move_arm_group, move_hand_group=move_hand_group)
        print("Do you want to repeat the pick and place routine? (y/n)")
        repeat = input()
    
    # Exit the script
    rospy.signal_shutdown("Pick and place routine finished")


if __name__ == '__main__':
    main()