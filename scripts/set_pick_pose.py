import sys
import rospy
import moveit_commander

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

    # Create a move group_arm object
    group_name = "panda_arm"
    move_arm_group = moveit_commander.MoveGroupCommander(group_name)

    # return the robot commander, move group, scene and display trajectory publisher
    return move_arm_group

def main():
    # Initialize the node
    rospy.init_node("set_pick_pose")

    # Initialize the moveit commander
    move_arm_group = initController(sys.argv)

    # Wait for the keyboard command to set the pick pose
    print("Press any key to set the pick pose")
    input()

    # Set the pick pose in the parameter server 
    # the pose is in the joint space
    rospy.set_param("/pick_pose", move_arm_group.get_current_joint_values())

    # Print the pick pose
    print("Current pose: ", move_arm_group.get_current_joint_values())
    print("Pick pose: ", rospy.get_param("/pick_pose"))

    # Shutdown the moveit commander
    moveit_commander.roscpp_shutdown()

    # Exit the script
    sys.exit()
    

if __name__ == "__main__":
    main()