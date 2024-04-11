import rospy
import sys

def main():
    # Initialize the node
    rospy.init_node("reset_pick_pose")

    rospy.set_param("/pick_pose", rospy.get_param("/default_pick_pose"))

    rospy.loginfo("Pick pose reset")

    rospy.signal_shutdown("Pick pose reseted successfully")
    
    # Exit the script
    sys.exit(0)


if __name__ == "__main__":
    main()
