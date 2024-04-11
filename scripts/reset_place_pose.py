import rospy
import sys

def main():
    # Initialize the node
    rospy.init_node("reset_place_pose")

    rospy.set_param("/place_pose", rospy.get_param("/default_place_pose"))

    rospy.loginfo("Place pose reset")

    rospy.signal_shutdown("Place pose reseted successfully")
    
    # Exit the script
    sys.exit(0)


if __name__ == "__main__":
    main()
