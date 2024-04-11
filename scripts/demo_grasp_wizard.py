import rospy
import actionlib
import franka_gripper.msg

def home_gripper():
    # Create an action client
    client = actionlib.SimpleActionClient('/franka_gripper/homing', franka_gripper.msg.HomingAction)

    # Wait for the action server to start
    rospy.logdebug("Waiting the server to start...")
    client.wait_for_server()

    # Create a goal
    goal = franka_gripper.msg.HomingGoal()

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    rospy.logdebug("Homing the gripper...")
    client.wait_for_result()

    # Get the result
    result = client.get_result()
    rospy.loginfo(f"Homing result: {result}")

    return result

def grasp():
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

def open_gripper():
    # Trying to open the gripper
    # To open the gripper I try to call franka_gripper::MoveAction(width, speed)
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Wait for the action server to start
    rospy.logdebug("Waiting the server to start...")
    client.wait_for_server()

    # Create a goal
    goal = franka_gripper.msg.MoveGoal()
    goal.width = 0.08 # m
    goal.speed = 0.1 # m/s

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    rospy.logdebug("Opening the gripper...")
    client.wait_for_result()

    # Get the result
    result = client.get_result()
    rospy.loginfo(f"Open gripper result: {result}")

    return result

def close_gripper():
    # Trying to close the gripper
    # To close the gripper I try to call franka_gripper::MoveAction(width, speed)
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Wait for the action server to start
    rospy.logdebug("Waiting the server to start...")
    client.wait_for_server()

    # Create a goal
    goal = franka_gripper.msg.MoveGoal()
    goal.width = 0.0 # m
    goal.speed = 0.1 # m/s

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    rospy.logdebug("Closing the gripper...")
    client.wait_for_result()

    # Get the result
    result = client.get_result()
    rospy.loginfo(f"Close gripper result: {result}")

    return result

def grasp_to(width, with_force=5, with_speed=0.1, with_tolerance=0.04):
    # Trying to grasp the object
    # To grasp the object I try to call franka_gripper::GraspAction(width, epsilon_inner, epsilon_outer, speed, force)
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Wait for the action server to start
    rospy.logdebug("Waiting the server to start...")
    client.wait_for_server()

    # Create a goal
    goal = franka_gripper.msg.GraspGoal()
    goal.width = width # m
    goal.speed = with_speed # m/s
    goal.force = with_force # N
    goal.epsilon.inner = with_tolerance
    goal.epsilon.outer = with_tolerance

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    rospy.logdebug("Grasping the object...")
    client.wait_for_result()

    # Get the result
    result = client.get_result()
    rospy.loginfo(f"Grasp result: {result}")

    return result

if __name__ == '__main__':
    rospy.init_node('homing_client')
    
    # Show a menu with the available options
    # 1. Homing
    # 2. Grasp
    option = ""
    while option != "q":
        print("Select an option:")
        print("1. Homing")
        print("2. Grasp")
        print("3. Grasp to")
        print("4. Open gripper")
        print("5. Close gripper")
        print("q to exit")
        option = input("Option: ")        

        if option == "1":
            home_gripper()
        elif option == "2":
            grasp()
        elif option == "3":
            width = input("Width: ")
            # Check if the input is a number
            if not width.replace(".", "").isdigit():
                print("Invalid width")
                continue
            width = float(width)

            with_force = input("Force or g to grasp: ")
            # Check if the input is a number
            if not with_force.replace(".", "").isdigit():
                if with_force == "g":
                    grasp_to(width)
                else:
                    print("Invalid force")
                    continue
            
            with_speed = input("Speed or g to grasp: ")
            # Check if the input is a number
            if not with_speed.replace(".", "").isdigit():
                if with_speed == "g":
                    grasp_to(width, float(with_force))
                else:
                    print("Invalid speed")
                    continue

            with_tolerance = input("Tolerance or g to grasp: ")
            # Check if the input is a number
            if not with_tolerance.replace(".", "").isdigit():
                if with_tolerance == "g":
                    grasp_to(width, float(with_force), float(with_speed))
                else:
                    print("Invalid tolerance")
                    continue

            grasp_to(width, float(with_force), float(with_speed), float(with_tolerance))         

        elif option == "4":
            open_gripper()
        elif option == "5":
            close_gripper()
        elif option != "q":
            print("Invalid option\n")
