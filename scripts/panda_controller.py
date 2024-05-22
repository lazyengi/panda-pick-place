import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy
import threading
from panda_pick_place_msgs.msg import StatusHeader, DetectionResults
from std_msgs.msg import UInt8
from rospy import Publisher, Subscriber
from typing import List

class PandaController:
    def __init__(self) -> None:
        self.nodes_count: int = 2
        self.nodes: List[str] = [
            "panda_vision",
            "panda_pick_place"
        ]
        self.ack: List[bool] = [
            False,
            False
        ]
        self.status: List[StatusHeader] = [
            StatusHeader(status = "waiting", message = "Handshaking...                 "),
            StatusHeader(status = "waiting", message = "Handshaking...                 ")
        ]
        self.pubs: List[Publisher] = []
        self.subs: List[Subscriber] = []

        # Initialize the pubs
        self.start_detection_pub = rospy.Publisher("/panda_vision/start_detection", UInt8, queue_size=10)
        self.homing_pub = rospy.Publisher("/panda_pick_place/homing", StatusHeader, queue_size=10)
        self.open_gripper_pub = rospy.Publisher("/panda_pick_place/open_gripper", StatusHeader, queue_size=10)
        self.close_gripper_pub = rospy.Publisher("/panda_pick_place/close_gripper", StatusHeader, queue_size=10)
        self.set_pick_pose_pub = rospy.Publisher("/panda_pick_place/set_pick_pose", StatusHeader, queue_size=10)
        self.set_place_pose_pub = rospy.Publisher("/panda_pick_place/set_place_pose", StatusHeader, queue_size=10)
        self.pick_pub = rospy.Publisher("/panda_pick_place/pick", StatusHeader, queue_size=10)
        self.go_to_pick_pose_pub = rospy.Publisher("/panda_pick_place/go_to_pick_pose", StatusHeader, queue_size=10)
        self.go_to_place_pose_pub = rospy.Publisher("/panda_pick_place/go_to_place_pose", StatusHeader, queue_size=10)

        # Handshaking with the nodes
        for i in range(len(self.nodes)):
            self.pubs.append(rospy.Publisher(f'/{self.nodes[i]}/status', StatusHeader, queue_size=10))
            self.subs.append(rospy.Subscriber(f'/{self.nodes[i]}/status', StatusHeader, self.node_status_callback, i))

        # Start the status publisher thread
        handshaking_thread = threading.Thread(target=self.handshaking)
        handshaking_thread.start()

        # Wait for the nodes to be ready
        loading = ["-", "\\", "|", "/"]
        i = 0
        print("\nWaiting for the nodes to be ready:")
        while not rospy.is_shutdown() and not self.nodes_ready():
            i = (i + 1) % len(loading)
            string = ""
            for j in range(self.nodes_count):
                string += f"[{'v' if self.status[j].status == 'ready' else loading[i]}] /{self.nodes[j]}: {self.status[j].message} - "
            # Remove last " - "
            string = string[:-3]
            print(string, flush=True, end="\r")
            rospy.sleep(0.1)
            
        string = ""
        for j in range(self.nodes_count):
            string += f"[v] /{self.nodes[j]}: {self.status[j].message} - "
        print(string, flush=True, end="\n\n")
        self.status = [
            None,
            None
        ]

        print("Nodes are ready! Starting the controller...")

    def nodes_ready(self) -> bool:
        """
        Check if all the nodes are ready

        Returns:
        - bool: True if all the nodes are ready, False otherwise
        """

        ready = True
        for i in range(self.nodes_count):
            if self.status[i] is None or self.status[i].status != "ready":
                ready = False
                break
        return ready

    def nodes_ack(self) -> bool:
        """
        Check if all the nodes are ready

        Returns:
        - bool: True if all the nodes are ready, False otherwise
        """

        ack = True
        for i in range(self.nodes_count):
            if not self.ack[i]:
                ack = False
                break
        return ack


    def handshaking(self) -> None:
        """
        Handshaking with other nodes
        """

        rate = rospy.Rate(rospy.get_param('/panda_pick_place/handshaking_freq', 3))  
        
        while not rospy.is_shutdown() and not self.nodes_ack():
            for i in range(self.nodes_count):
                if not self.ack[i]:
                    self.pubs[i].publish(StatusHeader(status = "syn", message = "Handshaking..."))
                else:
                    self.status[i].message = "Handshaking done!              "
            rate.sleep()

        for i in range(self.nodes_count):
            self.pubs[i].publish(StatusHeader(status = "syncdone", message = "Handshaking done!"))

    def node_status_callback(self, msg: StatusHeader, node_index: int)->None:
        """
        Callback function for the status messages

        Parameters:
        - msg (StatusHeader): The status message
        - node (String): The node name

        Returns:
        None
        """

        # Check if the handshaking is done
        if not self.ack[node_index] and msg.status == "synack":
            self.ack[node_index] = True
            return
        elif (msg.status == "syn" or msg.status == "synack" or msg.status == "syncdone"):
            return
        
        # Listen to the status messages
        if self.status[node_index] is None or msg.status == "error":
            if msg.status == "warning":
                rospy.logwarn(msg.message)
                return
            elif msg.status == "error":
                rospy.logerr(msg.message)
                # Send a command to all nodes to stop
                for i in range(len(self.nodes)):
                    self.pubs[i].publish(StatusHeader(status = "shutdown", message = f" "))
                rospy.signal_shutdown(f"{self.nodes[node_index]} error: {msg.message}")
            elif msg.status == "info":
                rospy.loginfo(msg.message)
                return
            
        # Update the status message in the init phase
        self.status[node_index] = msg

    def start_detection(self) -> None:
        """
        Start the object detection
        """

        # Publish a message to /panda_vision/start_detection topic
        self.start_detection_pub.publish(2)

        # Wait for the detection to be done
        rospy.wait_for_message("/panda_vision/detected_objects", DetectionResults)

    def homing(self) -> None:
        """
        Move the robot to the home position
        """

        # Publish a message to /panda_pick_place/homing topic
        self.homing_pub.publish(StatusHeader(status = "homing", message = "Moving the robot to the home position..."))

    def open_gripper(self) -> None:
        """
        Open the gripper
        """

        # Publish a message to /panda_pick_place/open_gripper topic
        self.open_gripper_pub.publish(StatusHeader(status = "open_gripper", message = "Opening the gripper..."))

    def close_gripper(self) -> None:
        """
        Close the gripper
        """

        # Publish a message to /panda_pick_place/close_gripper topic
        self.close_gripper_pub.publish(StatusHeader(status = "close_gripper", message = "Closing the gripper..."))


    def set_pick_pose(self) -> None:
        """
        Set the current pose as the pick pose
        """
        
        # Publish a message to /panda_pick_place/set_pick_pose topic
        self.set_pick_pose_pub.publish(StatusHeader(status = "set_pick_pose", message = "Setting the current pose as the pick pose..."))        

    def set_place_pose(self) -> None:
        """
        Set the current pose as the place pose
        """

        # Publish a message to /panda_pick_place/set_place_pose topic
        self.set_place_pose_pub.publish(StatusHeader(status = "set_place_pose", message = "Setting the current pose as the place pose..."))        

    def pick(self) -> None:
        """
        Pick the object
        """

        # Publish a message to /panda_pick_place/pick topic
        self.pick_pub.publish(StatusHeader(status = "pick", message = "Picking the object..."))

        # Wait for the pick to be done
        rospy.wait_for_message("/panda_pick_place/pick", StatusHeader)

    def go_to_pick_pose(self) -> None:
        """
        Move the robot to the pick pose
        """

        # Publish a message to /panda_pick_place/go_to_pick_pose topic
        self.go_to_pick_pose_pub.publish(StatusHeader(status = "go_to_pick_pose", message = "Moving the robot to the pick pose..."))

    def go_to_place_pose(self) -> None:
        """
        Move the robot to the place pose
        """

        # Publish a message to /panda_pick_place/go_to_place_pose topic
        self.go_to_place_pose_pub.publish(StatusHeader(status = "go_to_place_pose", message = "Moving the robot to the place pose..."))

    def shutdown(self) -> None:
        """
        Shutdown the controller
        """

        # Send a command to other nodes to stop
        for i in range(len(self.nodes)):
            self.pubs[i].publish(StatusHeader(status = "shutdown", message = f" "))
        rospy.signal_shutdown("Shutdown")

def main():
    rospy.init_node('panda_controller')

    try:    
        controller = PandaController()

        while not rospy.is_shutdown():
            # Print a menu with the available options
            print("What do you want to do?")
            print("1. Home the robot")
            print("2. Open the gripper")
            print("3. Close the gripper")
            print("4. Set the current pose as the pick pose")
            print("5. Set the current pose as the place pose")
            print("6. Start detection")
            print("7. Pick")
            print("8. Go to pick pose")
            print("9. Go to place pose")
            print("-1. Exit")

            try:
                option = int(input("Option: "))
                if option == 1:
                    controller.homing()
                    continue
                elif option == 2:
                    controller.open_gripper()
                    continue
                elif option == 3:
                    controller.close_gripper()
                    continue
                elif option == 4:
                    controller.set_pick_pose()
                    continue
                elif option == 5:
                    controller.set_place_pose()
                    continue
                elif option == 6:
                    controller.start_detection()
                    continue
                elif option == 7:
                    controller.pick()
                    continue
                elif option == 8:
                    controller.go_to_pick_pose()
                    continue
                elif option == 9:
                    controller.go_to_place_pose()
                    continue
                elif option == -1:
                    controller.shutdown()
                    break
                else:
                    print("Invalid option!")
            except ValueError:
                print("Invalid option!")
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()