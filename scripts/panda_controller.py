import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy
import threading
from panda_pick_place_msgs.msg import StatusHeader, DetectionResults, DetectionRequest
from std_msgs.msg import UInt8
from rospy import Publisher, Subscriber
from typing import List
import tkinter as tk
from tkinter import filedialog, ttk
from threading import Timer

class PandaController:
    def __init__(self) -> None:
        self.nodes_count: int = 2
        self.nodes: List[str] = [
            "panda_vision",
            "panda_pick_place"
        ]
        self.ack: List[bool] = [
            False,
            False,
        ]
        self.status: List[StatusHeader] = [
            StatusHeader(status = "waiting", message = "Handshaking...                 "),
            StatusHeader(status = "waiting", message = "Handshaking...                 "),
        ]
        self.pubs: List[Publisher] = []
        self.subs: List[Subscriber] = []
        self.debug = False
        self.ready = False
        self.busy = False
        self.timer = None

        # Initialize the pubs
        self.start_detection_pub = rospy.Publisher("/panda_vision/start_detection", DetectionRequest, queue_size=10)
        self.homing_pub = rospy.Publisher("/panda_pick_place/homing", StatusHeader, queue_size=10)
        self.open_gripper_pub = rospy.Publisher("/panda_pick_place/open_gripper", StatusHeader, queue_size=10)
        self.close_gripper_pub = rospy.Publisher("/panda_pick_place/close_gripper", StatusHeader, queue_size=10)
        self.set_pick_pose_pub = rospy.Publisher("/panda_pick_place/set_pick_pose", StatusHeader, queue_size=10)
        self.set_place_pose_pub = rospy.Publisher("/panda_pick_place/set_place_pose", StatusHeader, queue_size=10)
        self.pick_place_pub = rospy.Publisher("/panda_pick_place/pick_place_routine", StatusHeader, queue_size=10)
        self.go_to_pick_pose_pub = rospy.Publisher("/panda_pick_place/go_to_pick_pose", StatusHeader, queue_size=10)
        self.go_to_place_pose_pub = rospy.Publisher("/panda_pick_place/go_to_place_pose", StatusHeader, queue_size=10)
        self.clean_table_pub = rospy.Publisher("/panda_pick_place/clean_table_routine", StatusHeader, queue_size=10)
        self.debug_pub = rospy.Publisher("/panda_pick_place/debug", StatusHeader, queue_size=10)
        self.debug_sub = rospy.Subscriber("/panda_pick_place/debug", StatusHeader, self.debug_callback, queue_size=10)

        self.build_gui()

    def build_gui(self) -> None:
        self.window = tk.Tk()
        self.window.title("Panda Controller GUI") 

        gui_width = rospy.get_param("/panda_pick_place/gui/width")
        gui_height = rospy.get_param("/panda_pick_place/gui/height")

        self.window.geometry(f"{gui_width}x{gui_height}")

        # Create an empty label for padding
        padding_label = tk.Label(self.window, text="")
        padding_label.grid(row=0, column=0, pady=20)

        # Section - FIRST ROW
        # Create the status labels and reset button
        self.status_labels = []
        self.message_labels = []
        for i in range(len(self.nodes)):
            node_label = tk.Label(self.window, text=f"{self.nodes[i]}:")
            node_label.config(font=("Lato", 14, "bold"))
            node_label.grid(row=0, column=i, padx=10, sticky="w")
            status_label = tk.Label(self.window, text="loading")
            status_label.grid(row=1, column=i, padx=10, sticky="w")
            message_label = tk.Label(self.window, text="Handshaking...")
            message_label.grid(row=2, column=i, padx=10, sticky="w")
            self.status_labels.append(status_label)
            self.message_labels.append(message_label)
        # Reset button
        reset_button = tk.Button(self.window, text="Reset Nodes", command=self.reset_nodes)
        reset_button.grid(row=1, column=len(self.nodes), columnspan=1, sticky="e", padx=10)
        # Stop button (red with darker red color)
        self.stop_button = tk.Button(self.window, text="⣾", command=self.shutdown)
        self.stop_button.grid(row=0, column=len(self.nodes), columnspan=1, sticky="e", padx=10, pady=5)
        # Debug button
        self.debug_button = tk.Button(self.window, text=f"Debug: off", command=self.toggle_debug)
        self.debug_button.grid(row=2, column=len(self.nodes), columnspan=1, sticky="e", padx=10, pady=10)
        self.window.columnconfigure(len(self.nodes), weight=1, minsize=(gui_width/rospy.get_param("/panda_pick_place/gui/columns_count")))

        # Divider
        ttk.Separator(self.window, orient="horizontal").grid(row=3, column=0, columnspan=(len(self.nodes)+1), sticky="ew", padx=10, pady=10)

        # Section - SECOND ROW
        # Create a row with the button to choose the new model, the current model path
        model_button = tk.Button(self.window, text="Choose model", command=self.choose_model)
        model_button.grid(row=4, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        self.model_path_label = tk.Label(self.window, text=rospy.get_param("/panda_pick_place/model/path"))
        self.model_path_label.grid(row=4, column=1, columnspan=len(self.nodes), sticky="w", padx=10, pady=10)

        # Divider
        ttk.Separator(self.window, orient="horizontal").grid(row=5, column=0, columnspan=(len(self.nodes)+1), sticky="ew", padx=10, pady=10)

        # Section - MENU
        # I put a row of padding
        padding = tk.Label(self.window, text="Actions")
        padding.grid(row=6, column=0, columnspan=4, padx=10, pady=20, sticky="w")
        # Fromm the 5th row I put the buttons for the actions
        homing_button = tk.Button(self.window, text="Home", command=self.homing)
        homing_button.grid(row=7, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        open_gripper_button = tk.Button(self.window, text="Open Gripper", command=self.open_gripper)
        open_gripper_button.grid(row=8, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        close_gripper_button = tk.Button(self.window, text="Close Gripper", command=self.close_gripper)
        close_gripper_button.grid(row=9, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        set_pick_pose_button = tk.Button(self.window, text="Set Pick Pose", command=self.set_pick_pose)
        set_pick_pose_button.grid(row=10, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        set_place_pose_button = tk.Button(self.window, text="Set Place Pose", command=self.set_place_pose)
        set_place_pose_button.grid(row=11, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        start_detection_button = tk.Button(self.window, text="Start Detection", command=self.start_detection)
        start_detection_button.grid(row=12, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        pick_button = tk.Button(self.window, text="Pick and place", command=self.pick_place_routine)
        pick_button.grid(row=13, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        go_to_pick_pose_button = tk.Button(self.window, text="Go to Pick Pose", command=self.go_to_pick_pose)
        go_to_pick_pose_button.grid(row=14, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        go_to_place_pose_button = tk.Button(self.window, text="Go to Place Pose", command=self.go_to_place_pose)
        go_to_place_pose_button.grid(row=15, column=0, columnspan=1, sticky="w", padx=10, pady=10)
        clean_table_button = tk.Button(self.window, text="Clean Table", command=self.clean_table_routine)
        clean_table_button.grid(row=16, column=0, columnspan=1, sticky="w", padx=10, pady=10)

        # Divider
        ttk.Separator(self.window, orient="horizontal").grid(row=17, column=0, columnspan=(len(self.nodes)+1), sticky="ew", padx=10, pady=10)

        # Start the Tkinter mainloop in a separate thread
        self.controller_thread = threading.Thread(target=self.controller_init)
        self.controller_thread.start()

        self.window.mainloop()

    def controller_init(self, handshaking=True) -> None:
        self.busy = True

        # Handshaking with the nodes
        for i in range(len(self.nodes)):
            self.pubs.append(rospy.Publisher(f'/{self.nodes[i]}/status', StatusHeader, queue_size=10))
            self.subs.append(rospy.Subscriber(f'/{self.nodes[i]}/status', StatusHeader, self.node_status_callback, i))

        # Starting handshaking thread
        if handshaking:
            self.handshaking_thread = threading.Thread(target=self.handshaking)
            self.handshaking_thread.start()
        
        # Wait for the nodes to be ready
        loading = ['⣾', '⣷', '⣯', '⣟', '⡿', '⢿', '⣻', '⣽']
        i = 0
        print("\nWaiting for the nodes to be ready:")
        while not rospy.is_shutdown() and not self.nodes_ready():
            i = (i + 1) % len(loading)
            self.update_stop_button(loading[i])
            string = ""
            for j in range(self.nodes_count):
                string += f"{'v' if self.status[j].status == 'ready' else loading[i]} /{self.nodes[j]}: {self.status[j].status} - "
            # Remove last " - "
            string = string[:-3]
            print(string, flush=True, end="\r")
            rospy.sleep(0.3)

        self.update_stop_button("Stop")
            
        string = ""
        for j in range(self.nodes_count):
            string += f"[v] /{self.nodes[j]}: {self.status[j].message} - "
        print(string, flush=True, end="\n\n")
        self.status = [
            None,
            None,
            None
        ]

        print("Nodes are ready! Starting the controller...")
        self.busy = False

    def choose_model(self) -> None:
        """
        Choose the model
        """

        # Open a file dialog to choose the model
        model_path = filedialog.askopenfilename(title = "Select a Model", filetypes = [("Model files", "*.pth"),("Model files", "*.pt")])

        # Update the model path label
        self.model_path_label.config(text=model_path)

        # Update the model path in the ROS parameter server
        rospy.set_param("/panda_pick_place/model/path", model_path)

    def toggle_debug(self) -> None:
        """
        Toggle the debug mode
        """

        try:
            # Schedule the GUI update on the main thread
            self.window.after(0, self._toggle_debug)
        except rospy.ROSInterruptException:
            pass

    def _toggle_debug(self) -> None:
        """
        Toggle the debug mode
        """

        self.debug = not self.debug 
        self.debug_button.config(text=f"Debug: {'on' if self.debug else 'off'}")

        # Adding the "GO" button at last row for the debug mode
        if self.debug:
            go_button = tk.Button(self.window, text="GO", command=self.go)
            go_button.grid(row=17, column=0, columnspan=1, sticky="w", padx=10, pady=10)

    def debug_callback(self, msg: StatusHeader) -> None:
        """
        Callback function for the debug messages

        Parameters:
        - msg (StatusHeader): The status message

        Returns:
        None
        """

        if msg.status != "ask":
            return

        if self.debug:
            rospy.sleep(0.5)
            self.debug_pub.publish(StatusHeader(status = "wait", message = "Debug mode is on!"))
            # Add the label to row 17 col 1 "breakpoint reached"
            debug_label = tk.Label(self.window, text="Breakpoint reached", fg="red")
            debug_label.grid(row=17, column=1, columnspan=len(self.nodes), padx=10, sticky="w")
        else:
            rospy.sleep(0.5)
            self.debug_pub.publish(StatusHeader(status = "go", message = "Debug mode is off!"))

    def go(self) -> None:
        """
        Go function for the debug mode
        """

        if self.debug:
            self.debug_pub.publish(StatusHeader(status = "go", message = "Debug mode is on!"))
        

    def update_stop_button(self, text: str) -> None:
        """
        Update the text of the stop button

        Parameters:
        - text (String): The text to display

        Returns:
        None
        """

        try:
            # Schedule the GUI update on the main thread
            self.window.after(0, self._update_stop_button, text)
        except rospy.ROSInterruptException:
            pass

    def _update_stop_button(self, text: str) -> None:
        try:
            if text == "Stop":
                self.stop_button.config(text=text, bg="#f59287", fg="darkred")
            else:
                self.stop_button.config(text=text)
        except rospy.ROSInterruptException:
            pass

    def update_status(self, node_index, status):
        try:
            # Schedule the GUI update on the main thread
            self.window.after(0, self._update_status, node_index, status)
        except rospy.ROSInterruptException:
            pass

    def _update_status(self, node_index, status):
        try:
            # Update the status label
            color = "green" if status == "ready" else "red" if status == "error" else "orange"
            self.status_labels[node_index].config(text="")
            self.status_labels[node_index].config(text=status, fg=color)
        except rospy.ROSInterruptException:
            pass

    def update_message(self, node_index, message):
        try:
            # Schedule the GUI update on the main thread
            self.window.after(0, self._update_message, node_index, message)
        except rospy.ROSInterruptException:
            pass

    def _update_message(self, node_index, message):
        try:
            # Update the message label
            self.message_labels[node_index].config(text="")
            self.message_labels[node_index].config(text=message)
        except rospy.ROSInterruptException:
            pass

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
        self.ready = ready
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

    def reset_nodes(self) -> None:
        """
        Reset the nodes
        """

        for i in range(self.nodes_count):
            self.status[i] = None
            self.ack[i] = False
            self.pubs[i].publish(StatusHeader(status = "reset", message = "Resetting..."))

    def handshaking(self) -> None:
        """
        Handshaking with other nodes
        """
        try:
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
        except rospy.ROSInterruptException:
            pass

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
        if msg.status == "waiting":
            self.status[node_index] = msg
            self.update_status(node_index, "waiting")
            self.update_message(node_index, msg.message)
            return
        elif msg.status == "ready":
            self.status[node_index] = msg
            self.update_status(node_index, "ready")
            self.update_message(node_index, msg.message)
            return
        elif msg.status == "warning":
            if not self.ready:
                self.status[node_index] = StatusHeader(status = "ready", message = " ")
            self.update_status(node_index, "warning")
            self.update_message(node_index, msg.message)
            if self.ready:
                rospy.logwarn(msg.message)
            return
        elif msg.status == "error":
            if not self.ready:
                self.status[node_index] = StatusHeader(status = "ready", message = " ")
            self.update_status(node_index, "error")
            self.update_message(node_index, msg.message)
            if self.ready:
                rospy.logerr(msg.message)
            return
        elif msg.status == "info":
            self.update_status(node_index, "info")
            self.update_message(node_index, msg.message)
            if self.ready:
                rospy.loginfo(msg.message)
            return
            
    def printOnThread(self, msg: str) -> None:
        """
        Print a message on a different thread

        Parameters:
        - msg (String): The message to print

        Returns:
        None
        """

        rospy.loginfo(msg)

    def start_detection(self) -> None:
        """
        Start the object detection
        """

        if self.busy:
            return

        # Publish a message to /panda_vision/start_detection topic
        self.start_detection_pub.publish(DetectionRequest(obj_count = 5, obj_id=-1, confidence=0.7))

        # Wait for the detection to be done
        rospy.wait_for_message("/panda_vision/detected_objects", DetectionResults)

    def homing(self) -> None:
        """
        Move the robot to the home position
        """

        if self.busy:
            return
        
        try:
            # Publish a message to /panda_pick_place/homing topic
            self.homing_pub.publish(StatusHeader(status = "homing", message = "Moving the robot to the home position..."))

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/homing", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.action_status_label = tk.Label(self.window, text="error", fg="red")

    def open_gripper(self) -> None:
        """
        Open the gripper
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/open_gripper topic
            self.open_gripper_pub.publish(StatusHeader(status = "open_gripper", message = "Opening the gripper..."))

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/open_gripper", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.action_status_label = tk.Label(self.window, text="error", fg="red")

    def close_gripper(self) -> None:
        """
        Close the gripper
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/close_gripper topic
            self.close_gripper_pub.publish(StatusHeader(status = "close_gripper", message = "Closing the gripper..."))

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/close_gripper", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))


    def set_pick_pose(self) -> None:
        """
        Set the current pose as the pick pose
        """

        if self.busy:
            return
        
        try:
            # Publish a message to /panda_pick_place/set_pick_pose topic
            self.set_pick_pose_pub.publish(StatusHeader(status = "set_pick_pose", message = "Setting the current pose as the pick pose..."))        

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/set_pick_pose", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))

    def set_place_pose(self) -> None:
        """
        Set the current pose as the place pose
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/set_place_pose topic
            self.set_place_pose_pub.publish(StatusHeader(status = "set_place_pose", message = "Setting the current pose as the place pose..."))        

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/set_place_pose", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))

    def timer_callback(self):
        if self.busy:
            self.busy = False

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        # Printing the results from the fifth row and third column
        self.update_action_response_with_status(StatusHeader(status = "error", message = "Timeout"))

    def pick_place_routine(self) -> None:
        """
        Pick the object
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/pick_place_routine topic
            self.pick_place_pub.publish(StatusHeader(status = "pick_place_routine", message = "Picking the object..."))
            self.busy = True
            # Subscribe to the /panda_pick_place/pick_place_routine topic
            status: StatusHeader = rospy.Subscriber(f'/panda_pick_place/pick_place_routine', StatusHeader, self._pick_place_routine_callback)

            # Wait for something to happen
            # Start a timer
            self.timer = Timer(10.0, self.timer_callback)  # 10 seconds
            self.timer.start()
            while not rospy.is_shutdown() and self.busy:
                rospy.sleep(0.1)

            # Stop the timer
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None

        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))
    
    def clean_table_routine(self) -> None:
        """
        Clean the table
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/clean_table_routine topic
            self.clean_table_pub.publish(StatusHeader(status = "clean_table_routine", message = "Cleaning the table..."))
            self.busy = True
            # Subscribe to the /panda_pick_place/clean_table_routine topic
            status: StatusHeader = rospy.Subscriber(f'/panda_pick_place/clean_table_routine', StatusHeader, self._clean_table_routine_callback)

            # Wait for something to happen
            # Start a timer
            self.timer = Timer(10.0, self.timer_callback)  # 10 seconds
            self.timer.start()
            while not rospy.is_shutdown() and self.busy:
                rospy.sleep(0.1)

            # Stop the timer
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None

        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))

    def _clean_table_routine_callback(self, msg: StatusHeader) -> None:
        """
        Callback function for the clean_table_routine messages

        Parameters:
        - msg (StatusHeader): The status message

        Returns:
        None
        """
    
        try:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(msg)

            if msg.status == "done":
                self.busy = False
            
                # Stop the timer
                if self.timer is not None:
                    self.timer.cancel()
                    self.timer = None
                return

            # Reset the timer
            if self.timer is not None:
                self.timer.cancel()
            self.timer = Timer(10.0, self.timer_callback)  # 10 seconds
            
        except Exception as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))
            self.busy = False

    def _pick_place_routine_callback(self, msg: StatusHeader) -> None:
        """
        Callback function for the pick_place_routine messages

        Parameters:
        - msg (StatusHeader): The status message

        Returns:
        None
        """
    
        try:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(msg)

            if msg.status == "done":
                self.busy = False
            
                # Stop the timer
                if self.timer is not None:
                    self.timer.cancel()
                    self.timer = None
                return

            # Reset the timer
            if self.timer is not None:
                self.timer.cancel()
            self.timer = Timer(10.0, self.timer_callback)  # 10 seconds
            
        except Exception as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))
            self.busy = False

    def update_action_response_with_status(self, msg):
        try:
            # Schedule the GUI update on the main thread
            self.window.after(0, self._update_action_response_with_status, msg)
        except rospy.ROSInterruptException:
            pass

    def _update_action_response_with_status(self, msg):
        try:
            # Printing the results from the fifth row and third column
            status_color = "green" if msg.status == "done" else "red" if msg.status == "error" else "orange"
            self.action_status_label = tk.Label(self.window, text=msg.status, fg=status_color)
            self.action_status_label.grid(row=6, column=1, columnspan=len(self.nodes), padx=10, sticky="w")
            self.action_message_label = tk.Label(self.window, text=msg.message)
            self.action_message_label.grid(row=7, column=1, rowspan=9, columnspan=len(self.nodes), padx=10, sticky="w")
        except rospy.ROSInterruptException:
            pass

    def go_to_pick_pose(self) -> None:
        """
        Move the robot to the pick pose
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/go_to_pick_pose topic
            self.go_to_pick_pose_pub.publish(StatusHeader(status = "go_to_pick_pose", message = "Moving the robot to the pick pose..."))

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/go_to_pick_pose", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            print("--------------------------629")
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            print(e)
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))

    def go_to_place_pose(self) -> None:
        """
        Move the robot to the place pose
        """

        if self.busy:
            return

        try:
            # Publish a message to /panda_pick_place/go_to_place_pose topic
            self.go_to_place_pose_pub.publish(StatusHeader(status = "go_to_place_pose", message = "Moving the robot to the place pose..."))

            # Wait for the homing to be done
            status: StatusHeader = rospy.wait_for_message("/panda_pick_place/go_to_place_pose", StatusHeader, timeout=10.0)

            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(status)
        except rospy.ROSException as e:
            # Printing the results from the fifth row and third column
            self.update_action_response_with_status(StatusHeader(status = "error", message = str(e)))

    def shutdown(self) -> None:
        """
        Shutdown the controller
        """

        # Send a command to other nodes to stop
        for i in range(len(self.nodes)):
            self.pubs[i].publish(StatusHeader(status = "shutdown", message = f" "))

        # Stop the other threads
        try:
            self.controller_thread.join(0.1)
            self.handshaking_thread.join(0.1)
        except RuntimeError:
            pass
        
        self.window.destroy()

        rospy.signal_shutdown("Shutdown")

def main():
    rospy.init_node('panda_controller')

    try:    
        controller = PandaController()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()