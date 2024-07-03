import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy

import tf2_ros
import geometry_msgs.msg
from panda_pick_place_msgs.msg import StatusHeader
import numpy as np
from tf.transformations import quaternion_from_matrix

class Broadcaster:
    def __init__(self):
        self.init_trigger = False
        self.ack_sent = False

        # Listen for the status messages for handshaking and start the init process
        self.status_pub = rospy.Publisher('/tf2_broadcaster/status', StatusHeader, queue_size=10)
        self.status_sub = rospy.Subscriber('/tf2_broadcaster/status', StatusHeader, self.status_callback, queue_size=10)
        while not self.init_trigger and not rospy.is_shutdown():
            rospy.sleep(0.2)
        self.status_pub.publish(status = "ready", message = "Broadcast is on!")

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



    def broadcast_tf(self):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        rate_freq = rospy.get_param('/panda_pick_place/tf/rate_freq')

        panda_ee_link = rospy.get_param('/panda_pick_place/tf/panda_ee_link')
        camera_link = rospy.get_param('/panda_pick_place/tf/camera_link')

        tf_matrix = np.array(rospy.get_param('/panda_pick_place/tf/tf_matrix')).reshape((4, 4))  # Assuming the matrix is flattened

        rate = rospy.Rate(rate_freq)

        t.header.frame_id = panda_ee_link
        t.child_frame_id = camera_link

        # Extract the translation from the transformation matrix
        t.transform.translation.x = tf_matrix[0, 3]
        t.transform.translation.y = tf_matrix[1, 3]
        t.transform.translation.z = tf_matrix[2, 3]

        # Extract the rotation from the transformation matrix and convert it to a quaternion
        q = quaternion_from_matrix(tf_matrix)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        while not rospy.is_shutdown():
            t.header.stamp = rospy.Time.now()
            br.sendTransform(t)
            rate.sleep()

def main():
    rospy.init_node('tf2_broadcaster')
    try:
        broadcaster = Broadcaster()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()