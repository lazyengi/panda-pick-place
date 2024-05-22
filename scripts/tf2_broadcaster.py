import sys
sys.path.insert(0, '/home/giuseppe/franka_ws/devel/lib/python3/dist-packages')
import rospy

import tf2_ros
import geometry_msgs.msg
  
import numpy as np
from tf.transformations import quaternion_from_matrix

def broadcast_tf():
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
    broadcast_tf()
    rospy.spin()

if __name__ == '__main__':
    main()