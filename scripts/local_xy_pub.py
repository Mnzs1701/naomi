#!/usr/bin/env python3

from hmac import trans_36
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion

tfTimeOffset = rospy.Duration(0.1)

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(1000.0)
    
    trans = Vector3()
    rot = Quaternion()

    tf_lookup = tfBuffer.lookup_transform('base_link', 'odomparent', rospy.Time(0))

    trans.x = tf_lookup.transform.translation.x
    trans.y = tf_lookup.transform.translation.y
    trans.z = tf_lookup.transform.translation.z

    rot = tf_lookup.transform.rotation

    while not rospy.is_shutdown():
        try:
            tstamped = tfBuffer.lookup_transform('base_link', 'odomparent', rospy.Time(0))
            trans.z = tstamped.transform.translation.z
            mapzTransMsg = TransformStamped()
            mapzTransMsg.transform.translation = trans
            mapzTransMsg.transform.rotation = rot
            mapzTransMsg.header.stamp = tstamped.header.stamp + tfTimeOffset
            mapzTransMsg.header.frame_id ='odomparent'
            mapzTransMsg.child_frame_id = 'dynamic_z_map'

            # br.sendTransform((0, 0, -trans[2]),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"dynamic_z_map","map")
            br.sendTransform(mapzTransMsg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()
    rospy.spin()