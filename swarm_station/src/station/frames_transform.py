#!/usr/bin/python3

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import TransformStamped

from swarm_checker.msg import SwarmState
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
import sys
import rospy

import tf2_py
import tf2_ros
import tf_conversions
import tf

# tf2 broadcaster for custom framess

def setListId():
    # set id list from SwarmState
    rospy.loginfo('Getting all connected clovers..')
    while True:
        try:
            message = rospy.wait_for_message("/swarm_checker/state", SwarmState, timeout=5)
            lista_id = message.connected_ids
            rospy.loginfo(f'Connected clovers are: {lista_id}')

            return lista_id

        except Exception as err:
            rospy.logerr("Could not get connected clovers, trying again..")
        # self.lista_id = [0, 1, 2]


def forEeachFrame(id, broadcaster):
    static = TransformStamped()

    # defining its content
    static.header.stamp = rospy.Time.now()
    static.header.frame_id = f"body{id}"
    static.child_frame_id = f"text{id}"

    static.transform.translation.x = 0
    static.transform.translation.y = 0
    static.transform.translation.z = 0.4

    quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    static.transform.rotation.x = quat[0]
    static.transform.rotation.y = quat[1]
    static.transform.rotation.z = quat[2]
    static.transform.rotation.w = quat[3]
    
    broadcaster.sendTransform(static)
    return 


def main():

    rospy.init_node('tf_markers_handler')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    id_list = setListId()
    rate = rospy.Rate(100)


    while True:

        try:
            for id in id_list:
            
                forEeachFrame(id, broadcaster)
                rospy.loginfo(f"Successfully created transform for frame body{id}")
                rate.sleep()
            
        except Exception as err:
            rospy.logerr(f"Could not create all transforms. Trying again.. - {err}")
            break
            
    rospy.spin()

if __name__ == "__main__":
    main()