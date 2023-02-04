#!/usr/bin/python3

# codigo legal para referencia: https://www.programcreek.com/python/?code=Kinovarobotics%2Fkinova-movo%2Fkinova-movo-master%2Fmovo_common%2Fmovo_ros%2Fsrc%2Fmovo%2Fmove_base.py
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Vector3
# from geometry_msgs.msg import Point
# from std_msgs.msg import ColorRGBA
# from std_msgs.msg import Header

import rospy

array = MarkerArray()


def show_marker(marker_array_, pos, ori, scale, color, lifetime):
    marker = Marker()
    
    
    marker.header.frame_id = "/table_top"
    # marker_.header.stamp = rospy.Time.now()
    marker.type = marker.CUBE
    marker.action = marker.ADD # eq a 0 - criar

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = ori[1]
    marker.pose.orientation.y = ori[2]
    marker.pose.orientation.z = ori[3]
    marker.pose.orientation.w = ori[0]

    marker.lifetime = rospy.Duration.from_sec(lifetime) # 0 -> forever
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.a = 1
    red_, green_, blue_ = color
    marker.color.r = red_
    marker.color.g = green_
    marker.color.b = blue_
    marker_array_.markers.append(marker)
    



if __name__ == "__main__":
    pose = [0, 0, 1]
    orientation = [ 0, 0, 0, 0]
    scale = [1, 1, 1]
    color = [0, 255, 0] # rgb
    # pose = Pose()
    # pose.position.

    show_marker(array, pose, orientation, scale, color, 0)
    # show_marker(array, [4, 0, 1], orientation, scale, color, 0)

    rospy.init_node("marker")

    publisher = rospy.Publisher("/vehicle_marker", MarkerArray, queue_size=10)
    rate = rospy.Rate(1)
    print("publishing: ", array)

    while not rospy.is_shutdown():
        
        publisher.publish(array)
        rospy.sleep(1)
