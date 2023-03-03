#!/usr/bin/python3


from swarm_visualizer.msg import SwarmStationCommands
from geometry_msgs.msg import Point
import rospy

rospy.init_node("command_publisher")

pub = rospy.Publisher("/marker_state", SwarmStationCommands, queue_size=10)

msg = SwarmStationCommands()

# Example of publishing swarm station commands via /marker_state topic

a = Point()
b = Point()
a.x, a.y, a.z = -5, -2, 0
b.x, b.y, b.z = 5, 5, 5
msg.command = 'rectangle'
msg.points = [a, b]

while True:
    pub.publish(msg)

    rospy.loginfo(f'publishing /n{msg}')
    a = input('press')
    rospy.Rate(100).sleep()
