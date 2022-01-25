'''
 * @file offb_node.py
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
'''

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
import math
import time
import numpy as np


current_state = State()
# current state recebe a mensagem de callback
def state_cb(msg_cb):
    current_state = msg_cb

print(current_state)

rospy.init_node("XABLAU_NO_TAKEOFF", anonymous=True)

rospy.Subscriber("clover0/mavros/state",State, state_cb, queue_size=10)   
local_pos_pub = rospy.Publisher("clover0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
arming_client = rospy.ServiceProxy("clover0/mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy("clover0/mavros/setmode", SetMode)

rate = rospy.Rate(20)

while(not rospy.is_shutdown() and current_state.connected):
    rospy.spin()
    rate.sleep()


pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

for i in range(0,100):
    local_pos_pub.publish(pose)
    rospy.spin()
    rate.sleep()

offb_set_mode = SetMode()
offb_set_mode.request.custom_mode = "OFFBOARD"

arm_cmd = CommandBool()
arm_cmd.request.value = True

last_request = time.time()
first_request = time.time()

while(not rospy.is_shutdown() and current_state.connected):
    if 