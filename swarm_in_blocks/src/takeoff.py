#!/usr/bin/python3

'''
 * @file offb_node.py
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
'''

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool, SetModeRequest, CommandBoolRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
import time

current_state = State()
# current state recebe a mensagem de callback
def state_cb(msg_cb):
    current_state = msg_cb

rospy.init_node("XABLAU_NO_TAKEOFF", anonymous=True)

rospy.Subscriber("clover0/mavros/state", State, state_cb, queue_size=10)   
local_pos_pub = rospy.Publisher("clover0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
arming_client = rospy.ServiceProxy("clover0/mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy("clover0/mavros/set_mode", SetMode)

rate = rospy.Rate(20)

rospy.loginfo("Waiting connected")
while(not rospy.is_shutdown() and current_state.connected):
    
    rospy.spin()
    rate.sleep()
rospy.loginfo("Connected")


pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

rospy.loginfo("Publishing first pose")
for i in range(0,20):
    local_pos_pub.publish(pose)
    # rospy.spin()
    rate.sleep()
rospy.loginfo("Published")

last_request = time.time()
first_request = time.time()

rospy.loginfo("Changing mode to OFFBOARD")
mode_sent = set_mode_client.call(SetModeRequest(custom_mode = "OFFBOARD"))

rospy.loginfo("Arming")
sucess = arming_client.call(CommandBoolRequest(value=True))

while(not rospy.is_shutdown()):
    # if (current_state.mode != "OFFBOARD" and (time.time() - last_request > 5)):
    #     rospy.loginfo("Changing mode to OFFBOARD")
    #     mode_sent = set_mode_client.call(custom_mode = "OFFBOARD")

    #     if(mode_sent):
    #         rospy.loginfo("Offboard enabled")
           
    #     last_request = time.time()
    # elif (not current_state.armed and (time.time() - last_request > 5)):
    #     rospy.loginfo("Arming")
    #     sucess = arming_client.call(value = "True")
    #     if(sucess):
    #         rospy.loginfo("Vehicle armed")
    #     last_request = time.time()
    
    local_pos_pub.publish(pose)        

    rospy.spin()
    rate.sleep()