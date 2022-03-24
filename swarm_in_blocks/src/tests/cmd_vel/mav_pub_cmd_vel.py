#!/usr/bin/python3

#     pub = rospy.Publisher("/clover2/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

import rospy
import random
import keyboard

from geometry_msgs.msg import Twist # twist message contains the values of speed in all directions

def move():
    speed_pub = rospy.Publisher('/clover0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)  
    rospy.init_node('talker', anonymous=True)  
    rate = rospy.Rate(5)
    count = 0

    while not rospy.is_shutdown(): 
        twist = Twist()  
        twist.linear.x = 2
        #twist.linear.z = 1
        # twist.angular.z = 1


        count += 1
        rospy.loginfo(" x linear: " + str(twist.linear.x))
        #rospy.loginfo(" z angular: " + str(twist.angular.z))
        #rospy.loginfo(" \n" + str(Twist))
        speed_pub.publish(twist)    
       # speed_pub.publish()
        #speed_pub.publish(Twist.angular.x)
        #rate.sleep()


         # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('q'):  # if key 'q' is pressed 
            print('You Pressed A Key!')
            


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass