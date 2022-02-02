#!/usr/bin/python3

import rospy
from clover import srv
from std_srvs.srv import Trigger
import sys
import time
import math

rospy.init_node("formation", anonymous=True)


nav0 = rospy.ServiceProxy("clover0/navigate", srv.Navigate)
nav1 = rospy.ServiceProxy("clover1/navigate", srv.Navigate)
nav2 = rospy.ServiceProxy("clover2/navigate", srv.Navigate)


land0 = rospy.ServiceProxy("clover0/land", Trigger)
land1 = rospy.ServiceProxy("clover1/land", Trigger)
land2 = rospy.ServiceProxy("clover2/land", Trigger)


# Initial positions
x_c0 = 0; y_c0 = 0
x_c1 = 0; y_c1 = 1
x_c2 = 0; y_c2 = 2


def menu():
    print("Take off first - start battle")
    print("1 - takeoff all")
    print("2 - iniciate")
    print("0 - initial position")
    print("L - land all")
    print("E - exit")



def takeoff_all():
    print("All drones taking off")
    nav0(x=0, y=0, z=1, auto_arm=True)
    nav1(x=0, y=0, z=1, auto_arm=True)
    nav2(x=0, y=0, z=1, auto_arm=True)
    
    print("Done\n")


def battle():
    dur = True

    while dur:
        


def land_all():
    print("All drones landing")
    land0()
    land1()
    land2()
    
    print("Done\n")


def init_pos():
    print("All drones returning to initial position")
    nav0(x=0, y=0, z=1)
    nav1(x=0, y=0, z=1)
    nav2(x=0, y=0, z=1)
    
    print("Done\n")




if __name__ == "__main__":
    while not rospy.is_shutdown():
        menu()
        #key= input("press a key for action")
        key=sys.stdin.read(1)
        if (key == str('1')):
            takeoff_all()
            rospy.sleep(5)
        elif (key == str('2')):
            battle()
            rospy.sleep(5)
        
        elif (key == str('0')):
            init_pos()
            rospy.sleep(5)
        elif (key == str('l')):
            land_all()
            rospy.sleep(5)
        elif (key == str('e')):
            break
