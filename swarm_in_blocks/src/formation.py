#!/usr/bin/python3

import rospy
from clover import srv
from std_srvs.srv import Trigger
import sys
import time
import math

rospy.init_node("formation", anonymous=True)

# rospy.Subscriber("clover0/mavros/state", State, state_cb, queue_size=10)   
# pub = rospy.Publisher("clover0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
# tel0 = rospy.ServiceProxy('clover0/get_telemetry', srv.GetTelemetry)
# tel1 = rospy.ServiceProxy('clover1/get_telemetry', srv.GetTelemetry)

nav0 = rospy.ServiceProxy("clover0/navigate", srv.Navigate)
nav1 = rospy.ServiceProxy("clover1/navigate", srv.Navigate)
nav2 = rospy.ServiceProxy("clover2/navigate", srv.Navigate)
nav3 = rospy.ServiceProxy("clover3/navigate", srv.Navigate)

land0 = rospy.ServiceProxy("clover0/land", Trigger)
land1 = rospy.ServiceProxy("clover1/land", Trigger)
land2 = rospy.ServiceProxy("clover2/land", Trigger)
land3 = rospy.ServiceProxy("clover3/land", Trigger)

# Initial positions
x_c0 = 0; y_c0 = 0
x_c1 = 0; y_c1 = 1
x_c2 = 0; y_c2 = 2
x_c3 = 0; y_c3 = 3

def takeoff_all():
    print("All drones taking off")
    nav0(x=0, y=0, z=1, auto_arm=True)
    nav1(x=0, y=0, z=1, auto_arm=True)
    nav2(x=0, y=0, z=1, auto_arm=True)
    nav3(x=0, y=0, z=1, auto_arm=True)
    print("Done\n")

def line(x0 = 0, y0 = 0, z0=1, L=1):
    print("Beginning line formation")
    print("C0")
    nav0(x = x0, y = y0, z = z0)
    rospy.sleep(7)
    print("C1")
    nav1(x = (x0+L), y = (y0-y_c1), z = z0)
    print("Line done\n")

def square(x0=0, y0=0, z0=1, L=2):
    print("Beginning square formation")
    #print("C0")
    nav0(x=x0, y=y0, z=z0)
    #rospy.sleep(10)
    #print("C1")
    nav1(x=(x0+L), y=(y0-y_c1), z=z0)
    #rospy.sleep(10)
    #print("C2")
    nav2(x=x0, y=(y0-y_c2+L), z=z0)
    #rospy.sleep(10)
    #print("C3")
    nav3(x=(x0+L), y=(y0-y_c3+L), z=z0)
    print("Square done\n")

def triangle(x0=0, y0=0, z0=1, L=1):
    print("Beginning triangle formation")
    print("C0")
    nav0(x=x0, y=y0, z=z0)
    rospy.sleep(10)
    print("C1")
    nav1(x=(x0+L), y=(y0-y_c1), z=z0)
    rospy.sleep(10)
    print("C2")
    nav2(x=(x0+L/2), y=(y0-y_c2+L), z=z0)
    print("Triangle done\n")

def init_pos():
    print("All drones returning to initial position")
    nav0(x=0, y=0, z=1)
    nav1(x=0, y=0, z=1)
    nav2(x=0, y=0, z=1)
    nav3(x=0, y=0, z=1)
    print("Done\n")

def land_all():
    print("All drones landing")
    land0()
    land1()
    land2()
    land3()
    print("Done\n")

def menu():
    print("Press")
    print("1 - takeoff all")
    print("2 - line formation")
    print("3 - triangle formation")
    print("4 - square formation")
    print("0 - initial position")
    print("L - land all")
    print("E - exit")

if __name__ == "__main__":
    while not rospy.is_shutdown():
        menu()
        #key= input("press a key for action")
        key=sys.stdin.read(1)
        if (key == str('1')):
            takeoff_all()
            rospy.sleep(5)
        elif (key == str('2')):
            line()
            rospy.sleep(5)
        elif (key == str('3')):
            triangle()
            rospy.sleep(5)
        elif (key == str('4')):
            square(x0=2, y0=2, L=2)
            rospy.sleep(5)
        elif (key == str('0')):
            init_pos()
            rospy.sleep(5)
        elif (key == str('l')):
            land_all()
            rospy.sleep(5)
        elif (key == str('e')):
            break

    # line(x0=2, y0=4, L=2)
    # square(x0=2, y0=2, L=2)
    # triangle(x0=2, y0=2, L=2)