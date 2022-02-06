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
nav3 = rospy.ServiceProxy("clover3/navigate", srv.Navigate)

land0 = rospy.ServiceProxy("clover0/land", Trigger)
land1 = rospy.ServiceProxy("clover1/land", Trigger)
land2 = rospy.ServiceProxy("clover2/land", Trigger)
land3 = rospy.ServiceProxy("clover3/land", Trigger)

# Number of clovers
N = 4

# Initial positions
init_x = []
init_y = []

for i in range(N):
    init_x = init_x + [0]
    init_y = init_y + [i]

def takeoff_all():
    print("All drones taking off")
    nav0(x=0, y=0, z=1, auto_arm=True)
    nav1(x=0, y=0, z=1, auto_arm=True)
    nav2(x=0, y=0, z=1, auto_arm=True)
    nav3(x=0, y=0, z=1, auto_arm=True)
    rospy.sleep(7)
    print("Done\n")

def line(x0 = 0, y0 = 0, z0=1, L=1):
    print("Beginning line formation")
    nav0(x = (x0+L), y = y0, z = z0)
    rospy.sleep(5)
    nav1(x = (x0+L/(N-2)), y = (y0-init_y[1]), z = z0)
    rospy.sleep(5)
    nav2(x = (x0+L/(N-1)), y = (y0-init_y[2]), z = z0)
    rospy.sleep(5)
    nav3(x = x0, y = (y0-init_y[3]), z = z0)
    rospy.sleep(10)
    print("Line done\n")

def square(x0=0, y0=0, z0=1, L=2):
    print("Beginning square formation")
    #print("C0")
    nav0(x=x0, y=y0, z=z0)
    #rospy.sleep(10)
    #print("C1")
    nav1(x=(x0+L), y=(y0-init_y[1]), z=z0)
    #rospy.sleep(10)
    #print("C2")
    nav2(x=x0, y=(y0-init_y[2]+L), z=z0)
    #rospy.sleep(10)
    #print("C3")
    nav3(x=(x0+L), y=(y0-init_y[3]+L), z=z0)
    rospy.sleep(10)
    print("Square done\n")

def triangle(x0=0, y0=0, z0=1, L=1):
    print("Beginning triangle formation")
    # print("C0")
    nav0(x=x0, y=y0, z=z0)
    # rospy.sleep(10)
    # print("C1")
    nav1(x=(x0+L), y=(y0-init_y[1]), z=z0)
    # rospy.sleep(10)
    # print("C2")
    nav2(x=(x0+L/2), y=(y0-init_y[2]+L), z=z0)
    rospy.sleep(10)
    print("Triangle done\n")

def init_pos():
    print("All drones returning to initial position")
    nav0(x=0, y=0, z=1)
    nav1(x=0, y=0, z=1)
    nav2(x=0, y=0, z=1)
    nav3(x=0, y=0, z=1)
    rospy.sleep(5)
    print("Done\n")

def land_all():
    print("All drones landing")
    land0()
    land1()
    land2()
    land3()
    rospy.sleep(5)
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
        key= input("\n")
        #key=sys.stdin.read(1)
        if (key == str('1')):
            takeoff_all()
            rospy.sleep(2)
        elif (key == str('2')):
            x0 = int(input("Insert initial x coordinate: "))
            y0 = int(input("Insert initial y coordinate: "))
            z0 = int(input("Insert the desired height: "))
            L = int(input("Insert the desired length: "))
            line(x0=x0, y0=y0, z0=z0, L=L)
            rospy.sleep(5)
        elif (key == str('3')):
            x0 = int(input("Insert initial x coordinate: "))
            y0 = int(input("Insert initial y coordinate: "))
            z0 = int(input("Insert the desired height: "))
            L = int(input("Insert the desired side length: "))
            triangle(x0=x0, y0=y0, z0=z0, L=L)
            rospy.sleep(5)
        elif (key == str('4')):
            x0 = int(input("Insert initial x coordinate: "))
            y0 = int(input("Insert initial y coordinate: "))
            z0 = int(input("Insert the desired height: "))
            L = int(input("Insert the desired side length: "))
            square(x0=x0, y0=y0, z0=z0, L=L)
            rospy.sleep(5)
        elif (key == str('0')):
            init_pos()
            rospy.sleep(2)
        elif (key == str('l') or key == str('L')):
            land_all()
            rospy.sleep(2)
        elif (key == str('e') or key == str('E')):
            break