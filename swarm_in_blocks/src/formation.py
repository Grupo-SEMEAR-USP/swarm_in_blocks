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
nav4 = rospy.ServiceProxy("clover4/navigate", srv.Navigate)
nav5 = rospy.ServiceProxy("clover5/navigate", srv.Navigate)
nav6 = rospy.ServiceProxy("clover6/navigate", srv.Navigate)
nav7 = rospy.ServiceProxy("clover7/navigate", srv.Navigate)
nav8 = rospy.ServiceProxy("clover8/navigate", srv.Navigate)

land0 = rospy.ServiceProxy("clover0/land", Trigger)
land1 = rospy.ServiceProxy("clover1/land", Trigger)
land2 = rospy.ServiceProxy("clover2/land", Trigger)
land3 = rospy.ServiceProxy("clover3/land", Trigger)
land4 = rospy.ServiceProxy("clover4/land", Trigger)
land5 = rospy.ServiceProxy("clover5/land", Trigger)
land6 = rospy.ServiceProxy("clover6/land", Trigger)
land7 = rospy.ServiceProxy("clover7/land", Trigger)
land8 = rospy.ServiceProxy("clover8/land", Trigger)

# Number of clovers
N = 9

# Initial positions
init_x = []
init_y = []

for i in range(N):
    init_x = init_x + [0]
    init_y = init_y + [i]

def takeoff_all():
    print("All drones taking off")
    for i in range(N):
        nav = "nav" + str(i)
        eval(nav)(z=1, auto_arm=True)
        print("Clover {} taking off".format(i))
    rospy.sleep(7)
    print("Done\n")

def line(z0=1, L=1):
    f = L/(N-1)
    print("Beginning line formation")
    for i in range(N):
        x0 = 0 - init_x[i]
        y0 = 0 - init_y[i]
        nav = "nav" + str(i)
        eval(nav)(x=(x0+f*(N-1-i)), y=y0, z=z0)
        rospy.sleep(2)
    rospy.sleep(5)
    print("Line done\n")

def side_sq(xi, yi, z0, L):
    q = 0
    j = 0
    n = int(1 + N/4)
    f = L/(n-1)
    while (q<N):
        for i in range(q,n+q):
            x0 = 0 - init_x[i]
            y0 = 0 - init_y[i]
            nav = "nav" + str(i)
            eval(nav)(x=(x0+f*(n-1-j)), y=y0+yi, z=z0)
            q = q+1
            j = j+1
            rospy.sleep(2)
        yi = yi + L/(n-1)
        j = 0

def square(x0=0, y0=0, z0=1, L=2):
    print("Beginning square formation")

    #if (N%9==0):
    side_sq(0,0,z0,L)
        #side_sq(0,L,z0,L)



    # nav0(x=x0, y=y0, z=z0)
    # nav1(x=(x0+L), y=(y0-init_y[1]), z=z0)
    # nav2(x=x0, y=(y0-init_y[2]+L), z=z0)
    # nav3(x=(x0+L), y=(y0-init_y[3]+L), z=z0)

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
    for i in range(N):
        nav = "nav" + str(i)
        eval(nav)(x=0, y=0, z=1)
        print("Clover {} returning to initial position".format(i))
    rospy.sleep(5)
    print("Done\n")

def land_all():
    print("All drones landing")
    for i in range(N):
        land = "land" + str(i)
        eval(land)()
        print("Clover {} landing".format(i))
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
            if (N < 2):
                print("You need at least 2 clovers!\n")
            else:
                z0 = int(input("Insert the desired height: "))
                L = int(input("Insert the desired length: "))
                line(z0=z0, L=L)
                rospy.sleep(5)
        elif (key == str('3')):
            if (N < 3):
                print("You need at least 3 clovers!\n")
            else:
                x0 = int(input("Insert initial x coordinate: "))
                y0 = int(input("Insert initial y coordinate: "))
                z0 = int(input("Insert the desired height: "))
                L = int(input("Insert the desired side length: "))
                triangle(x0=x0, y0=y0, z0=z0, L=L)
                rospy.sleep(5)
        elif (key == str('4')):
            if (N < 4):
                print("You need at least 4 clovers!\n")
            else:
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
            rospy.sleep(5)
        elif (key == str('e') or key == str('E')):
            break