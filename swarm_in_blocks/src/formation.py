#!/usr/bin/python3

from matplotlib import projections
import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

def plot_preview(coord):
    #start_form='False'
    plt.figure(figsize=(8, 8))
    plt.subplots_adjust(bottom = 0.2)
    plt.plot(coord[:,0],coord[:,1],'ro')
    plt.axis([-1,11,-1,11])
    plt.grid(True)
    plt.xticks(np.linspace(0,10,11))
    plt.yticks(np.linspace(0,10,11))
    posit = plt.axes([0.4, 0.1, 0.2, 0.05])
    button = Button(posit,'Confirm')
    #button.on_clicked(start_form='True')
    plt.show(block=False)
    #return start_form

def plot_preview_3d(coord):
    #start_form='False'
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111,projection='3d')
    plt.subplots_adjust(bottom = 0.2)
    ax.plot(coord[:,0],coord[:,1],coord[:,2],'ro')
    #plt.axis([-1,11,-1,11])
    plt.grid(True)
    plt.xticks(np.linspace(0,10,11))
    plt.yticks(np.linspace(0,10,11))
    posit = plt.axes([0.4, 0.1, 0.2, 0.05])
    button = Button(posit,'Confirm')
    #button.on_clicked(start_form='True')
    plt.show(block=False)
    #return start_form
      
def takeoff_all(self):
    coord = np.empty((0,4))
    print("All drones taking off")
    for clover in self.swarm:
        point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
        clover.navigate(x=0, y=0, z=1, auto_arm=True)
        coord = np.concatenate((coord,[point]))
    plot_preview(coord)
    return coord

def initial_position(self):
    coord = np.empty((0,4))
    print("All drones returning")
    for clover in self.swarm:
        point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
        clover.navigate(x=0, y=0, z=1)
        coord = np.concatenate((coord,[point]))
    plot_preview(coord)
    return coord

def land_all(self):
    coord = np.empty((0,4))
    for clover in self.swarm:
        #clover.land()
        point = [self.init_x[clover.id],self.init_y[clover.id],0,1]
        coord = np.concatenate((coord,[point]))
    plot_preview(coord)
    return coord

#---Formations---

def line(self, N, L=1):
    coord = np.empty((0,4))
    z0 = 1
    f = L/(N-1)
    print("Beginning line formation")
    for clover in self.swarm:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        point = [round(f*(N-1-clover.id),2), 0, z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        #rospy.sleep(2)
    plot_preview(coord)
    #rospy.sleep(5)
    print("Line done\n")
    return coord


def circle(self, N, xc=4, yc=4, r=2):
    coord = np.empty((0,4))
    z0 = 1
    print("Beginning circle formation")
    angle = 2*np.pi/N
    for clover in self.swarm:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        xi = r*np.cos(clover.id*angle)
        yi = r*np.sin(clover.id*angle)
        point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        #rospy.sleep(5)
    plot_preview(coord)
    #rospy.sleep(5)
    print("Circle done\n")
    return coord


def square(self, N, type="full", L=2):
    coord = np.empty((0,4))
    z0 = 1
    print("Beginning square formation")
    yi = 0
    n = int(1 + N/4)

    if (type=="empty"):
        if (N%4 == 0):
            (q, coord) = square_side(self, N, L, q=0, n=n, yi=0, coord=coord)
            while (q<N-n):
                yi = yi + L/(n-1)
                (q, coord) = square_side(self, N, L, q=q, n=2, yi=yi, coord=coord)
            (q, coord) = square_side(self, N, L, q=q, n=n, yi=L, coord=coord)
        else:
            (q, coord) = square_side(self, N, L, q=0, n=n+1, yi=0, coord=coord)
            if (N%4 > 1):
                m = n+1
            else:
                m = n
            while (q<N-n):
                yi = yi + L/(m-1)
                (q, coord) = square_side(self, N, L, q=q, n=2, yi=yi, coord=coord)
            if (N%4 < 3):
                (q, coord) = square_side(self, N, L, q=q, n=n, yi=L, coord=coord)
            else:
                (q, coord) = square_side(self, N, L, q=q, n=n+1, yi=L, coord=coord)                

    elif (type=="full"):
        (q, coord) = square_side(self, N, L, q=0, n=n, yi=0, coord=coord)
        while (q<N):
            if (round(np.sqrt(N),2) == int(np.sqrt(N)) or N%4==0):
                yi = yi + L/(n-1)
                (q, coord) = square_side(self, N, L, q=q, n=n, yi=yi, coord=coord)
            else:
                yi = yi + L/n
                (q, coord) = square_side(self, N, L, q=q, n=(N%4), yi=yi, coord=coord)
                if (N-q == n):
                    (q, coord) = square_side(self, N, L, q=q, n=n, yi=L, coord=coord)
    plot_preview(coord)
    #rospy.sleep(5)
    print("Square done\n")
    return coord


#---3D Formations---
def cube(self, N, L):
    coord = np.empty((0,4))
    print("Beginning cube formation")
    n = np.cbrt(N)
    z = 1
    while (round(n,2) != int(n)):
        N +=1
        n = np.cbrt(N)
    q = 0
    for i in range(0, int(n)):
        yi = 0
        for i in range(0,int(n)):
            (q, coord) = square_side(self, n**2, L, q, int(n), yi, z0=z, coord=coord)
            yi = yi + L/(n-1)
        z = z + L/(n-1)
    plot_preview_3d(coord)
    return coord

def sphere(self, N, xc=4, yc=4, zc=4, r=2):
    coord = np.empty((0,4))
    print("Beginning circle formation")
    theta = 2*np.pi/N
    phi = 2*np.pi/N
    for clover in self.swarm:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        xi = r*np.cos(clover.id*theta)*np.sin(clover.id*phi)
        yi = r*np.sin(clover.id*theta)*np.sin(clover.id*phi)
        zi = r*np.cos(clover.id*phi)
        point = [round(xc+xi,2), round(yc+yi,2), round(zc+zi,2), 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        #rospy.sleep(5)
    plot_preview_3d(coord)
    #rospy.sleep(5)
    print("Circle done\n")
    return coord

#---Support Functions---

def square_side(self, N, L, q, n, yi, coord, z0=1):
    j = 0
    if (n == 1):
        f = L/2
        j = -1
    else:
        f = L/(n-1)
    for clover in self.swarm[q:n+q]:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        point = [round(f*(n-1-j),2), yi, z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        q += 1
        j += 1
        #rospy.sleep(2)
        if (q==N):
            break
    return(q, coord)


# if __name__ == "__main__":
#     while not rospy.is_shutdown():
#         break
