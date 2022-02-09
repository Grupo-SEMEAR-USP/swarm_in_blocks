#!/usr/bin/python3

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
      
def takeoff_all(self):
    coord = np.empty((0,4))
    print("All drones taking off")
    for clover in self.swarm:
        point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
        #clover.navigate(x=0, y=0, z=1, auto_arm=True)
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
        clover.land()
        point = [self.init_x[clover.id],self.init_y[clover.id],0,1]
        coord = np.concatenate((coord,[point]))
    plot_preview(coord)
    return coord

#---Formations---

def line(self, z0=1, L=1):
    coord = np.empty((0,4))
    N = self.num_of_clovers
    f = L/(N-1)
    print("Beginning line formation")
    for clover in self.swarm:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        point = [round(f*(N-1-clover.id),2), 0, z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        rospy.sleep(2)
    plot_preview(coord)
    rospy.sleep(5)
    print("Line done\n")
    return coord


def circle(self, xc=4, yc=4, z0=1, r=2):
      coord = np.empty((0,4))
      N = self.num_of_clovers
      print("Beginning circle formation")
      angle = 2*math.pi/N
      for clover in self.swarm:
         x0 = 0 - self.init_x[clover.id]
         y0 = 0 - self.init_y[clover.id]
         xi = r*math.cos(clover.id*angle)
         yi = r*math.sin(clover.id*angle)
         point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
         #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
         coord = np.concatenate((coord,[point]))
         rospy.sleep(5)
      plot_preview(coord)
      rospy.sleep(5)
      print("Circle done\n")
      return coord


def square(self, type="full", z0=1, L=2):
    coord = np.empty((0,4))
    N = self.num_of_clovers
    print("Beginning square formation")
    yi = 0
    n = int(1 + N/4)

    if (type=="empty"):
        if (N%4 == 0):
            (q, coord) = square_side(self, q=0, n=n, yi=0, z0=z0, L=L, coord=coord)
            while (q<N-n):
                yi = yi + L/(n-1)
                (q, coord) = square_side(self, q=q, n=2, yi=yi, z0=z0, L=L, coord=coord)
            (q, coord) = square_side(self, q=q, n=n, yi=L, z0=z0, L=L, coord=coord)
        else:
            (q, coord) = square_side(self, q=0, n=n+1, yi=0, z0=z0, L=L, coord=coord)
            if (N%4 > 1):
                m = n+1
            else:
                m = n
            while (q<N-n):
                yi = yi + L/(m-1)
                (q, coord) = square_side(self, q=q, n=2, yi=yi, z0=z0, L=L, coord=coord)
            if (N%4 < 3):
                (q, coord) = square_side(self, q=q, n=n, yi=L, z0=z0, L=L, coord=coord)
            else:
                (q, coord) = square_side(self, q=q, n=n+1, yi=L, z0=z0, L=L, coord=coord)                

    elif (type=="full"):
        (q, coord) = square_side(self, q=0, n=n, yi=0, z0=z0, L=L, coord=coord)
        while (q<N):
            if (math.sqrt(N) == int(math.sqrt(N))):
                yi = yi + L/(n-1)
                (q, coord) = square_side(self, q=q, n=n, yi=yi, z0=z0, L=L, coord=coord)
            else:
                yi = yi + L/n
                (q, coord) = square_side(self, q=q, n=(N%4), yi=yi, z0=z0, L=L, coord=coord)
                if (N-q == n):
                    (q, coord) = square_side(self, q=q, n=n, yi=L, z0=z0, L=L, coord=coord)
    plot_preview(coord)
    rospy.sleep(5)
    print("Square done\n")
    return coord


#---Support Functions---

def square_side(self, q, n, yi, z0, L, coord):
    j = 0
    N = self.num_of_clovers
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
        rospy.sleep(2)
        if (q==N):
            break
    return(q, coord)


# if __name__ == "__main__":
#     while not rospy.is_shutdown():
#         break
