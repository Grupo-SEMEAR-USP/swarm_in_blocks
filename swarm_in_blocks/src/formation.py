#!/usr/bin/python3

from tkinter.simpledialog import SimpleDialog
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

def array(N):
    #Formação da matriz
    matriz=[]
    for i in range(N):
        line=[]
        for j in range(4):
            line.append(0.0)
        matriz.append(line)
    return matriz

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
        #clover.navigate(x=0, y=0, z=1)
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
            if (np.sqrt(N) == int(np.sqrt(N)) or N%4==0):
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

def triangle_matriz(N):
    triangle_side = array(N)
    L=2
    #Variáveis contadoras
    if(N<5):
        c1=1    #Primeira variável independente
    else:
        c1=1/2
    c2=0        #Segunda variável independente
    c3=1        #Parametro para base do triângulo 
    p=1         #Parametro de subtração

    id_list = []
    reta = math.sqrt(3) #Coeficiente angular

    #Laço que define as variáveis a partir do numero de drones
    for index in range(N):
        id_list.append(index)

        if(index%3==0):
            if(index>3):
                L += 1
                
        if(index>7):
            if(N%2!=0 or N%3==0):
                c3 = 1/2

        if((index+1)%3==0 and index>7):
            p+=1

    #######################################
    id=int(np.median(id_list)) #Media dos ids
    h = (math.sqrt(3)*L)/2     #Altura do triângulo 

    #Verificações
    if(N%2==0 and N%3!=0):
        S=N-1

    if(N%2!=0 and N>7):
        S=N-p
    if(N%3==0):
        S=N-p
    for c in range(0,4):
        
        for l in range(0,N):
        #Define o x     
            if(c==0): 
                if(l<=id and reta*c1*l<=h):
                    triangle_side[l][c]  = round(reta*c1*l,2)

                else:
                    triangle_side[l][c] = round(reta*c1*c2,2)
                    c2+=1
                
                if(l>=S and S>2):
                    triangle_side[l][c] = 0
                    
        #Define o y  
            elif(c==1):
                if(l<=id and reta*c1*l<=h):
                    triangle_side[l][c] = c1*l
                    c2=0
                else:
                    triangle_side[l][c] = L-c1*c2
                    c2+=1

                if(l>=S):
                    triangle_side[l][c] = c3
                    c3+=1
            #Define o z  
            elif(c==2):
                triangle_side[l][c] = 1

        #Define o quarto parametro
            elif(c==3):
                triangle_side[l][c] = 1

    return triangle_side

def triangle(self,N):
    side = triangle_matriz(N)
    z0=1
    
    for clover in self.swarm:
        x0 = 0 - self.init_x[clover.id]
        y0 = 0 - self.init_y[clover.id]
        clover.navigate(x=x0+side[clover.id][0], y=y0+side[clover.id][1],z=side[clover.id][2])
    print(side)
    return side
         

#---3D Formations---
def cube(self, N, L):
    coord = np.empty((0,4))
    print("Beginning cube formation")
    n = np.cbrt(N)
    z = 1
    if (n == int(n)):
        q = 0
        for i in range(0, int(n)):
            yi = 0
            for i in range(0,int(n)):
                (q, coord) = square_side(self, n**2, L, q, int(n), yi, z0=z, coord=coord)
                yi = yi + L/(n-1)
            z = z + L/(n-1)
    plot_preview_3d(coord)
    return coord

def piramide_matriz(N):
    piramide_array = array(N)
    for index in range(N):
        if(index%3==0):
            if(index>3):
                L += 1

    h = (math.sqrt(3)*L)/2
    L1=L
    cp=0
    z=1
    for c in range(0,4):
        for l in range(0,N):
            if(c==0):
                if(l%3==0):
                    L-=1/2
                if((l-1)%3==0):
                    piramide_array[l][c]=round(h,2)
                else:
                    piramide_array[l][c]=0
                if(l==N-1):
                    piramide_array[l][c]=round(h/2, 2)
            
            if(c==1):
                if(l%3==0):
                    L1 -= 1/2

                if((l-2)%3==0):
                    piramide_array[l][c]=L1

                else:
                    piramide_array[l][c]=cp
                    cp+=1/2

                if(l==N-1):
                    piramide_array[l][c]=L/2

            if(c==2):

                piramide_array[l][c]=z

                if((l-2)%3==0):
                    z+=1
            
            if(c==3):
                piramide_array[l][c]=1

def piramide(self, N):
        sides = piramide_matriz(N)
        for clover in self.swarm:
            x0 = 0 - self.init_x[clover.id]
            y0 = 0 - self.init_y[clover.id]
            clover.navigate(x=x0+sides[clover.id][0], y=y0+sides[clover.id][1],z=sides[clover.id][2])
        print(sides)
        return sides         

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
