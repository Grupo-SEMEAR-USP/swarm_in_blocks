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
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

pi = np.pi

#---Formations---

def line(N, L=1):
    coord = np.empty((0,4))
    z0 = 1
    f = L/(N-1)
    print("Beginning line formation")
    for idx in range(N):
        point = [round(f*(N-1-idx),2), 0, z0, 1]
        coord = np.concatenate((coord,[point]))
    print("Line done\n")
    return coord

def circle(N, L=2):
    xc = yc = 0
    coord = np.empty((0,4))
    z0 = 1
    print("Beginning circle formation")
    angle = 2*pi/N
    for idx in range(N):
        xi = L*np.cos(idx*angle)
        yi = L*np.sin(idx*angle)
        point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
    print("Circle done\n")
    return coord

def full_square(N, L=2):
    coord = np.empty((0,4))
    z0 = 1
    print("Beginning full square formation")
    yi = 0
    n = int(1 + N/4)             
    (q, coord) = square_side(N, L, q=0, n=n, yi=0, coord=coord)
    while (q<N):
        if (round(np.sqrt(N),2) == int(np.sqrt(N)) or N%4==0):
            yi = yi + L/(n-1)
            (q, coord) = square_side(N, L, q=q, n=n, yi=yi, coord=coord)
        else:
            yi = yi + L/n
            (q, coord) = square_side(N, L, q=q, n=(N%4), yi=yi, coord=coord)
            if (N-q == n):
                (q, coord) = square_side(N, L, q=q, n=n, yi=L, coord=coord)
    print("Square done\n")
    return coord

def empty_square(N, L=2):
    coord = np.empty((0,4))
    z0 = 1
    print("Beginning empty square formation")
    yi = 0
    n = int(1 + N/4)
    if (N%4 == 0):
        (q, coord) = square_side(N, L, q=0, n=n, yi=0, coord=coord)
        while (q<N-n):
            yi = yi + L/(n-1)
            (q, coord) = square_side(N, L, q=q, n=2, yi=yi, coord=coord)
        (q, coord) = square_side(N, L, q=q, n=n, yi=L, coord=coord)
    else:
        (q, coord) = square_side(N, L, q=0, n=n+1, yi=0, coord=coord)
        if (N%4 > 1):
            m = n+1
        else:
            m = n
        while (q<N-n):
            yi = yi + L/(m-1)
            (q, coord) = square_side(N, L, q=q, n=2, yi=yi, coord=coord)
        if (N%4 < 3):
            (q, coord) = square_side(N, L, q=q, n=n, yi=L, coord=coord)
        else:
            (q, coord) = square_side(N, L, q=q, n=n+1, yi=L, coord=coord)
    print("Square done\n")
    return coord

def triangle(N, L=2):
    coord = np.empty((0,4))
    Ld = 2
    #Variáveis contadoras
    if(N<5):
        c1=1                #variável independente
    else:
        c1=1/2
    cx=0                    #variável contadora para o x
    cy=0                    #variável contadora para o y 
    p=1                     #Parametro de subtração
    id_list = []
    reta = np.sqrt(3)       #Coeficiente angular

    #Laço que define as variáveis a partir do numero de drones
    for index in range(N):
        id_list.append(index)
        if(index%3==0):
            if(index>3):
                Ld += 1            

        if((index+1)%3==0 and index>7):
            p+=1
    
    if(Ld>L):
        print("Side size is not enough")
        print(f"New Side = {Ld}")
        L=Ld

    
    c3=L/2                     #Parametro para base do triângulo
    id=int(np.median(id_list)) #Mediana dos ids
    h = (np.sqrt(3)*L)/2       #Altura do triângulo 
    
    #Verificações
    if(N%2==0 and N%3!=0):
        S=N-1
    elif(N%2!=0 and N>7):
        S=N-p
    elif(N%3==0 and N>3):
        S=N-p
    else:
        S=N
    if(N>7):
        if(N%2!=0 or N%3==0):
            c3 = L/(p+1)

    for l in range(0,N):
        for c in range(0,4):  
        #Define o x     
            if(c==0): 
                if(l<=id and reta*c1*l<=h):
                    x=round(reta*c1*l,2)
                else:
                    x=round(reta*c1*cx,2)
                    cx+=1
                
                if(l>=S and S>2):
                        x=0
                    
        #Define o y  
            elif(c==1):
                if(l<=id and reta*c1*l<=h):
                    y=c1*l
                    cy=0
                else:
                    y=L-c1*cy
                    cy+=1
                if(l>=S):
                    y=c3
                    c3+=1
        #Define o z  
            elif(c==2):
                z=1.0
                if(l>=S and S!=N):
                    z = 3.0

        #Define o quarto parametro
        point=[x,y,z,1]
        coord = np.concatenate((coord,[point]))

    # for clover in self.swarm:
    #     x0 = 0 - self.init_x[clover.id]
    #     y0 = 0 - self.init_y[clover.id]
    #     clover.navigate(x=x0+coord[clover.id][0], y=y0+coord[clover.id][1],z=coord[clover.id][2])

    #     if(clover.id>=S):
    #         rospy.sleep(5)
    #         clover.navigate(x=x0+coord[clover.id][0], y=y0+coord[clover.id][1],z=1)
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
    return coord

def sphere(N, L=2):
    xc = yc = zc = 0
    coord = np.empty((0,4))
    print("Beginning circle formation")
    theta = 2*pi/N
    phi = 2*pi/N
    for i in range(0, int(2*pi)):
        xi = L*np.cos(i*theta)*np.sin(i*phi)
        yi = L*np.sin(i*theta)*np.sin(i*phi)
        zi = L*np.cos(i*phi)
        point = [round(xc+xi,2), round(yc+yi,2), round(zc+zi,2), 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
    print("Circle done\n")
    return coord

def pyramid(self, N, L):
    coord = np.empty((0,4))
    N=self.num_of_clovers
    Ld=2
    
    for index in range(N):
        if(index%3==0):
            if(index>3):
                Ld += 1
    if(Ld>L):
        print("Side size is not enough")
        print(f"New Side = {Ld}")
        L=Ld    
    h = round(((np.sqrt(3)*L)/2),2)
    L1=L
    cpx=0
    cpy=0
    z=1
    for l in range(0,N):
        for c in range(0,4): 
            #Define o x
                if(c==0):
                    if((l-1)%3==0):
                        x=h
                        h-=1/2
                    else:
                        x=cpx

                    if((l-2)%3==0):
                        cpx+=1/2
                        
                    if(l==N-1):
                        x=round(h/2, 2)
                        
            #Define o y
                if(c==1):
                    if(l%3==0 and l>0):
                        L1 -= 1/2

                    if((l-2)%3==0):
                        y=L1
                    elif((l-1)%3==0):
                        y=L1/2
                    else:
                        y=cpy
                        cpy+=1/2

                    if(l==N-1):
                        y=L/2
            
            #Define o z
                if(c==2):
                    if((l-2)%3==0):
                        z+=1
            
        point=[x,y,z,1]
        coord = np.concatenate((coord,[point]))      

    # for clover in self.swarm:
    #     x0 = 0 - self.init_x[clover.id]
    #     y0 = 0 - self.init_y[clover.id]
    #     clover.navigate(x=x0+coord[clover.id][0], y=y0+coord[clover.id][1],z=coord[clover.id][2])
    return coord         

#---Support Functions---

def square_side(N, L, q, n, yi, coord, z0=1):
    j = 0
    if (n == 1):
        f = L/2
        j = -1
    else:
        f = L/(n-1)
    for idx in range(q,n+q):
        point = [round(f*(n-1-j),2), yi, z0, 1]
        #clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
        coord = np.concatenate((coord,[point]))
        q += 1
        j += 1
        if (q==N):
            break
    return(q, coord)

