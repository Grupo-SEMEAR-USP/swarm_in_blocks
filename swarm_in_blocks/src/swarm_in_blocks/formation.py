#!/usr/bin/python3
import numpy as np
import logging
import random
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
import transform

pi = np.pi

#---Formations---

def line(N, L=1):
    if L/N < 1:
        L = N-1
        print("Distance between drones is too short\nSetting new length as {}".format(L))
    coord = np.empty((0,4))
    z0 = 1
    f = L/(N-1)
    logging.debug("Beginning line formation")
    for idx in range(N):
        point = [round(f*(N-1-idx),2), 0, z0, 1]
        coord = np.concatenate((coord,[point]))
    coord = transform.translateFormation(coord, -L/2, 0, 0)
    logging.debug("Line done\n")
    return coord

def circle(N, L=2):
    xc = yc = 0
    if 2*pi*L < N:
        L = round(N/(2*pi),2)
        print("Distance between drones is too short\nSetting new length as {}".format(L))
    coord = np.empty((0,4))
    z0 = 1
    logging.debug("Beginning circle formation")
    angle = 2*pi/N
    for idx in range(N):
        xi = L*np.cos(idx*angle)
        yi = L*np.sin(idx*angle)
        point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
        coord = np.concatenate((coord,[point]))
    logging.debug("Circle done\n")
    return coord

def full_square(N, L=2):
    n = int(np.sqrt(N)) 
    if L/np.ceil(N/n) < 1:
        L = np.ceil(N/n)
        print("Distance between drones is too short\nSetting new length as {}".format(L))
    coord = np.empty((0,4))
    z0 = 1
    logging.debug("Beginning full square formation")
    yi = 0            
    (q, coord) = square_side(N, L, q=0, n=n, yi=0, coord=coord)
    while (q<N):
        if (round(np.sqrt(N),2) == int(np.sqrt(N))):
            yi = yi + L/(n-1)
            (q, coord) = square_side(N, L, q=q, n=n, yi=yi, coord=coord)
        elif (N%n == 0):
            yi = yi + L/((N//n)-1)
            (q, coord) = square_side(N, L, q=q, n=n, yi=yi, coord=coord)
        else:
            yi = yi + L/(N//n)
            (q, coord) = square_side(N, L, q=q, n=n, yi=yi, coord=coord)
    coord = transform.translateFormation(coord, -L/2, -L/2, 0)
    logging.debug("Square done\n")
    return coord

def empty_square(N, L=2):
    if L/np.ceil((N/4)+1) < 1:
        L = np.ceil((N/4)+1)
        print("Distance between drones is too short\nSetting new length as {}".format(L))
    coord = np.empty((0,4))
    z0 = 1
    logging.debug("Beginning empty square formation")
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
    coord = transform.translateFormation(coord, -L/2, -L/2, 0)
    logging.debug("Square done\n")
    return coord

def triangle(N, L=2):
    logging.debug("Beginning triangle formation")
    coord = np.empty((0,4))
    Ld = 2                  #Ld is the default size for the side
    #Variáveis contadoras
    cx=0                    #counter variable for x
    cy=0                    #counter variable for y 
    p=1                     #Subtraction Parameter
    id_list = []
    reta = np.sqrt(3)       #Coeficiente angular

    #Loop that sets the variables from the number of drones
    for index in range(N):
        id_list.append(index)
        if(index%3==0):
            if(index>3):
                Ld += 1            

        if((index+1)%3==0 and index>7):
            p+=1
    
    if(Ld>L):
        logging.debug("Side size is not enough")
        logging.debug(f"New Side = {Ld}")
        L=Ld

    
    c3=L/2                     #Parameter for the base of the triangle
    id=int(np.median(id_list)) #Median of ids
    h = (np.sqrt(3)*L)/2       #Height of the triangle 
    
    #Checks

    if(N%2==0 and N%3!=0):
        S=N-p                  # p are drones base 
    elif(N%2!=0 and N>7):
        S=N-p
    elif(N%3==0 and N>3):
        S=N-p
    else:
        S=N
    if(N>7):
        if(N%2!=0 or N%3==0):
            c3 = L/(p+1)      # c3 is the pose to drones base
        if(N%2==0 and N>12):
            c3 = L/p

    for l in range(0,N):
        z=1.0
        if(l<=id and (h/Ld)*l<=h):
            x=round((h/Ld)*l,2)
            y=(L/Ld)*(l/2)
            cy=0

            if(l==id and N<5):
                x=round(h,2)
                y = L/2

        else:
            x=round((h/Ld)*cx,2)
            y = L - (L/Ld)*(cy/2)
            cx+=1
            cy+=1
            if(l==id and N<5):
                x=round(h,2)
                y=L/2
        
        if(l>=S):
            y=c3
            c3+=L/Ld
            if(S>2):
                x=0
            if(S!=N):
                z=1.5  

        point=[x,y,z,1]
        coord = np.concatenate((coord,[point]))
    coord = transform.translateFormation(coord, -L*np.sqrt(3)/6, -L/2, 0)
    logging.debug("Triangle done\n")
    return coord             
        
   
#---3D Formations---
def cube(N, L):
    coord = np.empty((0,4))
    logging.debug("Beginning cube formation")
    n = np.cbrt(N)
    drones_amount = N
    z = 1
    # Increases the amount of clovers until be possible to make a complete cube
    while (round(n,2) != int(n)):
        drones_amount +=1
        n = np.cbrt(drones_amount)
    q = 0
    # Makes the cube layer by layer
    for i in range(0, int(n)):
        yi = 0
        for i in range(0,int(n)):
            (q, coord) = square_side(n**2, L, q, int(n), yi, z0=z, coord=coord)
            yi = yi + L/(n-1)
        z = z + L/(n-1)
    selected_coords = partial_formation(N, coord)
    selected_coords = transform.translateFormation(selected_coords, -L/2, -L/2, 0)
    return selected_coords

def sphere(N, L=2):
    xc = yc = zc = 0
    coord = np.empty((0,4))
    logging.debug("Beginning circle formation")
    theta = 2*pi/N
    phi = 2*pi/N
    for i in range(0, N):
        xi = L*np.cos(i*theta)*np.sin(i*phi)
        yi = L*np.sin(i*theta)*np.sin(i*phi)
        zi = L*np.cos(i*phi)
        point = [round(xc+xi,2), round(yc+yi,2), round(zc+zi,2), 1]
        coord = np.concatenate((coord,[point]))
    logging.debug("Circle done\n")
    return coord

def pyramid(N, L):
    coord = np.empty((0,4))
    Ld=2
    c=1
    for index in range(N):
        if(index%3==0):
            if(index>3):
                Ld += 1
            if(index>10):
                c+=1/3
    if(Ld>L):
        logging.debug("Side size is not enough")
        logging.debug(f"New Side = {Ld}")
        L=Ld    
    h = round(((np.sqrt(3)*L)/2),2)
    cpy=0
    z=1
    for idx in range(N):
        if((idx-1)%3==0):
            point=[(L/Ld)*(cpy/2), L - (L/Ld)*(cpy/2), z, 1]
            cpy+=1
            
            
        if((idx-2)%3==0):
            point=[h - 0.7*(L/Ld)*(cpy/2)*round(np.sqrt(3),2) + c*(L/Ld), L/2, z, 1]
            z+=1
            
        
        if((idx%3)==0):
            point = [(L/Ld)*(cpy/2),(L/Ld)*(cpy/2), z, 1]
        

        coord = np.concatenate((coord,[point]))
    
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
        coord = np.concatenate((coord,[point]))
        q += 1
        j += 1
        if (q==N):
            break
    return(q, coord)

def partial_formation(N, coord):
    random_coords = random.sample(range(0, len(coord)), N)
    random_coords.sort()
    selec_coords = coord[random_coords, :]
    return selec_coords