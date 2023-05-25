#!/usr/bin/python3
import numpy as np
import logging
import random
import rospy
import socket

from swarm_client import SwarmClient
from swarm_client import CloverInstance

pi = np.pi

def return_clovers(id_list):
    clovers_obj = []
    for id in id_list:
        clover = CloverInstance(id)
        clovers_obj.append(clover)
        # clovers_obj.append(clover_server)

    # rospy.loginfo(f'CloverList: {clovers_obj}')
    rospy.sleep(4)
    return clovers_obj



def square3d(id_list, height = 1, L = 1.5):
        
    clovers = return_clovers(id_list)

    waitTime = 5

    rospy.loginfo('Square 3d program starting')
    

    for clover in clovers:
        rospy.loginfo('Sending blink command')
        clover.led_effect('blink', 255, 0, 255)

    for clover in clovers:
        rospy.loginfo('Takeoff initialized')
        clover.navigate(0,0,height, frame_id='body')

    rospy.sleep(waitTime+2)

    for clover in clovers:
        clover.led_effect('rainbow')

    for clover in clovers:
        rospy.loginfo('Starting moviment')
        clover.navigate(0,0,1, frame_id='body')

    rospy.sleep(waitTime)

    clovers[0].led_effect('wipe', 255, 0, 0)
    clovers[1].led_effect('wipe', 0, 255, 0)

    for clover in clovers:
        clover.navigate(2,0,0, frame_id='body')

    rospy.sleep(waitTime+3)

    clovers[0].led_effect('wipe', 0, 255, 0)
    clovers[1].led_effect('wipe', 255, 0, 0)
    
    for clover in clovers:
        clover.navigate(0,0,-1, frame_id='body')    

    rospy.sleep(waitTime)

    clovers[0].led_effect('wipe', 255, 255, 0)
    clovers[1].led_effect('wipe', 255, 255, 0)

        
    for clover in clovers:
        clover.navigate(-2,0,0, frame_id='body')    

    rospy.sleep(waitTime+3)

    for clover in clovers:
        clover.led_effect('rainbow')

    rospy.sleep(waitTime)

    for clover in clovers:
        clover.land()



# def circle(N, L=2):
#     xc = yc = 0
#     if 2*pi*L < N:
#         L = round(N/(2*pi),2)
#         print("Distance between drones is too short\nSetting new length as {}".format(L))
#     coord = np.empty((0,4))
#     z0 = 1
#     logging.debug("Beginning circle formation")
#     angle = 2*pi/N
#     for idx in range(N):
#         xi = L*np.cos(idx*angle)
#         yi = L*np.sin(idx*angle)
#         point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
#         coord = np.concatenate((coord,[point]))
#     logging.debug("Circle done\n")
#     return coord



if __name__ == '__main__':
    rospy.init_node('formations')

    N = rospy.get_param('numberClovers')
    id_list = []
    for i in range(N):
        id_list.append(i-1)

    square3d(id_list)