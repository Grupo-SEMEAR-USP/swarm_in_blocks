#!/usr/bin/python3

import rospy
import socket

from swarm_client import SwarmClient
from swarm_client import CloverInstance


def return_clovers(id_list):
    clovers_obj = []
    for id in id_list:
        clover = CloverInstance(id)
        clovers_obj.append(clover)
        # clovers_obj.append(clover_server)

    # rospy.loginfo(f'CloverList: {clovers_obj}')
    rospy.sleep(4)
    return clovers_obj


def led():
    id_list = [0, 1]
     
    clovers = return_clovers(id_list)
    rospy.loginfo('Sending msgs to server')

    for clover in clovers:
        rospy.loginfo('Sending fill effect')
        clover.led_effect('fill', 0, 0, 0)

    rospy.sleep(2)

    for clover in clovers:
        rospy.loginfo('Sending blink command')
        clover.led_effect('blink', 0,0, 255)
    
    rospy.sleep(2)

    clovers[0].led_effect('rainbow')
    clovers[1].led_effect('blink', 255, 0, 0)

    rospy.sleep(2)
    clovers[0].led_effect('wipe', 0, 255, 0)
    clovers[1].led_effect('wipe', 0, 255, 255)

def fly_single_drone(clover=0, height=1):
    # id_list = [clover]

    clover = CloverInstance(clover)

    rospy.sleep(4)

    rospy.loginfo(f"Taking off clover {id} to height {height}")
    clover.navigate(0,0,1, frame_id='body', auto_arm=True)
    rospy.sleep(5)
    clover.navigate(1, 1.5, height, frame_id='aruco_map_detected', auto_arm=True)
    rospy.sleep(8)
    clover.navigate(1, 1, height, frame_id='aruco_map_detected', auto_arm=True)
    rospy.sleep(8)
    # clover.navigate(1, 1, 0, frame_id='aruco_map', auto_arm=True)
    # rospy.sleep(5)
    clover.land()
     
    # while True:
    #     clover.navigate("")


def fly_both_drones():
    id_list = [0, 1]
    clovers = return_clovers(id_list)
    height = 1

    rospy.loginfo('Swarm navigation program starting')

    # rospy.sleep(4)
    for clover in clovers:
        rospy.loginfo('Sending blink command')
        clover.led_effect('blink', 255, 0, 255)

    rospy.sleep(3)

    for clover in clovers:
        rospy.loginfo('Take off initialized')
        clover.navigate(0,0, height, frame_id='body', auto_arm=True)
    
    rospy.sleep(8)

    for clover in clovers:
        rospy.loginfo('Sending blink command')
        clover.led_effect('blink', 0, 0, 255)

    rospy.sleep(6)
    cont = 0

    for clover in clovers:
        clover.land()
    # while cont < 6:
    #     for clover in clovers:
    #         clover.navigate(0,0,0, frame_id='body')
    #     cont += 1
    #     rospy.sleep(1)

    for clover in clovers:
        rospy.loginfo('Landing all drones')
        clover.land()        

    



if __name__ == '__main__':
    rospy.init_node('script')

    # fly_single_drone(clover=0, height=1)
    fly_both_drones()
    # led()