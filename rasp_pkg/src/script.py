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

    rospy.loginfo(f'CloverList: {clovers_obj}')
    return clovers_obj


def main():
    id_list = [0, 1]
     
    clovers = return_clovers(id_list)
    rospy.loginfo('Sending msgs to server')

    for clover in clovers:
        rospy.loginfo('Sending blink command')
        clover.led_effect('blink', 255)
    
    rospy.sleep(5)

    clovers[0].led_effect('rainbow')
    clovers[1].led_effect('blink', 0, 0, 255)


if __name__ == '__main__':
    rospy.init_node('script')
    main()