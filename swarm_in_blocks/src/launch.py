#!/usr/bin/python3

import rospy
import os
import time
from multiprocessing import Process

def startRoscore(verbose=False):
    
    cmd = 'roscore'
    
    if not verbose:
        cmd = cmd + ' 1>/dev/null'
    
    os.system(cmd)

def launchGazebo(verbose=False):
    
    cmd = 'roslaunch swarm_in_blocks gazebo.launch'

    if not verbose:
        cmd = cmd + ' 1>/dev/null'
    
    os.system(cmd)
    rospy.loginfo("Gazebo simulator started.")

def launchSingleVehicle(id, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0, verbose=False):
    
    cmd = f'roslaunch --wait swarm_in_blocks single_vehicle.launch ID:={id} x:={x} y:={y} z:={z} R:={roll} P:={pitch} Y:={yaw}'

    if not verbose:
        cmd = cmd + ' 1>/dev/null'

    os.system(cmd)
    rospy.loginfo(f"Clover {id} launched.")

def spawnGazeboAndVehicles(num_of_clovers, init_formation_coords):

    Process(target=startRoscore).start()
    time.sleep(3)
    
    Process(target=launchGazebo).start()

    for idx in range(num_of_clovers):
        id = idx
        x = init_formation_coords[id][0]
        y = init_formation_coords[id][1]
        Process(target=launchSingleVehicle,args=(id, x, y)).start()
    rospy.loginfo("Simulation architeture done")
    
if __name__ == '__main__':

    num_of_clovers = 2
    spawnGazeboAndVehicles(num_of_clovers,[[0,0,0.3,1],[0,1,0.3,1]])



