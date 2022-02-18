#!/usr/bin/python3

import rospy
import os
import time
from multiprocessing import Process
import subprocess
import signal

def startRoscore(verbose=False):
    
    cmd = 'roscore'

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            print("Entrered if")
            print(p.poll())
            p.terminate()
        p.terminate()

def launchGazebo(verbose=False):
    
    cmd = ['roslaunch', 'swarm_in_blocks', 'gazebo.launch']

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def launchSingleVehicle(id, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0, verbose=False):
    
    cmd = ['roslaunch','--wait', 'swarm_in_blocks', 'single_vehicle.launch',
            f'ID:={id}', f'x:={x}', f'y:={y}', f'z:={z}',
            f'R:={roll}', f'P:={pitch}', f'Y:={yaw}']

    try:
        p = subprocess.Popen(cmd,stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.PIPE)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def spawnGazeboAndVehicles(num_of_clovers, init_formation_coords):

    processes = []
    p = Process(target=startRoscore)
    p.start()
    processes.append(p)
    time.sleep(3)
    
    p = Process(target=launchGazebo)
    p.start()
    processes.append(p)

    for idx in range(num_of_clovers):
        id = idx
        x = init_formation_coords[id][0]
        y = init_formation_coords[id][1]
        p = Process(target=launchSingleVehicle, args=(id, x, y))
        p.start()
        processes.append(p)
    
    time.sleep(5*num_of_clovers)
    
    def handler(signal_received, frame):
        for p in processes:
            p.terminate()
        for p in processes:
            while p.is_alive():
                continue
        exit()

    signal.signal(signal.SIGINT, handler)
    rospy.loginfo("Simulation architeture done")
    
if __name__ == '__main__':

    num_of_clovers = 2
    spawnGazeboAndVehicles(num_of_clovers,[[0,0,0.3,1],[0,1,0.3,1]])



