#!/usr/bin/python3

import logging
import rospy
import time
from multiprocessing import Process
import subprocess
import signal
import traceback
import argparse

def startRoscore(output_screen=True):
    
    cmd = 'roscore'

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    except:
        print(traceback.format_exc())
    finally:
        if p.poll() is None:
            logging.debug(p.poll())
            p.terminate()
        p.terminate()

def launchGazebo(output_screen=True):
    
    cmd = ['roslaunch', 'swarm_in_blocks', 'gazebo.launch']

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def launchBlocklyBackEnd(output_screen=True):
    
    cmd = ['roslaunch', 'swarm_in_blocks', 'blocks.launch']

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)

def launchSingleVehicle(id, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0, output_screen=True):
    
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

def simulation(num_of_clovers, init_formation_coords, output_screen=True):

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
    
    time.sleep(2*num_of_clovers)
    
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

    parser = argparse.ArgumentParser(description="Configuration of launch modes on Swarm In Blocks project.")
    parser.add_argument('--mode', type=str, required=True)
    subparser = parser.add_subparsers(dest='command')
    planning = subparser.add_parser('--planing')
    simulation = subparser.add_parser('--simulation')
    navigation = subparser.add_parser('--navigation')

    planning.add_argument('--blocks', type=bool, default=True)
    
    simulation.add_argument('--num', type=int, default=2)
    simulation.add_argument('--formation', type=str, default='full_square')
    simulation.add_argument('--led', type=bool, default=True)
    simulation.add_argument('--camera', type=bool, default=False)
    simulation.add_argument('--rangefinder', type=bool, default=True)
    simulation.add_argument('--flashlight', type=bool, default=False)

    navigation.add_argument('--blocks', type=bool, default=True)

    args = parser.parse_args()

    if args.command == 'simulation':
        
    num_of_clovers = 2
    spawnGazeboAndVehicles(num_of_clovers,[[0,0,0.3,1],[0,1,0.3,1]])

    

    

