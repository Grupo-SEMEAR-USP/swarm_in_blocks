#!/usr/bin/python3

import logging

from click import launch
import rospy
import time
from multiprocessing import Process
import subprocess
import signal
import numpy as np
import argparse
import sys
import os

# local import
sys.path.insert(0, os.path.dirname(__file__))
import formation

def strToBool(string):
    assert type(string)==str, "Input needs to be str."
    return str in ['true', 'True', '1', 'y', 'yes', 't']

def boolToStrLower(boolean):
    # analyse if its already on str lower mode
    if boolean in ['true', 'false']:
        return boolean
    # else, transform to str lower
    assert type(boolean)==bool, "Input needs to be bool"
    return str(boolean).lower()

def startRoscore(output_screen=True):
    
    cmd = 'roscore'

    try:
        if output_screen:
            p = subprocess.Popen(cmd)
        else:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    except Exception as e:
        print(e)
    finally:
        if p.poll() is None:
            logging.debug(p.poll())
            p.terminate()
        p.terminate()

def launchGazebo(output_screen=True):
    
    cmd = ['roslaunch', 'swarm_in_blocks', 'gazebo.launch']
    
    try:
        if output_screen:
            p = subprocess.Popen(cmd)
        else:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def launchBlocklyBackEnd(new_page=True, output_screen=True):
    
    new_page = boolToStrLower(new_page)
    cmd = ['roslaunch', 'swarm_in_blocks', 'blocks.launch', f'newpage:={new_page}']
    
    try:
        if output_screen:
            p = subprocess.Popen(cmd)
        else:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def launchSingleVehicle(idx, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0, led=True, camera=False, rangefinder=True, flashlight=False, output_screen=True):
    
    led = boolToStrLower(led)
    camera = boolToStrLower(camera)
    rangefinder = boolToStrLower(rangefinder)
    flashlight = boolToStrLower(flashlight)
    cmd = ['roslaunch','--wait', 'swarm_in_blocks', 'clover_simulated.launch',
            f'ID:={idx}', f'x:={x}', f'y:={y}', f'z:={z}',
            f'R:={roll}', f'P:={pitch}', f'Y:={yaw}', f'led:={led}', f'camera:={camera}', f'rangefinder:={rangefinder}', f'flashlight:={flashlight}']

    try:
        if output_screen:
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        else:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stdin=subprocess.PIPE)
        returncode = p.wait()
    finally:
        if p.poll() is None:
            p.terminate()
        p.terminate()

def launchSimulation(num_of_clovers, init_formation_coords, led=True, camera=False, rangefinder=True, flashlight=False, new_blocks_page=True, output_screen=True, roscore=True, gazebo=True, blocks=True):

    processes = []
    if roscore:
        p = Process(target=startRoscore, args=(output_screen,))
        p.start()
        processes.append(p)
        time.sleep(3)
    
    if gazebo:
        p = Process(target=launchGazebo, args=(output_screen,))
        p.start()
        processes.append(p)

    if blocks:
        p = Process(target=launchBlocklyBackEnd, args=(output_screen,))
        p.start()
        processes.append(p)

    for idx in range(num_of_clovers):
        x = init_formation_coords[idx][0]
        y = init_formation_coords[idx][1]
        p = Process(target=launchSingleVehicle, args=(idx,), kwargs=dict(x=x, y=y, led=led, camera=camera, rangefinder=rangefinder, flashlight=flashlight, output_screen=output_screen))
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

def launchPlanning(blocks, output_screen):

    processes = []
    if blocks:
        p = Process(target=launchBlocklyBackEnd, args=(output_screen,))
        p.start()
        processes.append(p)
    
    rospy.loginfo("Planning architeture done")
    
    def handler(signal_received, frame):
        for p in processes:
            
            p.terminate()
        for p in processes:
            while p.is_alive():
                continue
        rospy.loginfo("Planning architeture terminated")
        exit()

    signal.signal(signal.SIGINT, handler)

def launchNavigation(blocks, output_screen):
    
    processes = []
    if blocks:
        p = Process(target=launchBlocklyBackEnd, args=(output_screen,))
        p.start()
        processes.append(p)
    
    rospy.loginfo("Navigation architeture done")
    
    def handler(signal_received, frame):
        for p in processes:
            
            p.terminate()
        for p in processes:
            while p.is_alive():
                continue
        rospy.loginfo("Navigation architeture terminated")
        exit()

    signal.signal(signal.SIGINT, handler)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Configuration of launch modes on Swarm In Blocks project.")
    # parser.add_argument('--mode', type=str, required=True)
    subparser = parser.add_subparsers(dest='command')
    planning = subparser.add_parser('planing')
    simulation = subparser.add_parser('simulation')
    navigation = subparser.add_parser('navigation')

    # planning args
    planning.add_argument('--blocks', type=str, default='true', help="Launch blocks back-end for blocks programming.")
    planning.add_argument('--quiet', '-q', type=str, default='false', help='Output muted')
    
    # simulation args
    simulation.add_argument('--number_of_clovers', '-n', type=int, default=2, help='Number of clovers of the swarm')
    simulation.add_argument('--formation', '-f', type=str, default='full_square', help='Initial formation of the swarm')
    simulation.add_argument('--led', '-l', type=str, default='true', help='Clovers with led or not')
    simulation.add_argument('--rangefinder', '-r', type=str, default='true', help='Clovers with rangefinder or not')
    simulation.add_argument('--camera', '-c', type=str, default='false', help='Clovers with camera or not. WARNING: cameras on simulation require a large processing power')   
    simulation.add_argument('--flashlight', type=str, default='false', help='Clovers with flashlight or not')
    simulation.add_argument('--quiet', '-q', type=str, default='false', help='Output muted')

    # navigation args
    navigation.add_argument('--blocks', type=str, default='true', help="Launch blocks back-end for blocks programming.")
    navigation.add_argument('--quiet', '-q', type=str, default='false', help='Output muted')

    args, _ = parser.parse_known_args()

    # launch simulation
    if args.command == 'simulation':
        
        # Warning: they are all str type
        led = args.led
        rangefinder = args.rangefinder
        camera = args.camera
        flashlight = args.flashlight

        output_screen = not strToBool(args.quiet)

        if (args.formation=='line'):
            form = formation.line(args.number_of_clovers, args.number_of_clovers)
        elif (args.formation=='full_square'):
            form = formation.full_square(args.number_of_clovers, np.sqrt(args.number_of_clovers))
        elif (args.formation=='empty_square'):
            form = formation.empty_square(args.number_of_clovers, np.sqrt(args.number_of_clovers))
        elif (args.formation=='circle'):
            form = formation.circle(args.number_of_clovers, np.sqrt(args.number_of_clovers))
        elif (args.formation=='triangle'):
            form = formation.triangle(args.number_of_clovers, args.number_of_clovers/3)
        else:
            raise Exception('Formation input doesn\'t match any built-in formations')

        launchSimulation(args.number_of_clovers, form, led=led, camera=camera, rangefinder=rangefinder, flashlight=flashlight, output_screen=output_screen, roscore=False, gazebo=False, blocks=False)
    
    # launch navigation
    if args.command == 'navigation':
        output_screen = not strToBool(args.quiet)
        blocks = strToBool(args.blocks)
        
        launchNavigation(blocks, output_screen)

    # launch navigation
    if args.command == 'planning':
        output_screen = not strToBool(args.quiet)
        blocks = strToBool(args.blocks)
        
        launchPlanning(blocks, output_screen)