#!/usr/bin/python3

#* Communication with index.html
# onclick of the index.html button "ready" and the AJAX script

from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

@app.route('/')
@app.route('/index.html')
def index():
    return render_template('index.html')


@app.route('/addnumber')
def add():
    a = request.args.get('a', 0, type=float)
    b = request.args.get('b', 0, type=float)
    return jsonify(result=a + b)

#* Launch part of code (ROS communication with the launch) 
import logging
import rospy
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
            logging.debug(p.poll())
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

def launchSingleVehicle(id, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0):
    
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
    num_of_clovers = 2
    spawnGazeboAndVehicles(num_of_clovers,[[0,0,0.3,1],[0,1,0.3,1]])
    app.run()