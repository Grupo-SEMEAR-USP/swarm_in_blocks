#!/usr/bin/python3
import time
from swarm_in_blocks.swarm import Swarm
import rospy

# Create my swarm
myswarm = Swarm()
# Init simulation mode
myswarm.startSimulation()

# get N
N = myswarm.connected_clovers

# Take off all drones and wait 
myswarm.takeOffAll(z=5)

myswarm.setFormation2D('circle', N, 2)
myswarm.applyFormation()

dtheta = 10
dt = 0.5
while not rospy.is_shutdown():
    myswarm.rotateFormation(dtheta, 0, 0)
    time.sleep(dt)
    myswarm.applyFormation(speed=10, wait=False)


