#!/usr/bin/python3
from swarm_in_blocks import swarm as sw

# creating myswarm object
myswarm = sw.Swarm()

# start simulation mode
myswarm.startSimulation()

# take off all clovers
myswarm.takeOffAll(z=1.5)