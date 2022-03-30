from swarm_in_blocks import swarm as sw

# Create my swarm
myswarm = sw.Swarm()
# Init simulation mode
myswarm.startSimulation()

# Take off all drones and wait 
myswarm.takeOffAll(z=10)

myswarm.setFormation('circle', )
