from swarm_in_blocks.swarm import Swarm
import rospy

# Number of clovers
N = 49

# Create my swarm
myswarm = Swarm(N)

# Initial formation
myswarm.setInitialFormation('circle', 5)

# Init simulation mode
myswarm.startPlanning()

# Click resume to continue the program
myswarm.plotPreview('2D')

# Take off all drones and wait 
myswarm.takeOffAll(z=10)

# Click resume to continue the program
myswarm.plotPreview('3D')

# empty square formation
myswarm.setFormation2D('empty_square', N, 10)

# full square formation
myswarm.setFormation2D('full_square', N, 10)

# cube formation
myswarm.setFormation3D('cube', N, 5)

# pyramid formation
myswarm.setFormation3D('pyramid', N, 5)

# Click resume to finish the program
myswarm.plotPreview('3D')