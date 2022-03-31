from swarm_in_blocks.swarm import Swarm
import rospy

# Number of clovers
N = 49

# Create my swarm with the number of drones that is launch on Gazebo
# It already gets the initial formation from each drone configuration
myswarm = Swarm()

# Init simulation mode
myswarm.startSimulation()

# Click resume to continue the program
myswarm.plotPreview('2D')

# Take off all drones and wait 
myswarm.takeOffAll(z=10)

# empty square formation
myswarm.setFormation2D('empty_square', N, 10)
myswarm.applyFormation()

# full square formation
myswarm.setFormation2D('full_square', N, 10)
myswarm.applyFormation()

# Click resume to finish the program
myswarm.plotPreview('3D')

# cube formation
myswarm.setFormation3D('cube', N, 5)
myswarm.applyFormation()

# pyramid formation
myswarm.setFormation3D('pyramid', N, 5)
myswarm.applyFormation()

# Click resume to finish the program
myswarm.plotPreview('3D')


