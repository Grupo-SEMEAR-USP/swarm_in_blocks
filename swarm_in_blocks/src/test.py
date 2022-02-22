
from swarm import Swarm
import plot
import rospy

swarm = Swarm(2)
swarm.startSimulation(already_launched=True)
swarm.takeoffAll()

input()

swarm.transformFormation(1,1,1,0,0,0,2,2,5)
swarm.applyFormation()

input()
