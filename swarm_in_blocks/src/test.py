from swarm import Swarm

swarm = Swarm(2)
swarm.startSimulation(already_launched=True)

# swarm.takeoff_all()
swarm.setFormation2D('line', swarm.num_of_clovers, 5)
swarm.transformFormation()
swarm.plot_preview()
swarm.applyFormation()

swarm.land_all()

