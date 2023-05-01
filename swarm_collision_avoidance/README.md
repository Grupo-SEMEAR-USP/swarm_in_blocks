# swarm_collision_avoidance

Algorithm created to treat the possible collisions between the clovers during their movements.

See user documentation of the clover_blocks at the [main Clover documentation site](https://clover.coex.tech/en/blocks.html).
For more info about swarm_clover_blocks, see the [offical documentation swarm in blocks](https://swarm-in-blocks.gitbook.io/swarm-in-blocks/introduction/swarm-in-blocks)

Internal package documentation is given below.

---

## Usability

This package does not have an interface for user interaction. It is just a package that, when run, promotes services on the clovers that prevents them from colliding.

---

## How it works

### Organization

The package has the CMakeList.txt and package.xml files, which are ROS standards. In addition to them, it has only the src folder.

| Package | Description |
| ------- | -------- |
| `src` | The collision handling algorithm is concentrated in a single file encoded in Python: swarm_collision_avoidance.py |

### node 'swarm_collision_avoidance'

After initializing node 'swarm_collision_avoidance', it was possible to access topics and services from other ROS nodes and allow the logic of the anti-collision algorithm to take place.

### SwarmInternalCollisionAvoidance Class

Class that will check and obtain information about all the drones in the swarm, opening the possibility of checking the current positions of the drones to prevent them from getting too close. 

In addition, it creates a vector of objects of the type SingleClover.

### SingleClover Class

The constructor of this class takes all the important information of a clover, creating several internal variables in the object that characterize it, such as id, initial position, among others.

### avoidCollision Method

This method analyzes, every 0.1 seconds, the position between each pair of clovers in the swarm. If one of these detected distances is less than the minimum safe distance, the angle between the trajectories is calculated using the scalar product concept. After that, a function is called that deals with the collisions according to the angle detected or the fact that one of the drones is stationary.

#### If one of the clovers is stopped

In this case, the first step is to stop the clover that was moving. The stationary clover is shifted orthogonally relative to the trajectory of what was moving. After that, the clover that was in motion follows its trajectory and the clover that has moved from its original position returns to it. This is done by the `__handleStationaryClover` method.

#### Parallel collision

Firstly, both clovers are paralyzed. After that, one of the shamrocks is arbitrarily chosen to move orthogonally relative to the trajectory of the other. After that, the clover that stood still follows its original trajectory. When the latter moves far enough away from the other, the latter is also released to follow its original trajectory.This is done by the `__handleParallelClovers` method.

<center><img src="parallel_collision_perspectiva (1).gif" width=700></center>

#### Non-parallel collision

The first act is also to paralyze the two drones. Arbitrarily, one of the clovers is released to follow his original trajectory. After reaching a safe distance, the other clover is also released to follow its original trajectory. This is done by the `__handleNonParalellClovers` method.

<center><img src="non_parallel_colision_perspectiva (2).gif" width=700></center>

