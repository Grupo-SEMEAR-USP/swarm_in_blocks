# swarm_in_blocks
###### This package plays a crucial role in launching and managing the navigation services of our swarm of drones. This package plays a crucial role in launching and managing the navigation services of the swarm of drones.

 - See user documentation of the clover_blocks at the [main Clover documentation site](https://clover.coex.tech/en/blocks.html).
 -  For more info about swarm_clover_blocks, see the [offical documentation swarm in blocks](https://swarm-in-blocks.gitbook.io/swarm-in-blocks/introduction/swarm-in-blocks)

Internal package documentation is given below.

| Package | Description |
| ------- | -------- |
| `src` |  The launch algorithm is concentrated in a single file encoded in Python:  launch.py |


---

## Usability

All the most recent available ROS, Gazebo and the Swarm In Blocks packages and a linux terminal to run the command and start all tree of drones functionalities.

---
## How it works


### Simulation
It calls the simulation indeed, using the Gazebo. It allows organizing the clovers in 2D formations, such as some geometries, letters and words, it's also possible to make canonical 3D geometries and generate formations based on loaded meshes or point clouds. We can also apply LED effects in the swarm, all connected and in real-time.
The simulation.launch initializes the simulation environment. Gazebo with the Gazebo ROS API, blocks back-end, swarm checker node and the simulated clovers start with the following command:
roslaunch swarm_in_blocks simulation.launch num:=2 formation:=line led:=true camera:=false 
#### Parameters:
`num`:=(1,2,3...) - Number of drones that will be launched on gazebo.
formation:=(line, empty_square, full_square, circle) - Initial formation of the swarm.

`led`:=(true, false) - Launch clovers with leds or not.

`camera`:=(true, false) -  Launch clovers with camera or not. Obs: camera takes much processing and might cause fps dropping in gazebo when launched for all the swarm.
`rangefinder`:=(true, false) - Launch clovers with rangefinder.
flashlight

#####Example:
``````
roslauch swarm_in_blocks simulation.launch num:=2 led:=true camera:=true
``````

### swarm_clover_blocks node

The package has the launch files to launch all the drones selected. it will iniciate all the drones services individually, connecting themselves. Also implementing the formation geometry, the LED, camera or range finder behavior and more necessary inicializations.
