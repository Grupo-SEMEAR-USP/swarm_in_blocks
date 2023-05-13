# swarm_examples

We have made examples for you to facilitate the learning process of our platform.

---

## Usability

There is no interface for the user to handle. These are examples that were written in python, which, when rotated, cause movements in the swarm.

---

## How it works


### Organization

The package has the CMakeList.txt and package.xml files, which are ROS standards. In addition to them, it has the src, launch and world folders.

| Package | Description |
| ------- | -------- |
| `launch`  | It contains the file that configures and begins the simulation based on the number of drones, the formation established by them, among other parameters.|
| `src`  | It contains several scripts, each of which promotes different modifications to the swarm.|
| `world`  | Contains a file that uses a language used to model virtual 3D environments for simulators (in our case, we use Gazebo).|
