# swarm_checker

Implementation of a ROS node that monitors the state of the entire clovers swarm. In each vehicle, the presence of nodes and services is verified and, after the collection of this data, the information is published in ROS Topic swarm_"/checker/state".

---

## Usability

This package does not have an interface for user interaction. It's just a packet that runs when the swarm is started that captures all the information from the clover and publishes it to a ROS Topic for other algorithms to make use of that data.
---

## How it works


### Organization

The package has the CMakeList.txt and package.xml files, which are ROS standards. In addition to them, it has the src and msg folders.

| Package | Description |
| ------- | -------- |
| `msg`  |  Defines a message named SwarmState that is used by the main SwarmChecker code. This message contains information about the clovers.|
| `src`  | Contains the file that is the center of the purpose of this package. This file scans all clovers and publishes the data to the topic.|


### `swarm_checker` node

After this node starts, a ROS Publisher is created that will publish to ROS Topic /swarm_checker/state.

#### checkKnownCloversLoop method

Starts the continuous swarm check by running a loop with a frequency of 1Hz, which calls various internal methods to make specific checks. 

Among the internal methods, we have functions that check the number of clovers in the swarm, that check if the MAVROS and simple_offboard nodes are working, that returns information about the connection, mode and armament of the clover and, finally, that check the availability of clover's services. At the end, the previously created ROS Publisher is triggered and the information is published to the topic.

#### Topics Published

* `~/swarm_checker/state`: Topic that contains the most varied information about all the clovers of the swarm. Among them, we highlight the number of clovers, the IDs of the clovers, the clovers that are connected, the clovers that are failing, among other data.
