# FPV

# Usability
The development of sFPV (Swarm First Person View) is intended to allow drone pilots to control the flight in real time through a camera installed on the drone.This year, we decided to restart its structure, making it run also completely on the web to integrate with the Swarm Station.

# How it works ?
# Organization
The package is organized in two main parts: the backend and the frontend. The backend is a ROS node, which implements all the services and topics needed for running Blockly-generated  JavaScript. The frontend is a web application, which provides a user interface for the backend.With this, it was possible to ensure the visualization of the drone image in real time with the possibility of handling it by computer keys, as well as drone data, such as battery, telemetry, CPU data and state, which provides the user with greater flight control of the drone, focusing on the importance of having greater safety when putting a drone to fly

| Package | Description |
| ------- | -------- |
| `msg`  |  Contains the messages that will be sent, basically the user's request and the id relative to the request |
| `src`  |  This is the source directory that contains the script files to run the fpv application |
| `launch` | The launch is the command that possibility the execution of the code in the terminal  |

# Frontend
The frontend application uses roslib.js library for communicating with backend node and other ROS resources. Our main developments were focused on this part of the package, so we will explain it in more detail.The web interface developed in HTML and CSS, that the user can use to control de drone.
<p align="center"> 
    <img width="600" src="https://github.com/Grupo-SEMEAR-USP/swarm_in_blocks/blob/master/assets/fpv_2023.gif"/>
</p>

# How to use it ?
For use the sFPV node of the swarm_in_blocks package, you need to have a simulation running on the gazebo. With this, you can open a terminal and run the following commands: 
`roslaunch swarm_in_blocks simulation.launch num:=2 formation:=line led:=true camera:=true`

With this command, we have the simulation in the gazebo for the establishment of connection with the drone image. If the connection is not established, it is necessary to execute the command roslaunch rosbridge_server rosbridge_websocket.launch port=9090  in order to obtain a simultaneous communication of the simulation with the web FPV.
After these commands, finally, it becomes necessary to run the fpv.hmtl code to obtain the FPV web interface. With these steps, we have the complete operation of FPV.

So, if you want more information about the sFPV, please check the documentation that is published in this link: https://swarm-in-blocks.gitbook.io/swarm-in-blocks/background-theory/systems/first-person-view-new.

