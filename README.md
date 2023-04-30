
[![Latest release](https://img.shields.io/github/v/release/aregtech/areg-sdk?label=Latest%20release&style=social)](https://github.com/aregtech/areg-sdk/releases/tag/v1.0.0)
[![Stars](https://img.shields.io/github/stars/aregtech/areg-sdk?style=social)](https://github.com/aregtech/areg-sdk/stargazers)
[![Fork](https://img.shields.io/github/forks/aregtech/areg-sdk?style=social)](https://github.com/aregtech/areg-sdk/network/members)
[![Watchers](https://img.shields.io/github/watchers/aregtech/areg-sdk?style=social)](https://github.com/aregtech/areg-sdk/watchers)

---

## Table of contents[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#table-of-contents)
- [Motivation](#motivation)
- [Composition](#composition)
- [Software build](#software-build)
    - [Clone sources](#clone-sources)
    - [Build with `cmake`](#build-with-cmake)
    - [Build with `make`](#build-with-make)
    - [Build with `msbuild`](#build-with-msbuild)
    - [Build with IDE](#build-with-ide)
- [Integration](#integration)
    - [Integrate for development](#integrate-for-development)
    - [Configure multicast router](#configure-multicast-router)
    - [Configure logging](#configure-logging)
- [Roadmap](#roadmap)
- [Interface-centricity](#interface-centricity)
- [More than embedded](#more-than-embedded)
- [Use cases and benefits](#use-cases-and-benefits)
    - [Distributed solution](#distributed-solution)
    - [Driverless devices](#driverless-devices)
    - [Real-time solutions](#real-time-solutions)
    - [Digital twin](#digital-twin)
    - [Simulation and tests](#simulation-and-tests)
- [Examples](#examples)
- [License](#license)
- [Call to action](#call-to-action)

---

## Introduction [![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#swarm_intro)

---

Nowadays, **swarms of drones** are getting more and more applications and being used in several different areas, from agriculture to surveillance and rescues. But controlling a high amount of drones isn't a simple task, demanding a lot of studies and complex software.

Swarm in Blocks (from it's origin in 2022) was born looking to make a *high-level interface based on the blocks language*, to make simple handling swarms, without requiring advanced knowledge in all the necessary platforms, creating tools to allow a lot of applications based on the user needs and also using the Clover platform.

In 2023, Swarm in Blocks has taken an even bigger step, looking to fulfill our biggest vision **"It's never been easy to Swarm"**, we talk to transcend the local scope of the past project and explore the biggest problems for implementing a Swarm. For Copterhack 2023, we present Swarm in Blocks 2.0, an even more complete platform with the purpose of facing the biggest difficulties of a Swarm in a simple and polished way.

<p align="center">
    <img width="600" src="assets/intro/clovers_leds.gif" />
</p>


### Swarm in Blocks 2022 

---

Swarm in Blocks is a CopterHack 2022 project. It's a high-level interface based on the blocks language, which consists of fitting code parts, like a puzzle. Each script represents a functionality, for example, conditional structures, loops, or functions that receive parameters and return an instruction to the swarm. 

<p align="center">
    <img width="500" src="assets/intro/blocks.gif" />
</p>

<p align="center">
    <img width="600" src="assets/intro/ring.gif" />
</p>


For more information on our project from last year, see our final article in [Swarm in Blocks 2022](https://clover.coex.tech/en/swarm_in_blocks.html).

Even with the huge facilities that the block platform offers, we realized that this was just the tip of the iceberg when it comes to deploying real swarms. Several other operational and conceptual problems in validating a real swarm still haunted the general public. With that, this year's project comes precisely with the purpose of tackling the main problems in validating a Swarm in a simple and polished way.

### What's new

---

As already mentioned, of the various problems that can increase the complexity of a real swarm, we decided to deal with the ones that most afflicted us and reintegrated our solutions into our central platform, building a single extremely complete and cohesive platform.

| Problem | Our Solution |
| -------- | -------- |
| Possible collision between drones (lack of safety especially for large Swarms)  | Collision Avoidance System  |
| Giant clutter to keep track of all Clovers in a swarm individually (several terminals, many simulateneo computers with several people to keep track of)  | Swarm Station  |
| Lack of basic features for handling a swarm pre-implemented in the Clover platform (such as access to battery data and raspberry computational power)  | Full integration of low level data in our Swarm Station  |
| Lack of security in indoor tests regarding the limitation of physical space (walls and objects) in the Swarm region  | Safe Area Pop Up in Swarm Station  |
| Decentralization of information and platforms for access  | Web Homepage  |
| Difficulty configuring physical drones for swarm  | Our complete documentation with pre-designed settings for swarms in our repo image  |
| Lack of a center for reports of successful tests with swarms of drones for the Clover platform describing the test conditions (odometry, etc.)  | Show off section in our Gitbook  |

And many other solutions are also featured on our platform, for more information please check the solutions described clearly and in detail throughout our **Gitbook**. We recommend reading in order to understand the fundamental precepts of our platform. 

> 📖 **Acess our [Gitbook](https://app.gitbook.com/s/C9O11TiXK1JPnlrpilLg/background-theory/system)!**

## Getting started [![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#swarm_intro)

---

Our platform was made to be extremely intuitive and easy to use. To start (after completing the installation we suggested in our gitbook), you can run the command: 

    roslaunch swarm_in_blocks simulation.launch num:=2 

After that, you can open your browser and access our homepage by typing `localhost` in the search bar.


## Usage - modes [![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#swarm_intro)

---

The Swarm in Blocks can be programmed either with the blocks interface or directly in Python and we developed three main launch modes, each one focused on a different application of the project, they are:


- *Planning Mode:* Its main goal is to allow the user to check the drones' layout, save and load formations, before starting the simulator or using real clovers. In order to need less computational power and avoid possible errors during the simulation.
- *Simulation Mode:* In this mode happens the simulation indeed, starting the Gazebo, the necessary ROS nodes and some other tools. It allows applying the developed features, which will be explained ahead and see how they would behave in real life.
- *Navigation Mode:* The last mode will support executing everything developed in real clovers so that it's possible to control a swarm with block programming. The biggest obstacle yet is the practical testing of this mode, due to the financial difficulty of acquiring a Clover swarm.


## New Swarm Features [![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#swarm_intro)

---

With our vision of solving the problems that most plague the deployment of a real swarm, we have developed several features (and even integrated platforms), below we will list our main developments:


### Swarm Station

---

The main feature from our platform is the *Swarm Station*, which is a **3d Web Visualizer** that shows in real time all the necessary information regarding the drones state, such as real time positioning and visualization, which clover is connected, the topics available and a lot more. Also, you can define a safe area to ensure each drones safety, forcing them to land in case they cross the forbidden area. The front end runs completely on the web browser, saving processing and installation resources. It also comes with a web terminal, allowing the user to open several instances of a terminal emulation in just one click.

<p align="center">
    <img width="700" src="assets/swarm_station/vid01.gif"/>
</p>

This package uses the ROS suite `rosbridge_server` to establish a communication between the ROS environment and the web server. 

To run it, we recommend using **Firefox** browser to assure stability. But feel free to test it on other navigators. 

If you launched our `simulation.launch` from the `swarm_in_blocks` package, then you just have to run

    roslaunch swarm_station swarm_station.launch

Otherwise, you have to make sure that the `rosbridge_websocket` is running on port `9090`:
    
    roslanch rosbridge_server rosbridge_websocket.launch port:=9090 
    
For more detailed instructions on how to use each single feature from the Swarm Station, check our [Gitbook page about the station](https://swarm-in-blocks.gitbook.io/swarm-in-blocks/). 

### Swarm Collision Avoidance

---

When many drones move close to each other, collisions are very likely to occur. To avoid this problem, an algorithm was developed to avoid collisions between drones. When analyzing a collision, 3 types of scenario are possible, the case where one clover is stationary and the other in motion, the case where both are in motion and with parallel trajectories, and finally the case where both are in motion and with non-parallel trajectory.

To turn on the collision avoidance, it is necessary to run:

    rosrun swarm_collision_avoidance swarm_collision_avoidance_node.py

<p align="center">
    <img width="600" src="assets/collision.gif" />
</p>

### Rasp Package

---

The Raspberry package was developed to instantiate a node that will be responsible for collecting essential processing, memory and temperature information from the raspberry and send it to the Swarm Station. It's the package that should be put on the `catkin_ws/src/` directory of each Raspberry Pi, because it also contains the `realClover.launch` needed to launch the swarm on real life.

### Swarm FPV

---

This package is a reformulation of one of the CopterHack 2022 implementations, the **Swarm First Person Viewer**. This year, we decided to restart its structure, making it run also completely on the web to integrate with the Swarm Station. It also depends on the `rosbridge_websocket` running on the port `9090` (default).

<p align="center"> 
    <img width="600" src="assets/fpv_2023.gif"/>
</p>

### Real Swarm

---

## Conclusion

---

Engineering and robotics challenges have always been the main driver of Team Athena, from which we seek to impact society through innovation. Last year, during CopterHack 2022, there was no lack of challenges of this type, and in them we grew and exceeded our limits, all to deliver the best possible project: Swarm in Blocks. All the motivation to facilitate a task as complex as the manipulation of swarms of drones, even through block programming, delighted us a lot and we hope that it delights all our users.

With that came the Swarm in Blocks 2.0, which brought with it innovations that optimized the clover's flight control and that could allow for greater emotions in the handling of the drone, in addition to focusing on greater flight safety.
The Swarm in Blocks 2.0 presents new features for this year, such as the Web terminal, First Person View (FPV), Collision Avoidance, Clover UI and Swarm Station.
However, the work will not stop there. Our goal is to further improve our system and next steps include validating Collision Avoidance outside the simulated world and performing performance tests with network communication solutions to optimize Real Swarm.

Finally, we thank the entire COEX team that made CopterHack 2023 possible and all the support given during the competition. We are Team Athena, creator of the Swarm in Blocks platform and we appreciate all your attention!

### The Atena Team 

Atena Team 2023 (Swarm in Blocks 2.0):

- Agnes Bressan de Almeida : [Github](https://github.com/AgnesBressan), [LinkedIn](https://www.linkedin.com/in/agnes-bressan-148615262/)
- Felipe Tommaselli: [Github](https://github.com/Felipe-Tommaselli), [LinkedIn](https://www.linkedin.com/in/felipe-tommaselli-385a9b1a4/)
- Gabriel Ribeiro Rodrigues Dessotti : [Github](), [LinkedIn](https://www.linkedin.com/in/gabriel-ribeiro-rodrigues-dessotti-8884a3216)
- José Carlos Andrade do Nascimento: [Github](https://github.com/joseCarlosAndrade), [LinkedIn](https://www.linkedin.com/in/jos%C3%A9-carlos-andrade-do-nascimento-71186421a)
- Lucas Sales Duarte : [Github](), [LinkedIn]()
- Matheus Della Rocca Martins : [Github](), [LinkedIn](https://www.linkedin.com/in/matheus-martins-9aba09212/)
- Nathan Fernandes Vilas Boas : [Github](), [LinkedIn](https://www.linkedin.com/mwlite/in/nathan-fernandes-vilas-boas-047616262)


In honor of Atena Team 2022:

- Guilherme Soares Silvestre : [Github](https://github.com/guisoares9), [LinkedIn](https://www.linkedin.com/in/guilherme-soares-silvestre-76570118b/)
- Eduardo Morelli Fares: [Github](https://github.com/faresedu), [LinkedIn](https://www.linkedin.com/in/eduardo-fares-a271561a0/)
- Felipe Tommaselli: [Github](https://github.com/Felipe-Tommaselli), [LinkedIn](https://www.linkedin.com/in/felipe-tommaselli-385a9b1a4/)
- João Aires C. F. Marsicano: [Github](https://github.com/Playergeek181), [LinkedIn](https://www.linkedin.com/in/joao-aires-correa-fernandes-marciano-53b426195/)
- José Carlos Andrade do Nascimento: [Github](https://github.com/joseCarlosAndrade), [LinkedIn](https://www.linkedin.com/in/jos%C3%A9-carlos-andrade-do-nascimento-71186421a)
- Rafael Saud C. Ferro: [Github](https://github.com/Rafael-Saud), [LinkedIn](https://www.linkedin.com/in/rafael-saud/)


