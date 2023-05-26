# Raspberry package
This package contains all the necessary files that need to go into the raspberry pi of each drone in order to accomplish all the features mentioned. For more detailed instructions, check out our [Gitbook](https://swarm-in-blocks.gitbook.io/).

## Usability

To use this package, you must first transfer it to the Raspberry Pi, which can be done by cloning the package:
 ```bash
  git clone --depth 1 https://github.com/Grupo-SEMEAR-USP/swarm_in_blocks.git swarm_in_blocks/rasp_pkg

```         
If there is no internet connection on the Raspberry Pi, you can transfer the package using the established SSH connection:            
```bash
  scp /path/to/local/folder/my-package pi@<RaspberryPi_IP>:~/my-package
```


The only requirement for this package is that you have installed the psutil library in order to collect information from the Raspberry Pi:
```bash
  pip install psutil
```

## Organization

Overall it's a simple package that it's organized like following:

| Package | Description |
| ------- | -------- |
| `launch`  | Contains the 2 mentioned launch files that: 1 - Initializes the Raspberry PI node; 2 - Initializes each drones launch inside its namespaces |
| `msg`  |  Defines the `raspData.msg` and also the `processInfo.msg`, although the last one has not been used yet. These are custom messages made to send resources data to the Swarm Station Backed  |
| `src`  |  That's the source directory that contains the script files used to run the Raspberry Pi node. It also contains the Swarm and Clover server nodes that make the connection between the master machine and each clover |


## How it works

This package has three essential parts: The server side, which consists of `clover_client.py`, which should run on each Raspberry pi, `swarm_server.py`, which should be running on the master machine (computer) and the `swarm_client.py` file that should be imported on the formation scripts, with the `CloverInstance` class, passing only the id as argument. Check the example scripts on this same folder. It's important to note that the user should put it's **local IP address** and the desired **port** on the `config.txt` file, besides also choosing the ID this clover will have on the network.

For the port, we recommend to leave **5000**, but feel free to change it in case of busy situations. The ID field should be changed on each clover. On the master machine, you can ignore it.

There's also the `realClover.launch`, responsible for implementing the swarm in blocks functionalities using namespaces. To use it, you must put every clover roscore to run on the same address. For more detailed instructions, check our documentation.

Lastly, there's the `rasp_resouce_publisher` node, responsible for extracting and publishing the data collected from the Rasperry Pi. For now, it only works when using the same roscore address for the master and the clovers.

## Server solution

This is the solution we recommend when talking about real drone swarms.

As previously said, you should put the necessary configuration on the `config.txt file`, and then put this package on every single clover that you want to connect to the swarm. After that, you just have to run the `swarm_server` on the master machine and the `clover_client` on every clover.

    rosrun rasp_pkg swarm_server.py # on the master computer 

    rosrun rasp_pkg clover_client.py # on each clover

Then, you just have to import the file `swarm_client.py` on a script that you want to test, and instantiate each clover like `clover0 = CloverInstance(id=0)`. Now, enjoy making the formations and LED effects. Check out our examples scripts.

## Raspberry Resources Publisher Node

It's a python node that keeps collecting information about the Raspberry Pi resources such as **processing states, memory usage, temperature and more**. It publishes this information on the `/cloverID/cpu_usage` topic, so the `swarm_station` backend is able to retrieve all this data with the `raspData.msg` custom ROS message.

## Real Swarm Launch 
That's the ROS launch file that initializes each drone on its own namespace. It's important to note that it's built from the **0.23v clovers image version**, and **it has not been implemented yet to the new 0.24v.** It's worth noting that this solution is sometimes unstable when it comes to network load. We'd definitely recommend using the server solution.
