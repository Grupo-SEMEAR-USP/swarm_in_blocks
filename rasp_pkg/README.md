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

## How it works

It contains two important parts:

## Raspberry Resources Publisher Node 
It's a python node that keeps collecting information about the Raspberry Pi resources such as **processing states, memory usage, temperature and more**. It publishes this information on the `/cloverID/cpu_usage` topic, so the `swarm_station` backend is able to retrieve all this data with the `raspData.msg` custom ROS message.

## Real Swarm Launch 
That's the ROS launch file that initializes each drone on its own namespace. It's important to note that it's built from the **0.23v clovers image version**, and **it has not been implemented yet to the new 0.24v.**

## Organization

Overall it's a simple package that it's organized like following:

| Package | Description |
| ------- | -------- |
| `launch`  | Contains the 2 mentioned launch files that: 1 - Initializes the Raspberry PI node; 2 - Initializes each drones launch inside its namespaces |
| `msg`  |  Defines the `raspData.msg` and also the `processInfo.msg`, although the last one has not been used yet. These are custom messages made to send resources data to the Swarm Station Backed  |
| `src`  |  That's the source directory that contains the script files used to run the Raspberry Pi node |
