# swarm_clover_blocks

Expansion of the adaptation of Blockly (Google's block programming language) made by the Clover platform, now with support for swarms of drones.

<img src="demo.gif" width=700>


See user documentation of the clover_blocks at the [main Clover documentation site](https://clover.coex.tech/en/blocks.html).
For more info about swarm_clover_blocks, see the [offical documentation swarm in blocks](https://swarm-in-blocks.gitbook.io/swarm-in-blocks/introduction/swarm-in-blocks)

Internal package documentation is given below.

---

## Usability

This packages is completely available in the browser, so you can use it on any device with a browser through localhost. The only requirement is that the device is connected to the same network as the physical drone or with the simulation running. If you configured the Apache web service as our documentation suggests, you can (with the drone or the simulation connected) type "localhost" in the browser and acess our homepage web interface. From there you can acess the blocks programming interface, that in fact it is what this package contains.

---

## How it works


### Organization

The package is organized in two main parts: the backend and the frontend. The backend is a ROS node, which implements all the services and topics needed for running Blockly-generated Python script. The frontend is a web application, which provides a user interface for the backend.

| Package | Description |
| ------- | -------- |
| `msg`  |  Contains the messages that will be sent, basically the user's request and the id relative to the request |
| `programs`  | Saves the pre-ready examples for programming in blocks, already adding the complete branch of blocks to fulfill a certain specific task. They are available through the front-end |
| `src`  | The heart of the backend, in short, launches all the connections the package needs concentrated in a 'clover_blocks.py' file  |
| `srv`  | Specifies the messages the services will use (Load, Run and Store) |
| `www`  |  Heart of the front-end, it contains the pure blockly and our adaptation of the ide, as well as several files of communication with the backend in javascript  |


### Frontend

The frontend files are located in [`www`](./www/) subdirectory. The frontend application uses [`roslib.js`](http://wiki.ros.org/roslibjs) library for communicating with backend node and other ROS resources. Our main developments were focused on this part of the package, so we will explain it in more detail.

The `blockly` folder contains the compressed blockly from google (without modifications). Our work are in the other files, like `blocks.js`, `index.html`, `main.css`, `main.js`, `python.js`, `ros.js` and `roslib.js`. Each has a unique role in the frontend application.

It is worth highlighting that this package no more it is called from the `.ros` folder in the root of the user's home directory, but from the `swarm_clover_blocks` folder in the `src` folder of the user's home directory by the homepage. The `.ros` folder had some disvantages, like the auto-erase and rebuild of the folder each time the user called some roslaunch command. With that in mind, we decided to change the folder to our repo, so that way we can keep full control of what we are doing with him. Now, the backend of the code works exactely the same as the original, but we have more freedom to change the frontend and the backend, when necessary.

### `swarm_clover_blocks` node

`swarm_clover_blocks` is the blocks programming backend, implementing all the services and topics needed for running Blockly-generated Python script. This backend application works exactly the same as the original `clover_blocks` package, but with some modifications to support the swarm of drones.

#### Services

* `~run` ([*clover_blocks/Run*](srv/Run.srv)) – run Blockly-generated program (in Python).
* `~stop` ([*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)) – terminate the running program.
* `~store` ([*clover_blocks/load*](srv/Store.srv)) – store a user program (to `<package_path>/programs` by default).
* `~load` ([*clover_blocks/load*](srv/Load.srv)) – load all the stored programs.

#### Parameters

* `~programs_dir` (*string*) – directory for user programs.

Parameters read by frontend:

* `~navigate_tolerance` (*float*) – distance tolerance in meters, used for navigate-like blocks (default: 0.2).
* `~navigate_global_tolerance` (*float*) – distance tolerance for global coordinates navigation (default: 1).
* `~yaw_tolerance` (*float*) – yaw angle tolerance in degrees, used in set_yaw block (default: 1).
* `~sleep_time` (*float*) – duration of sleep in loop cycles, used for navigate-like blocks (default: 0.2).
* `~confirm_run` (*bool*) – enable confirmation to run the program (default: true).

These parameters also can be set as URL GET-parameters, for example:

```
http://<hostname>/clover_blocks/?navigate_tolerance=0.5&sleep_time=0.1
```

#### Topics Published

* `~running` ([*std_msgs/Bool*](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html)) – indicates if the program is currently running.
* `~block` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – current executing block (maximum topic rate is limited).
* `~error` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – user program errors and exceptions.
* `~prompt` ([*clover_blocks/Prompt*](msg/Prompt.msg)) – user input request (includes random request ID string).

This topic is published from the frontend side:

* `~prompt/<request_id>` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – user input response.
