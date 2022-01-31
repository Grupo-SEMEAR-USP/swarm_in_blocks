import roslaunch 
import rospy
import time

numero_drones = 11

# INICIANDO NODES
# package = "swarm_in_blocks"
# executable = "takeoff.py"
# node = roslaunch.core.Node(package, executable)

# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()

# process = launch.launch(node)
# print(process.is_alive())
# time.sleep(5)

# process.stop()

# INCIANDO SERVIÇOS


# sem in-line arguments
def no_arg():
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jose_carlosa13/catkin_ws/src/swarm_in_blocks/swarm_in_blocks/launch/swarm_main.launch"])
    launch.start()
    rospy.loginfo("started")

    rospy.sleep(3)
    try:
        launch.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        launch.shutdown()
        # 3 seconds later


# com in-line arguments
def with_arg(num):
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = ["/home/jose_carlosa13/catkin_ws/src/swarm_in_blocks/swarm_in_blocks/launch/swarm_main.launch","num:="+ str(num)]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    parent.start()
    rospy.loginfo("started")

    rospy.sleep(3)
    try:
        parent.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        parent.shutdown()
        # 3 seconds later

#launch.shutdown()

with_arg(numero_drones)