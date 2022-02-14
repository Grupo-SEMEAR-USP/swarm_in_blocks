#!/usr/bin/python3


import roslaunch
import rospy

def launchGazebo(uuid):
    cli_args = ["swarm_in_blocks", "gazebo.launch"]
    path = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    parent = roslaunch.parent.ROSLaunchParent(uuid, [path])
    parent.start()
    rospy.loginfo("started")

    rospy.sleep(1)

    return parent

def launchSingleVehicle(uuid, id, x=0, y=0, z=0.3, roll=0, pitch=0, yaw=0):
    
    cli_args = ["swarm_in_blocks", "single_vehicle.launch",
                "ID:=" + str(id),
                "x:=" + str(x),
                "y:=" + str(y),
                "z:=" + str(z),
                "R:=" + str(roll),
                "P:=" + str(pitch),
                "Y:=" + str(yaw)]

    roslaunch_args = cli_args[2:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    parent.start()
    rospy.loginfo("started")
    return parent

def spawnGazeboAndVehicles(num_of_clovers):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    parents = []
    gazebo = launchGazebo(uuid)
    parents.append(gazebo)
    
    side_x = 1
    # side_y = 5
    x = 0
    y = 0
    for i in range(num_of_clovers):
        if i >= 10:
            i = i+1
        if x >= side_x:
            x = 0
            y += 1

        parent = launchSingleVehicle(uuid, i, x=x, y=y)
        parents.append(parent)
        x+=1
    
    rospy.sleep(1)
    try:
        for parent in parents:
            parent.spin()
        
    finally:
        # After Ctrl+C, stop all nodes from running
        rospy.loginfo("Killing vehicles launches")
        for parent in parents:
            parent.shutdown()



if __name__ == '__main__':

    num_of_clovers = 6
    spawnGazeboAndVehicles(num_of_clovers)

    

    

