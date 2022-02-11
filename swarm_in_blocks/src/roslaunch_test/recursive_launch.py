from click import launch
import roslaunch 
import rospy
import time
import os

numero_drones = 4


'''
path relativo para o launch
'''
fileMain = '/swarm_main.launch'
fileRec = '/swarm_recursive.launch'
pathMain = os.getcwd().replace('/src/roslaunch_test', '') + '/launch' + fileMain
pathRec = os.getcwd().replace('/src/roslaunch_test', '') + '/launch' + fileRec


listM = []
listArg = []


# Initializing node for handling launch files
rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)



# sem in-line arguments
def no_arg(path):

    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
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

def with_arg(launch, num):
    
    cli_args = [launch,"num:="+ str(num)]
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



# Launching swarm_main (spawning gazebo, etc)

cli_main = ['swarm_in_blocks', 'swarm_main.launch', '']
roslaunch_main = roslaunch.rlutil.resolve_launch_arguments(cli_main)
listM.append(roslaunch_main)

roslaunch_args = cli_main[2:]
listArg.append(roslaunch_args)
#launch_files = [(roslaunch_main, arg)]




# ============ SPAWNING DRONES ================


# genereting launch strings


def recursive(file, id):
    cli_args = ['swarm_in_blocks', file, 'num:=' + str(id), 'ID:=' + str(id)]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    print(roslaunch_file)
    listM.append(roslaunch_file)

    roslaunch_args = cli_args[2:]
    print(roslaunch_args)
    listArg.append(roslaunch_args)

    
    #final = [roslaunch_file, roslaunch_args]

    # returns the final string for each launch
    #return final



# defining launch files

# for i in range(numero_drones):
    
#     #launch_files = launch_files + recursive('swarm_recursive.launch', i)
#     recursive('swarm_recursive.launch', i)
# #print(launch_files)

# cli_args = ['swarm_in_blocks', 'swarm_recursive.launch', 'num:=' + str(id), 'ID:=' + str(id)]
# roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
#roslaunch_args = cli_args[2:]

launch_files = [(roslaunch.rlutil.resolve_launch_arguments(['swarm_in_blocks', 'swarm_recursive.launch', 'num:=' + str(id), 'ID:=' + str(id)]), ['num:=' + str(id), 'ID:=' + str(id)]) for id in range(numero_drones)]
#print(launch_files)
#launch_files = list(zip(listM, listArg))
#print(launch_files)

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()
try:
    parent.spin()
finally:
    # After Ctrl+C, stop all nodes from running
    parent.shutdown()
    # 3 seconds later