#!/usr/bin/python3

import os
import subprocess
import rospy

def main():
    # iniciating a ros node
    rospy.init_node('web_terminal')

    while not rospy.is_shutdown():
        try:
            # Get the current username
            USER = os.getlogin()
            # rospy.loginfo(USER)
            subprocess.run(["node", f'/home/{USER}/.ros/www/swarm_clover_blocks/web_terminal/server.js'])
            # subprocess.run(["node", f'/home/{USER}/.ros/www/clover/web_terminal/backend.js'])
            
        except rospy.ServiceException as e:
            print("WebTerminal node failed %s", e)



if __name__ == '__main__':
    # main()
    try:
        main()
    except rospy.ROSInterruptException:
        pass