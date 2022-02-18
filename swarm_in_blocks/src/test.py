import sys
import subprocess
import os
from multiprocessing import Process

def test():
    os.system('roslaunch swarm_in_blocks swarm.launch 1>/dev/null')

# subprocess.run('roslaunch swarm_in_blocks swarm.launch', stdout = open('/dev/null', 'w'))

# Process(target=test).start()

subprocess.Popen(['roslaunch','swarm_in_blocks','swarm.launch'], stdout=open('/dev/null', 'w'))

while True:
    print("aaaa")
    