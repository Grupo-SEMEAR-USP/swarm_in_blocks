#!/usr/bin/python3

'''
This node should run on each clover individually, connecting to the master machine that runs the server.
'''

import socket
import rospy
from mavros_msgs import srv 

from clover import srv
from std_srvs.srv import Trigger

class SwarmClient:
    def __init__(self, host="192.168.0.207", port=5000, id=0):
        self.id = id
        # host = "192.168.0.207"  # as both code is running on same pc
        # port = 5000  # socket server port number

        self.client_socket = socket.socket()  # instantiate
        self.client_socket.connect((host, port))  # connect to the server

        # message = input(" -> ")  # take input

        self.swarm_commands = {
            'navigate' : self.navigate_client,
            'led_effect' : self.led_effect_client,
            'land' : self.land_client
        }

        self.data = ''

    def configure(self):

        # logging.debug("Waiting clover services...")
        rospy.loginfo("Waiting clover services...")

        rospy.wait_for_service(f"navigate", timeout=1)
        self.navigate = rospy.ServiceProxy(f"navigate", srv.Navigate)

        rospy.wait_for_service(f"land", timeout=1)
        self.land = rospy.ServiceProxy(f"land", Trigger)

        rospy.wait_for_service(f"led/set_effect", timeout=1)
        self.set_effect = rospy.ServiceProxy(f"led/set_effect", srv.SetLEDEffect)
    
    def navigate_client(self):
        message = self.data

        rospy.loginfo("Navigate message..")

        data = message.split(' ')
        x = float(data[2])
        y = float(data[3])
        z = float(data[4])
        vel = float(data[5])
        frame_id = data[6]
        arm = True

        self.navigate(x=x, y=y, z=z, speed=vel, frame_id=frame_id, auto_arm=arm)

    def led_effect_client(self):
        rospy.loginfo("Changing led effect..")

        message = self.data

        data = message.split(' ')
        # print(data)
        effect = data[2]
        r = int(data[3])
        g = int(data[4])
        b = int(data[5])

        self.set_effect(effect=effect, r=r, g=g, b=b)
    
    def land_client(self):
        
        rospy.loginfo("Landing..")
        
        self.land()

    def receive(self):
        rospy.loginfo("Waiting for server messages...")
        while True:
            #client_socket.send(message.encode())  # send message
            self.data = self.client_socket.recv(1024).decode()  # receive response
            
            split = self.data.split(' ')
            if int(split[0]) == self.id:
                command = self.data.split(' ')[1]
                print(f"Clover {self.id} received a {command} command!")
                try:
                    self.swarm_commands[command]()
                except Exception as err:
                    print("Command not executed: ", err)
            
                if self.data == 'exit':
                    self.client_socket.close()
                    break
                print('Received from server: ' + self.data)  # show in terminal

                    #message = input(" -> ")  # again take input

        self.client_socket.close()  # close the connection


if __name__ == '__main__':
    rospy.init_node("swarm_client")
    client = SwarmClient()
    client.configure()
    client.receive()


# clover comando 
# clover0 led_effect effect r g b ; (effect -> string fast, blink, etc; r,g,b-> int 0-255)
# clover0 navigate x y z vel frame_id arm ; (x,y,z -> float ou None, caso desconsidere)
# clover0 land ; (sem parametro)
