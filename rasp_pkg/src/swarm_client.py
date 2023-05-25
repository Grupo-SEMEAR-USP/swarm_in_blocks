#!/usr/bin/python3
'''
The user should call a CloverInstance for each clover, then be able to give formations instructions. 
'''

import rospy
import socket
from std_msgs.msg import String


class SwarmClient:
    '''
    Class that is initialized only once each formation script
    '''
    def __init__(self, host = "192.168.0.207", port = 5000, id=0):
        self.id = id

        print(f"Starting server at {host} {port}")
        

        self.client_socket = socket.socket()  # instantiate
        self.client_socket.connect((host, port))  # connect to the server

    def send_message(self, message):
        try:
            self.client_socket.send(message.encode())
        except Exception as err:
            rospy.logerr(f"Could not send message to server, {err}")
        
    # def close_connection(self):
    #     self.client_socket.close()


class CloverInstance:
    '''
    Clover instance that holds all the message functions for swarm controlling
    '''
    client = SwarmClient()

    def __init__(self, id):
        self.id = id
        # self.client = SwarmClient()
        self.swarm_publisher = rospy.Publisher("swarm_control", String, queue_size=10)

    def __send_message(self, message):
        # CloverInstance.client.send_message(message=message)
        self.swarm_publisher.publish(message)

    def led_effect(self, effect, r=0, g=0, b=0):
        try:
            message = f'{self.id} led_effect {effect} {r} {g} {b}'
            self.__send_message(message=message)
        except Exception as err:
            rospy.logerr(f"Could not send led_effect for clover {id}, {err}")

    def navigate(self, x, y, z, vel = 0.5, frame_id = 'body', auto_arm = True):
        try:
            print("sending navigate message")
            message = f'{self.id} navigate {x} {y} {z} {vel} {frame_id} {auto_arm}'
            self.__send_message(message=message)
        
        except Exception as err:
            rospy.logerr(f" Could not send navigate command for clover {id}, {err}")

    def land(self):
        try:
            message = f'{self.id} land'
            self.__send_message(message=message)
        except Exception as err:
            rospy.logerr(f"Could not send land for clover {id}, {err}")

    def landAll(self, clover_list):
        for clover in clover_list:
            clover.land()
        

    # def close_connection(self):
    #     CloverInstance.client.close()




if __name__ == '__main__':
    rospy.init_node('swarm_client')
    swarm_server = SwarmClient()
    # swarm_server.server_program()
