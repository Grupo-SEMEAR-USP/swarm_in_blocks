#!/usr/bin/python3
'''
This node has to run on the master machine, with the proper host address and port number.
'''

import rospy
import socket
import signal
from std_msgs.msg import String
import threading

number = 2

class SwarmServer:
    def __init__(self, host = "192.168.0.207", port = 5000, id=0):
        print(f"Starting server at {host} {port}")
        self.server_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        self.server_socket.bind((host, port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        self.close_clients = False
        self.server_socket.listen(20)
        self.conn = []

        self.__thread = threading.Thread(target= self.__accept_clients, daemon=True)
        self.__thread.start()

        # ros callback
        rospy.Subscriber("swarm_control", String, self.__send_ros_message)

        # signal handling
        signal.signal(signal.SIGINT, self.__signal_handler)
        signal.signal(signal.SIGTSTP, self.__exit_handler)

    
    def __accept_clients(self):
        while not self.close_clients:
            conn, address = self.server_socket.accept()  # accept new connection
            self.conn.append(conn)
            rospy.loginfo(f"Connection from: {address}")

    def __signal_handler(self, signum, frame):
        rospy.logwarn("Exiting by ^C, closing connection. Consider doing this by typing exit")
        self.close_connection()

    def __exit_handler(self, signum, frame):
        rospy.logwarn("Exiting by ^Z, closing connection. Consider doing this by typing exit")
        self.close_connection()

    def __send_message(self, message):
        try:
            for conn in self.conn:
                conn.send(message.encode())

        except Exception as err:
            rospy.logerr(f"Could not send message: {err}")


    def __send_ros_message(self, message):
        try:
            for conn  in self.conn:
                rospy.sleep(0.1)
                conn.send(message.data.encode())
        except Exception as err:
            rospy.logerr(f"Could not send socket message: {err}")

    def server_program(self):
        rospy.loginfo("Type exit to close server: ")
        while True:
        
            data = input(' -> ')
            if data == 'exit':
                break

            self.__send_message(data)
        self.close_connection()  # close the connection

    
    def close_connection(self):
        self.close_clients = True
        self.server_socket.close()

        for conn in self.conn:
            conn.close()


if __name__ == '__main__':
    rospy.init_node('swarm_server')
    swarm_server = SwarmServer()
    swarm_server.server_program()

    swarm_server.close_connection()
