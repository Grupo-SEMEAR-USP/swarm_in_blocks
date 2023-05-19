#!/usr/bin/python3
'''
This node has to run on the master machine, with the proper host address and port number.
'''

import rospy
import socket

class SwarmServer:
    def __init__(self, host = "192.168.0.207", port = 5000):
        print(f"Starting server at {host} {port}")
        self.server_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        self.server_socket.bind((host, port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        self.server_socket.listen(2)
        self.conn, self.address = self.server_socket.accept()  # accept new connection
        print("Connection from: " + str(self.address))

    def send_message(self, message):
        self.conn.send(message.encode())

    def server_program(self):

        while True:
            # receive data stream. it won't accept data packet greater than 1024 bytes
            data = self.conn.recv(1024).decode()
            if not data:
                # if data is not received break
                break
            print("from connected user: " + str(data))
            data = input(' -> ')
            self.conn.send(data.encode())  # send data to the client

        self.conn.close()  # close the connection




if __name__ == '__main__':
    rospy.init_node('swarm_server')
    swarm_server = SwarmServer()
    # swarm_server.server_program()
