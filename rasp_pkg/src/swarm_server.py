#!/usr/bin/python3
'''
This node has to run on the master machine, with the proper host address and port number.
'''

import rospy
import socket
import signal

number = 2

class SwarmServer:
    def __init__(self, host = "192.168.0.207", port = 5000, id=0):
        print(f"Starting server at {host} {port}")
        self.server_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        self.server_socket.bind((host, port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        self.server_socket.listen(10)
        self.conn = []
        for i in range(number):
            conn, address = self.server_socket.accept()  # accept new connection
            self.conn.append(conn)
            print("Connection from: " + str(address))

        # signal handling
        signal.signal(signal.SIGINT, self.__signal_handler)
        signal.signal(signal.SIGTSTP, self.__exit_handler)

    def __signal_handler(self, signum, frame):
        rospy.logwarn("Existing by ^C, closing connection. Consider doing this by typing exit")
        self.close_connection()

    def __exit_handler(self, signum, frame):
        rospy.logwarn("Existing by ^Z, closing connection. Consider doing this by typing exit")
        self.close_connection()

    def send_message(self, message):
        for conn in self.conn:
            conn.send(message.encode())

    def server_program(self):
        rospy.loginfo("Type exit to close server: ")
        while True:
        
            data = input(' -> ')
            if data == 'exit':
                break

            self.send_message(data)
        self.close_connection()  # close the connection

    
    def close_connection(self):
        self.server_socket.close()

        for conn in self.conn:
            conn.close()


if __name__ == '__main__':
    rospy.init_node('swarm_server')
    swarm_server = SwarmServer()
    swarm_server.server_program()

    swarm_server.close_connection()
