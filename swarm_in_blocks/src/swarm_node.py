import rospy
import rosservice
import rosnode
import logging

import numpy as np

# Mavros msgs
from mavros_msgs import srv
from mavros_msgs.msg import State

# Clover services
from clover import srv
from std_srvs.srv import Trigger

# Swarm msg 
from swarm_in_blocks.msg import SwarmState

class SingleClover: 
#Create and call all servicers, subscribers and clover topics

    def __init__(self, name, id):
        self.name = name
        self.id = id
        self.current_state = State()

        # Configure clover services and topics
        self.configureMavros()

    def stateCb(self, msg_cb):
        self.current_state = msg_cb
        self.connected = msg_cb.connected
        self.armed = msg_cb.armed
        self.mode = msg_cb.mode

    def configureMavros(self):
        self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)

    def checkSimpleOffboardServices(self):
        try:
            rospy.wait_for_service(f"{self.name}/get_telemetry" , timeout=1)
            rospy.wait_for_service(f"{self.name}/navigate", timeout=1)
            rospy.wait_for_service(f"{self.name}/navigate_global", timeout=1)
            rospy.wait_for_service(f"{self.name}/set_position", timeout=1)
            rospy.wait_for_service(f"{self.name}/set_velocity", timeout=1)
            rospy.wait_for_service(f"{self.name}/set_attitude", timeout=1)
            rospy.wait_for_service(f"{self.name}/set_rates", timeout=1)
            rospy.wait_for_service(f"{self.name}/land", timeout=1)
        except:
            rospy.logerr(f"simple_offboard services missing for clover{self.id}.")
            return False
        
        return True
    
    def checkLedServices(self):
        try:
            rospy.wait_for_service(f"{self.name}/led/set_leds", timeout=1)
            rospy.wait_for_service(f"{self.name}/led/set_effect", timeout=1)
        except:
            rospy.logwarn_once(f"led services disabled for clover{self.id}")
            return False
        
        return True

class SwarmChecker:
    def __init__(self):
        
        self.connected_clovers = 0
        self.failed_clovers = 0
        self.armed_clovers = 0
        self.offboard_mode_clovers = 0

        self.all_clovers_ids = []
        self.ids_connected = []
        self.ids_armed = []
        self.ids_with_led = []

        # Private vars
        self.__nodes_ok = []
        self.__mavros_conn_ok = []
        self.__offboard_ok = []
        self.__armed_ok = []
        self.__led_ok = []
        rospy.init_node("swarm_checker")

    def createCloversObjects(self):
        for index in range(self.num_of_clovers):
            clover_object = SingleClover(f"clover{index}", index)
            clover_object.init_coord = self.init_formation_coords[index]
            self.clovers_list.append(clover_object)

    def getNodes(self):
        self.node_list = rosnode.get_node_names()

    def checkNumOfClovers(self):
        num_of_clovers = 0
        for node in self.node_list:
            if ('clover' in node) and ('mavros' in node):
                num_of_clovers += 1
        
        self.num_of_clovers = num_of_clovers
    
    def checkNodes(self):
        nodes_ok = []
        for clover_id in range(self.num_of_clovers):
            passed = True
            for node in self.nodes:
                if f"clover{clover_id}" in node:
                    if not ('mavros' in node):
                        passed = False
                        rospy.logerr(f"mavros node missing on clover{clover_id}")
                    if not ('simple_offboard' in node):
                        passed = False
                        rospy.logerr(f"simple_offboard node missing on clover{clover_id}")
            nodes_ok.append(passed)
        self.__nodes_ok = nodes_ok
    
    def checkServices(self):
        offboard_ok = []
        led_ok = []
        for clover in self.clovers_list:
            # Simple offboard services check
            res_off = clover.checkSimpleOffboardServices()
            offboard_ok.append(res_off)
            
            # Led services check
            res_led = clover.checkLedServices()
            led_ok.append(res_led)
        
        self.__led_ok = led_ok

    def checkStatusOfClovers(self):
        mavros_conn_ok = []
        offboard_mode_ok = []
        armed_ok = []
        
        for clover in self.clovers_list:
            connected = clover.connected
            mode = clover.mode
            armed = clover.armed

            mavros_conn_ok.append(connected)
            armed_ok.append(armed)
            if mode.lower() == 'offboard':
                offboard_mode_ok.append(True)
            else:
                offboard_mode_ok.append(False)
        
        self.__mavros_conn_ok = mavros_conn_ok
        self.__offboard_ok = offboard_mode_ok
        self.__armed_ok = armed_ok
    
    def checkLoop(self):

        rospy.loginfo("swarm_checker is waiting for clovers...")
        while not rospy.is_shutdown():
            self.getNodes()
            self.checkNumOfClovers()
            if self.num_of_clovers != 0: 
                rospy.loginfo(f"{self.num_of_clovers} clovers detected.")

                rospy.loginfo("Analysing clovers nodes...")
                self.checkNodes()
                
                rospy.loginfo("Analysing clovers services...")
                self.createCloversObjects()
                self.checkServices()
            
    def publishKNownClovers(self):

        self.clovers_list = np.argwhere(self.__nodes_ok == True)
        self.
    def checkKnownCloversLoop(self):

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            self.checkStatusOfClovers()
            self.checkServices()
            self.publishKnowClovers

