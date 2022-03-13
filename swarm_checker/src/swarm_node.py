import rospy
import rosservice
import rosnode
import rosparam
import logging
from multiprocessing import Process
import numpy as np

# Mavros msgs
from mavros_msgs import srv
from mavros_msgs.msg import State

# Clover services
from clover import srv
from std_srvs.srv import Trigger

# Swarm msg 
from swarm_checker.msg import SwarmState

class SingleClover: 
#Create and call all servicers, subscribers and clover topics

    def __init__(self, name, id):
        self.name = name
        self.id = id

        self.connected = None
        self.armed = None
        self.mode = None

        # Configure clover services and topics
        self.configureMavros()

    def stateCb(self, msg_cb):
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
        # Principal vars
        self.all_clovers = 0
        self.connected_clovers = 0
        self.failed_clovers = 0
        self.armed_clovers = 0
        self.offboard_mode_clovers = 0

        self.all_clovers_ids = []
        self.connected_ids = []
        self.failed_ids = []
        self.armed_ids = []
        self.with_led_ids = []

        # Private vars
        self.__nodes_ok = []
        self.__mavros_conn_ok = []
        self.__offboard_ok = []
        self.__armed_ok = []
        self.__led_ok = []

        # List of clover objects
        self.clovers_obj_list = []
        rospy.init_node("swarm_checker")
        self.setPublisher()
    
    def setPublisher(self):
        self.pub = rospy.Publisher("~state", SwarmState, queue_size=10)

    def updateCloversObjects(self):
        for clover_id in self.all_clovers_ids:
            clover_object = SingleClover(f"clover{clover_id}", clover_id)
            self.clovers_obj_list.append(clover_object)

    def checkNumOfClovers(self):
        params_list = rosparam.list_params(ns='')
        clovers_ids_list = []

        # Iterate for each param and get the /clover{id}/clover_id
        for param in params_list:
            if ('clover' in param) and ('clover_id' in param):
                clover_id = rospy.get_param(param)
                clovers_ids_list.append(clover_id)

        #Sort clover ids list for better processing
        clovers_ids_list.sort()

        # Analyse num of clovers
        num_of_clovers = len(clovers_ids_list)

        # Compare with known clovers
        # 1: If num_of_clovers is smaller than known num 
        if num_of_clovers < self.all_clovers:
            mask = np.isin(self.all_clovers_ids, clovers_ids_list)
            clovers_ids_out = np.where(mask==False)[0]
            for clover_id_out in clovers_ids_out:
                rospy.logwarn(f"Detected that clover{clover_id_out} has missed connection. Waiting for reconnection...")
        
        # 2: If num of clovers is greater than known num
        if num_of_clovers > self.all_clovers:
            mask = np.isin(clovers_ids_list, self.all_clovers_ids)
            clovers_ids_new = np.where(mask==False)[0]
            for clover_id_new in clovers_ids_new:
                rospy.loginfo(f"Detected clover{clover_id_new} as a new clover connected.")

        if num_of_clovers != self.all_clovers:
            # Update ids list and num_of_clovers
            self.all_clovers_ids = clovers_ids_list
            self.all_clovers = num_of_clovers
            self.updateCloversObjects()
    
    def checkNodes(self):
        nodes = rosnode.get_node_names()
        nodes_ok = [False]*self.all_clovers
        for clover in self.clovers_obj_list:
            passed = False
            for node in nodes:
                if f"clover{clover.id}" in node:
                    if 'mavros' in node:
                        passed_mavros = True
                        rospy.logerr(f"mavros node missing on clover{clover.id}")
                    if 'simple_offboard' in node:
                        passed_off = True
                        rospy.logerr(f"simple_offboard node missing on clover{clover.id}")
            nodes_ok[clover.id] = passed
        

        for node in nodes:
            if ('clover' in node) and (('mavros' in node) or ('simple_offboard' in node)):
                for clover in self.clovers_obj_list:
                    if f"clover{clover.id}" in node:


        self.__nodes_ok = nodes_ok
    
    def checkServices(self):
        offboard_ok = [False]*self.all_clovers
        led_ok = [False]*self.all_clovers

        for clover in self.clovers_obj_list:
            # Simple offboard services check
            res_off = clover.checkSimpleOffboardServices()
            offboard_ok[clover.id] = res_off
            
            # Led services check
            res_led = clover.checkLedServices()
            led_ok[clover.id] = res_led
        
        self.__led_ok = led_ok

    def checkStatusOfClovers(self):
        mavros_conn_ok = [False]*self.all_clovers
        offboard_mode_ok = [False]*self.all_clovers
        armed_ok = [False]*self.all_clovers
        
        for clover in self.clovers_obj_list:
            connected = clover.connected
            mode = clover.mode
            armed = clover.armed

            mavros_conn_ok[clover.id] = connected
            armed_ok[clover.id] = armed
            if (not mode is None) and mode.lower() == 'offboard':
                offboard_mode_ok[clover.id] = True
            else:
                offboard_mode_ok[clover.id] = False
        
        self.__mavros_conn_ok = mavros_conn_ok
        self.__offboard_ok = offboard_mode_ok
        self.__armed_ok = armed_ok
            
    def publishKnownClovers(self):
        # self.all_clovers_ids = np.where(self.__nodes_ok == True)
        self.connected_ids = np.argwhere(self.__mavros_conn_ok == True).ravel()
        self.failed_ids = np.argwhere((self.__mavros_conn_ok == False) or (self.__nodes_ok == False)).ravel()
        self.armed_ids = np.argwhere(self.__armed_ok == True).ravel()
        
        # self.all_clovers = len(self.all_clovers_ids)
        self.connected_clovers = len(self.connected_ids)
        self.failed_clovers = len(self.failed_ids)
        self.armed_clovers = len(self.armed_ids)

        msg = SwarmState()
        
        msg.all_clovers = self.all_clovers
        msg.connected_clovers = self.connected_clovers
        msg.failed_clovers = self.failed_clovers
        msg.armed_clovers = self.armed_clovers

        msg.all_clovers_ids = self.all_clovers_ids
        msg.connected_ids = self.connected_ids
        msg.failed_ids = self.failed_ids
        msg.armed_ids = self.armed_ids

        self.pub.publish(msg)

    def checkKnownCloversLoop(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.checkNumOfClovers()
            self.checkNodes()
            self.checkStatusOfClovers()
            self.checkServices()
            self.publishKnownClovers()
            rate.sleep()

    def checkNewCloversLoop(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.checkNumOfClovers()
            self.updateCloversObjects()
            rate.sleep()

if __name__ == '__main__':
    swarm_checker = SwarmChecker()
    swarm_checker.checkKnownCloversLoop()
