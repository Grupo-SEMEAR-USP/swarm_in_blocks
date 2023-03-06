#!/usr/bin/python3

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from swarm_checker.msg import SwarmState
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
from swarm_visualizer.msg import SwarmStationCommands


import tf
import sys

import rospy
import tf2_ros

from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from swarm_checker.msg import SwarmState

import numpy as np
from threading import Thread
import traceback
import logging


class SingleClover:
    # Create and call all servicers, subscribers and clover topics
    def __init__(self, name, clover_id):
        self.name = name
        self.id = clover_id
        self.init_coord = np.array([])
        self.pose = np.array([])

        # Configure clover services and topics
        self.configure()

    def __localPositionCallback(self, local_position):
        
        if self.init_coord.size != 0:
            x = local_position.pose.position.x + self.init_coord[0]
            y = local_position.pose.position.y + self.init_coord[1]
            z = local_position.pose.position.z + self.init_coord[2]
            self.pose = np.array([x, y, z])

    def configure(self):

        logging.debug("Waiting clover services...")
        rospy.loginfo("Waiting clover services...")

        rospy.wait_for_message(f"{self.name}/mavros/local_position/pose", PoseStamped, timeout=1)
        rospy.Subscriber(f"{self.name}/mavros/local_position/pose", PoseStamped, self.__localPositionCallback)

        rospy.wait_for_service(f"{self.name}/get_telemetry", timeout=1)
        self.get_telemetry = rospy.ServiceProxy(
            f"{self.name}/get_telemetry", srv.GetTelemetry)

        rospy.wait_for_service(f"{self.name}/navigate", timeout=1)
        self.navigate = rospy.ServiceProxy(
            f"{self.name}/navigate", srv.Navigate)

        rospy.wait_for_service(f"{self.name}/navigate_global", timeout=1)
        self.navigate_global = rospy.ServiceProxy(
            f"{self.name}/navigate_global", srv.NavigateGlobal)

        rospy.wait_for_service(f"{self.name}/set_position", timeout=1)
        self.set_position = rospy.ServiceProxy(
            f"{self.name}/set_position", srv.SetPosition)

        rospy.wait_for_service(f"{self.name}/set_velocity", timeout=1)
        self.set_velocity = rospy.ServiceProxy(
            f"{self.name}/set_velocity", srv.SetVelocity)

        rospy.wait_for_service(f"{self.name}/set_attitude", timeout=1)
        self.set_attitude = rospy.ServiceProxy(
            f"{self.name}/set_attitude", srv.SetAttitude)

        rospy.wait_for_service(f"{self.name}/set_rates", timeout=1)
        self.set_rates = rospy.ServiceProxy(
            f"{self.name}/set_rates", srv.SetRates)

        rospy.wait_for_service(f"{self.name}/land", timeout=1)
        self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)

        rospy.wait_for_service(f"{self.name}/led/set_effect", timeout=1)
        self.set_effect = rospy.ServiceProxy(
            f"{self.name}/led/set_effect", srv.SetLEDEffect)

    def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):

        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed,
                      frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(
                frame_id='navigate_target' + str(self.id))
            if np.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)





class SafeZone:
    def __init__(self):
        self.lista_id = []
        self.isSafeZoneActive = False

    def setListId(self):
        # set id list from SwarmState
        rospy.loginfo('Getting all connected clovers..')
        while True:
            try:
                message = rospy.wait_for_message("/swarm_checker/state", SwarmState, timeout=5)
                self.lista_id = message.connected_ids
                rospy.loginfo(f'Connected clovers are: {self.lista_id}')
                break

            except Exception as err:
                rospy.logerr("Could not get connected clovers, trying again")
            # self.lista_id = [0, 1, 2]

    def checkPosition(self):
        pass

def main():
    safe_zone = SafeZone()
    
    while not rospy.is_shutdown():

        safe_zone.checkPosition() 

if __name__ == "__main__":
    main()