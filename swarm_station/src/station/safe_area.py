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

        # rospy.wait_for_service(f"{self.name}/get_telemetry", timeout=1)
        # self.get_telemetry = rospy.ServiceProxy(
        #     f"{self.name}/get_telemetry", srv.GetTelemetry)

        # rospy.wait_for_service(f"{self.name}/navigate", timeout=1)
        # self.navigate = rospy.ServiceProxy(
        #     f"{self.name}/navigate", srv.Navigate)

        # rospy.wait_for_service(f"{self.name}/navigate_global", timeout=1)
        # self.navigate_global = rospy.ServiceProxy(
        #     f"{self.name}/navigate_global", srv.NavigateGlobal)

        # rospy.wait_for_service(f"{self.name}/set_position", timeout=1)
        # self.set_position = rospy.ServiceProxy(
        #     f"{self.name}/set_position", srv.SetPosition)

        # rospy.wait_for_service(f"{self.name}/set_velocity", timeout=1)
        # self.set_velocity = rospy.ServiceProxy(
        #     f"{self.name}/set_velocity", srv.SetVelocity)

        # rospy.wait_for_service(f"{self.name}/set_attitude", timeout=1)
        # self.set_attitude = rospy.ServiceProxy(
        #     f"{self.name}/set_attitude", srv.SetAttitude)

        # rospy.wait_for_service(f"{self.name}/set_rates", timeout=1)
        # self.set_rates = rospy.ServiceProxy(
        #     f"{self.name}/set_rates", srv.SetRates)

        rospy.wait_for_service(f"{self.name}/land", timeout=1)
        self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)

        # rospy.wait_for_service(f"{self.name}/led/set_effect", timeout=1)
        # self.set_effect = rospy.ServiceProxy(
        #     f"{self.name}/led/set_effect", srv.SetLEDEffect)

    def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):

        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed,
                      frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(
                frame_id='navigate_target' + str(self.id))
            if np.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)


# string command
# geometry_msgs/Point[] points
# float32 length
# float32 radius


class SafeZone:
    def __init__(self):
        # id list
        self.lista_id = []
        
        # drone list
        self.swarm = []
        self.landed_drones = []
        self.all_of_clovers = 0
        
        # state of safe zone
        self.isSafeZoneActive = False
        
        # Initial formation
        self.init_formation_coords = []

        self.available_commands = "rectangle;circle"

        # message template
        self.safe_zone_data = {
            'command' : 'null',
            'points' : [Point(), Point()],
            'length' : 0,
            'radius' : 0
        }

        self.safeTypes = {
            "rectangle" : self.rectangle_check,
            "circle" : self.circle_check,
            "null" : print('empty')
        }

    def handleOutOfBoundaryDrone(self, clover):
        
        if (clover.id in self.landed_drones):
            return

        rospy.logwarn(f"WARNING - Cover {clover.id} out of boundary\nLanding.. ")
        self.landed_drones.append(clover.id)
        thr = Thread(target=clover.land)
        thr.start()
        thr.join()

        # clover.land() 

    def rectangle_check(self):
        # for
        # print('rectangle check')
        for clover in self.swarm:
            # print(clover.pose, self.safe_zone_data['points'][0])
            # rospy.loginfo(f"{self.safe_zone_data['points'][1].x} ")
            if clover.pose[0] < self.safe_zone_data['points'][0].x or clover.pose[0] > self.safe_zone_data['points'][1].x:
                self.handleOutOfBoundaryDrone(clover)
            if clover.pose[1] < self.safe_zone_data['points'][0].y or clover.pose[1] > self.safe_zone_data['points'][1].y:
                self.handleOutOfBoundaryDrone(clover)
            if clover.pose[2] > self.safe_zone_data['points'][0].z:
                self.handleOutOfBoundaryDrone(clover)

    def circle_check(self):
        pass

    def setSubscribers(self):
        rospy.Subscriber("/marker_state", SwarmStationCommands, callback=self.setSafeZoneModeCallback)

    def setSafeZoneModeCallback(self, data):
        if data.command in self.available_commands: # to do -  add more cases
            self.isSafeZoneActive = True

            # setting template to initialize safe zone backend
            for count, points in enumerate(data.points):
                self.safe_zone_data['points'][count] = points
                # self.safe_zone_data['poits'][count] = points


                # self.safe_zone_data['points'][0] = points[0]
                # self.safe_zone_data['points'][1] = points[1]
                # self.safe_zone_data['points'][2] = points[2]

            self.safe_zone_data['command'] = data.command
            self.safe_zone_data['length'] = data.length
            self.safe_zone_data['radius'] = data.radius

            rospy.loginfo(f"Turning safe zone active with data: \n{self.safe_zone_data}")

      
    def setListId(self):
        # set id list from SwarmState
        rospy.loginfo('Getting all connected clovers..')
        while True:
            # try:
            message = rospy.wait_for_message("/swarm_checker/state", SwarmState, timeout=5)
            self.lista_id = message.connected_ids
            self.all_of_clovers = len(self.lista_id)
            rospy.loginfo(f'Connected clovers are: {self.lista_id}')

            self.getInitialFormation()

            for idx, id in enumerate(self.lista_id):
                clover = SingleClover(f"clover{id}", id)
                clover.init_coord = self.init_formation_coords[idx]
                self.swarm.append(clover)
                
                print(self.swarm)
            break

            # except Exception as err:
            #     rospy.logerr(f"Could not get connected clovers, trying again - {err}")
            # self.lista_id = [0, 1, 2]

    def getInitialFormation(self):
        coords = np.zeros((self.all_of_clovers, 4))
        coords[:] = np.NaN

        for idx, clover_id in enumerate(self.lista_id):
            try:
                x = rospy.get_param(f"/clover{clover_id}/initial_pose/x")
                y = rospy.get_param(f"/clover{clover_id}/initial_pose/y")
                z = rospy.get_param(f"/clover{clover_id}/initial_pose/z")
                coords[idx] = [x, y, z, 1]
            except Exception:
                print(traceback.format_exc())
                continue
        
        # See if there is a clover that wasnt able to retrieve it initial pose
        initial_pose_failed = np.argwhere(np.isnan(coords))
        initial_pose_failed = initial_pose_failed[:,0]
        initial_pose_failed = np.unique(initial_pose_failed)

        self.init_formation_coords = coords

    def checkPosition(self):
        if self.isSafeZoneActive:
            
            pass
        pass

def main():
    rospy.init_node('safe_area')
    safe_zone = SafeZone()

    safe_zone.setSubscribers()
    safe_zone.setListId()

    while not rospy.is_shutdown():

        # safe_zone.checkPosition() 
        if safe_zone.isSafeZoneActive and safe_zone.safe_zone_data['command'] != '':
            # print(safe_zone.safeTypes['rectangle'])
            safe_zone.safeTypes[safe_zone.safe_zone_data['command']]()
        
        rospy.Rate(100).sleep()

if __name__ == "__main__":
    main()
    rospy.spin()