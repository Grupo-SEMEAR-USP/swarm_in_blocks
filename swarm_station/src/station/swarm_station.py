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
import tf2_ros
import tf
import sys
import rospy
import threading

from clover import srv
from std_srvs.srv import Trigger

import math


class SwarmStation:
    def __init__(self):
        self.vehicle_marker_array = MarkerArray()
        self.safe_marker_array = MarkerArray()
        self.text_marker_array = MarkerArray()
        self.base_marker_array = MarkerArray()

        self.marker_single = Marker()
        self.lista_id = []
        self.initial_pos = []

        self.isSafeZoneActive = False

        self.mesh_path_base = "package://swarm_station/meshes/clover4/clover_body_solid.dae"
        self.mesh_path = "package://swarm_station/meshes/clover4/clover_guards_transparent.dae"

        self.landServices = []
        self.ledServices = []

    def setPublishers(self):
        self.markerPublisher = rospy.Publisher("/vehicle_marker", MarkerArray, queue_size=10)
        self.safeMarkerPublisher = rospy.Publisher("/safe_marker", MarkerArray, queue_size=10)
        self.textMarkerPublisher = rospy.Publisher("/text_marker", MarkerArray, queue_size=10)
        self.baseMarkerPublisher = rospy.Publisher("/base_vehicle_marker", MarkerArray, queue_size=10)


    def setSubscribers(self):
        # rospy.Subscriber("/clover0/mavros/local_position/pose", PoseStamped, callback=self.callback)
        rospy.Subscriber("/marker_state", SwarmStationCommands, callback=self.markerCallback)


    def setServices(self):
        try:
            for id in self.lista_id:
                rospy.wait_for_service(f"/clover{id}/land", timeout=1)
                self.landServices.append(rospy.ServiceProxy(f"/clover{id}/land", Trigger))

                rospy.wait_for_service(f"/clover{id}/led/set_effect", timeout=1)
                self.ledServices.append(rospy.ServiceProxy(f"/clover{id}/led/set_effect", srv.SetLEDEffect))

        except Exception as err:
            rospy.logerr(f'Could not initialize services for swarm station backend. Please consider running it again - LAND ALL NOT AVAILABLE - {err}')


    def markerCallback(self, data):
        rospy.loginfo('callback')
        if data.command == 'reload':
            rospy.loginfo('Republishing markers on reload request..')
            self.markerPublisher.publish(self.vehicle_marker_array)
            self.textMarkerPublisher.publish(self.text_marker_array)
            self.baseMarkerPublisher.publish(self.base_marker_array)

            if self.isSafeZoneActive == True:
                self.safeMarkerPublisher.publish(self.safe_marker_array)
        
        elif data.command == 'rectangle':
            rospy.loginfo('Creating safe zone markers on request')
            self.isSafeZoneActive = True
            # Publihsing safe zone markers based on swarm station request  
            pointA = Point()
            pointB = Point()
            pointC = Point()
            pointD = Point()

            # pointA.x, pointA.y, pointA.z = -6.5, -6.4, 0
            # pointC.x, pointC.y, pointC.z = 7.5, 8.4, 0
            pointA = data.points[0]
            pointC = data.points[1]

            # height = pointA.z
            height = data.length
            pointB.x, pointB.y, pointB.z = pointA.x, pointC.y, 0
            pointD.x, pointD.y, pointD.z = pointC.x, pointA.y, 0

            
            point_list = []
            # for i in range(height):
            i = 0
            while i <= height:
            #     pointA.z, pointB.z, pointC.z, pointD = height, height, height, height
                a = Point()
                b = Point()
                c = Point()
                d = Point()
                a.x,a.y,a.z = pointA.x, pointA.y, i
                b.x,b.y,b.z = pointB.x, pointB.y, i
                c.x,c.y,c.z = pointC.x, pointC.y, i
                d.x,d.y,d.z = pointD.x, pointD.y, i
                point_list.append(a)
                point_list.append(b)
                point_list.append(b)
                point_list.append(c)
                point_list.append(c)
                point_list.append(d)
                point_list.append(d)
                point_list.append(a)

                # point_list.append(pointA)
                # point_list.append(pointB)
                # point_list.append(pointB)
                # point_list.append(pointC)
                # point_list.append(pointC)
                # point_list.append(pointD)
                # point_list.append(pointD)
                # point_list.append(pointA)
                # pointA.z +=1
                i += 0.8
            # print(point_list)
            color = [255, 0, 255]
            
            type = 'line'
            frame = '/map'
            
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0

            scale = [4,4,4]

            try:    
                rospy.loginfo('Creating line marker')
                self.create_marker(frame=frame, type=type, pose=pose, scale=scale, color=color, lifetime=0, action=0, vertices=point_list)
                rospy.loginfo('Marker line created')

                print(self.safe_marker_array)

                self.safeMarkerPublisher.publish(self.safe_marker_array)

            except Exception as err:
                rospy.logerr(err)

        elif data.command == 'circle':
            rospy.loginfo('Creating cirlce safe zone markers on request')
            self.isSafeZoneActive = True
            # Publihsing safe zone markers based on swarm station request  
            pointA = Point()
            # pointB = Point()
            # pointC = Point()
            # pointD = Point()

            # pointA.x, pointA.y, pointA.z = -6.5, -6.4, 0
            # pointC.x, pointC.y, pointC.z = 7.5, 8.4, 0
            pointA = data.points[0]
            radius = data.radius
            # pointC = data.points[1]

            height = data.length
            # height = 8
            # pointB.x, pointB.y, pointB.z = pointA.x, pointC.y, 0
            # pointD.x, pointD.y, pointD.z = pointC.x, pointA.y, 0

            
            point_list = []
            i=0
            ant = Point()
            while i <= height:
                ant.x = pointA.x + radius * math.cos(math.radians(0))
                ant.y = pointA.y + radius * math.sin(math.radians(0))
                ang = 1
                while ang <= 360:
                    a = Point()
                    b = Point()

                    x = pointA.x + radius * math.cos(math.radians(ang))
                    y = pointA.y + radius * math.sin(math.radians(ang))
                    z = i

                    a.x = ant.x
                    a.y = ant.y
                    a.z = z

                    b.x = x
                    b.y = y
                    b.z = z

                    ant = b
                    point_list.append(a)
                    point_list.append(b)
                    ang += 1

                i += 0.5

            color = [255, 0, 255]
            
            type = 'line'
            frame = '/map'
            
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0

            scale = [4,4,4]

            try:    
                rospy.loginfo('Creating circle line marker')
                self.create_marker(frame=frame, type=type, pose=pose, scale=scale, color=color, lifetime=0, action=0, vertices=point_list)
                rospy.loginfo('Circle marker line created')

                print(self.safe_marker_array)

                self.safeMarkerPublisher.publish(self.safe_marker_array)

            except Exception as err:
                rospy.logerr(err)
    
                
        elif data.command == 'land_all':
            threads = list()

            rospy.logwarn('Landing all drones on request..')            
            for service in self.landServices:
                # service()
                thr = threading.Thread(target=service)
                threads.append(thr)
                thr.start()
                
            for index, thread in enumerate(threads):
                thread.join()
            

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
        


    def setInitialPose(self):
        
        for id in self.lista_id:
            init_x = rospy.get_param(f'/clover{id}/initial_pose/x')
            init_y = rospy.get_param(f'/clover{id}/initial_pose/y')

            self.initial_pos.append([init_x, init_y])
        
        rospy.loginfo(self.initial_pos)

    def waitMessage(self):
        for id in self.lista_id:
            try:
                # message = rospy.wait_for_message(f'/clover{id}/mavros/local_position/pose', PoseStamped, timeout=5)

                # cur_pose = message.pose
                # cur_pose.position.x += 1
                # self.vehicle_marker_array.markers[id].pose = message.pose
                # print('aaa')
                # self.vehicle_marker_array.markers[id].pose.position.x += self.initial_pos[id][0]
                # self.vehicle_marker_array.markers[id].pose.position.y += self.initial_pos[id][1]
                self.vehicle_marker_array.markers[id].id = id
                self.vehicle_marker_array.markers[id].header.frame_id = f'/body{id}'

                self.base_marker_array.markers[id].id = id
                self.base_marker_array.markers[id].header.frame_id = f'/body{id}'

                # self.text_marker_array.markers[id].pose = message.pose
                # # print('aaa')
                # self.text_marker_array.markers[id].pose.position.x += self.initial_pos[id][0]
                # self.text_marker_array.markers[id].pose.position.y += self.initial_pos[id][1]

                # dot not comment these 2 lines, they define the identifier and frame reference for the markers
                self.text_marker_array.markers[id].id = id
                self.text_marker_array.markers[id].header.frame_id = f'/text{id}'
                rospy.loginfo(f'Marker {id} initialized successfully')

            except Exception as err:
                rospy.logerr('Not all markers successfully initialized', err)
        # print()


    def callback(self, data):
        # print("a")
        self.vehicle_marker_array.markers[0].pose = data.pose
        # print(self.marker_array)
        try:
            self.markerPublisher.publish(self.vehicle_marker_array)
            sys.exit(1)
        except Exception as err:
            print(err)

    def create_marker(self, frame, type, pose, scale, color, lifetime, action=0, vertices=0, id_t = 0):
        marker = Marker()
        rospy.logwarn(frame)
        marker.header.frame_id = frame
        # marker_.header.stamp = rospy.Time.now()
        # marker.type = marker.ARROW

        marker.action = action # eq a 0 - criar/ modificar

        rospy.loginfo("appending")
        marker.pose = pose

        marker.lifetime = rospy.Duration.from_sec(lifetime) # 0 -> forever
        # marker.scale.x = scale[0]
        # marker.scale.y = scale[1]
        # marker.scale.z = scale[2]
        marker.color.a = 1

        try:
            if type == 'drone':
                marker.type = marker.MESH_RESOURCE
                marker.mesh_resource = self.mesh_path

                vector = Vector3()
                vector.x = scale[0]
                vector.y = scale[1]
                vector.z = scale[2]
                # marker.scale.x = scale[0]
                # marker.scale.y = scale[1]
                # marker.scale.z = scale[2]

                marker.mesh_use_embedded_materials = False 

                marker.scale = vector
                marker.color.r = 1
                marker.color.g = 255
                marker.color.b = 1

                self.vehicle_marker_array.markers.append(marker)

            elif type == 'line':
                marker.type = marker.LINE_LIST
                red_, green_, blue_ = color
                marker.color.r = red_
                marker.color.g = green_
                marker.color.b = blue_         

                marker.points = vertices

                vector = Vector3()
                vector.x = scale[0]
                vector.y = scale[1]
                vector.z = scale[2]
                # marker.scale.x = scale[0]
                # marker.scale.y = scale[1]
                # marker.scale.z = scale[2]
                marker.scale = vector
                marker.color.a = 1
                self.safe_marker_array.markers.append(marker)
            
            elif type == 'text':
                marker.type = marker.TEXT_VIEW_FACING
                red_, green_, blue_ = color
                marker.color.r = red_
                marker.color.g = green_
                marker.color.b = blue_             
            
                marker.text = f"Clover {id_t}"
                vector = Vector3()
                vector.x = scale[0]
                vector.y = scale[1]
                vector.z = scale[2]
                # marker.scale.x = scale[0]
                # marker.scale.y = scale[1]
                # marker.scale.z = scale[2]
                marker.scale = vector
                marker.color.a = 1

                self.text_marker_array.markers.append(marker)

            elif type == 'base':
                marker.type = marker.MESH_RESOURCE
                marker.mesh_resource = self.mesh_path_base

                vector = Vector3()
                vector.x = scale[0]
                vector.y = scale[1]
                vector.z = scale[2]
                # marker.scale.x = scale[0]
                # marker.scale.y = scale[1]
                # marker.scale.z = scale[2]

                marker.mesh_use_embedded_materials = True 

                marker.scale = vector
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0

                self.base_marker_array.markers.append(marker)

        except Exception as err:
            rospy.logerr(f"COULD NOT CREATE MARKER OF TYPE {type} - {err}")
        rospy.loginfo("done")

    
    def marker_list(self):
        self.tf_handler = tf.Transformer(True, rospy.Duration(5))

        for id in self.lista_id:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0

            # Vehicle marker
            self.create_marker(frame=f"/body{id}", type="drone", pose=pose, scale=[1,1,1], color=[255, 255, 0], lifetime=0)

            self.create_marker(frame=f'/body{id}', type="base", pose=pose, scale=[1,1,1], color=[0,0,0], lifetime=0)

            #  Text marker
            # self.createTfFrame(id)
            text_s = 0.45
            self.create_marker(frame=f"/text{id}", type="text", pose=pose, scale=[text_s, text_s, text_s], color=[255, 255, 255], lifetime=0, id_t=id)


            rospy.loginfo(f'Current id: {id}')


def main():
    rospy.init_node("marker_handler")
    obj = SwarmStation()

    # obj.get_param()
    obj.setPublishers() 
    obj.setSubscribers()  
    obj.setListId() # get id array from connected clovers
    obj.setServices()
    obj.marker_list() # create a marker for each id
    obj.setInitialPose() # configure initial pose array
    obj.waitMessage() # links mavros pose to marker's
    
    try:
        for i in range(3):
            obj.markerPublisher.publish(obj.vehicle_marker_array)
            obj.textMarkerPublisher.publish(obj.text_marker_array)
            obj.baseMarkerPublisher.publish(obj.base_marker_array)
    except Exception as err:
        rospy.logerr(f"Could not publish markers - {err}")
        sys.exit(1)

    rospy.spin()
    

if __name__ == "__main__":
    main()