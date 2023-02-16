#!/usr/bin/python3

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from swarm_checker.msg import SwarmState
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
import sys
import rospy


class MarkerObj:
    def __init__(self):
        self.marker_array = MarkerArray()
        self.marker_single = Marker()
        self.lista_id = []
        self.initial_pos = []

    def setPublishers(self):
        self.markerPublisher = rospy.Publisher("/vehicle_marker", MarkerArray, queue_size=10)


    def setSubscribers(self):
        # rospy.Subscriber("/clover0/mavros/local_position/pose", PoseStamped, callback=self.callback)
        rospy.Subscriber("/marker_state", String, callback=self.markerCallback)

    def markerCallback(self, data):
        if data.data == 'reload':
            rospy.loginfo('Republishing markers on reload request..')
            self.markerPublisher.publish(self.marker_array)

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
                message = rospy.wait_for_message(f'/clover{id}/mavros/local_position/pose', PoseStamped, timeout=5)

                cur_pose = message.pose
                # cur_pose.position.x += 1
                self.marker_array.markers[id].pose = message.pose
                print('aaa')
                self.marker_array.markers[id].pose.position.x += self.initial_pos[id][0]
                self.marker_array.markers[id].pose.position.y += self.initial_pos[id][1]
                self.marker_array.markers[id].id = id
                self.marker_array.markers[id].header.frame_id = f'/base_link{id}'
                print(f'Marker {id} initialized successfully')
            except Exception as err:
                print('Not all markers successfully initialized', err)
        # print()


    def callback(self, data):
        # print("a")
        self.marker_array.markers[0].pose = data.pose
        # print(self.marker_array)
        try:
            self.markerPublisher.publish(self.marker_array)
            sys.exit(1)
        except Exception as err:
            print(err)
    

    def create_marker(self, frame, type, pose, scale, color, lifetime):
        marker = Marker()
    
        marker.header.frame_id = frame
        # marker_.header.stamp = rospy.Time.now()
        marker.type = marker.ARROW
        
        marker.action = 0 # eq a 0 - criar/ modificar

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation.x = pose.orientation.x
        marker.pose.orientation.y = pose.orientation.y
        marker.pose.orientation.z = pose.orientation.z
        marker.pose.orientation.w = pose.orientation.w

        marker.lifetime = rospy.Duration.from_sec(lifetime) # 0 -> forever
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = 1
        red_, green_, blue_ = color
        marker.color.r = red_
        marker.color.g = green_
        marker.color.b = blue_  
        self.marker_array.markers.append(marker)
    
    def marker_list(self):
        for id in self.lista_id:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0
            self.create_marker(frame=f"/base_link{id}", type="", pose=pose, scale=[1,1,1], color=[255, 255, 0], lifetime=0)

            print('a', id)
    

def main():
    rospy.init_node("marker_handler")
    obj = MarkerObj()

   
    # id list of functional clovers from swarm state
    # list = [0, 1, 2]


    obj.setPublishers() 
    obj.setSubscribers()  
    obj.setListId() # get id array from connected clovers
    obj.marker_list() # create a marker for each id
    obj.setInitialPose() # configure initial pose array
    obj.waitMessage() # links mavros pose to marker's
    
    # rospy.spin()

    # while not rospy.is_shutdown():
    #     obj.markerPublisher.publish(obj.marker_array)
    #     if input("enter para proximo pub") == 'quit':
    #         break
    obj.markerPublisher.publish(obj.marker_array)
    rospy.spin()
    

if __name__ == "__main__":
    main()