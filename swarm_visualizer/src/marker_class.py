#!/usr/bin/python3

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
import sys
import rospy


class MarkerObj:
    def __init__(self):
        self.marker_array = MarkerArray()
        self.marker_single = Marker()

    def setPublishers(self):
        self.markerPublisher = rospy.Publisher("/vehicle_marker", MarkerArray, queue_size=10)


    def setSubscribers(self):
        rospy.Subscriber("/clover0/mavros/local_position/pose", PoseStamped, callback=self.callback)


    def waitMessage(self, lista_id):
        for id in lista_id:
            message = rospy.wait_for_message(f'/clover{id}/mavros/local_position/pose', PoseStamped, timeout=5)

            cur_pose = message.pose
            cur_pose.position.x += 1
            self.marker_array.markers[id].pose = message.pose
            self.marker_array.markers[id].pose.position.y += id
            self.marker_array.markers[id].id = id
            print(id)

    

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
    
    def marker_list(self, id_list):
        for id in id_list:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0
            self.create_marker(frame=f"/base_link{id}", type="", pose=pose, scale=[1,1,1], color=[255, 255, 0], lifetime=0)
    

def main():
    rospy.init_node("marker_handler")
    obj = MarkerObj()

   

    list = [0, 1]

    obj.marker_list(list)

    obj.setPublishers()
    # obj.setSubscribers()
    ## test 2
    obj.waitMessage(list)
    # rospy.spin()
    while not rospy.is_shutdown():
        obj.markerPublisher.publish(obj.marker_array)
        a = input("enter para proximo pub")
    

if __name__ == "__main__":
    main()