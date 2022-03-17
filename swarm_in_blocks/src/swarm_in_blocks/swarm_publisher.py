import logging
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from swarm_in_blocks.msg import SwarmApiState

class SwarmPublisher:
    def __init__(self):
        self.state_pub = rospy.Publisher("~state", SwarmApiState, queue_size=10)
        self.formation_name_pub = rospy.Publisher("~formation/name", String, queue_size=10)
        self.formation_pose_pub = rospy.Publisher("~formation/pose", Pose, queue_size=10)
        self.formation_coords_pub = rospy.Publisher("~formation/coordinates", PoseArray, queue_size=10)

    def publishSwarmAssets(self, swarm_obj):
        
        self.formation_name_pub.publish(swarm_obj.curr_formation_name)

        pose_msg = Pose()
        if swarm_obj.curr_formation_pose.size != 0:
            pose_msg.position.x = swarm_obj.curr_formation_pose[0]
            pose_msg.position.y = swarm_obj.curr_formation_pose[1]
            pose_msg.position.z = swarm_obj.curr_formation_pose[2]
        self.formation_pose_pub.publish(pose_msg)

        pose_arr_msg = PoseArray()
        pose_arr_msg.header.stamp = rospy.Time.now()
        # pose_arr_msg.poses is one list that you migh want to append Pose objects
        for coord in swarm_obj.curr_formation_coords:
            pose = Pose()
            pose.position.x = coord[0]
            pose.position.y = coord[1]
            pose.position.z = coord[2]
            pose_arr_msg.poses.append(pose)
        self.formation_coords_pub.publish(pose_arr_msg)

    def publishSwarmStatus(self, swarm_obj):
        
        swarm_state = SwarmState()
        swarm_state.header.stamp = rospy.Time.now()
        swarm_state.name = swarm_obj.swarm_name
        swarm_state.status = swarm_obj.status
        swarm_state.mode = swarm_obj.mode
        swarm_state.connected_clovers = swarm_obj.connected_clovers
        swarm_state.armed_clovers = swarm_obj.armed_clovers
        self.state_pub.publish(swarm_state)
    
    def publishStatusLoop(self, swarm_obj):
        logging.debug("Starting publishing swarm status.")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publishSwarmStatus(swarm_obj)
            rate.sleep()
    
    def publishAssetsLoop(self, swarm_obj):
        logging.debug("Starting publishing swarm assets.")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publishSwarmAssets(swarm_obj)
            rate.sleep()
