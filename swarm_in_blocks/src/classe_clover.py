#!/usr/bin/python3

import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool, SetModeRequest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger

class Make_Clover:

   def _init_(self, name, id):
      self.name = name
      self.id = id

      self.current_state = State()

      # Configure clover services and topics
      self.configure()

   def state_cb(self, msg_cb):
      self.current_state = msg_cb

   def configure(self):
      self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.state_cb, queue_size=10)    
      self.local_pos_pub = rospy.Publisher(f"{self.name}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
      self.arming_client = rospy.ServiceProxy(f"{self.name}/mavros/cmd/arming",  CommandBool)
      self.set_mode_client = rospy.ServiceProxy(f"{self.name}/mavros/set_mode", SetMode)
      self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
      self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
      self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
      self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
      self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
      self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
      self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
      self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger) 


class Swarm_Clover(Make_Clover):
   
   
   def takeoff(self):
      rate = rospy.Rate(20)
      rospy.loginfo("Waiting connected")

      while(not rospy.is_shutdown() and self.current_state.connected):
         #rospy.spin()
         rate.sleep()
      rospy.loginfo("Connected")

      pose = PoseStamped()
      pose.pose.position.x = 0
      pose.pose.position.y = 0
      pose.pose.position.z = 2

      rospy.loginfo("Publishing first pose")
      for i in range(0,50):
         self.local_pos_pub.publish(pose)
         # rospy.spin()
         rate.sleep()
      rospy.loginfo("Published")

      last_request = time.time()
      first_request = time.time()

      rospy.loginfo("Changing mode to OFFBOARD")
      #mode_sent = SetModeRequest(None,"OFFBOARD")
      mode_sent = self.set_mode_client.call(SetModeRequest(None,"OFFBOARD"))
      #print("OFFBOARD FOI")
      #print(mode_sent)
      rospy.loginfo("Arming")
      sucess = self.arming_client.call(True)

      while(not rospy.is_shutdown()):
         if (self.current_state.mode != "OFFBOARD" and (time.time() - last_request > 5)):
               rospy.loginfo("Changing mode to OFFBOARD")
               mode_sent = self.set_mode_client.call(SetModeRequest(None,"OFFBOARD"))

               if(mode_sent):
                  rospy.loginfo("Offboard enabled")
               
               last_request = time.time()
         elif (not self.current_state.armed and (time.time() - last_request > 5)):
               rospy.loginfo("Arming")
               sucess = self.arming_client.call(True)
               if(sucess):
                  rospy.loginfo("Vehicle armed")
         
               last_request = time.time()
         
         self.local_pos_pub.publish(pose)        

         #rospy.spin()
         rate.sleep()
