#!/usr/bin/python3

'''
 * @file offb_node.py
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
'''

from distutils.util import subst_vars
from http import client
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

   def _init_(self):
      self.current_state = State()

   def state_cb(self, msg_cb):
      self.msg_cb = msg_cb
      self.current_state = msg_cb
     

   def nodes(self, name):
      self.name = name 
      rospy.init_node("class_clover_node", anonymous=True) 
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


class Swarm_Clover:

   def __init__(self, number_clover):
       self.number_clover = number_clover
       clover_object_list = []
       for n in range(number_clover):
          clover_object = Make_Clover()
          clover_object_list.append(clover_object)
      
     
       


   '''def takeoff(self, z):
      print("Takeoff all clovers")
      self.z = z
      print("All drones taking off")
      for i in range(self.number_clover):
        nav = "nav" + str(i)
        eval(nav)(z=1, auto_arm=True)
        print("Clover {} taking off".format(i))
      rospy.sleep(7)
      print("Done\n")'''

     



