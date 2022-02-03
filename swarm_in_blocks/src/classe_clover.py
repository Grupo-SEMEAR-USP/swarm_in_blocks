#!/usr/bin/python3

import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger

class SingleClover:

   def __init__(self, name, id):
      self.name = name
      self.id = id

      self.current_state = State()

      # Configure clover services and topics
      self.configure()

   def stateCb(self, msg_cb):
      self.current_state = msg_cb

   def configure(self):
      self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)    
      self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
      self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
      self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
      self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
      self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
      self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
      self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
      self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger) 


class Swarm:
   def __init__(self, number_clover):
      rospy.init_node('swarm')
      self.number_clover = number_clover
      self.swarm = []
      for index in range(number_clover):
         clover_object = SingleClover(f"clover{index}", index)
         self.swarm.append(clover_object)
   
   def takeOffAll(self):

      for clover in self.swarm:
         clover.navigate(x=0, y=0, z=1, auto_arm=True)

   def launchSwarm(self):
      pass

if __name__ == "__main__":

   swarm = Swarm(4)

   swarm.takeOffAll()



