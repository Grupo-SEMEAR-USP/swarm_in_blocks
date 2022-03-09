import rospy
import logging

# Mavros msgs
from mavros_msgs import srv
from mavros_msgs.msg import State

# Clover services
from clover import srv
from std_srvs.srv import Trigger

class SingleClover: 
#Create and call all servicers, subscribers and clover topics

    def __init__(self, name, id):
        self.name = name
        self.id = id

        self.current_state = State()

        # Configure clover services and topics
        self.checkConfiguration()

    def stateCb(self, msg_cb):
        self.current_state = msg_cb
        self.connected = msg_cb.connected
        self.armed = msg_cb.armed
        self.mode = msg_cb.mode

    def checkConfiguration(self):

        # logging.debug("Waiting clover services...")
        # rospy.loginfo("Waiting clover services...")
        self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)
        
        rospy.wait_for_service(f"{self.name}/get_telemetry")
        self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
        
        rospy.wait_for_service(f"{self.name}/navigate")
        self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
        
        rospy.wait_for_service(f"{self.name}/navigate_global")
        self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
        
        rospy.wait_for_service(f"{self.name}/set_position")
        self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
        
        rospy.wait_for_service(f"{self.name}/set_velocity")
        self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
        
        rospy.wait_for_service(f"{self.name}/set_attitude")
        self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
        
        rospy.wait_for_service(f"{self.name}/set_rates")
        self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
        
        rospy.wait_for_service(f"{self.name}/land")
        self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)