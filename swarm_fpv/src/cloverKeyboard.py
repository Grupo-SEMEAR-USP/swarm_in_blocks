import rospy 
from clover import srv
from std_srvs.srv import Trigger
import sys

# Class created in order to facilitate the control of each drone individually
class DroneKeyboard:
    def __init__(self, id):
        
        self.name = f'clover{id}'
        self.body = 'body' + str(id)
        self.base_link = 'base_link' + str(id)
        self.id = id

        try:
            rospy.wait_for_service(f"{self.name}/get_telemetry")
            rospy.wait_for_service(f"{self.name}/navigate")
            rospy.wait_for_service(f"{self.name}/set_velocity")
            rospy.wait_for_service(f"{self.name}/land")
           
            self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
            self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
            self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
            self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)
      
        except:
            
            sys.exit('Connection not succeeded - Please check if you have a simulation going on')

            
    # Takeoff method
    def takeoff(self):
        #print(self.get_telemetry())
        if not self.get_telemetry().armed:
            self.navigate(y=2, frame_id=self.body, auto_arm=True)
            print('Taking off...')
        else:
            print('Drone already armed!')

    # Position control method
    def move(self, axis, sp=1):
        if axis == 'x+':
            print('indo para frente')
            self.set_velocity(vx=sp, frame_id=self.body)
        elif axis == 'x-':
            self.set_velocity(vx=-sp, frame_id=self.body)
        elif axis == 'y+':
            self.set_velocity(vy=+sp, frame_id=self.body)
        elif axis == 'y-':
            self.set_velocity(vy=-sp, frame_id=self.body)
        elif axis == 'up':
            self.set_velocity(vz=sp, frame_id=self.body)
        elif axis == 'right':
            self.navigate(yaw=float('nan'), yaw_rate=-1, frame_id=self.body)
        elif axis == 'left':
            self.navigate(yaw=float('nan'), yaw_rate=1, frame_id=self.body)
        elif axis == 'down':
            if self.get_telemetry().z > 1:
                self.set_velocity(vz=-sp, frame_id=self.body)
            else:
                print('Drone is too close to the ground!')

    def stop(self):
        telem = self.get_telemetry('odom')
        self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0, frame_id=self.body)
        # self.navigate(x=telem.x, y=telem.y, z=telem.z, yaw=float('nan'))
        print()