from turtle import down
from pynput.keyboard import Key, Listener, KeyCode

import rospy 

from clover import srv
from std_srvs.srv import Trigger

import sys
import time
# name = '/clover'
# rospy.wait_for_service(f"{name}/get_telemetry")
# rospy.wait_for_service(f"{name}/navigate")
# rospy.wait_for_service(f"{name}/set_velocity")
# rospy.wait_for_service(f"{name}/land")

# set_velocity = rospy.ServiceProxy("set_velocity", srv.SetVelocity)
# navigate = rospy.ServiceProxy("navigate", srv.Navigate)
# get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
# land = rospy.ServiceProxy("land", Trigger)


#sname = 'clover1'



class DroneKeyboard:
    def __init__(self, id):
        #print('inicio  do init')
        self.name = f'clover{id}'
        self.body = 'body' + str(id)
        #print(self.name)
        try:
            rospy.wait_for_service(f"{self.name}/get_telemetry")
            rospy.wait_for_service(f"{self.name}/navigate")
            rospy.wait_for_service(f"{self.name}/set_velocity")
            rospy.wait_for_service(f"{self.name}/land")
            #print(self.name)
            self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
            self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
            self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
            self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)
            #print('fim do init')
        except:
            
            sys.exit('Connection not succeeded - Please check if you have a simulation going on')

            

    def takeoff(self, clover=1):
        #print(self.get_telemetry())
        if not self.get_telemetry().armed:
            self.navigate(y=2, frame_id=self.body, auto_arm=True)
            print('Taking off...')
        else:
            print('Drone already armed!')


    def move(self, axis, sp=2, clover=1):
        if axis == 'x+':
            self.set_velocity(vx=sp, frame_id=self.body)
        elif axis == 'x-':
            self.set_velocity(vx=-sp, frame_id=self.body)
        elif axis == 'y+':
            self.set_velocity(vy=-sp, frame_id=self.body)
        elif axis == 'y-':
            self.set_velocity(vy=+sp, frame_id=self.body)
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

    def stop(self, clover=1):
        self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0, frame_id=self.body)


def press(key):
    
    #print(key)
    if key == KeyCode.from_char('t'):
        drone.takeoff()
    elif key == KeyCode.from_char('w'):
        drone.move('up')
    elif key == KeyCode.from_char('s'):
        drone.move('down')
    elif key == KeyCode.from_char('a'):
        drone.move('left')
    elif key == KeyCode.from_char('d'):
        drone.move('right')
    elif key == KeyCode.from_char('l'):
        drone.land()
    elif key == KeyCode.from_char('g'):
        print(drone.get_telemetry())
    elif key == KeyCode.from_char('q'):
        print('Quitting..')
        return False

    if key == Key.up:
        drone.move('x+')
        
    if key == Key.right:
        drone.move('y+')

    if key == Key.left:
        drone.move('y-')
    if key == Key.down:
        drone.move('x-')
        
def release(key):
        # if key == Key.space:
        #     return False
    drone.stop()



print("Drone ID:")
id_drone = input()


drone = DroneKeyboard(id_drone)
print('Successfully initiaded! The following keys are accepted as commands:')
print('''
Navigation:
Keyarrows - (standart navigation keys)
W - Up (increases altitude)
S - Down (decreases altitude)
D - Turn right in rotation
A - Turn left in rotation
L - Land
===============
Utilities:
G - Get drone info (uses get_telemetry method)
''')

with Listener(
    on_press=press,
    on_release=release) as listener:
    time.sleep(0.5)
    listener.join()

#drone = DroneKeyboard(1)