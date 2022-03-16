from termios import VEOL
import rospy 
import tkinter as tk
from clover import srv
from std_srvs.srv import Trigger

# Proxys definitions
name = '/clover0'
rospy.wait_for_service(f"{name}/get_telemetry")
rospy.wait_for_service(f"{name}/navigate")
rospy.wait_for_service(f"{name}/set_velocity")
rospy.wait_for_service(f"{name}/land")
set_velocity = rospy.ServiceProxy(f"{name}/set_velocity", srv.SetVelocity)
navigate = rospy.ServiceProxy(f"{name}/navigate", srv.Navigate)
get_telemetry = rospy.ServiceProxy(f"{name}/get_telemetry", srv.GetTelemetry)
land = rospy.ServiceProxy(f"{name}/land", Trigger)


def takeoff(clover=0):
    if not get_telemetry().armed:
        navigate(y=2, frame_id='body', auto_arm=True)
        print('Taking off...')
    else:
        print('Drone already armed!')

def move(axis, sp=1, clover=0):
    if axis == 'x+':
        set_velocity(vx=sp, frame_id='body')
    elif axis == 'x-':
        set_velocity(vx=-sp, frame_id='body')
    elif axis == 'y+':
        set_velocity(vy=-sp, frame_id='body')
    elif axis == 'y-':
        set_velocity(vy=+sp, frame_id='body')
    elif axis == 'up':
        set_velocity(vz=sp, frame_id='body')
    elif axis == 'down':
        if get_telemetry().z > 1:
            set_velocity(vz=-sp)
        else:
            print('Drone is too close to the ground!')

def stop(clover=0):
    set_velocity(vx=0, vy=0, vz=0, frame_id='body')



def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
        elif event.char == 's':
            stop()

        elif event.char == 'u':
            move('up')
        elif event.char == 'j':
            move('down')
        elif event.char == 't':
            takeoff()
        elif event.char == 'l':
            land()


    else: #-- non standard keys
        if event.keysym == 'Up':
            print("up")
            move('x+')

        elif event.keysym == 'Down':
            print("down")
            move('x-')
            
        elif event.keysym == 'Left':
            print("left")
            move('y-')
            
        elif event.keysym == 'Right':
            print("right")
            move('y+')
            
        

def main():
    root = tk.Tk()
    root.bind_all('<Key>', key)
   

    root.mainloop()



if __name__ == "__main__":
    main()