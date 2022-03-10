#!/usr/bin/python3
#/clover0/main_camera/image_raw   #  <--- topic to be subscribed


"""
usage:
rosrun proteus_demo ImageView.py image:=/ATRV/CameraMain
image:=/clover0/main_camera/image_raw
image:=/clover0/main_camera/parameter_updates
"""


from typing import NoReturn
import new_pynput
import threading
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import CompressedImage
import time
import wx
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from threading import Thread

subscribers = [] # lista com o atual subscriber

clovers = [] # lista com o numero em str dos topicos a serem inscritos

keyboard_clover = [] # list with drone objects for later control

bridge = CvBridge()

node_path = os.path.dirname(os.path.realpath(__file__))

class ImageViewApp(wx.App):
    def OnInit(self):
        # class vars
        self.last_key = ''
        self.last_thrd = Thread()
        
        # wx
        self.frame = wx.Frame(None, title = "ROS Image View", size=(500, 560))
        self.frame.SetIcon(wx.Icon(os.path.join(node_path, 'logo.ico')))
        self.panel = ImageViewPanel(self.frame)
        self.panel.setup()
        self.frame.Show(True)
        self.InitUI()
        
        return True

    def InitUI(self):
        midp = wx.Panel(self.panel, pos = (0, 310), size = (500, 600))
        midp.SetBackgroundColour("#616161")
        sizer = wx.GridBagSizer(5, 5)
        # http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html

        # configuraçao da GUI (parte interativa)
        font = wx.SystemSettings.GetFont(wx.SYS_SYSTEM_FONT)
        font.SetPointSize(14)
        
        text = wx.StaticText(midp, label="Topic:")
        text.SetFont(font)
        sizer.Add(text, pos=(1, 1), flag=wx.EXPAND, border = 30)
        
        self.list = wx.Choice(midp, choices=clovers)
        sizer.Add(self.list, pos=(1, 3), span=(2, 14), flag=wx.ALIGN_CENTER|wx.EXPAND)

        self.list.Bind(wx.EVT_CHOICE, self.onChoice)
        
        icon = wx.StaticBitmap(midp, bitmap=wx.Bitmap(os.path.join(node_path, 'verde-claro.png')))
        sizer.Add(icon, pos=(5, 6), flag=wx.BOTTOM|wx.ALIGN_BOTTOM,border=5)

        # Keybidings events:
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_KEY_UP, self.OneKeyUp)
        self.Bind(wx.EVT_CHAR, self.OnKeyDown)
        #self.SetFocus()

        midp.SetSizer(sizer)



    def onChoice(self, event): # Deals with Choice event
        choice = self.list.GetCurrentSelection() # Return the wished id  

        for subs in subscribers: # Unsubscribes all topics so it won't accumulate 
            subs.unregister()

        subscribers.append(rospy.Subscriber(f'/clover{choice}/main_camera/image_raw/compressed', CompressedImage, handle_image,queue_size=10)) # se inscreve no topico pedido e ativa handle_image
        print(choice)        

        # Erases previous object so that only one drone is controlled at a time
        keyboard_clover.clear()
        
        # Creates a new drone object that contains controlling methods 
        drone = new_pynput.DroneKeyboard(choice)
        keyboard_clover.append(drone)
        print(keyboard_clover)
        # print(dir(drone))



    def OnKeyDown(self, event=None):
        #print(event.GetKeyCode())
        print('key down')
        key = event.GetKeyCode()
        # keyU = event.GetUnicodeKey()
        if self.last_key == key:
            event.Skip()
        if self.last_key != key:
            self.last_key = key
            print(f'GetKeyCode: {key}')

            # Function that handles the key pressed
            if self.last_thrd.is_alive():
                self.last_thrd.join()
            thrd = Thread(target=mov_control, args=(key,))
            thrd.start()
            self.last_thrd = thrd
        
        

    def OneKeyUp(self, event=None):
        print('key released')
        self.last_key = ''
        # Stops all objects that are currently being used
        if keyboard_clover:
            for obj in keyboard_clover:
                if self.last_thrd.is_alive():
                    self.last_thrd.join()
                thrd = Thread(target=obj.stop)
                thrd.start()
                self.last_thrd = thrd


class ImageViewPanel(wx.Panel):
    def setup(self):
        self.SetBackgroundColour("#bdbdbd")
        self.staticbmp = wx.StaticBitmap(self, pos=(90, 30)) #, , pos=(90, 50)
        self.timer = wx.Timer(self)
        self.timer.Start(1000./60)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_TIMER, self.NextFrame)

        self.img = np.zeros((640,480,3))
        self.staticbmp = wx.Bitmap.FromBuffer(self.img.shape[1], self.img.shape[0], self.img)

    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def update(self, image):
        self.img = image
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.staticbmp = wx.Bitmap.FromBuffer(self.img.shape[1], self.img.shape[0], self.img)
        
        #sizer.Add(bmp, pos=(0,2))
        
        # dc = wx.BufferedPaintDC(self)
        # dc.DrawBitmap(self.bmp, 0, 0)


    # The following events were written in order to improve camera performance
    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.staticbmp, 90, 30)
        
    def NextFrame(self, event):
        
        # self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.staticbmp.CopyFromBuffer(self.img)
        self.Refresh()


t0 = 0
def handle_image(ros_image):
    # make sure we update in the UI thread
    
    # encoded = np.frombuffer(image.data, np.uint32)
    # image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    cv_image = bridge.compressed_imgmsg_to_cv2(ros_image)
    wx.CallAfter(wx.GetApp().panel.update, cv_image)
    global t0
    #print(time.perf_counter() - t0)
    t0 = time.perf_counter()
    #wx.GetApp().panel.update(image)
    # http://wiki.wxpython.org/LongRunningTasks



# Key input analisys 
def mov_control(key):
    if keyboard_clover: 
            # Keyboard input for arrow keys
            if key in [wx.WXK_UP]:
                print('UP')
                keyboard_clover[0].move('x+')
                
            if key in [wx.WXK_DOWN]:
                print('DOWN')
                keyboard_clover[0].move('x-')

            if key in [wx.WXK_LEFT]:
                print('LEFT')
                keyboard_clover[0].move('y+')

            if key in [wx.WXK_RIGHT]:
                print('RIGHT')
                keyboard_clover[0].move('y-')
            
            # Keyboard input for command letters
            if key == 84: # t
                #print('Taking off..')
                keyboard_clover[0].takeoff()
            
            if key == 76: # l
                print('Landing drone..')
                keyboard_clover[0].land()

            if key == 87: # w
                print('Flying up')
                keyboard_clover[0].move('up')
            
            if key == 65: # a
                print('Turning left')
                keyboard_clover[0].move('left')

            if key == 83: # s
                print('Flying down')
                keyboard_clover[0].move('down')

            if key == 68: # d
                print('Turning right')
                keyboard_clover[0].move('right')

    else:
        print('No drone has been initialized yet!')


# Filtrates all topics in order to find how many drones are publishing
# It's only called once
def topics_sorter():
    id_r = 0
    for str in rospy.get_published_topics():
        ref = '/main_camera/image_raw'
        refc = '/main_camera/image_raw/c'
        reft = '/main_camera/image_raw/t'
        if  ref in str[0] and refc not in str[0] and reft not in str[0]: # faz a filtragem correta

            clovers.append(f'{id_r}')
            id_r = id_r + 1


def main():
    app = ImageViewApp()
    rospy.init_node('ImageView')
    app.MainLoop()
    return 0


if __name__ == "__main__":
    #threading.Thread(target=init_pynput).start()
    topics_sorter() # separa os topicos desejados
    print(clovers)
    main()