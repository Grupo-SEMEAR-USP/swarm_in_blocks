#!/usr/bin/python3

# Tools
import time
import wx
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from threading import Thread

# ROS imports
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import CompressedImage

# Local import
from cloverKeyboard import DroneKeyboard


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
        
        # max_width_x, max_width_y = wx.DisplaySize()
        # self.window_size = (max_width_x*2//3, max_width_y*5//6)

        index = 0
        display = wx.Display(index)
        
        _, _, max_width_x, max_width_y = display.GetGeometry()
        self.window_size = (max_width_x*2//3, max_width_y*5//6)
        print(self.window_size)
        # wx
        self.frame = wx.Frame(None, title = "First Person View - Swarm In Blocks",  style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER, size=self.window_size)
        # self.frame = wx.Frame(None, title = "ROS Image View", size=self.window_size)
        self.frame.SetIcon(wx.Icon(os.path.join(node_path, 'assets', 'logo.ico')))
        self.panel = ImageViewPanel(self.frame)
        self.InitUI()
        self.panel.setup(self)
        # self.frame.Maximize(True)
        self.frame.Centre()
        self.frame.Show(True)
        
        return True

    def InitUI(self):
        self.panel_div_y = int(3/5*self.window_size[1])
        self.initial_panel_pos = (0, self.panel_div_y)
        panel_size_y = int(2/3*self.window_size[1])
        self.initial_panel_size = (self.window_size[0], panel_size_y)
        
        self.midp = wx.Panel(self.panel, pos = self.initial_panel_pos, size=self.initial_panel_size)
        self.midp.SetBackgroundColour("black")
        sizer = wx.GridBagSizer(4,2)

        # configuraçao da GUI (parte interativa)
        font = wx.SystemSettings.GetFont(wx.SYS_SYSTEM_FONT)
        font.SetPointSize(14)
        
        # text = wx.StaticText(self.midp, label="Clover:")
        # text.SetFont(font)
        # sizer.Add(text, (0,0), (1,1), wx.ALIGN_CENTER, 0)
        
        self.list = wx.Choice(self.midp, choices=clovers)
        sizer.Add(self.list, (1,5), (1,1), wx.ALIGN_CENTER, 10)

        self.list.Bind(wx.EVT_CHOICE, self.onChoice)
        
        self.toggle = wx.ToggleButton(self.midp, -1, label='Active')
        sizer.Add(self.toggle, (2,5), (1,1),  wx.ALIGN_CENTER, 10)

        icon = wx.StaticBitmap(self.midp, bitmap=wx.Bitmap(os.path.join(node_path, 'assets', 'logomark.png')))
        sizer.Add(icon, (3,5), (1,1), wx.ALIGN_CENTER, 10)
        # icon2 = wx.StaticBitmap(self.midp, bitmap=wx.Bitmap(os.path.join(node_path, 'assets', 'logomark.png')))
        # sizer.Add(icon2, (3,1), (1,1), wx.ALIGN_CENTER, 10)

        # line
        line = wx.StaticLine(self.midp)
        sizer.Add(line, pos=(1, 6), span=(7, 1),
            flag=wx.EXPAND|wx.BOTTOM, border=50)

        # line2 = wx.StaticLine(self.midp)
        # sizer.Add(line2, pos=(0, 1), span=(1, 5),
        #     flag=wx.EXPAND|wx.BOTTOM, border=50)

        # text
        control_text = wx.StaticText(self.midp)
        control_text.SetFont(font)
        # sizer.Add(control_text, pos=(3, 8), flag=wx.ALIGN_CENTER)

        # joystick addition
        joystick = wx.StaticBitmap(self.midp, bitmap=wx.Bitmap(os.path.join(node_path, 'assets', 'joy.jpg')))
        # sizer.Add(joystick, (3, 1), (1, 1), wx.ALIGN_CENTER, 10)
        
        joystick.SetPosition((self.window_size[0] - 300, 30))

        # sizer.AddGrowableRow(0)
        # sizer.AddGrowableRow(1)
        # sizer.AddGrowableRow(2)
        # sizer.AddGrowableCol(0)
        # sizer.AddGrowableCol(1)
        self.midp.SetSizer(sizer)
        
        # Keybidings events:
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_KEY_UP, self.OneKeyUp)
        self.Bind(wx.EVT_CHAR, self.OnKeyDown)
        # self.Bind(wx.EVT_SIZE, self.OnResizeWindow)
        self.toggle.SetFocus()
    
    # def OnResizeWindow(self, event):
    #     pass
    #     # self.frame.Maximize(True)
    #     # event.Skip()
    
    def onChoice(self, event): # Deals with Choice event
        choice = self.list.GetCurrentSelection() # Return the wished id  
        self.toggle.SetFocus()

        for subs in subscribers: # Unsubscribes all topics so it won't accumulate 
            subs.unregister()

        subscribers.append(rospy.Subscriber(f'/clover{choice}/main_camera/image_raw/compressed', CompressedImage, handle_image,queue_size=10)) # se inscreve no topico pedido e ativa handle_image
        print(choice)        

        # Erases previous object so that only one drone is controlled at a time
        keyboard_clover.clear()
        
        # Creates a new drone object that contains controlling methods 
        drone = DroneKeyboard(choice)
        keyboard_clover.append(drone)
        print(keyboard_clover)
        # print(dir(drone))

    def OnKeyDown(self, event):
        #print(event.GetKeyCode())
        print('key down')
        self.toggle.SetFocus()
        key = event.GetKeyCode()
        # keyU = event.GetUnicodeKey()
        if self.last_key == key or self.last_key == key + 32 or self.last_key == key - 32:
            event.Skip()
            return
        else:
            self.last_key = key
            print(f'GetKeyCode: {key}')

            # Function that handles the key pressed
            if self.last_thrd.is_alive():
                # self.last_thrd.join()
                event.Skip()
                return
            thrd = Thread(target=mov_control, args=(key,))
            thrd.start()
            self.last_thrd = thrd

    def OneKeyUp(self, event):
        print('key released')
        print(len(keyboard_clover))
        self.last_key = ''
        self.toggle.SetFocus()
        # Stops all objects that are currently being used
        if keyboard_clover:
            for obj in keyboard_clover:
                tick = time.time()
                if self.last_thrd.is_alive():
                    event.Skip()
                    return
                # while self.last_thrd.is_alive():
                    
                #     if time.time()-tick > 5:
                #         print("Clover do not respond! Waiting...")
                #     if time.time()-tick > 10:
                #         print("Clover might be disconnected. Stop waiting")
                #         break
                    # self.last_thrd.join()
                    
                thrd = Thread(target=obj.stop)
                thrd.start()
                self.last_thrd = thrd
                event.Skip()


class ImageViewPanel(wx.Panel):
    def setup(self, parent):
        self.img = np.zeros((480, 640,3))
        self.SetBackgroundColour("#bdbdbd")
        self.bmp_size = (parent.window_size[0]//2 - self.img.shape[1]//2, parent.panel_div_y//2 - self.img.shape[0]//2)
        self.staticbmp = wx.StaticBitmap(self, pos=self.bmp_size) #, , pos=(90, 50)
        self.timer = wx.Timer(self)
        self.timer.Start(1000//60)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_TIMER, self.NextFrame)

        
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
        dc.DrawBitmap(self.staticbmp, self.bmp_size[0], self.bmp_size[1])
        
    def NextFrame(self, event):
        
        # self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.staticbmp.CopyFromBuffer(self.img)
        self.Refresh()

def handle_image(ros_image):
    # make sure we update in the UI thread
    cv_image = bridge.compressed_imgmsg_to_cv2(ros_image)
    cv_image = cv2.resize(cv_image, (640,480))
    wx.CallAfter(wx.GetApp().panel.update, cv_image)

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
#/clover0/main_camera/image_raw   #  <--- topic to be subscribed
def topics_sorter():
    id_r = 0
    topic_list = rospy.get_published_topics()
    topic_list.sort()
    for str in topic_list:
        ref = '/main_camera/image_raw'
        refc = '/main_camera/image_raw/c'
        reft = '/main_camera/image_raw/t'
        if  ref in str[0] and refc not in str[0] and reft not in str[0]: # faz a filtragem correta

            clovers.append(f'clover{id_r}')
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