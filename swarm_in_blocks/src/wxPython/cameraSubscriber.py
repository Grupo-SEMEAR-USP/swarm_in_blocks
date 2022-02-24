#!/usr/bin/python3
#/clover0/main_camera/image_raw   #  <--- topic to be subscribed


"""
usage:
rosrun proteus_demo ImageView.py image:=/ATRV/CameraMain
image:=/clover0/main_camera/image_raw
image:=/clover0/main_camera/parameter_updates
"""




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

subscribers = [] # lista com o atual subscriber

clovers = [] # lista com o numero em str dos topicos a serem inscritos

bridge = CvBridge()
class ImageViewApp(wx.App):
    def OnInit(self):
        self.frame = wx.Frame(None, title = "ROS Image View", size=(500, 560))
        self.frame.SetIcon(wx.Icon('logo.ico'))
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
        
        icon = wx.StaticBitmap(midp, bitmap=wx.Bitmap('verde-claro.png'))
        sizer.Add(icon, pos=(5, 6), flag=wx.BOTTOM|wx.ALIGN_BOTTOM,border=5)

        midp.SetSizer(sizer)

    def onChoice(self, event): # lida com os eventos de choice
        choice = self.list.GetCurrentSelection() # retorna o id do topico a ser visualizado 


        for subs in subscribers: # se desinscreve de todos os topicos para evitar sobreposição e acumulo
            subs.unregister()

        subscribers.append(rospy.Subscriber(f'/clover{choice}/main_camera/image_raw/compressed', CompressedImage, handle_image,queue_size=10)) # se inscreve no topico pedido e ativa handle_image
        print(choice)
        #print(rospy.get_published_topics())


class ImageViewPanel(wx.Panel):
    def setup(self):
        self.SetBackgroundColour("#bdbdbd")
        self.staticbmp = wx.StaticBitmap(self, pos=(90, 30)) #, , pos=(90, 50)
        self.timer = wx.Timer(self)
        self.timer.Start(1000./60)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_TIMER, self.NextFrame)

        self.img = np.zeros((640,480,3))
        self.bmp = wx.Bitmap.FromBuffer(self.img.shape[1], self.img.shape[0], self.img)

    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def update(self, image):
        # cria um tipo de "dado" bitmap caso nao fosse e seta de acordo com os canais
        # a informação coletada nao entrava nos requisitos do if, portanto pus para serem executadas diretamente
        
        # http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html
     
        # transforma o buffer em um bitmap e insere na gui no painel criado (self)
        
        self.img = image
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.bmp = wx.Bitmap.FromBuffer(self.img.shape[1], self.img.shape[0], self.img)
        
        #sizer.Add(bmp, pos=(0,2))
        
        # dc = wx.BufferedPaintDC(self)
        # dc.DrawBitmap(self.bmp, 0, 0)

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bmp, 0, 0)

    def NextFrame(self, event):
        
        # self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.bmp.CopyFromBuffer(self.img)
        self.Refresh()

t0 = 0
def handle_image(ros_image):
    # make sure we update in the UI thread
    
    # encoded = np.frombuffer(image.data, np.uint32)
    # image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    cv_image = bridge.compressed_imgmsg_to_cv2(ros_image)
    wx.CallAfter(wx.GetApp().panel.update, cv_image)
    global t0
    print(time.perf_counter() - t0)
    t0 = time.perf_counter()
    #wx.GetApp().panel.update(image)
    # http://wiki.wxpython.org/LongRunningTasks




def main():
    app = ImageViewApp()
    rospy.init_node('ImageView')
    #rospy.sleep(0.1)
    
    
    # rospy.Subscriber(setID(id_ros), Image, handle_image)
    #print(__doc__)
    
    app.MainLoop()
    return 0


def topics_sorter():
    id_r = 0
    for str in rospy.get_published_topics():
        ref = '/main_camera/image_raw'
        refc = '/main_camera/image_raw/c'
        reft = '/main_camera/image_raw/t'
        if  ref in str[0] and refc not in str[0] and reft not in str[0]: # faz a filtragem correta

            clovers.append(f'{id_r}')
            id_r = id_r + 1


if __name__ == "__main__":
    topics_sorter() # separa os topicos desejados
    print(clovers)
    main()