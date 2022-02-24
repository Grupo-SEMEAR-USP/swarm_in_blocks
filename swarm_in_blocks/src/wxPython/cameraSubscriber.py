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
from sensor_msgs.msg import Image

import wx

subscribers = [] # lista com o atual subscriber

clovers = [] # lista com o numero em str dos topicos a serem inscritos


class ImageViewApp(wx.App):
    def OnInit(self):
        self.frame = wx.Frame(None, title = "ROS Image View", size=(500, 560))
        self.frame.SetIcon(wx.Icon('logo.ico'))
        self.panel = ImageViewPanel(self.frame)
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

        subscribers.append(rospy.Subscriber(f'/clover{choice}/main_camera/image_raw', Image, handle_image)) # se inscreve no topico pedido e ativa handle_image
        print(choice)
        #print(rospy.get_published_topics())


class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def update(self, image):
        # cria um tipo de "dado" bitmap caso nao fosse e seta de acordo com os canais
        # a informação coletada nao entrava nos requisitos do if, portanto pus para serem executadas diretamente
        self.SetBackgroundColour("#bdbdbd")
    
        # http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html
     

        # transforma o buffer em um bitmap e insere na gui no painel criado (self)
        self.staticbmp = wx.StaticBitmap(self, pos=(90, 30)) #, , pos=(90, 50)
        bmp = wx.Bitmap.FromBuffer(image.width, image.height, image.data)
        #sizer.Add(bmp, pos=(0,2))
        self.staticbmp.SetBitmap(bmp)


def handle_image(image):
    # make sure we update in the UI thread
    wx.CallAfter(wx.GetApp().panel.update, image)
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