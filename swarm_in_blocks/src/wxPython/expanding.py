#!/usr/bin/python3

"""
ZetCode wxPython tutorial

In this example we create review
layout with wx.FlexGridSizer.

author: Jan Bodnar
website: www.zetcode.com
last modified: July 2020
"""

import wx
import rospy
from sensor_msgs.msg import Image
import sys
from matplotlib.pyplot import flag
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')



class Example(wx.Frame):

    def __init__(self, parent, title):
        super(Example, self).__init__(parent, title=title)

        self.InitUI()
        self.Centre()
        self.Show()

    def InitUI(self, image):

        sizer = wx.FlexGridSizer(3, 2, 15, 15)
        hbox = wx.BoxSizer(wx.HORIZONTAL)

        panel = wx.Panel(self)
        text = wx.StaticText(panel, label="texto")

        sizer.AddMany([(text)])

        hbox.Add(sizer, proportion=1, flag=wx.ALL | wx.EXPAND, border=15)
        panel.SetSizer(hbox)
      
def handle_image(image):
    # make sure we update in the UI thread
    #wx.CallAfter(ex.InitUI, image)
    # http://wiki.wxpython.org/LongRunningTasks
    
    
    return image

def ImageSubscriber(image):
    app = wx.App()
    ex = Example(None, title='ROS VISUALIZATION')
    ex.Show()
    app.MainLoop()


def main():

    

    rospy.init_node('ImageView')
    rospy.Subscriber('/clover0/main_camera/image_raw', Image, ImageSubscriber)


if __name__ == '__main__':
    main()