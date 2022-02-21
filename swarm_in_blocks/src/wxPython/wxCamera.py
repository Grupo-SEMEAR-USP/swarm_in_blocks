#!/usr/bin/python3
#/clover0/main_camera/image_raw   #  <--- topic to be subscribed


"""
usage:
rosrun proteus_demo ImageView.py image:=/ATRV/CameraMain
"""
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Image

import wx
import sys

class ImageViewApp(wx.App):
    def OnInit(self):
        self.frame = wx.Frame(None, title = "ROS Image View", size = (256, 256))
        self.panel = ImageViewPanel(self.frame)
        self.frame.Show(True)
        return True

class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def update(self, image):
        # http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html
        if not hasattr(self, 'staticbmp'):
            self.staticbmp = wx.StaticBitmap(self)
            frame = self.GetParent()
            frame.SetSize((image.width, image.height))
        if image.encoding == 'rgba8':
            bmp = wx.BitmapFromBufferRGBA(image.width, image.height, image.data)
            self.staticbmp.SetBitmap(bmp)
        elif image.encoding == 'rgb8':
            bmp = wx.BitmapFromBuffer(image.width, image.height, image.data)
            self.staticbmp.SetBitmap(bmp)

def handle_image(image):
    # make sure we update in the UI thread
    wx.CallAfter(wx.GetApp().panel.update, image)
    # http://wiki.wxpython.org/LongRunningTasks

def main(argv):
    app = ImageViewApp()
    rospy.init_node('ImageView')
    rospy.Subscriber('/clover0/main_camera/image_raw', Image, handle_image)
    print(__doc__)
    app.MainLoop()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))