from matplotlib.pyplot import eventplot
import wx
import logging as log

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(200,100))

        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_KEY_UP, self.OnKeyDown)
        self.Bind(wx.EVT_CHAR, self.OnKeyDown)
        self.SetFocus()
        self.Show(True)

    def OnKeyDown(self, event=None):
        print(event)
        log.warning("OnKeyDown event %s" % (event))
        print('down')

if __name__ == "__main__":
    app = wx.App(False)
    gui = MainWindow(None, "test")
    app.MainLoop()