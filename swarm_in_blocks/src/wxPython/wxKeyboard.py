import wx

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(200,100))

        self.panel = wx.Panel(self, wx.ID_ANY)
        self.panel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.panel.Bind(wx.EVT_KEY_UP, self.OneKeyUp)
        self.panel.Bind(wx.EVT_CHAR, self.OnKeyDown)
        self.panel.SetFocus()
        self.Show(True)

    def OnKeyDown(self, event=None):
        #print(event.GetKeyCode())
        key = event.GetKeyCode()
        if key in [wx.WXK_UP]:
            print('UP')
        if key in [wx.WXK_DOWN]:
            print('DOWN')
        if key in [wx.WXK_LEFT]:
            print('LEFT')
        if key in [wx.WXK_RIGHT]:
            print('RIGHT')
        
    def OneKeyUp(self, event=None):
        print('key released')
if __name__ == "__main__":
    app = wx.App(False)
    gui = MainWindow(None, "test")
    app.MainLoop()
