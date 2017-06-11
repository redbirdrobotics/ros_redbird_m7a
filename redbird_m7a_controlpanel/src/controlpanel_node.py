#!/usr/bin/env python

"""Redbird Robotics GUI"""
import wx
import random
import rospy
import time
import math
from geometry_msgs.msg import PoseStamped, TwistStamped

# The frame contains the parent panels and its children so the app can interact with it
class RedbirdFrame(wx.Frame):

    def __init__(self, parent):
        super(RedbirdFrame, self).__init__(parent)
        # This properly "destroys" the GUI when closed
        self.Bind(wx.EVT_CLOSE, self.destroyWindow)

        # Redbird icon for title bar
        ico = wx.Icon('redbird.ico', wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        # SetDoubleBuffered ensures the background properly resets
        self.SetDoubleBuffered(True)

        self.initUI()

    # This properly "destroys" the GUI process when closed
    def destroyWindow(self, event):
        self.Destroy();

    # Initialize the user interface of the frame
    def initUI(self):
        # Create the redbirdPanel with its widgets already there
        redbirdPanel = RedbirdPanel(self)

        self.SetTitle('Redbird Robotics Control Panel')
        self.Centre()  # Guess wxPython creators were British...
        self.Show(True)


# The RedbirdPanel is where most of the layout and widgets are handled
class RedbirdPanel(wx.Panel):
    def __init__(self, parent):
        super(RedbirdPanel, self).__init__(parent)
        print "init RedbirdPanel"

        # Use a GridBagSizer for a dynamic layout of widgets
        # Add widgets here
        gbs = wx.GridBagSizer(5, 5)

        # Title
        title = wx.StaticText(self, label="Redbird Control Panel", style=wx.TRANSPARENT_WINDOW)
        titleFont = wx.Font(18, wx.DECORATIVE, wx.ITALIC, wx.NORMAL)
        title.SetForegroundColour('red')
        title.SetBackgroundColour('white')
        title.SetFont(titleFont)
        gbs.Add(title, pos=(0, 0), span=(1, 5), flag=wx.LEFT | wx.RIGHT | wx.ALIGN_CENTER_HORIZONTAL, border=5)

        # Top Buttons Panel
        topButtonsPanel = TopButtonsPanel(self)
        gbs.Add(topButtonsPanel, pos=(2, 0), span=(1, 5), flag=wx.ALIGN_CENTER_HORIZONTAL)

        # Flight Info Panel
        flightInfoPanel = FlightInfoPanel(self)
        gbs.Add(flightInfoPanel, pos=(4, 0), span=(1, 5), flag=wx.ALIGN_CENTER_HORIZONTAL)

        # Kill Button
        button = wx.Button(self, id=-1, label="Kill")
        gbs.Add(button, pos=(6, 0), span=(1,5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        button.Bind(wx.EVT_BUTTON, self.killClicked)

        # These lines make the GradBagSizer resize windows properly
        gbs.AddGrowableCol(gbs.GetEffectiveColsCount() - 1)  # This took forever to figure out :/
        gbs.AddGrowableRow(gbs.GetEffectiveRowsCount() - 1)
        print "rows:", gbs.GetEffectiveRowsCount()
        print "columns:", gbs.GetEffectiveColsCount()
        self.SetSizerAndFit(gbs)

        # For the background
        self.SetBackgroundStyle(wx.BG_STYLE_ERASE)
        self.bmp = wx.Bitmap("redbirdlogo.png")
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.onEraseBackground)

    # This properly updates the background when the size of the window changes
    def onEraseBackground(self, evt):
        dc = evt.GetDC()

        if not dc:
            dc = wx.ClientDC(self)
            rect = self.GetUpdateRegion().GetBox()
            dc.SetClippingRect(rect)
        dc.Clear()

        sizeObj = self.GetSize()
        newHeight = sizeObj.height
        newWidth = sizeObj.width
        img = wx.Image("redbirdlogo.png", wx.BITMAP_TYPE_ANY)
        img = img.Scale(newWidth, newHeight)
        self.bmp = img.ConvertToBitmap()
        dc.DrawBitmap(self.bmp, 0, 0)

    def killClicked(self, event):
        print "Clicked kill button"


class TopButtonsPanel(wx.Panel):
    def __init__(self, parent):
        super(TopButtonsPanel, self).__init__(parent)#, style=wx.SIMPLE_BORDER | wx.ALIGN_CENTER)
        print "init topButtonsPanel"

        self.SetBackgroundColour('white')
        buttonGBS = wx.GridBagSizer(5, 5)

        # Start Button
        startButton = wx.Button(self, id=-1, label="Start")
        buttonGBS.Add(startButton, pos=(0, 0), flag=wx.ALL, border=1)
        startButton.Bind(wx.EVT_BUTTON, self.startClicked)

        # Stop Button
        stopButton = wx.Button(self, id=-1, label="Stop")
        buttonGBS.Add(stopButton, pos=(0, 1), flag=wx.TOP | wx.BOTTOM, border=1)
        stopButton.Bind(wx.EVT_BUTTON, self.stopClicked)

        # Land Button
        landButton = wx.Button(self, id=-1, label="Land")
        buttonGBS.Add(landButton, pos=(0, 2), flag=wx.TOP | wx.BOTTOM, border=1)
        landButton.Bind(wx.EVT_BUTTON, self.landClicked)

        # Hold Button
        holdButton = wx.Button(self, id=-1, label="Hold")
        buttonGBS.Add(holdButton, pos=(0, 3), flag=wx.TOP | wx.BOTTOM, border=1)
        holdButton.Bind(wx.EVT_BUTTON, self.holdClicked)

        # Home Button
        homeButton = wx.Button(self, id=-1, label="Home")
        buttonGBS.Add(homeButton, pos=(0, 4), flag=wx.TOP | wx.BOTTOM, border=1)
        homeButton.Bind(wx.EVT_BUTTON, self.homeClicked)

        buttonGBS.AddGrowableCol(buttonGBS.GetEffectiveColsCount() - 1)
        buttonGBS.AddGrowableRow(buttonGBS.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(buttonGBS)

    # This method is called when the Start button is clicked
    def startClicked(self, event):
        print "Clicked start button"

    # This method is called when the Stop button is clicked
    def stopClicked(self, event):
        print "Clicked stop button"

    # This method is called when the Land button is clicked
    def landClicked(self, event):
        print "Clicked land button"

    def holdClicked(self, event):
        print "Clicked hold button"

    def homeClicked(self, event):
        print "Clicked home button"


# Flight Info Panel to store Flight Information
# The Flight Info Panel will be store within the RedbirdPanel
class FlightInfoPanel(wx.Panel):
    def __init__(self, parent):
        super(FlightInfoPanel, self).__init__(parent, style=wx.SIMPLE_BORDER)
        print "init FlightInfoPanel"

	rospy.init_node("rcp", anonymous=True)
        velocitySubscriber = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.updateVelocity)
	altitudeSubscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.updateAltitude)


        self.SetBackgroundColour('white')
        self.SetFont(wx.Font(pointSize=14, family=wx.ROMAN, style=wx.NORMAL, weight=wx.FONTWEIGHT_NORMAL, underline=False))
        # Box Sizer to store Flight Info in the Flight Info Panel
        box = wx.GridBagSizer(5, 5)
        
        # Dividing lines to make the Flight Info Panel prettier
        line = wx.StaticLine(self)
        line2 = wx.StaticLine(self)
        line3 = wx.StaticLine(self)
        line4 = wx.StaticLine(self)
        line5 = wx.StaticLine(self)

        # Velocity
        self.velocityLabel = wx.StaticText(self, label="Velocity")
        self.velocity = "KEEP THIS LONG"; # Required for when the text changes
        self.liveVelocity = wx.StaticText(self, label=self.velocity)
        box.Add(self.velocityLabel, pos=(0, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveVelocity, pos=(0, 1), flag=wx.TOP, border=5)

        box.Add(line, pos=(1, 0), span=(1, 2), flag=wx.EXPAND)

        # Altitude
        self.altitudeLabel = wx.StaticText(self, label="Altitude")
        self.altitude = "0";
        self.liveAltitude = wx.StaticText(self, label=self.altitude)
        box.Add(self.altitudeLabel, pos=(2, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveAltitude, pos=(2, 1), flag=wx.ALL)

        box.Add(line2, pos=(3, 0), span=(1, 2), flag=wx.EXPAND)

        # Yaw
        self.yawLabel = wx.StaticText(self, label="Yaw")
        self.yaw = "0";
        self.liveYaw = wx.StaticText(self, label=self.yaw)
        box.Add(self.yawLabel, pos=(4, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveYaw, pos=(4, 1), flag=wx.ALL)

        box.Add(line3, pos=(5, 0), span=(1, 2), flag=wx.EXPAND)

        # Pitch
        self.pitchLabel = wx.StaticText(self, label="Pitch")
        self.pitch = "0";
        self.livePitch = wx.StaticText(self, label=self.pitch)
        box.Add(self.pitchLabel, pos=(6, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.livePitch, pos=(6, 1), flag=wx.ALL)

        box.Add(line4, pos=(7, 0), span=(1, 2), flag=wx.EXPAND)

        # Roll
        self.rollLabel = wx.StaticText(self, label="Roll")
        self.roll = "0";
        self.liveRoll = wx.StaticText(self, label=self.roll)
        box.Add(self.rollLabel, pos=(8, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveRoll, pos=(8, 1), flag=wx.ALL)

        box.Add(line5, pos=(9, 0), span=(1, 2), flag=wx.EXPAND)

        box.AddGrowableCol(box.GetEffectiveColsCount() - 1)
        box.AddGrowableRow(box.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(box)

    # Referece http://wiki.ros.org/mavros for what each msg object is returning
    def updateVelocity(self, msg):
        xVelocity = msg.twist.linear.x
        yVelocity = msg.twist.linear.y
        zVelocity = msg.twist.linear.z
        velocity = math.sqrt(math.pow(xVelocity, 2) + math.pow(yVelocity, 2) + math.pow(zVelocity, 2))
        wx.CallAfter(self.liveVelocity.SetLabel, (str(velocity) + ""))

    def updateAltitude(self, msg):
        wx.CallAfter(self.liveAltitude.SetLabel, (str(msg.pose.position.z) + ""))

    def updateYaw(self, event):
        self.liveYaw.SetLabel(str(random.randint(1, 40)) + " deg/sec")

    def updatePitch(self, event):
        self.livePitch.SetLabel(str(random.randint(1, 40)) + " degrees")

    def updateRoll(self, event):
        self.liveRoll.SetLabel(str(random.randint(1, 40)) + " degrees")

# Customizes the default wxPython App object
class App(wx.App):

    # When the App object is initialized, this constructs the frame
    # note: OnInit is a special wxPython
    def OnInit(self):
        self.frame = RedbirdFrame(None)  # pass parent=None into Frame
        self.SetTopWindow(self.frame)
        self.frame.Show(True)

        return True  # True indicates that processing should continue after initialization


# This if statement allows the script to run only from interpreter and not if
# this class is imported
if __name__ == '__main__':
    app = App()
    app.MainLoop()
