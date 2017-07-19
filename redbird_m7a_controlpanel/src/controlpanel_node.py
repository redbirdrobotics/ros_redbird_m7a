#!/usr/bin/env python

"""Redbird Robotics GUI"""
import wx
import random
# import rospy
import time
import math
# from geometry_msgs.msg import PoseStamped, TwistStamped
import wx.lib.scrolledpanel
import string
import wx.grid

maxWidth = 0

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
        # Maximize, then get width so that the max width can be applied to Ros Logger Panel
        self.Maximize(True)
        sizeObject = self.GetSize()
        global maxWidth
        maxWidth = sizeObject.width

        # Create the redbirdPanel with its three rows of info there
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

        # # Title
        # title = wx.StaticText(self, label="Redbird Control Panel", style=wx.TRANSPARENT_WINDOW)
        # titleFont = wx.Font(18, wx.DECORATIVE, wx.ITALIC, wx.NORMAL)
        # title.SetForegroundColour('red')
        # title.SetBackgroundColour('white')
        # title.SetFont(titleFont)
        # gbs.Add(title, pos=(0, 0), span=(1, 5), flag=wx.LEFT | wx.RIGHT | wx.TOP | wx.ALIGN_CENTER_HORIZONTAL,
        #         border=10)

        # Localization Info
        self.localizationGrid = wx.grid.Grid(self)
        self.localizationGrid.CreateGrid(14, 3)
        self.localizationGrid.SetColLabelValue(0, "Found")
        self.localizationGrid.SetColLabelValue(1, "Color")
        self.localizationGrid.SetColLabelValue(2, "Position")

        # Simulation Info
        self.simulationGrid = wx.grid.Grid(self)
        self.simulationGrid.CreateGrid(14, 2)
        self.simulationGrid.SetColLabelValue(0, "Color")
        self.simulationGrid.SetColLabelValue(1, "Confidence")

        self.SetFound(2)

        self.localizationGrid.SetCellValue(0, 0, "WORK")
        self.localizationGrid.AutoSize()
        self.simulationGrid.AutoSize()

        # Draw Robot Positions Grid Panel
        self.drawGridPanel = DrawGridPanel(self)
        self.drawGridPanel.Bind(wx.EVT_PAINT, self.redrawGrid)

        topInfoRow = wx.BoxSizer(wx.HORIZONTAL)

        topInfoRow.Add(self.localizationGrid, flag = wx.ALL,border= 1)
        topInfoRow.Add(self.simulationGrid, flag = wx.ALL,border= 1)

        topInfoRow.Add(self.drawGridPanel, wx.ALL, 1)

        gbs.Add(topInfoRow, pos=(0, 0), span=(1, 5), flag=wx.ALIGN_CENTER_HORIZONTAL)

        middleInfoRow = wx.BoxSizer(wx.HORIZONTAL)

        self.vehicle_state_panel = VehicleStatePanel(self)
        self.flight_state_panel = FlightStatePanel(self)
        self.buttons_panel = ButtonsPanel(self)
        middleInfoRow.Add(self.vehicle_state_panel, flag=wx.ALL, border=1)
        middleInfoRow.Add(self.flight_state_panel, flag=wx.ALL, border=1)
        middleInfoRow.Add(self.buttons_panel, flag=wx.ALL, border=1)

        gbs.Add(middleInfoRow, pos= (1,0), span=(1,5), flag=wx.ALIGN_CENTER_HORIZONTAL)

        # Ros Logger takes up all of bottom
        self.ros_logger_panel = RosLoggerPanel(self)
        gbs.Add(self.ros_logger_panel, pos=(2, 0), span=(1, 5), flag=wx.ALIGN_CENTER_HORIZONTAL | wx.EXPAND)


        # This timer refreshes the robot position drawing
        self.drawGridTimer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.refreshWindow, self.drawGridTimer)
        self.drawGridTimer.Start(1000)

        # These lines make the GridBagSizer resize windows properly
        gbs.AddGrowableCol(gbs.GetEffectiveColsCount() - 1)  # This took forever to figure out :/
        gbs.AddGrowableRow(gbs.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(gbs)
        self.Layout()

        # For the background
        self.SetBackgroundStyle(wx.BG_STYLE_ERASE)
        self.bmp = wx.Bitmap("redbirdlogo.png")
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.onEraseBackground)

    def SetFound(self, row):
        self.localizationGrid.SetCellValue(row, 0, "True")

    def refreshWindow(self, event):
        self.drawGridPanel.Refresh()

    def redrawGrid(self, event):
        dc = wx.PaintDC(self.drawGridPanel)
        for i in range(20):
            dc.DrawLine(0, i * 20, 400, i * 20)
            dc.DrawLine(i * 20, 0, i * 20, 400)

        dc.SetBrush(wx.Brush(wx.Colour(255, 0, 0)))
        for i in range(5):
            xRand = random.randint(0, 20)
            yRand = random.randint(0, 20)
            dc.DrawRectangle(xRand * 20, yRand * 20, 20, 20)

        dc.SetBrush(wx.Brush(wx.Colour(0, 255, 0)))
        for i in range(5):
            xRand = random.randint(0, 20)
            yRand = random.randint(0, 20)
            dc.DrawRectangle(xRand * 20, yRand * 20, 20, 20)

        dc.SetBrush(wx.Brush(wx.Colour(128, 0, 128)))
        for i in range(4):
            xRand = random.randint(0, 20)
            yRand = random.randint(0, 20)
            dc.DrawRectangle(xRand * 20, yRand * 20, 20, 20)

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



# This is the panel where the robot positions are drawn on
class DrawGridPanel(wx.Panel):
    def __init__(self, parent):
        super(DrawGridPanel, self).__init__(parent, size=(400, 400))
        print "init DrawGridPanel"


class ButtonsPanel(wx.Panel):
    def __init__(self, parent):
        super(ButtonsPanel, self).__init__(parent)  # , style=wx.SIMPLE_BORDER | wx.ALIGN_CENTER)
        print "init buttonsPanel"

        self.SetBackgroundColour('white')
        buttonGBS = wx.GridBagSizer(5, 5)

        # Start Button
        # startButton = wx.Button(self, id=-1, label="Start")

        # Ensure the services have fully started up
        # rospy.wait_for_service('get_flights')

        # # Get flight script names
        # get_flights_srv = rospy.ServiceProxy('get_flights', GetFlights)
        # flight_names = get_flight_srv(empty=[])

        flight_names = ['---FLIGHT---', 'test_flight1', 'test_flight2']
        # print "flight names == " + flight_names
        self.startDropDown = wx.Choice(self, choices=flight_names, name="Available flights")
        self.startDropDown.SetSelection(0)

        # buttonGBS.Add(startButton, pos=(0, 0), flag=wx.ALL, border=1)
        buttonGBS.Add(self.startDropDown, pos=(0, 0), span=(1, 5), flag=wx.ALL, border=1)
        self.startDropDown.Bind(wx.EVT_CHOICE, self.chooseFlightScript)
        # startButton.Bind(wx.EVT_BUTTON, self.startClicked)

        # Stop Button
        stopButton = wx.Button(self, id=-1, label="Stop/Land")
        buttonGBS.Add(stopButton, pos=(1, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        stopButton.Bind(wx.EVT_BUTTON, self.stopClicked)

        # Land Button
        landButton = wx.Button(self, id=-1, label="Land")
        buttonGBS.Add(landButton, pos=(2, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        landButton.Bind(wx.EVT_BUTTON, self.landClicked)

        # Hold Button
        holdButton = wx.Button(self, id=-1, label="Hold")
        buttonGBS.Add(holdButton, pos=(3, 0),  span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        holdButton.Bind(wx.EVT_BUTTON, self.landClicked)

        # Kill Button
        button = wx.Button(self, id=-1, label="Kill")
        buttonGBS.Add(button, pos=(4, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        button.Bind(wx.EVT_BUTTON, self.killClicked)



        buttonGBS.AddGrowableCol(buttonGBS.GetEffectiveColsCount() - 1)
        buttonGBS.AddGrowableRow(buttonGBS.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(buttonGBS)

    # This method is called when the user selects a flight script
    def chooseFlightScript(self, event):
        selection = self.startDropDown.GetString(self.startDropDown.GetSelection())
        # Make sure the flight suggestion from the dropdown menu wasn't accidentally selected
        if selection != "---FLIGHT---":
            print "You chose " + selection

    # This method is called when the Start button is clicked
    def startClicked(self, event):
        print "Clicked start button"

    # This method is called when the Stop button is clicked
    def stopClicked(self, event):
        print "Clicked stop button"

    # This method is called when the Land button is clicked
    def landClicked(self, event):
        print "Clicked land button"

    # This method is called when the Kill button is clicked
    def killClicked(self, event):
        print "Clicked kill button"



# Flight Info Panel to store Flight Information
# The Flight Info Panel will be store within the RedbirdPanel
class VehicleStatePanel(wx.Panel):
    def __init__(self, parent):
        super(VehicleStatePanel, self).__init__(parent, style=wx.SIMPLE_BORDER)
        print "init VehicleStatePanel"
        # rospy.init_node("rcp", anonymous=True)
        # velocitySubscriber = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.updateVelocity)
        # altitudeAndAttitudeSubscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
        #                                                  self.updateAltitudeAndAttitude)

        self.SetBackgroundColour('white')
        self.SetFont(
            wx.Font(pointSize=14, family=wx.ROMAN, style=wx.NORMAL, weight=wx.FONTWEIGHT_NORMAL, underline=False))
        # Gridbag Sizer to store Flight Info in the Flight Info Panel
        box = wx.GridBagSizer(5, 5)

        # Dividing lines to make the Flight Info Panel prettier
        line = wx.StaticLine(self)
        line2 = wx.StaticLine(self)
        line3 = wx.StaticLine(self)
        line4 = wx.StaticLine(self)
        line5 = wx.StaticLine(self)

        # Velocity
        self.velocityLabel = wx.StaticText(self, label="Velocity")
        self.velocity = "KEEP THIS LONG";  # Required for when the text changes
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

        # Ensures proper sizing
        box.AddGrowableCol(box.GetEffectiveColsCount() - 1)
        box.AddGrowableRow(box.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(box)

    # Reference http://wiki.ros.org/mavros for what each msg object is returning
    def updateVelocity(self, msg):
        # Calculates the magnitude of velocity
        xVelocity = msg.twist.linear.x
        yVelocity = msg.twist.linear.y
        zVelocity = msg.twist.linear.z
        velocity = math.sqrt(math.pow(xVelocity, 2) + math.pow(yVelocity, 2) + math.pow(zVelocity, 2))
        wx.CallAfter(self.liveVelocity.SetLabel, str("%.3f" % velocity) + " m/s")

    def updateAltitudeAndAttitude(self, msg):
        # Updates altitude
        wx.CallAfter(self.liveAltitude.SetLabel, str("%.3f" % msg.pose.position.z) + " m")

        # Updates attitude (yaw, pitch, and roll(
        wx.CallAfter(self.liveYaw.SetLabel, str("%.3f" % msg.pose.orientation.z) + " deg")  # Yaw
        wx.CallAfter(self.livePitch.SetLabel, str("%.3f" % msg.pose.orientation.y) + " deg")  # Pitch
        wx.CallAfter(self.liveRoll.SetLabel, str("%.3f" % msg.pose.orientation.x) + " deg")  # Roll

# ============================================================================================

class FlightStatePanel(wx.Panel):
    def __init__(self, parent):
        super(FlightStatePanel, self).__init__(parent, style=wx.SIMPLE_BORDER)
        print "init FlightStatePanel"
        # rospy.init_node("rcp", anonymous=True)
        # velocitySubscriber = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.updateVelocity)
        # altitudeAndAttitudeSubscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
        #                                                  self.updateAltitudeAndAttitude)

        self.SetBackgroundColour('white')
        self.SetFont(
            wx.Font(pointSize=14, family=wx.ROMAN, style=wx.NORMAL, weight=wx.FONTWEIGHT_NORMAL, underline=False))
        # Gridbag Sizer to store Flight Info in the Flight Info Panel
        box = wx.GridBagSizer(5, 5)

        # Dividing lines to make the Flight Info Panel prettier
        line = wx.StaticLine(self)
        line2 = wx.StaticLine(self)
        # line3 = wx.StaticLine(self)
        # line4 = wx.StaticLine(self)
        # line5 = wx.StaticLine(self)

        # Current Flight
        self.currentFlightLabel = wx.StaticText(self, label="Current Flight")
        self.currentFlight = "KEEP THIS LONG";  # Required for when the text changes
        self.liveCurrentFlight = wx.StaticText(self, label=self.currentFlight)
        box.Add(self.currentFlightLabel, pos=(0, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveCurrentFlight, pos=(0, 1), flag=wx.TOP, border=5)

        box.Add(line, pos=(1, 0), span=(1, 2), flag=wx.EXPAND)

        # Time Elapsed
        self.timeElapsedLabel = wx.StaticText(self, label="Time Elapsed")
        self.timeElapsed = "0";
        self.liveTimeElapsed = wx.StaticText(self, label=self.timeElapsed)
        box.Add(self.timeElapsedLabel, pos=(2, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveTimeElapsed, pos=(2, 1), flag=wx.ALL)

        box.Add(line2, pos=(3, 0), span=(1, 2), flag=wx.EXPAND)

        # Yaw
        self.batteryStateLabel = wx.StaticText(self, label="Battery State")
        self.batteryState = "0";
        self.liveBatteryState = wx.StaticText(self, label=self.batteryState)
        box.Add(self.batteryStateLabel, pos=(4, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveBatteryState, pos=(4, 1), flag=wx.ALL)

        # Ensures proper sizing
        box.AddGrowableCol(box.GetEffectiveColsCount() - 1)
        box.AddGrowableRow(box.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(box)

    # # Reference http://wiki.ros.org/mavros for what each msg object is returning
    # def updateVelocity(self, msg):
    #     # Calculates the magnitude of velocity
    #     xVelocity = msg.twist.linear.x
    #     yVelocity = msg.twist.linear.y
    #     zVelocity = msg.twist.linear.z
    #     velocity = math.sqrt(math.pow(xVelocity, 2) + math.pow(yVelocity, 2) + math.pow(zVelocity, 2))
    #     wx.CallAfter(self.liveVelocity.SetLabel, str("%.3f" % velocity) + " m/s")
    #
    # def updateAltitudeAndAttitude(self, msg):
    #     # Updates altitude
    #     wx.CallAfter(self.liveAltitude.SetLabel, str("%.3f" % msg.pose.position.z) + " m")
    #
    #     # Updates attitude (yaw, pitch, and roll(
    #     wx.CallAfter(self.liveYaw.SetLabel, str("%.3f" % msg.pose.orientation.z) + " deg")  # Yaw
    #     wx.CallAfter(self.livePitch.SetLabel, str("%.3f" % msg.pose.orientation.y) + " deg")  # Pitch
    #     wx.CallAfter(self.liveRoll.SetLabel, str("%.3f" % msg.pose.orientation.x) + " deg")  # Roll






# ============================================================================================

class RosLoggerPanel(wx.Panel):
    def __init__(self, parent):
        super(RosLoggerPanel, self).__init__(parent, style=wx.SIMPLE_BORDER)
        print "init RosLoggerPanel"
        global maxWidth
        print "maxWidth == " + str(maxWidth)
        self.t3 = wx.TextCtrl(self, size= (maxWidth, 170), style=wx.TE_MULTILINE | wx.EXPAND)

        # Timers
        self.log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.addLog, self.log_timer)
        self.log_timer.Start(1000)

    def addLog(self, evt):
        digits = "".join([random.choice(string.digits) for i in xrange(8)])
        chars = "".join([random.choice(string.letters) for i in xrange(15)])
        self.t3.AppendText("\nLOG: " + digits + chars + "MOOO")


# Customizes the default wxPython App object
class App(wx.App):
    # When the App object is initialized, this constructs the frame
    # note: OnInit is a special wxPython initialization constructor
    def OnInit(self):
        self.frame = RedbirdFrame(None)  # pass parent=None into Frame
        self.SetTopWindow(self.frame)
        self.frame.Show(True)

        return True  # True indicates that processing should continue after initialization


# This if statement allows the script to run only from interpreter and not if
# this class is imported
if __name__ == '__main__':
    app = App(0)
    app.MainLoop()
