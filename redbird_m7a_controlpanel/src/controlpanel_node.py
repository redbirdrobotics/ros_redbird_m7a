#!/usr/bin/env python

"""Redbird Robotics GUI"""
import wx
import random
import rospy
import time
import math
import tf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from redbird_m7a_msgs.msg import FlightState
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Log
from redbird_m7a_msgs.srv import GetFlights, StartFlight
import wx.lib.scrolledpanel
import string
import wx.grid

maxWidth = 0 # Use maxWidth to use in the full length Ros Logger Panel

# The frame contains the parent panels and its children so the app can interact with it
class RedbirdFrame(wx.Frame):
    def __init__(self, parent):
        super(RedbirdFrame, self).__init__(parent, pos=wx.DefaultPosition, size=(1500, 1000), style=wx.DEFAULT_FRAME_STYLE)
        # This properly "destroys" the GUI when closed
        self.Bind(wx.EVT_CLOSE, self.destroyWindow)

        # Redbird icon for title bar
        #ico = wx.Icon('redbird.ico', wx.BITMAP_TYPE_ICO)
        #self.SetIcon(ico)

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
        #global maxWidth
        #maxWidth = sizeObject.widthT

        # Create the redbirdPanel with its three rows of info there
        redbirdPanel = RedbirdPanel(self)

        self.SetTitle('Redbird Robotics Control Panel')
        self.Centre()  # Guess wxPython creators were British...
        self.Show(True)

# The RedbirdPanel is where most of the layout and widgets are handled
class RedbirdPanel(wx.Panel):
    def __init__(self, parent):
        super(RedbirdPanel, self).__init__(parent, size=(-1,-1))
        print "init RedbirdPanel"

        # Use a GridBagSizer for a dynamic layout of widgets
        # Add widgets here
        gbs = wx.GridBagSizer(5, 5)

        """
        # BEGIN MIDDLE INFO ROW
        # Localization Info
        """
        self.localizationGrid = wx.grid.Grid(self)
        """
        self.localizationGrid.CreateGrid(14, 3)
        self.localizationGrid.SetColLabelValue(0, "Found")
        self.localizationGrid.SetColLabelValue(1, "Color")
        self.localizationGrid.SetColLabelValue(2, "Position")

        # Simulation Info
        """
        self.simulationGrid = wx.grid.Grid(self)
        """
        self.simulationGrid.CreateGrid(14, 2)
        self.simulationGrid.SetColLabelValue(0, "Color")
        self.simulationGrid.SetColLabelValue(1, "Confidence")

        # self.SetFound(2, "True")

        self.localizationGrid.AutoSize()
        self.simulationGrid.AutoSize()


        # Draw Robot Positions Grid Panel
        """
        self.drawGridPanel = DrawGridPanel(self)
        """
        self.drawGridPanel.Bind(wx.EVT_PAINT, self.redrawGrid)

        topInfoRow = wx.BoxSizer(wx.HORIZONTAL)

        topInfoRow.Add(self.localizationGrid, flag = wx.ALL,border= 1)
        topInfoRow.Add(self.simulationGrid, flag = wx.ALL,border= 1)

        topInfoRow.Add(self.drawGridPanel, wx.ALL, 1)

        gbs.Add(topInfoRow, pos=(0, 0), span=(1, 5), flag=wx.ALIGN_CENTER_HORIZONTAL)
        """

        # END TOP INFO ROW

        # BEGIN MIDDLE INFO ROW
        middleInfoRow = wx.BoxSizer(wx.HORIZONTAL)

        self.vehicle_state_panel = VehicleStatePanel(self)
        self.flight_state_panel = FlightStatePanel(self)
        self.buttons_panel = ButtonsPanel(self)
        middleInfoRow.Add(self.vehicle_state_panel, flag=wx.ALL, border=1)
        middleInfoRow.Add(self.flight_state_panel, flag=wx.ALL, border=1)
        middleInfoRow.Add(self.buttons_panel, flag=wx.ALL, border=1)

        gbs.Add(middleInfoRow, pos= (1,0), span=(1,5), flag=wx.ALIGN_CENTER_HORIZONTAL)
        # END MIDDLE INFO ROW

        # Ros Logger takes up all of bottom (third) row
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

        #####################################################################################################################

        # For the background
        # self.SetBackgroundStyle(wx.BG_STYLE_ERASE)
        # self.bmp = wx.Bitmap("redbirdlogo.png")
        # self.Bind(wx.EVT_ERASE_BACKGROUND, self.onEraseBackground)

        #####################################################################################################################

        # CHANGE THIS TIMER TO ADD THE CORRECT FOUND FUNCTIONALITY========================================================
        # This timer sets changes the found column
        self.found_log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.SetFound, self.found_log_timer)
        self.found_log_timer.Start(1000)

        # CHANGE THIS TIMER TO ADD THE CORRECT COLOR FUNCTIONALITY========================================================
        # This timer changes the color column
        self.color_log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.SetColor, self.color_log_timer)
        self.color_log_timer.Start(1000)

        # CHANGE THIS TIMER TO ADD THE CORRECT POSITION FUNCTIONALITY========================================================
        # This timer changes the position column
        self.position_log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.SetPosition, self.position_log_timer)
        self.position_log_timer.Start(1000)

        # CHANGE THIS TIMER TO ADD THE CORRECT CONFIDENCE FUNCTIONALITY========================================================
        # This timer changes the confidence column
        self.confidence_log_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.SetConfidence, self.confidence_log_timer)
        self.confidence_log_timer.Start(1000)

    # This sets the element in the found column to either true or false
    def SetFound(self, event): # row, tOrFString): UNCOMMENT THIS TO INCLUDE ROW AND TRUE/FALSE STRING
        randomRow = random.randint(0,13)
        randomInt = random.randint(1,2)
        if randomInt == 1:
            randomTrueOrFalse = "True"
        else:
            randomTrueOrFalse = "False"
        self.localizationGrid.SetCellValue(randomRow, 0, randomTrueOrFalse)

    # This sets the element in the color column to either green or red
    def SetColor(self, event):  # row, tOrFString): UNCOMMENT THIS TO INCLUDE ROW AND TRUE/FALSE STRING
        randomInt = random.randint(1, 2)
        randomRow = random.randint(0, 13)
        if randomInt == 1:
            color = "Green"
        else:
            color = "Red"
        self.localizationGrid.SetCellValue(randomRow, 1, color)
        self.simulationGrid.SetCellValue(randomRow, 0, color)

    # This sets the element in the confidence column
    def SetConfidence(self, event):  # row, tOrFString): UNCOMMENT THIS TO INCLUDE ROW AND TRUE/FALSE STRING
        randomConfidence = random.randint(0, 100)
        randomRow = random.randint(0, 13)
        self.simulationGrid.SetCellValue(randomRow, 1, str(randomConfidence) + "%")

    def SetPosition(self, event):  # row, tOrFString): UNCOMMENT THIS TO INCLUDE ROW AND TRUE/FALSE STRING
        randomPos = random.randint(0, 200)
        randomRow = random.randint(0, 13)
        self.localizationGrid.SetCellValue(randomRow, 2, str(randomPos))

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
        super(ButtonsPanel, self).__init__(parent)# , style=wx.SIMPLE_BORDER | wx.ALIGN_CENTER)
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

        ###########################################################################################################################

        # Wait for service startup
        rospy.wait_for_service('/redbird/flight_director/get_flights')
        rospy.wait_for_service('/redbird/flight_director/start_flight')

        # Setup services
        self._get_flights_serv = rospy.ServiceProxy('/redbird/flight_director/get_flights', GetFlights)
        self._start_flights_serv = rospy.ServiceProxy('/redbird/flight_director/start_flight', StartFlight)

        ###########################################################################################################################

        self.flight_names = self._get_flights_serv(Empty()).flights

        # print "flight names == " + flight_names
        self.startDropDown = wx.Choice(self, choices=self.flight_names, name="Available flights")
        self.startDropDown.SetSelection(0)

        # buttonGBS.Add(startButton, pos=(0, 0), flag=wx.ALL, border=1)
        buttonGBS.Add(self.startDropDown, pos=(0, 0), span=(1, 5), flag=wx.ALL, border=1)
        self.startDropDown.Bind(wx.EVT_CHOICE, self.chooseFlightScript)
        # startButton.Bind(wx.EVT_BUTTON, self.startClicked)

        # Start Button
        startButton = wx.Button(self, id=-1, label="Start")
        buttonGBS.Add(startButton, pos=(1, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        startButton.Bind(wx.EVT_BUTTON, self.startClicked)

        # Stop/Land Button
        landButton = wx.Button(self, id=-1, label="Stop/Land")
        """
        buttonGBS.Add(landButton, pos=(2, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        landButton.Bind(wx.EVT_BUTTON, self.landClicked)
        """

        # Kill Button
        button = wx.Button(self, id=-1, label="Kill")
        """
        buttonGBS.Add(button, pos=(3, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        button.Bind(wx.EVT_BUTTON, self.killClicked)
        """

        # Refresh Button
        button = wx.Button(self, id=-1, label="Refresh")
        buttonGBS.Add(button, pos=(2, 0), span=(1, 5), flag=wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_HORIZONTAL, border=1)
        button.Bind(wx.EVT_BUTTON, self.refreshClicked)

        buttonGBS.AddGrowableCol(buttonGBS.GetEffectiveColsCount() - 1)
        buttonGBS.AddGrowableRow(buttonGBS.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(buttonGBS)

        self.grabData()

    # This method is called when the user selects a flight script
    def chooseFlightScript(self, event):
        selection = self.startDropDown.GetString(self.startDropDown.GetSelection())
        # Make sure the flight suggestion from the dropdown menu wasn't accidentally selected
        if selection != "---FLIGHT---":
            print "You chose " + selection

    # This method is called when the Start button is clicked
    def startClicked(self, event):
        print self.startDropDown.GetString(self.startDropDown.GetSelection())
        print self._start_flights_serv(self.startDropDown.GetString(self.startDropDown.GetSelection())) 

    # This method is called when the Stop button is clicked
    def stopClicked(self, event):
        print "Clicked stop button"

    # This method is called when the Land button is clicked
    def landClicked(self, event):
        print "Clicked land button"

    # This method is called when the Kill button is clicked
    def killClicked(self, event):
        print "Clicked kill button"

    # This method is called when the Refresh button is clicked
    def refreshClicked(self, event):
        self.flight_names = self._get_flights_serv(Empty()).flights
        self.flight_names = sorted(self.flight_names, key=len)
        self.flight_names.reverse()
        self.startDropDown = wx.Choice(self, choices=self.flight_names, name="Available flights")
        self.startDropDown.SetSelection(0)

    def grabData(self):
        self.flight_names = self._get_flights_serv(Empty()).flights
        self.flight_names = sorted(self.flight_names, key=len)
        self.flight_names.reverse()
        self.startDropDown = wx.Choice(self, choices=self.flight_names, name="Available flights")
        self.startDropDown.SetSelection(0)


# Flight Info Panel to store Flight Information
# The Flight Info Panel will be store within the RedbirdPanel
class VehicleStatePanel(wx.Panel):
    def __init__(self, parent):
        super(VehicleStatePanel, self).__init__(parent)#, style=wx.SIMPLE_BORDER)
        print "init VehicleStatePanel"

        # UNCOMMENT THIS TO GET THE VEHICLE STATE DATA====================================================================
        self.velocitySubscriber = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.updateVelocity)
        self.altitudeAndAttitudeSubscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.updateAltitudeAndAttitude)

        self.SetBackgroundColour('white')
        self.SetFont(
            wx.Font(pointSize=14, family=wx.ROMAN, style=wx.NORMAL, weight=wx.FONTWEIGHT_NORMAL, underline=False))
        # Gridbag Sizer to store Flight Info in the Flight Info Panel
        box = wx.GridBagSizer(5, 5)

        # Dividing lines to make the Flight Info Panel prettier
        line = wx.StaticLine(self)
        line1 = wx.StaticLine(self)
        line2 = wx.StaticLine(self)
        line3 = wx.StaticLine(self)
        line4 = wx.StaticLine(self)
        line5 = wx.StaticLine(self)
        line6 = wx.StaticLine(self)
        line7 = wx.StaticLine(self)

        # Example ros values for startup
        velocity_ros = 23.1234
        velocityX_ros = 23.1234
        velocityY_ros = 23.1234
        velocityZ_ros = 23.1234
        altitude_ros = 23.1234
        yaw_ros = 23.1234
        pitch_ros = 23.1234
        roll_ros = 23.1234

        # Velocity
        self.velocityLabel = wx.StaticText(self, label="Velocity: ")
        self.velocity = str("%07.3f" % velocity_ros) + " m/s"  # Required for when the text changes
        self.liveVelocity = wx.StaticText(self, label=self.velocity)
        box.Add(self.velocityLabel, pos=(0, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveVelocity, pos=(0, 1), flag=wx.TOP, border=5)

        box.Add(line, pos=(1, 0), span=(1, 2), flag=wx.EXPAND)

        # VelocityX
        self.velocityXLabel = wx.StaticText(self, label="Velocity X: ")
        self.velocityX = str("%07.3f" % velocityX_ros) + " m/s"  # Required for when the text changes
        self.liveVelocityX = wx.StaticText(self, label=self.velocityX)
        box.Add(self.velocityXLabel, pos=(2, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveVelocityX, pos=(2, 1), flag=wx.TOP, border=5)

        box.Add(line1, pos=(3, 0), span=(1, 2), flag=wx.EXPAND)

        # VelocityY
        self.velocityYLabel = wx.StaticText(self, label="Velocity Y: ")
        self.velocityY = str("%07.3f" % velocityY_ros) + " m/s"  # Required for when the text changes
        self.liveVelocityY = wx.StaticText(self, label=self.velocityY)
        box.Add(self.velocityYLabel, pos=(4, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveVelocityY, pos=(4, 1), flag=wx.TOP, border=5)

        box.Add(line2, pos=(5, 0), span=(1, 2), flag=wx.EXPAND)

        # VelocityZ
        self.velocityZLabel = wx.StaticText(self, label="Velocity Z: ")
        self.velocityZ = str("%07.3f" % velocityZ_ros) + " m/s"  # Required for when the text changes
        self.liveVelocityZ = wx.StaticText(self, label=self.velocityZ)
        box.Add(self.velocityZLabel, pos=(6, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveVelocityZ, pos=(6, 1), flag=wx.TOP, border=5)

        box.Add(line3, pos=(7, 0), span=(1, 2), flag=wx.EXPAND)

        # Altitude
        self.altitudeLabel = wx.StaticText(self, label="Altitude: ")
        self.altitude = str("%07.3f" % altitude_ros) + " m"
        self.liveAltitude = wx.StaticText(self, label=self.altitude)
        box.Add(self.altitudeLabel, pos=(8, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveAltitude, pos=(8, 1), flag=wx.ALL)

        box.Add(line4, pos=(9, 0), span=(1, 2), flag=wx.EXPAND)

        # Yaw
        self.yawLabel = wx.StaticText(self, label="Yaw: ")
        self.yaw = str("%07.3f" % yaw_ros) + " " + (u'\N{DEGREE SIGN}')
        self.liveYaw = wx.StaticText(self, label=self.yaw)
        box.Add(self.yawLabel, pos=(10, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveYaw, pos=(10, 1), flag=wx.ALL)

        box.Add(line5, pos=(11, 0), span=(1, 2), flag=wx.EXPAND)

        # Pitch
        self.pitchLabel = wx.StaticText(self, label="Pitch: ")
        self.pitch = str("%07.3f" % pitch_ros) + " " + (u'\N{DEGREE SIGN}')
        self.livePitch = wx.StaticText(self, label=self.pitch)
        box.Add(self.pitchLabel, pos=(12, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.livePitch, pos=(12, 1), flag=wx.ALL)

        box.Add(line6, pos=(13, 0), span=(1, 2), flag=wx.EXPAND)

        # Roll
        self.rollLabel = wx.StaticText(self, label="Roll: ")
        self.roll = str("%07.3f" % roll_ros) + " " + (u'\N{DEGREE SIGN}')
        self.liveRoll = wx.StaticText(self, label=self.roll)
        box.Add(self.rollLabel, pos=(14, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveRoll, pos=(14, 1), flag=wx.ALL)

        box.Add(line7, pos=(15, 0), span=(1, 2), flag=wx.EXPAND)

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
        wx.CallAfter(self.liveVelocity.SetLabel, str("{:.2f}".format(velocity)) + " m/s")
        wx.CallAfter(self.liveVelocityX.SetLabel, str("{:.2f}".format(xVelocity)) + " m/s")
        wx.CallAfter(self.liveVelocityY.SetLabel, str("{:.2f}".format(yVelocity)) + " m/s")
        wx.CallAfter(self.liveVelocityZ.SetLabel, str("{:.2f}".format(zVelocity)) + " m/s")

    def updateAltitudeAndAttitude(self, msg):
        # Updates altitude
        wx.CallAfter(self.liveAltitude.SetLabel, str("{:.2f}".format(msg.pose.position.z)) + " m")

        # Updates attitude (yaw, pitch, and roll)
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        roll = math.degrees(euler[0])
        pitch = math.degrees(euler[1])
        yaw = math.degrees(euler[2])
        if yaw >= 0: wx.CallAfter(self.liveYaw.SetLabel, str("{:.2f}".format(yaw)) + " " + (u'\N{DEGREE SIGN}'))  # Yaw
        else: wx.CallAfter(self.liveYaw.SetLabel, str("{:.2f}".format(yaw)) + " " + (u'\N{DEGREE SIGN}'))  # Yaw
        if pitch >= 0: wx.CallAfter(self.livePitch.SetLabel, str("{:.2f}".format(pitch)) + " " + (u'\N{DEGREE SIGN}'))  # Pitch
        else: wx.CallAfter(self.livePitch.SetLabel, str("{:.2f}".format(pitch)) + " " + (u'\N{DEGREE SIGN}'))  # Pitch
        if roll >= 0: wx.CallAfter(self.liveRoll.SetLabel, str("{:.2f}".format(roll)) + " " + (u'\N{DEGREE SIGN}'))  # Roll
        else: wx.CallAfter(self.liveRoll.SetLabel, str("{:.2f}".format(roll)) + " " + (u'\N{DEGREE SIGN}'))  # Roll


class FlightStatePanel(wx.Panel):
    def __init__(self, parent):
        super(FlightStatePanel, self).__init__(parent)#, style=wx.SIMPLE_BORDER)
        print "init FlightStatePanel"

        # UNCOMMENT THIS AND CORRECT THE MAVROS SUBSCRIBER INFO TO GET FLIGHT STATE DATA
        rospy.init_node("rcp", anonymous=True)
        self.flightStateSubscriber = rospy.Subscriber("/redbird/flight_director/flight_state", FlightState, self.updateFlightState)
        self.batterySubscriber = rospy.Subscriber("/mavros/battery", BatteryState, self.updateBattery)

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
        line6 = wx.StaticLine(self)

        # GET THE VARIABLES FROM ROS HERE
        currentFlight_ros = "testflight"
        timeElapsed_ros = 23.1234
        previousTime_ros = 23.1234
        batteryState_ros = 23.1234
        batteryVoltage_ros = 23.1234
        batteryCurrent_ros = 23.1234

        # Current Flight
        self.currentFlightLabel = wx.StaticText(self, label="Current Flight: ")
        self.currentFlight = currentFlight_ros + "\t\t\t"  # Required for when the text changes
        self.liveCurrentFlight = wx.StaticText(self, label=self.currentFlight)
        box.Add(self.currentFlightLabel, pos=(0, 0), span=(1, 1), flag=wx.TOP | wx.LEFT, border=5)
        box.Add(self.liveCurrentFlight, pos=(0, 1), flag=wx.TOP, border=5)

        box.Add(line, pos=(1, 0), span=(1, 2), flag=wx.EXPAND)

        # Time Elapsed
        self.timeElapsedLabel = wx.StaticText(self, label="Time Elapsed: ")
        self.timeElapsed = str("%07.3f" % timeElapsed_ros) + " s"
        self.liveTimeElapsed = wx.StaticText(self, label=self.timeElapsed)
        box.Add(self.timeElapsedLabel, pos=(2, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveTimeElapsed, pos=(2, 1), flag=wx.ALL)

        box.Add(line2, pos=(3, 0), span=(1, 2), flag=wx.EXPAND)

        # Previous Time
        self.previousTimeLabel = wx.StaticText(self, label="Previous Time: ")
        self.previousTime = str("%07.3f" % previousTime_ros) + " s"
        self.livePreviousTime = wx.StaticText(self, label=self.previousTime)
        box.Add(self.previousTimeLabel, pos=(4, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.livePreviousTime, pos=(4, 1), flag=wx.ALL)

        box.Add(line3, pos=(5, 0), span=(1, 2), flag=wx.EXPAND)


        # Battery Percent
        self.batteryStateLabel = wx.StaticText(self, label="Battery Percent: ")
        self.batteryState = str("%07.3f" % batteryState_ros) + " %"
        self.liveBatteryState = wx.StaticText(self, label=self.batteryState)
        box.Add(self.batteryStateLabel, pos=(6, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveBatteryState, pos=(6, 1), flag=wx.ALL)

        box.Add(line4, pos=(7, 0), span=(1, 2), flag=wx.EXPAND)

        # Battery Voltage
        self.batteryVoltageLabel = wx.StaticText(self, label="Battery Voltage: ")
        self.batteryVoltage = str("%07.3f" % batteryVoltage_ros) + " %"
        self.liveBatteryVoltage = wx.StaticText(self, label=self.batteryVoltage)
        box.Add(self.batteryVoltageLabel, pos=(8, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveBatteryVoltage, pos=(8, 1), flag=wx.ALL)

        box.Add(line5, pos=(9, 0), span=(1, 2), flag=wx.EXPAND)

        # Battery Current
        self.batteryCurrentLabel = wx.StaticText(self, label="Battery Current: ")
        self.batteryCurrent = str("%07.3f" % batteryCurrent_ros) + " %"
        self.liveBatteryCurrent = wx.StaticText(self, label=self.batteryCurrent)
        box.Add(self.batteryCurrentLabel, pos=(10, 0), span=(1, 1), flag=wx.LEFT, border=5)
        box.Add(self.liveBatteryCurrent, pos=(10, 1), flag=wx.ALL)

        box.Add(line6, pos=(11, 0), span=(1, 2), flag=wx.EXPAND)

        # Ensures proper sizing
        box.AddGrowableCol(box.GetEffectiveColsCount() - 1)
        box.AddGrowableRow(box.GetEffectiveRowsCount() - 1)
        self.SetSizerAndFit(box)

    # FIX THESE UPDATE FUNCTIONS TO CORRECTLY SET THE LABEL===============================================================
    # Reference http://wiki.ros.org/mavros for what each msg object is returning
    def updateFlightState(self, msg):
        # Updates current flight time
        if msg.is_flying: 
            elapsed = msg.header.stamp.secs - msg.current_flight.start_time
        elif not msg.is_flying and msg.current_flight.start_time > 0:
            elapsed = msg.current_flight.end_time - msg.current_flight.start_time
        else:
            elapsed = 0

        wx.CallAfter(self.liveTimeElapsed.SetLabel, str("{:d}".format(int(elapsed))) + " s") #==================================
        
        # Update flight name
        wx.CallAfter(self.liveCurrentFlight.SetLabel, str(msg.current_flight.name)) #==================================

        # Updates previous flight time
        if msg.previous_flight.start_time > 0:
            prev_elapsed = msg.previous_flight.end_time - msg.previous_flight.start_time
        else: 
            prev_elapsed = 0

        wx.CallAfter(self.livePreviousTime.SetLabel, str("{:d}".format(int(prev_elapsed))) + " s") #==================================
 
    def updateBattery(self, msg):
        # Updates Battery State
        try:
            wx.CallAfter(self.liveBatteryState.SetLabel, str("{:.2f}".format(msg.percentage * 100.0)) + " %") #=======================
            wx.CallAfter(self.liveBatteryVoltage.SetLabel, str("{:.2f}".format(msg.voltage)) + " V") #=======================    
            wx.CallAfter(self.liveBatteryCurrent.SetLabel, str("{:.2f}".format(msg.current)) + " A") #=======================
        except:
            pass        

class RosLoggerPanel(wx.Panel):
    def __init__(self, parent):
        super(RosLoggerPanel, self).__init__(parent) #,size=parent.GetSize(), style=wx.SIMPLE_BORDER)
        print "init RosLoggerPanel"
        global maxWidth # use global maxWidth so the text control can span the whole width
        self.t3 = wx.TextCtrl(self, size=(900,400), style=wx.TE_MULTILINE | wx.EXPAND | wx.TE_READONLY)

        self.flightLogSubscriber = rospy.Subscriber("/rosout", Log, self.addLog)

        # Timer
        # self.log_timer = wx.Timer(self)
        # self.Bind(wx.EVT_TIMER, self.addLog, self.log_timer)
        # self.log_timer.Start(1000)

    # APPEND LOG INFO TO SELF.T3.APPENDTEXT AS A STRING===================================================================
    def addLog(self, msg):
        try:
            self.t3.AppendText(msg.msg + "\n") #=================
        except:
            pass


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
