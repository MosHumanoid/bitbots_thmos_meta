import numpy as np
import math
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox, \
    QInputDialog, QShortcut
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QPixmap, QTransform

import os
import rospkg, rospy
import time
import multiprocessing as mp
import yaml

from label_pool import BallPool, RobotPool, CrossPool, OpponentPool, UndefinedPool, TeammatePool, AngularLabel, Arrowlabel

from status_msg import StatusMsg
from trajectory_msg import TrajectoryMsg
from detection_msg import DetectionMsg
from position_msg import PositionMsg

from name import Name


class RobotInformation:

    def __init__(self, robotID, frame, colorString):
        self.id = robotID
        self.frame = frame
        self.color = colorString
        self.rob_label = None # Qlabel
        self.pixmap = None

        self.ang_label = None
        self.arrow_label = None

        self.x = 0
        self.y = 0
        self.angel = 0

        self.team_color = ""
        self.oppo_color = ""

        self.currentBallLabel = []
        self.currentObstacleLabel = []
        self.currentTeammateLabel = []
        self.currentOpponentsLabel = []
        self.currentPathLabel = []

        self.hide = False
        self.teammateVisible = True
        self.obstacleVisible = True
        self.opponentVisible = True
        self.ballVisible = True
        self.pathVisible = True


class SingleField:

    def __init__(self, gameFrame, fieldImageLabel):
        self.frame = gameFrame
        self.field = fieldImageLabel

        rp = rospkg.RosPack()
        ip_filename = os.path.join(rp.get_path('bitbots_live_tool_rqt'), 'resource', 'ip_config.yaml')
        with open(ip_filename, "r") as file:
            config = yaml.load(file)
            self.fieldfilename = config.get("FIELD_IMAGE")
            self.field_scale_global = config.get("FIELD_SCALE")
            self.default_positions = config.get("DEFAULT_POSITIONS")
        file.close()

        print("Config: " + self.fieldfilename + ", " + str(self.field_scale_global))

        rp = rospkg.RosPack()
        self.fieldfilename = os.path.join(BallPool.rp.get_path('bitbots_live_tool_rqt'), 'resource', self.fieldfilename)
        self.fieldPixmap = QPixmap(self.fieldfilename)
        self.field.setPixmap(self.fieldPixmap)

        field_width = self.fieldPixmap.width()
        field_height = self.fieldPixmap.height()

        self.field_aspect = float(field_width) / float(field_height)
        self.frame_aspect = float(self.frame.width()) / float(self.frame.height())

        self.field_border = 40 # margin to frame

        self.transform = QTransform()

        self.fieldIsSwitched = False

        self.icon_timeout = 3

        self.team_colors = {3: "cyan", 2: "magenta"}

        if self.field_aspect >= self.frame_aspect: #
            self.field_size_x = self.frame.width() - self.field_border*2
            self.field_size_y = self.field_size_x / self.field_aspect
        else:
            self.field_size_y = self.frame.height() - self.field_border * 2
            self.field_size_x = self.field_size_y / self.field_aspect

        self.field.setScaledContents(True)
        self.field.setFixedSize(self.field_size_x, self.field_size_y)

        self.field_scale = float(self.field_size_x) / float(field_width)
        #print(self.field_scale)

        self.screenMidX = int(self.frame.width() / 2)
        self.screenMidY = int(self.frame.height() / 2)

        self.field.move(self.screenMidX - int(self.field.width() / 2), self.screenMidY - int(self.field.height()/ 2))

        self.colors = ["red", "green", "yellow", "blue"]

        self.robots = {} # dict where keys are robot IDs
        self.tabToRobot = {} # dict where keys are tab-nrs [0-3]
        self.ball_pool = BallPool(self.frame, size=42)
        self.rob_pool = RobotPool(self.frame, size=48)
        self.opp_pool = OpponentPool(self.frame, size=42)
        self.team_pool = TeammatePool(self.frame, size=42)
        self.cross_pool = CrossPool(self.frame, size=34)
        self.undef_pool = UndefinedPool(self.frame, size=42)

    # logic

    def setSide(self, default):
        self.fieldIsSwitched = not default

    # adds new robot, color is taken automatically. Color can be given manually
    def addRobot(self, robotID, color=""):
        if color == "" or color not in self.colors:
            if len(self.colors) > 0:
                color = self.colors.pop()
            else:
                color = "red"
        rob_info = RobotInformation(robotID, self.frame, color)
        rob_info.rob_label = self.rob_pool.getRobotLabel(color)
        rob_info.pixmap = rob_info.rob_label.pixmap() # save original pixmap for rotation
        rob_info.rob_label.update()
        rob_info.ang_label = AngularLabel(self.frame)
        rob_info.arrow_label = Arrowlabel(self.frame)


        self.frame.update()
        self.robots[robotID] = rob_info
        self.tabToRobot[ len(self.tabToRobot) ] = rob_info


        defpos = {"x": 0, "y": 0}
        if len(self.default_positions) > 0:
            defpos = self.default_positions.pop()
        self.setRobotPosition(robotID, defpos.get("x"), defpos.get("y"), 0) # sets default position

        """
        if self.robotsVisible:
            rob_info.rob_label.show()
        else:
            rob_info.rob_label.hide()"""
        rob_info.rob_label.show()
        rob_info.ang_label.show()
        rob_info.arrow_label.show()

        print(self.tabToRobot)
        print(self.robots)

        self.upadteAllVisiblities(rob_info)

        return rob_info


    def _meter_to_UI(self, val):
        return (val / self.field_scale_global) * self.field_scale


    # sets the robots position in meter!!!!
    def setRobotPosition(self, robotID, x, y, angle):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            if x != None:
                if self.fieldIsSwitched:
                    rob.x = -x
                else:
                    rob.x = x
            if y != None:
                if self.fieldIsSwitched:
                    rob.y = -y
                else:
                    rob.y = y
            if angle != None:
                if self.fieldIsSwitched:
                    rob.angel = -angle + self.degToRadians(180)
                else:
                    rob.angel = -angle

            transform = QTransform()
            transform.rotateRadians(rob.angel)

            rotPixMap = QPixmap(rob.rob_label.originalPixmap).transformed( transform )
            rob.rob_label.setPixmap(rotPixMap)
            rob.rob_label.setScaledContents(True)

            addScale = (abs(math.sin(rob.angel*2))) * (math.sqrt(2) - 1)
            newsize = self.rob_pool.size + (self.rob_pool.size*addScale)

            rob.rob_label.setFixedSize(newsize, newsize)
            #print(addScale, newsize, rob.rob_label.width())
            rob.rob_label.setScaledContents(True)

            rob.rob_label.move(self.screenMidX - int(rob.rob_label.width() / 2) + self._meter_to_UI(rob.x), \
                               self.screenMidY - int(rob.rob_label.height() / 2) - self._meter_to_UI(rob.y))
            rob.ang_label.move( rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.ang_label.width() / 2,\
                                rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.ang_label.height() / 2)

            rob.arrow_label.move(rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.arrow_label.width() / 2,\
                                rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.arrow_label.height() / 2)

            rob.arrow_label.setRoboAngle(self.radToDeg(rob.angel))



            rob.ang_label.setAngles(self.radToDeg(-rob.angel), None)

            #rob.rob_label.update()
            #rob.rob_label.repaint()


    def setBallsFor(self, robotID, listPositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)

            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentBallLabel)):
                self.ball_pool.returnBallLabel( rob.currentBallLabel.pop(), rob.color )
            for pos in listPositions:

                if time_secs - lastUpdate < self.icon_timeout:

                    bpx, bpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    ball = self.ball_pool.getBallLabel( rob.color )
                    ball.move(self.screenMidX - int(ball.width() / 2) + self._meter_to_UI(bpx), \
                              self.screenMidY - int(ball.height() / 2) + self._meter_to_UI(bpy))
                    rob.currentBallLabel.append(ball)

            self.upadteAllVisiblities(rob)





    def setPathsFor(self, robotID, listPaths):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            for i in range(len(rob.currentPathLabel)):
                self.cross_pool.returnCrossLabel( rob.currentPathLabel.pop(), rob.color )
            for pos in listPaths:
                cross = self.cross_pool.getCrossLabel( rob.color )

                if self.fieldIsSwitched:
                    cross.move(self.screenMidX - int(cross.width() / 2) - self._meter_to_UI(pos[0]), \
                              self.screenMidY - int(cross.height() / 2) - self._meter_to_UI(pos[1]))
                else:
                    cross.move(self.screenMidX - int(cross.width() / 2) + self._meter_to_UI(pos[0]), \
                               self.screenMidY - int(cross.height() / 2) + self._meter_to_UI(pos[1]))

                cross.show()
                rob.currentPathLabel.append(cross)
            self.upadteAllVisiblities(rob)


    def setTeammatesFor(self, robotID, listPositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)

            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentTeammateLabel)):
                self.team_pool.returnTeammateLabel( rob.currentTeammateLabel.pop(), rob.color )
            for pos in listPositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    bpx, bpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    teamm = self.team_pool.getTeammateLabel( rob.color )
                    teamm.move(self.screenMidX - int(teamm.width() / 2) + self._meter_to_UI(bpx), \
                              self.screenMidY - int(teamm.height() / 2) + self._meter_to_UI(bpy))
                    teamm.show()
                    rob.currentTeammateLabel.append(teamm)
            self.upadteAllVisiblities(rob)


    def setOpponentsFor(self, robotID, listPositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)

            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentOpponentsLabel)):
                self.opp_pool.returnOpponentLabel( rob.currentOpponentsLabel.pop(), rob.color )
            for pos in listPositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    bpx, bpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    opp = self.opp_pool.getOpponentLabel( rob.color )
                    opp.move(self.screenMidX - int(opp.width() / 2) + self._meter_to_UI(bpx), \
                              self.screenMidY - int(opp.height() / 2) + self._meter_to_UI(bpy))
                    opp.show()
                    rob.currentOpponentsLabel.append(opp)
            self.upadteAllVisiblities(rob)


    # obstacles are undefined
    def setObstaclesFor(self, robotID, listObstacles, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)

            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentObstacleLabel)):
                self.undef_pool.returnUndefLabel( rob.currentObstacleLabel.pop(), rob.color )
            for pos in listObstacles:
                if time_secs - lastUpdate < self.icon_timeout:
                    bpx, bpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    obs = self.undef_pool.getUndefLabel( rob.color )
                    obs.move(self.screenMidX - int(obs.width() / 2) + self._meter_to_UI(bpx), \
                              self.screenMidY - int(obs.height() / 2) + self._meter_to_UI(bpy))
                    obs.show()
                    rob.currentObstacleLabel.append(obs)
            self.upadteAllVisiblities(rob)


    # Message decoding ==============================================================================

    def setTrajectoryMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)

        # simple absolute target position with target angle
        if data.has_key(TrajectoryMsg.label_moveToX):
            self.setPathsFor(robotID, [(data.get(TrajectoryMsg.label_moveToX), data.get(TrajectoryMsg.label_moveToY), data.get(TrajectoryMsg.label_finalAngle))])

        # Twist vectors: rotation
        if data.has_key(TrajectoryMsg.label_rotateVel):
            rob = self.robots.get(robotID)
            rob.ang_label.setAngles(self.radToDeg(-rob.angel), self.radToDeg(data.get(TrajectoryMsg.label_rotateVel)))

        # Twist vectors: linear velocity
        if data.has_key(TrajectoryMsg.label_moveVelX):
            rob = self.robots.get(robotID)
            rob.arrow_label.setLinearAngle(self._meter_to_UI(data.get(TrajectoryMsg.label_moveVelX)), self._meter_to_UI(data.get(TrajectoryMsg.label_moveVelY)))
            rob.arrow_label.move(rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.arrow_label.width() / 2, \
                                 rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.arrow_label.height() / 2)
            rob.arrow_label.setRoboAngle(self.radToDeg(rob.angel))


    def setDetectionMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
        rob = self.robots.get(robotID)
        if data.has_key(DetectionMsg.label_ball_info):
            self.setBallsFor(robotID, [(data.get(DetectionMsg.label_ball_info).get("x"), data.get(DetectionMsg.label_ball_info).get("y"))], \
                             data.get(DetectionMsg.label_ball_info).get(Name.last_update))
        if data.has_key(DetectionMsg.label_obstacles):
            lsUndef = []
            lsTeammate = []
            lsOpponent = []
            for ob in data.get(DetectionMsg.label_obstacles):
                ob_color = ob.get(DetectionMsg.label_obstacle_info).get(DetectionMsg.label_color)
                if ob_color == rob.team_color: #mitspieler +2 weil die farben
                    lsTeammate.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])
                elif ob_color == rob.oppo_color: #gegner
                    lsOpponent.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])
                else: #undefined
                    lsUndef.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])

            self.setObstaclesFor(robotID, lsUndef, data.get(DetectionMsg.label_last_obstacle_update))
            self.setTeammatesFor(robotID, lsTeammate, data.get(DetectionMsg.label_last_obstacle_update))
            self.setOpponentsFor(robotID, lsOpponent, data.get(DetectionMsg.label_last_obstacle_update))


    def setPositionMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
            print("add robot: " + robotID)

        if(data.has_key(PositionMsg.label_pos)):
            self.setRobotPosition(robotID, data.get(PositionMsg.label_pos).get("x"), data.get(PositionMsg.label_pos).get("y"),\
                                  data.get(PositionMsg.label_orientation).get(PositionMsg.label_yaw))

    def setStatusMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
            print("add robot: " + robotID)
        if data.has_key(StatusMsg.labelTeamColor):
            rob = self.robots.get(robotID)
            statusColor = data.get(StatusMsg.labelTeamColor)
            statusColor = (1 - statusColor) + 2 # /gamestate und /obstacles have different constants for colors
            oppoColor = data.get(StatusMsg.labelTeamColor) + 2
            rob.team_color = statusColor
            rob.oppo_color = oppoColor
            #print(rob.team_color, rob.oppo_color)



    # visibility ===================================================================================

    # hides all of the tabs components if param hide is true
    # num is the tabs number
    #def hideAll(self, hide, num):
    #    Wird aber momentan ueber information_tab geregelt

    def changeVisibilityFor(self, ls, visible):
        for e in ls:
            if visible:
                e.show()
            else:
                e.hide()


    def upadteAllVisiblities(self, rob):
        self.changeVisibilityFor(rob.currentTeammateLabel, rob.teammateVisible)
        self.changeVisibilityFor(rob.currentObstacleLabel, rob.obstacleVisible)
        self.changeVisibilityFor(rob.currentOpponentsLabel, rob.opponentVisible)
        self.changeVisibilityFor(rob.currentBallLabel, rob.ballVisible)
        self.changeVisibilityFor(rob.currentPathLabel, rob.pathVisible)


    def setOwnRobotsVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            if visible:
                rob.rob_label.show()
                rob.ang_label.show()
                rob.arrow_label.show()
            else:
                rob.rob_label.hide()
                rob.ang_label.hide()
                rob.arrow_label.hide()


    def setTeammateVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.teammateVisible = visible
            self.upadteAllVisiblities(rob)


    def setOpponentVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.opponentVisible = visible
            self.upadteAllVisiblities(rob)


    def setPathVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.pathVisible = visible
            self.upadteAllVisiblities(rob)

    """
    def setObstacleVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.obstacleVisible = visible
            self.upadteAllVisiblities(rob)
    """


    def setBallVisibility(self, visible, num):
        #print(num)
        #print(self.tabToRobot)
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.ballVisible = visible
            #print("ball visible: " + str(rob.ballVisible))
            self.upadteAllVisiblities(rob)


    def setUndefVisibility(self, visible, num):
        if self.tabToRobot.has_key(num):
            rob = self.tabToRobot.get(num)
            rob.obstacleVisible = visible
            self.upadteAllVisiblities(rob)



    # Helper ===================================================================

    def vec_rotate(self, x, y, angle_rad):
        xneu = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        yneu = y * math.cos(angle_rad) + x * math.sin(angle_rad)
        return [xneu, yneu]

    def relToAbs(self, fromx, fromy, fromAng, relx, rely):
        rx, ry = self.vec_rotate(relx, rely, fromAng)
        return (fromx + rx, fromy + ry)


    def radToDeg(self, rads):
        return rads * 57.29578

    def degToRadians(self, deg):
        return deg / 57.29578