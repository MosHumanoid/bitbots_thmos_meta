import os
import rospkg
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui

import math
import rospy
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtWidgets import QGraphicsPixmapItem
from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtWidgets import QGraphicsScene
from PyQt5.QtWidgets import QWidget
from python_qt_binding import loadUi

from rqt_gui_py.plugin import Plugin
from geometry_msgs.msg import PoseWithCovarianceStamped

from dynamic_reconfigure.server import Server
from tf.transformations import euler_from_quaternion
#from humanoid_league_field_rqt.cfg import field_rqt_params


class HumanoidLeagueFieldRqt(Plugin):
    """ This class provides a rqt plugin which shows a 2d view of a RoboCup Soccer field.
     Different objects can be shown. Their position changing is handled in the respective callbacks."""

    def __init__(self, context):
        super(HumanoidLeagueFieldRqt, self).__init__(context)
        self.setObjectName('2dField')

        # field values
        self.field_length = 9
        self.field_width = 6
        self.image_width = 1100.0
        self.image_height = 800.0
        self.field_length_img = 900
        self.field_width_img = 600
        self.scale = 1  # scaling factor between meters in reality and pixels on display

        # object values
        self.robot_size = 75
        self.robot_pen_width = 5
        self.ball_size = 50
        self.opacity = 0.5

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('humanoid_league_field_rqt'), 'resource', '2dField.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('2dFieldUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        self._widget.resize_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.resize_push_button.pressed.connect(self.resize_field)

        # set drawing space
        self.view = self._widget.graphics_view
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(QColor(255, 255, 255))

        # field object
        rp = rospkg.RosPack()
        image_path = rp.get_path('humanoid_league_field_rqt') + "/resource/field.png"
        field_image = QPixmap(image_path)
        self.field = QGraphicsPixmapItem(field_image)
        self.field.setPos(0, 0)
        self._scene.addItem(self.field)


        # robot
        self.robot_pen = QPen()
        self.robot_pen.setWidth(self.robot_pen_width)
        self.robot_brush = QBrush(QColor(255, 0, 0))

        self.robot = QGraphicsEllipseItem(0, 0, self.robot_size, self.robot_size, self.field)
        self.robot.setBrush(self.robot_brush)
        self.robot.setPen(self.robot_pen)
        self.robot.setVisible(False)
        self.robot.setOpacity(self.opacity)

        self.robot_pers = QGraphicsEllipseItem(0, 0, self.robot_size, self.robot_size, self.field)
        self.robot_pers.setBrush(self.robot_brush)
        self.robot_pers.setPen(self.robot_pen)
        self.robot_pers.setVisible(False)
        self.robot_pers.setOpacity(self.opacity)

        self.robot_team = QGraphicsEllipseItem(0, 0, self.robot_size, self.robot_size, self.field)
        self.robot_team.setBrush(self.robot_brush)
        self.robot_team.setPen(self.robot_pen)
        self.robot_team.setVisible(False)
        self.robot_team.setOpacity(self.opacity)

        # ball
        self.ball_pen = QPen(QColor(255, 165, 0))
        self.ball_pen.setWidth(2)

        self.ball_personal_brush = QBrush(QColor(255, 165, 0))
        #self.ball_personal_brush.setStyle("HorPattern")
        self.ball_personal = QGraphicsEllipseItem(0, 0, self.ball_size, self.ball_size, self.field)
        self.ball_personal.setPen(self.ball_pen)
        self.ball_personal.setBrush(self.ball_personal_brush)
        self.ball_personal.setVisible(False)
        self.ball_personal.setOpacity(self.opacity)

        self.ball_team_brush = QBrush(QColor(255, 165, 0))
        #self.ball_team_brush.setStyle("VerPattern")
        self.ball_team = QGraphicsEllipseItem(0, 0, self.ball_size, self.ball_size, self.field)
        self.ball_team.setPen(self.ball_pen)
        self.ball_team.setBrush(self.ball_team_brush)
        self.ball_team.setVisible(False)
        self.ball_team.setOpacity(self.opacity)



        # set the right positions and sizes
        self.resize_field()
        self.view.setScene(self._scene)

        # todo dispaly data from models
        # rospy.Subscriber("/local_model", LocalModel, self.local_model_update, queue_size=100)
        # rospy.Subscriber("/global_model", GlobalModel, self.global_model_update, queue_size=100)

        #self.dyn_reconf = Server(field_rqt_params, self.reconfigure)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.position_cb, queue_size=10)

        context.add_widget(self._widget)

    def reconfigure(self, config, level):
        # get bools of what is active and not from dyn reconfigure
        position = config["position"]
        personal = config["personal"]
        team = config["team"]

        # set visibilities accordingly
        self.robot.setVisible(position)

        self.robot_pers.setVisible(personal)
        self.ball_personal.setVisible(personal)

        self.robot_team.setVisible(team)
        self.ball_team.setVisible(team)

    def get_center_point_xy(self):
        # to make it always fit to the current scale
        return (self.field_length / 2 * self.scale,
                self.field_width / 2 * self.scale)

    def resize_field(self):
        # fits the field into current window size
        size = self._widget.size()
        x_scale = size.width() / self.image_width
        y_scale = size.height() / self.image_height
        self.scale = min(x_scale, y_scale)
        self.field.setScale(self.scale)
        self.view.centerOn(size.width() / 2, size.height() / 2)
        self.field.offset()

    def set_scaled_position(self, item, x, y, height, width):
        item_scale = min(self.scale + 0.5, 1)
        item.setScale(item_scale)
        x *= self.field_length_img / self.field_length  # scaling from meters to pixels on original image
        x += self.image_width / 2  # transform from upper left corner (qt coord system) to center point (robocup coord sys)
        x -= width * item_scale / 2
        x = max(min(x, self.image_width - 50), 0)  # dont let it get outside of the window

        y *= self.field_width_img / self.field_width
        y += self.image_height / 2
        y -= height * item_scale / 2
        y = max(min(y, self.image_height - 50), 0)

        item.setX(x)
        item.setY(y)

        return x, y

    def position_cb(self, msg):
        self.set_scaled_position(self.robot, msg.pose.pose.position.x, msg.pose.pose.position.y, self.robot_size, self.robot_size)
        angle = math.degrees(euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]) * 16  # to get to qt coordinate system
        # set orientation
        self.robot.setStartAngle(angle)
        self.robot.setSpanAngle(360 * 16 - 1)
        self.robot_brush.setColor(QColor(47, 83, 140))
        self.robot.setBrush(self.robot_brush)
        self.robot.setVisible(True)
