import os
import rospkg
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui

import math
import rospy
import tf2_ros
import numpy as np
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
from PyQt5.QtWidgets import QGraphicsTextItem
from PyQt5.QtWidgets import QWidget
from python_qt_binding import loadUi

from rqt_gui_py.plugin import Plugin

from humanoid_league_msgs.msg import PoseWithCertainty, PoseWithCertaintyArray, \
    GoalRelative, ObstacleRelativeArray

from tf2_geometry_msgs import PointStamped

from dynamic_reconfigure.server import Server


# from humanoid_league_msgs.cfg import field_rqt_params


class HumanoidLeagueRelativeRqt(Plugin):
    """ This class provides a rqt plugin which shows a 2d view of a RoboCup Soccer field.
     Different objects can be shown. Their position changing is handled in the respective callbacks."""

    def __init__(self, context):
        super(HumanoidLeagueRelativeRqt, self).__init__(context)
        self.setObjectName('2dField')

        # color values
        self.green = np.array([30, 186, 44])
        self.red = np.array([186, 45, 30])

        # outlines
        self.outline_thickness = 9
        self.goal_outline = QColor(250, 0, 0)
        self.ball_outline = QColor(0, 0, 250)
        self.obstacle_outline = QColor(30, 30, 30)

        # image values
        self.image_width = 1200.0
        self.image_height = 1200.0
        self.meter_length_img = 50
        self.scale = 1  # scaling factor between meters in reality and pixels on display

        # object values
        self.obstacle_size = 75
        self.obstacle_pen_width = 5
        self.ball_size = 50
        self.opacity = 0.75
        self.post_size = 50
        self.center_size = 30
        self.obsacle_size = 50

        self.ball_active = True
        self.goal_active = True

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('humanoid_league_relative_rqt'), 'resource', 'relative.ui')
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

        # radar object
        rp = rospkg.RosPack()
        image_path = rp.get_path('humanoid_league_relative_rqt') + "/resource/radar.png"
        field_image = QPixmap(image_path)
        self.radar = QGraphicsPixmapItem(field_image)
        self.radar.setPos(0, 0)
        self._scene.addItem(self.radar)

        # legend
        self.legend = QGraphicsTextItem(self.radar)
        html_legend = ('<p style="font-size: 50px;">Legend</p> \
        <div style="font-size: 30px"><li>\
            <ul style="color: rgb{};">Goal outline</ul>\
            <ul style="color: rgb{};">Ball outline</ul>\
            <ul style="color: rgb{};">Obstacle outline</ul>\
            <ul style="color: rgb{};">Low confidence</ul>\
            <ul style="color: rgb{};">High confidence</ul>\
        </li></div>'.format(self.QColor2String(self.goal_outline),
                            self.QColor2String(self.ball_outline),
                            self.QColor2String(self.obstacle_outline),
                            self.Npcolor2String(self.red),
                            self.Npcolor2String(self.green)))
        self.legend.setHtml(html_legend)
        self.legend.setPos(self.image_width,50)

        # ball
        self.ball_pen = QPen(self.ball_outline)
        self.ball_pen.setWidth(self.outline_thickness)

        # goal posts
        self.post_pen = QPen(self.goal_outline)
        self.post_pen.setWidth(self.outline_thickness)

        self.left_post = QGraphicsEllipseItem(0, 0, self.post_size, self.post_size, self.radar)
        self.left_post.setPen(self.post_pen)
        self.left_post.setVisible(False)
        self.left_post.setOpacity(self.opacity)

        self.right_post = QGraphicsEllipseItem(0, 0, self.post_size, self.post_size, self.radar)
        self.right_post.setPen(self.post_pen)
        self.right_post.setVisible(False)
        self.right_post.setOpacity(self.opacity)

        # goal center
        self.center_pen = QPen(self.goal_outline)
        self.center_pen.setWidth(self.outline_thickness)

        self.center = QGraphicsEllipseItem(0, 0, self.center_size, self.center_size, self.radar)
        self.center.setPen(self.center_pen)
        self.center.setVisible(False)
        self.center.setOpacity(self.opacity)

        # Obstacles
        self.obstacle_pen = QPen(self.obstacle_outline)
        self.obstacle_pen.setWidth(self.outline_thickness)

        self.obstacles = []

        # set the right positions and sizes
        self.resize_field()
        self.view.setScene(self._scene)

        # self.dyn_reconf = Server(field_rqt_params, self.reconfigure)

        rospy.Subscriber("balls_relative", PoseWithCertaintyArray, self.balls_cb, queue_size=100)
        rospy.Subscriber("goal_relative", GoalRelative, self.goal_cb, queue_size=100)
        rospy.Subscriber("obstacles_relative", ObstacleRelativeArray, self.obstacle_cb, queue_size=100)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        context.add_widget(self._widget)

    def QColor2String(self, color):
        return ("({},{},{})".format(color.red(), color.green(), color.blue()))

    def Npcolor2String(self, color):
        return ("({},{},{})".format(color[0], color[1], color[2]))

    def reconfigure(self, config, level):
        # set visibilities accordingly
        self.ball_active = config["ball"]
        self.goal_active = config["goal"]

        # todo
        # config["obstacles"]
        # config["lines"]

    def resize_field(self):
        # fits the field into current window size
        size = self._widget.size()
        x_scale = size.width() / self.image_width
        y_scale = size.height() / self.image_height
        self.scale = min(x_scale, y_scale)
        self.radar.setScale(self.scale)
        self.view.centerOn(size.width() / 2, size.height() / 2)
        self.radar.offset()

    def set_scaled_position(self, item, x, y, height, width):
        item_scale = min(self.scale + 0.5, 1)
        item.setScale(item_scale)
        x *= self.meter_length_img  # scaling from meters to pixels on original image
        x += self.image_width / 2  # transform from upper left corner (qt coord system) to center point (robocup coord sys)
        x -= width * item_scale / 2
        x = max(min(x, self.image_width - 50), 0)  # dont let it get outside of the window

        y *= self.meter_length_img
        y += self.image_height / 2
        y -= height * item_scale / 2
        y = max(min(y, self.image_height - 50), 0)

        item.setX(y)
        item.setY(x)

    def balls_cb(self, msg):
        ball_msgs = msg.poses

        for ball_msg in ball_msgs:
            ball_point = PointStamped()
            ball_point.header.frame_id = msg.header.frame_id
            ball_point.header.stamp = msg.header.stamp
            ball_point.point = ball_msg.pose.pose.position
            try:
                ball_point = self.tf_buffer.transform(ball_point, "base_footprint")
            except tf2_ros.LookupException:
                rospy.logwarn("Could not transform from " + msg.header.frame_id + " to 'base_footprint'")
                return None

            ball = QGraphicsEllipseItem(0, 0, self.ball_size, self.ball_size, self.radar)
            ball.setPen(self.ball_pen)
            ball.setVisible(False)
            ball.setOpacity(self.opacity)
            self.set_scaled_position(ball, -1 * ball_point.point.x, -1 * ball_point.point.y, self.ball_size, self.ball_size)

            color_tuple = self.confidence2color(ball_msg.confidence)
            ball.setBrush(QBrush(QColor(*color_tuple)))
            ball.setVisible(self.ball_active)

    def goal_cb(self, msg):
        self.draw_goal_part(self.left_post, msg, msg.left_post, self.post_size)
        self.draw_goal_part(self.right_post, msg, msg.right_post, self.post_size)
        self.draw_goal_part(self.center, msg, msg.center_direction, self.center_size)

    def draw_goal_part(self, obj, msg, point, size):
        goal_point = PointStamped()
        goal_point.header.frame_id = msg.header.frame_id
        goal_point.header.stamp = msg.header.stamp
        goal_point.point = point
        try:
            goal_point = self.tf_buffer.transform(goal_point, "base_footprint")
        except tf2_ros.LookupException:
            rospy.logwarn("Could not transform from " + msg.header.frame_id + " to 'base_footprint'")
            return None

        self.set_scaled_position(obj, -1 * goal_point.point.x, -1 * goal_point.point.y, size, size)
        color_tuple = self.confidence2color(msg.confidence)
        obj.setBrush(QBrush(QColor(*color_tuple)))
        obj.setVisible(self.goal_active)

    def confidence2color(self, confidence):
        diff = self.green - self.red
        ball_color = self.red + diff * confidence
        return tuple(ball_color)


    def obstacle_cb(self, msg):
        for item in self.obstacles:
            self._scene.removeItem(item)

        for obstacle_msg in msg.obstacles:
            obstacle = QGraphicsEllipseItem(0, 0, self.obsacle_size, self.obsacle_size, self.radar)
            obstacle.setPen(self.obstacle_pen)
            obstacle.setOpacity(self.opacity)

            obstacle_point = PointStamped()
            obstacle_point.header.frame_id = msg.header.frame_id
            obstacle_point.header.stamp = msg.header.stamp
            obstacle_point.point = obstacle_msg.pose.pose.pose.position

            try:
                obstacle_point = self.tf_buffer.transform(obstacle_point, "base_footprint")
            except tf2_ros.LookupException:
                rospy.logwarn("Could not transform from " + msg.header.frame_id + " to 'base_footprint'")
                return None

            self.set_scaled_position(obstacle, -1 * obstacle_point.point.x, -1 * obstacle_point.point.y, self.obsacle_size, self.obsacle_size)
            color_tuple = self.confidence2color(obstacle_msg.pose.confidence)
            obstacle.setBrush(QBrush(QColor(*color_tuple)))
            obstacle.setVisible(True)
            self.obstacles.append(obstacle)
