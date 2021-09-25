#!/usr/bin/env python3

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
import traceback

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from humanoid_league_msgs.msg import PoseWithCertaintyStamped, PoseWithCertaintyArray, \
    ObstacleRelative, ObstacleRelativeArray, PoseWithCertainty
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion
import tf2_ros
import numpy as np

BALL_DIAMETER = 0.13
GOAL_WIDTH = 1.5
GOAL_HEIGHT = 1.1
POST_DIAMETER = 0.1
OBSTACLE_NUMBER = 4
OBSTACLE_HEIGT = 0.8
OBSTACLE_DIAMETER = 0.2

rospy.init_node("humanoid_league_interactive_marker")
tf_buffer = tf2_ros.Buffer(rospy.Duration(30))
tf_listener = tf2_ros.TransformListener(tf_buffer)


class RobocupInteractiveMarker(object):
    def __init__(self, server):
        self.server = server
        self.pose = Pose()
        self.publish = True
        self.int_marker = None
        self.make_marker()
        self.menu_handler = MenuHandler()
        item = self.menu_handler.insert("publish", callback=self.menu_callback)
        self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)

    def feedback(self, feedback):
        self.pose = feedback.pose
        self.server.applyChanges()

    def menu_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.publish = False
        else:
            self.publish = True
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_marker(self):
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "map"
        self.int_marker.pose = self.pose
        self.int_marker.scale = 1

        self.int_marker.name = self.marker_name

        control = InteractiveMarkerControl()
        control.orientation.w = math.sqrt(2) / 2
        control.orientation.x = 0
        control.orientation.y = math.sqrt(2) / 2
        control.orientation.z = 0
        control.interaction_mode = self.interaction_mode
        self.int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        markers = self.make_individual_markers(self.int_marker)
        for marker in markers:
            control.markers.append(marker)
        control.always_visible = True
        self.int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(self.int_marker, self.feedback)


class BallMarker(RobocupInteractiveMarker):
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.marker_name = "ball"
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.absolute_publisher = rospy.Publisher("balls_absolute", PoseWithCertaintyArray, queue_size=1)
        self.relative_publisher = rospy.Publisher("balls_relative", PoseWithCertaintyArray, queue_size=1)
        super(BallMarker, self).__init__(server)
        self.pose.position.x = 1.0

    def make_individual_markers(self, msg):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = BALL_DIAMETER
        marker.scale.y = BALL_DIAMETER
        marker.scale.z = BALL_DIAMETER
        marker.pose.position.z = BALL_DIAMETER / 2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4

        return (marker,)

    def publish_marker(self, e):
        # construct PoseWithCertaintyArray() message for map frame
        ball_absolute = PoseWithCertainty()
        ball_absolute.pose.pose = self.pose
        ball_absolute.confidence = 1.0

        balls_absolute = PoseWithCertaintyArray()
        balls_absolute.header.stamp = rospy.get_rostime()
        balls_absolute.header.frame_id = "map"
        balls_absolute.poses = [ball_absolute]

        # publish the new ball position
        if self.publish:
            self.absolute_publisher.publish(balls_absolute)

        # check if ball is also visible for the robot and publish on relative topic if this is the case
        # only works if camera info is provided
        if self.cam_info:
            try:
                ball_point_stamped = PointStamped()
                ball_point_stamped.header.stamp = rospy.Time.now()
                ball_point_stamped.header.frame_id = "map"
                ball_point_stamped.point = ball_absolute.pose.pose.position
                ball_in_camera_optical_frame = tf_buffer.transform(ball_point_stamped, self.cam_info["frame_id"],
                                                                   timeout=rospy.Duration(0.5))
                if ball_in_camera_optical_frame.point.z >= 0:
                    p = [ball_in_camera_optical_frame.point.x, ball_in_camera_optical_frame.point.y,
                         ball_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    # make sure that the transformed pixel is inside the resolution and positive.
                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        ball_in_footprint_frame = tf_buffer.transform(ball_in_camera_optical_frame, "base_footprint",
                                                                      timeout=rospy.Duration(0.5))
                        ball_relative = PoseWithCertainty()
                        ball_relative.pose.pose.position = ball_in_footprint_frame.point
                        ball_relative.confidence = 1.0

                        balls_relative = PoseWithCertaintyArray()
                        balls_relative.header = ball_in_footprint_frame.header
                        balls_relative.poses = [ball_relative]
                        self.relative_publisher.publish(balls_relative)
            except tf2_ros.LookupException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None
            except tf2_ros.ExtrapolationException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None


class GoalMarker(RobocupInteractiveMarker):
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.marker_name = "goal"
        self.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        self.absolute_publisher = rospy.Publisher("goal_absolute", PoseWithCertaintyArray, queue_size=1)
        self.relative_publisher = rospy.Publisher("goal_relative", PoseWithCertaintyArray, queue_size=1)
        self.relative_posts_publisher = rospy.Publisher("goal_posts_relative", PoseWithCertaintyArray, queue_size=1)
        super(GoalMarker, self).__init__(server)
        self.menu_handler = MenuHandler()
        self.pose.position.x = 3.0

    def make_individual_markers(self, msg):
        lpost = Marker()
        lpost.type = Marker.CYLINDER
        lpost.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_HEIGHT)
        lpost.color.r = 1.0
        lpost.color.g = 1.0
        lpost.color.b = 1.0
        lpost.color.a = 0.4
        lpost.pose.position = Point(0, GOAL_WIDTH / 2, GOAL_HEIGHT / 2)

        rpost = Marker()
        rpost.type = Marker.CYLINDER
        rpost.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_HEIGHT)
        rpost.color.r = 1.0
        rpost.color.g = 1.0
        rpost.color.b = 1.0
        rpost.color.a = 0.4
        rpost.pose.position = Point(0, - GOAL_WIDTH / 2, GOAL_HEIGHT / 2)

        bar = Marker()
        bar.type = Marker.CYLINDER
        bar.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_WIDTH)
        bar.color.r = 1.0
        bar.color.g = 1.0
        bar.color.b = 1.0
        bar.color.a = 0.4
        bar.pose.position = Point(0, 0, GOAL_HEIGHT)
        bar.pose.orientation = Quaternion(math.sqrt(2) / 2, 0, 0, math.sqrt(2) / 2)

        return (lpost, rpost, bar)

    def publish_marker(self, e):
        # construct GoalRelative message
        goal_absolute = PoseWithCertaintyArray()
        goal_absolute.header.stamp = rospy.get_rostime()
        goal_absolute.header.frame_id = "map"
        # calculate the positions of the right and the left post
        orientation = euler_from_quaternion((self.pose.orientation.x, self.pose.orientation.y,
                                             self.pose.orientation.z, self.pose.orientation.w))
        angle = orientation[2]
        left_post = PoseWithCertainty()
        left_post.pose.pose.position.x = self.pose.position.x - math.sin(angle) * GOAL_WIDTH / 2
        left_post.pose.pose.position.y = self.pose.position.y + math.cos(angle) * GOAL_WIDTH / 2
        left_post.confidence = 1.0

        right_post = PoseWithCertainty()
        right_post.pose.pose.position.x = self.pose.position.x + math.sin(angle) * GOAL_WIDTH / 2
        right_post.pose.pose.position.y = self.pose.position.y - math.cos(angle) * GOAL_WIDTH / 2
        right_post.confidence = 1.0


        goal_absolute.poses.append(left_post)
        goal_absolute.poses.append(right_post)

        # publish the new goal position
        if self.publish:
            self.absolute_publisher.publish(goal_absolute)

        # check if goal is also visible for the robot and publish on relative topic if this is the case
        # only works if camera info is provided
        if self.cam_info:
            try:
                goal_relative = PoseWithCertaintyArray()
                goal_relative.header.stamp = rospy.Time.now()
                goal_relative.header.frame_id = "base_footprint"

                goal_left_point_stamped = PointStamped()
                goal_left_point_stamped.header.stamp = goal_relative.header.stamp
                goal_left_point_stamped.header.frame_id = "map"
                goal_left_point_stamped.point = left_post.pose.pose.position
                left_post_in_camera_optical_frame = tf_buffer.transform(goal_left_point_stamped,
                                                                        self.cam_info["frame_id"],
                                                                        timeout=rospy.Duration(0.5))

                lpost_visible = False
                rpost_visible = False

                if left_post_in_camera_optical_frame.point.z >= 0:
                    p = [left_post_in_camera_optical_frame.point.x, left_post_in_camera_optical_frame.point.y,
                         left_post_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        left_post = PoseWithCertainty()
                        left_post.pose.pose.position = tf_buffer.transform(left_post_in_camera_optical_frame,
                                                                           "base_footprint",
                                                                           timeout=rospy.Duration(0.5)).point
                        goal_relative.poses.append(left_post)
                        lpost_visible = True

                goal_right_point_stamped = PointStamped()
                goal_right_point_stamped.header.stamp = goal_relative.header.stamp
                goal_right_point_stamped.header.frame_id = "map"
                goal_right_point_stamped.point = right_post.pose.pose.position
                right_post_in_camera_optical_frame = tf_buffer.transform(goal_right_point_stamped,
                                                                         self.cam_info["frame_id"],
                                                                         timeout=rospy.Duration(0.5))
                if right_post_in_camera_optical_frame.point.z >= 0:
                    p = [right_post_in_camera_optical_frame.point.x, right_post_in_camera_optical_frame.point.y,
                         right_post_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        right_post = PoseWithCertainty()
                        right_post.pose.pose.position = tf_buffer.transform(right_post_in_camera_optical_frame,
                                                                            "base_footprint",
                                                                            timeout=rospy.Duration(0.5)).point
                        goal_relative.poses.append(right_post)
                        rpost_visible = True

                # publish goal relative msg
                if rpost_visible or lpost_visible:
                    self.relative_publisher.publish(goal_relative)
                    self.relative_posts_publisher.publish(goal_relative)

            except tf2_ros.LookupException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None
            except tf2_ros.ExtrapolationException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None


class ObstacleMarker(RobocupInteractiveMarker):
    def __init__(self, server, cam_info, name):
        self.cam_info = cam_info
        self.marker_name = name
        self.type = 0  # unknown
        self.player_number = 0
        self.confidence = 1.0
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        super(ObstacleMarker, self).__init__(server)
        sub_menu_handle = self.menu_handler.insert("Color")
        h_mode_last = self.menu_handler.insert("red", parent=sub_menu_handle, callback=self.colorCb)
        h_mode_last = self.menu_handler.insert("blue", parent=sub_menu_handle, callback=self.colorCb)
        h_mode_last = self.menu_handler.insert("unknown", parent=sub_menu_handle, callback=self.colorCb)
        self.menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)
        self.pose.position.x = 2.0

    def colorCb(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            # unchecking something should lead to unknown color
            self.type = 0
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(5, MenuHandler.CHECKED)
            self.int_marker.controls[1].markers[0].type.r = 0.0
            self.int_marker.controls[1].markers[0].type.g = 0.0
            self.int_marker.controls[1].markers[0].type.b = 0.0
        else:
            if item == 3:
                self.type = 2
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 1.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 0.0
            elif item == 4:
                self.type = 3
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 0.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 1.0
            elif item == 5:
                self.type = 0
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 0.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 0.0
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_individual_markers(self, msg):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale = Vector3(POST_DIAMETER, POST_DIAMETER, OBSTACLE_HEIGT)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        marker.pose.position = Point(0, 0, OBSTACLE_HEIGT / 2)

        return (marker,)

    def get_absolute_message(self):
        msg = ObstacleRelative()
        msg.pose.pose.pose.position = self.pose.position
        msg.height = OBSTACLE_HEIGT
        msg.width = OBSTACLE_DIAMETER
        msg.type = self.type
        msg.pose.confidence = self.confidence
        msg.playerNumber = self.player_number
        return msg

    def get_relative_msg(self):
        # check if obstacle is also visible for the robot and only then return
        if self.cam_info:
            try:
                obstacle_point_stamped = PointStamped()
                obstacle_point_stamped.header.stamp = rospy.Time.now()
                obstacle_point_stamped.header.frame_id = "map"
                obstacle_point_stamped.point = self.pose.position
                obstacle_in_camera_optical_frame = tf_buffer.transform(obstacle_point_stamped,
                                                                       self.cam_info["frame_id"],
                                                                       timeout=rospy.Duration(0.5))
                if obstacle_in_camera_optical_frame.point.z >= 0:
                    p = [obstacle_in_camera_optical_frame.point.x, obstacle_in_camera_optical_frame.point.y,
                         obstacle_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    # make sure that the transformed pixel is inside the resolution and positive.
                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        obstacle_relative = ObstacleRelative()
                        ball_in_footprint_frame = tf_buffer.transform(obstacle_in_camera_optical_frame,
                                                                      "base_footprint",
                                                                      timeout=rospy.Duration(0.5))
                        obstacle_relative.pose.pose.pose.position = ball_in_footprint_frame.point
                        obstacle_relative.pose.confidence = self.confidence
                        obstacle_relative.playerNumber = self.player_number
                        obstacle_relative.type = self.type
                        return obstacle_relative
            except tf2_ros.LookupException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None
            except tf2_ros.ExtrapolationException as ex:
                rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                return None

        return None


class ObstacleMarkerArray:
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.absolute_publisher = rospy.Publisher("obstacles_absolute", ObstacleRelativeArray, queue_size=1)
        self.relative_publisher = rospy.Publisher("obstacles_relative", ObstacleRelativeArray, queue_size=1)
        self.obstacles = []
        for i in range(0, OBSTACLE_NUMBER):
            self.obstacles.append(ObstacleMarker(server, cam_info, "obstacle_" + str(i)))

    def publish_marker(self, e):
        absolut_msg = ObstacleRelativeArray()
        absolut_msg.header.stamp = rospy.Time.now()
        absolut_msg.header.frame_id = "map"
        absolut_obstacles = []
        relative_msg = ObstacleRelativeArray()
        relative_msg.header.stamp = rospy.Time.now()
        relative_msg.header.frame_id = "base_footprint"
        relative_obstacles = []

        for obstacle in self.obstacles:
            # always publish on absolute
            absolut_obstacles.append(obstacle.get_absolute_message())

            rel_msg = obstacle.get_relative_msg()
            if rel_msg:
                relative_obstacles.append(rel_msg)

        absolut_msg.obstacles = absolut_obstacles
        relative_msg.obstacles = relative_obstacles

        self.absolute_publisher.publish(absolut_msg)
        self.relative_publisher.publish(relative_msg)


if __name__ == "__main__":

    # get camera info
    cam_info = rospy.get_param("/camera_info", None)
    if not cam_info:
        rospy.logwarn("No camera info provided from parameter server. Will only publish absolute messages!")
    # retrieve InteractiveMarkerServer and setup subscribers and publishers
    server = InteractiveMarkerServer("basic_controls")
    ball = BallMarker(server, cam_info)
    goal = GoalMarker(server, cam_info)
    obstacles = ObstacleMarkerArray(server, cam_info)

    server.applyChanges()

    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), ball.publish_marker)
    rospy.Timer(rospy.Duration(0.1), goal.publish_marker)
    rospy.Timer(rospy.Duration(0.1), obstacles.publish_marker)

    # run and block until finished
    rospy.spin()
