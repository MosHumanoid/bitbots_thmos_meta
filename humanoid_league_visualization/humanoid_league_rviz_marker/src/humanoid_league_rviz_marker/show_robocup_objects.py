#!/usr/bin/env python3

import rospy
import time

from geometry_msgs.msg import Pose, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from humanoid_league_msgs.msg import PoseWithCertaintyStamped, PoseWithCertaintyArray, PoseWithCertainty, \
    ObstacleRelativeArray, ObstacleRelative


class ShowRobocupObjects:
    """This class provides RViz markers coresponding to the recognized relative objects.
    The generation of messages is handled directly in the respective callback methods."""

    def __init__(self):
        rospy.init_node("show_robocup_objects")
        # todo make dyn reconf able
        # todo add line markers
        self.marker_publisher = rospy.Publisher("robocup_markers", Marker, queue_size=10)
        self.marker_array_publisher = rospy.Publisher("robocup_marker_arrays", MarkerArray, queue_size=10)

        # object properties
        self.ball_diameter = 0.13
        self.ball_lifetime = int(5 * (10 ** 9))
        self.post_diameter = 0.10
        self.post_height = 1.10
        self.goal_lifetime = int(5 * (10 ** 9))
        self.obstacle_height = 1.0
        self.obstacle_lifetime = int(5 * (10 ** 9))
        self.obstacle_def_width = 0.3

        # --- initilize message objects ---
        # Most of the message information stay the same. It is more performant to set them just one time
        # ball
        self.marker_ball_rel = Marker()  # type: Marker
        self.marker_ball_rel.ns = "rel_ball"
        self.marker_ball_rel.id = 0
        self.marker_ball_rel.type = Marker.SPHERE
        self.marker_ball_rel.action = Marker.MODIFY
        scale = Vector3(self.ball_diameter, self.ball_diameter, self.ball_diameter)
        self.marker_ball_rel.scale = scale
        self.ball_color = ColorRGBA()
        self.ball_color.r = 1.0
        self.ball_color.a = 1.0
        self.marker_ball_rel.color = self.ball_color
        self.marker_ball_rel.lifetime = rospy.Duration(nsecs=self.ball_lifetime)
        # goal
        # -post 1
        self.marker_goal_rel1 = Marker()  # type:Marker
        self.marker_goal_rel1.ns = "rel_goal"
        self.marker_goal_rel1.id = 0
        self.marker_goal_rel1.type = Marker.CYLINDER
        self.marker_goal_rel1.action = Marker.MODIFY
        self.goal_post1_pose = Pose()
        self.goal_post1_pose.orientation.x = 0
        self.goal_post1_pose.orientation.y = 0
        self.goal_post1_pose.orientation.z = 0
        self.goal_post1_pose.orientation.w = 1
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_rel1.scale = scale
        self.post1_color = ColorRGBA()
        self.post1_color.r = 1.0
        self.post1_color.g = 1.0
        self.post1_color.b = 1.0
        self.post1_color.a = 1.0
        self.marker_goal_rel1.color = self.post1_color
        self.marker_goal_rel1.lifetime = rospy.Duration(nsecs=self.goal_lifetime)
        # -post 2
        self.marker_goal_rel2 = Marker()  # type:Marker
        self.marker_goal_rel2.ns = "rel_goal"
        self.marker_goal_rel2.id = 1
        self.marker_goal_rel2.type = Marker.CYLINDER
        self.marker_goal_rel2.action = Marker.MODIFY
        self.goal_post2_pose = Pose()
        self.goal_post2_pose.orientation.x = 0
        self.goal_post2_pose.orientation.y = 0
        self.goal_post2_pose.orientation.z = 0
        self.goal_post2_pose.orientation.w = 1
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_rel2.scale = scale
        self.post2_color = ColorRGBA()
        self.post2_color.r = 1.0
        self.post2_color.g = 1.0
        self.post2_color.b = 1.0
        self.post2_color.a = 1.0
        self.marker_goal_rel2.color = self.post2_color
        self.marker_goal_rel2.lifetime = rospy.Duration(nsecs=self.goal_lifetime)
        # goal parts
        self.goal_parts_marker = MarkerArray()
        # obstacle
        self.marker_obstacle = Marker()
        self.marker_obstacle.lifetime = rospy.Duration(nsecs=self.obstacle_lifetime)
        self.marker_obstacle.ns = "rel_obstacle"
        self.obstacle_color = ColorRGBA()
        self.obstacle_pose = Pose()
        self.obstacle_pose.orientation.x = 0
        self.obstacle_pose.orientation.y = 0
        self.obstacle_pose.orientation.z = 0
        self.obstacle_pose.orientation.w = 1
        self.marker_obstacle.type = Marker.CUBE

        # todo also display data from world model
        rospy.Subscriber("balls_relative", PoseWithCertaintyArray, self.balls_cb, queue_size=10)
        rospy.Subscriber("goal_relative", PoseWithCertaintyArray, self.goal_cb, queue_size=10)
        rospy.Subscriber("goal_posts_relative", PoseWithCertaintyArray, self.goal_parts_cb, queue_size=10)
        rospy.Subscriber("obstacles_relative", ObstacleRelativeArray, self.obstacle_cb, queue_size=10)

        # we do everything in the callbacks
        rospy.spin()

    def balls_cb(self, msg: PoseWithCertaintyArray):
        self.ball_frame = msg.header.frame_id
        self.marker_ball_rel.header = msg.header
        balls = msg.poses

        for ball in balls:
            self.marker_ball_rel.pose = ball.pose.pose
            self.ball_color.a = ball.confidence
            self.marker_ball_rel.color = self.ball_color
            self.marker_publisher.publish(self.marker_ball_rel)

    def goal_cb(self, msg: PoseWithCertaintyArray):
        # first post
        if len(msg.poses) > 0:
            self.marker_goal_rel1.header = msg.header
            self.goal_post1_pose.position = msg.poses[0].pose.pose.position
            self.goal_post1_pose.position.z = self.post_height / 2
            self.marker_goal_rel1.pose = self.goal_post1_pose
            self.post1_color.a = msg.poses[0].confidence
            self.marker_goal_rel1.color = self.post1_color
            self.marker_publisher.publish(self.marker_goal_rel1)

        # second post
        if len(msg.poses) > 1:
            self.marker_goal_rel2.header = msg.header
            self.goal_post2_pose.position = msg.poses[1].pose.pose.position
            self.goal_post2_pose.position.z = self.post_height / 2
            self.marker_goal_rel2.pose = self.goal_post2_pose
            self.post2_color.a = msg.poses[1].confidence
            self.marker_goal_rel2.color = self.post2_color
            self.marker_publisher.publish(self.marker_goal_rel2)

    def goal_parts_cb(self, msg: PoseWithCertaintyArray):
        arr = []
        i = 0
        for post in msg.poses:
            post_marker = Marker()
            pose = Pose()
            pose.position = post.pose.pose.position
            post_marker.pose = pose
            post_marker.pose.position.z = self.post_height /2
            post_marker.pose.orientation = Quaternion()
            post_marker.pose.orientation.x = 0
            post_marker.pose.orientation.y = 0
            post_marker.pose.orientation.z = 0
            post_marker.pose.orientation.w = 1
            post_marker.type = Marker.CYLINDER
            post_marker.action = Marker.MODIFY
            post_marker.id = i
            post_marker.ns = "rel_goal_parts"
            color = ColorRGBA()
            color.r = 1.0
            color.g = 1.0
            color.b = 1.0
            color.a = 1.0
            post_marker.color = color
            post_marker.lifetime = rospy.Duration(nsecs=self.goal_lifetime)
            scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
            post_marker.scale = scale
            post_marker.header = msg.header

            arr.append(post_marker)
            i += 1

        self.goal_parts_marker.markers = arr
        self.marker_array_publisher.publish(self.goal_parts_marker)

    def obstacle_cb(self, msg: ObstacleRelativeArray):
        for idx, obstacle in enumerate(msg.obstacles):
            self.marker_obstacle.header = msg.header
            self.marker_obstacle.id = idx
            self.obstacle_color.a = obstacle.pose.confidence
            # color depding on type of obstacle
            if obstacle.type == obstacle.ROBOT_CYAN:
                self.obstacle_color.r = 0.0
                self.obstacle_color.g = 0.0
                self.obstacle_color.b = 1.0
            elif obstacle.type == obstacle.ROBOT_MAGENTA:
                self.obstacle_color.r = 1.0
                self.obstacle_color.g = 0.0
                self.obstacle_color.b = 0.0
            elif obstacle.type == obstacle.ROBOT_UNDEFINED:
                self.obstacle_color.r = 0.0
                self.obstacle_color.g = 1.0
                self.obstacle_color.b = 0.0
            else:
                self.obstacle_color.r = 0.5
                self.obstacle_color.g = 0.5
                self.obstacle_color.b = 0.5
            self.marker_obstacle.color = self.obstacle_color

            # size if provided
            if obstacle.width > 0.0:
                scale = Vector3(obstacle.width, obstacle.width, self.obstacle_height)
            else:
                scale = Vector3(self.obstacle_def_width, self.obstacle_def_width, self.obstacle_height)
            self.marker_obstacle.scale = scale

            # position
            self.obstacle_pose.position = obstacle.pose.pose.pose.position
            self.obstacle_pose.position.z = self.obstacle_height / 2
            self.marker_obstacle.pose = self.obstacle_pose

            self.marker_publisher.publish(self.marker_obstacle)


if __name__ == "__main__":
    marker_node = ShowRobocupObjects()
