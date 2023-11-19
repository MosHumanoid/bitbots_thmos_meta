#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
import math
import os

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand

import sys, select, termios, tty
import actionlib
from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
'''
__ids__ = [
    "HeadPan",
    "HeadTilt",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbow",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbow",
    "LHipYaw",
    "LHipRoll",
    "LHipPitch",
    "LKnee",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipYaw",
    "RHipRoll",
    "RHipPitch",
    "RKnee",
    "RAnklePitch",
    "RAnkleRoll"
]
'''
__ids__ = [
    "neck", 
    "head", 
    "L_arm_1", 
    "L_arm_2", 
    "L_arm_3", 
    "R_arm_1", 
    "R_arm_2", 
    "R_arm_3",                                
    "L_leg_1", 
    "L_leg_2", 
    "L_leg_3", 
    "L_leg_4", 
    "L_leg_5", 
    "L_leg_6",                                
    "R_leg_1", 
    "R_leg_2", 
    "R_leg_3", 
    "R_leg_4", 
    "R_leg_5", 
    "R_leg_6"
]
__velocity__ = 5.0
__accelerations__ = -1.0
__max_currents__ = -1.0

walkready = JointCommand(
    joint_names=__ids__,
    velocities=[__velocity__] * len(__ids__),
    accelerations=[__accelerations__] * len(__ids__),
    max_currents=[__max_currents__] * len(__ids__),
    positions=[
        0.0,  # HeadPan
        0.0,  # HeadTilt
        0.0,  # LShoulderPitch
        0.0,  # LShoulderRoll
        0.79,  # LElbow
        0.0,  # RShoulderPitch
        0.0,  # RShoulderRoll
        -0.79,  # RElbow
        -0.0112,  # LHipYaw
        0.0615,  # LHipRoll
        0.4732,  # LHipPitch
        1.0058,  # LKnee
        -0.4512,  # LAnklePitch
        0.0625,  # LAnkleRoll
        0.0112,  # RHipYaw
        -0.0615,  # RHipRoll
        -0.4732,  # RHipPitch
        -1.0059,  # RKnee
        0.4512,  # RAnklePitch
        -0.0625,  # RAnkleRoll
    ])
msg = """
BitBots Teleop
--------------
Walk around:            Move head:
    q    w    e         u    i    o
    a    s    d         j    k    l
                        m    ,    .  

q/e: turn left/right    k: zero head position
a/d: left/rigth         i/,: up/down
w/s: forward/back       j/l: left/right
                        u/o/m/.: combinations     

Controls increase / decrease with multiple presses.
SHIFT increases with factor 10

y: kick left    Y: walk kick left
c: kick right   C: walk kick right

f: play walkready animation
r: reset robot in simulation
R: reset ball in simulation

CTRL-C to quit




"""

moveBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    'W': (10, 0, 0),
    'S': (-10, 0, 0),
    'A': (0, 10, 0),
    'D': (0, -10, 0),
    'Q': (0, 0, 10),
    'E': (0, 0, -10),
}
headBindings = {
    'u': (1, 1),
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'm': (-1, 1),
    ',': (-1, 0),
    '.': (-1, -1),
    'U': (10, 10),
    'I': (10, 0),
    'O': (10, -10),
    'J': (0, 10),
    'L': (0, -10),
    'M': (-10, 10),
    ';': (-10, 0),
    ':': (-10, -10)
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


head_pan_pos = 0
head_tilt_pos = 0


def joint_state_cb(msg):
    global head_pan_pos
    global head_tilt_pos

    if "neck" in msg.name and "head" in msg.name:
        head_pan_pos = msg.position[msg.name.index("neck")]
        head_tilt_pos = msg.position[msg.name.index("head")]


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    # Walking Part
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=True)

    x_speed_step = 0.01
    y_speed_step = 0.01
    turn_speed_step = 0.01

    x = 0
    y = 0
    th = 0
    status = 0

    # Head Part
    rospy.Subscriber("joint_states", JointState, joint_state_cb, queue_size=1)
    '''
    if rospy.get_param("sim_active", default=False):
        head_pub = rospy.Publisher("head_motor_goals", JointCommand, queue_size=1)
    else:
        head_pub = rospy.Publisher("DynamixelController/command", JointCommand, queue_size=1)
    '''
    head_pub = rospy.Publisher("DynamixelController/command", JointCommand, queue_size=1)
    head_msg = JointCommand()
    head_msg.max_currents = [-1] * 2
    head_msg.velocities = [5] * 2
    head_msg.accelerations = [40] * 2
    head_msg.joint_names = ['neck', 'head']
    head_msg.positions = [0] * 2

    head_pan_step = 0.05
    head_tilt_step = 0.05

    walk_kick_pub = rospy.Publisher("kick", Bool, queue_size=1)
    joint_pub = rospy.Publisher("DynamixelController/command", JointCommand, queue_size=1)

    reset_robot = rospy.ServiceProxy("/initial_pose", Empty)
    reset_ball = rospy.ServiceProxy("/reset_ball", Empty)

    print(msg)

    goal_left = KickGoal()
    goal_left.header.stamp = rospy.Time.now()
    frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
    goal_left.header.frame_id = frame_prefix + "base_footprint"
    goal_left.ball_position.x = 0.2
    goal_left.ball_position.y = 0.1
    goal_left.ball_position.z = 0
    goal_left.kick_direction = Quaternion(*quaternion_from_euler(0, 0, 0))
    goal_left.kick_speed = 1

    goal_right = KickGoal()
    goal_right.header.stamp = rospy.Time.now()
    frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
    goal_right.header.frame_id = frame_prefix + "base_footprint"
    goal_right.ball_position.x = 0.2
    goal_right.ball_position.y = -0.1
    goal_right.ball_position.z = 0
    goal_right.kick_direction = Quaternion(*quaternion_from_euler(0, 0, 0))
    goal_right.kick_speed = 1

    client = actionlib.SimpleActionClient('dynamic_kick', KickAction)

    try:
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x += moveBindings[key][0] * x_speed_step

                x = round(x, 2)
                y += moveBindings[key][1] * y_speed_step
                y = round(y, 2)
                th += moveBindings[key][2] * turn_speed_step
                th = round(th, 2)
            elif key in headBindings.keys():
                head_msg.positions[0] = head_pan_pos + headBindings[key][1] * head_pan_step
                head_msg.positions[1] = head_tilt_pos + headBindings[key][0] * head_tilt_step
                head_pub.publish(head_msg)
            elif key == 'k' or key == 'K':
                # put head back in init
                head_msg.positions[0] = 0
                head_msg.positions[1] = 0
                head_pub.publish(head_msg)
            elif key == 'y':
                # kick left
                client.send_goal(goal_left)
            elif key == 'c':
                # kick right
                client.send_goal(goal_right)
            elif key == 'Y':
                # kick left walk
                walk_kick_pub.publish(False)
            elif key == 'C':
                # kick right walk
                walk_kick_pub.publish(True)
            elif key == 'f':
                # play walkready animation
                walkready.header.stamp = rospy.Time.now()
                joint_pub.publish(walkready)
            elif key == 'r':
                # reset robot in sim
                try:
                    reset_robot()
                except:
                    pass
            elif key == 'R':
                # reset ball in sim
                try:
                    reset_ball()
                except:
                    pass
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th
            pub.publish(twist)
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")

    except Exception as e:
        print(e)

    finally:
        print("\n")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
