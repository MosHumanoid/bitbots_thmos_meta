from io import BytesIO
import math
import rospy
from humanoid_league_msgs.msg import HeadMode as HeadModeMsg
from bitbots_msgs.msg import JointCommand
from bitbots_head_behavior.collision_checker import CollisionChecker
import tf2_ros as tf2

class HeadCapsule:
    def __init__(self, blackboard):
        self.blackboard = blackboard

        # possible variables
        self.head_mode = None

        # preparing message for more performance
        self.pos_msg = JointCommand()
        # self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.pos_msg.joint_names = ["neck", "head"]
        self.pos_msg.positions = [0, 0]
        self.pos_msg.velocities = [0, 0]
        self.pos_msg.accelerations = [17, 17]
        self.pos_msg.max_currents = [-1, -1]

        self.position_publisher = None  # type: rospy.Publisher
        self.visual_compass_record_trigger = None  # type: rospy.Publisher

        self.tf_buffer = tf2.Buffer(rospy.Duration(5))
        # tf_listener is necessary, even though unused!
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        self.current_head_position = [0, 0]

        self.collision_checker = CollisionChecker()

    def head_mode_callback(self, msg):
        """
        ROS Subscriber callback for /head_mode message.
        Saves the messages head mode on the blackboard
        """
        self.head_mode = msg.headMode

    #################
    # Head position #
    #################

    def _calculate_lower_speed(self, delta_fast_joint, delta_my_joint, speed):
        """
        Calculates the speed for the axis with the non maximum velocity.
        :param delta_fast_joint: The radians delta of the faster joint.
        :param delta_my_joint: The radians delta of the joint that should be slowed down.
        :param speed: The speed of the faster joint.
        :return: Speed of this slower joint.
        """
        estimated_time = delta_fast_joint / speed
        # Prevent zero division if goal is reached
        if estimated_time != 0:
            return delta_my_joint / estimated_time
        else:
            return 0

    def send_motor_goals(self, pan_position, tilt_position, pan_speed=1.5, tilt_speed=1.5, current_pan_position=None, current_tilt_position=None, clip=True):
        """
        :param pan_position: pan in radians
        :param tilt_position: tilt in radians
        :param pan_speed:
        :param tilt_speed:
        :param clip: clip the motor values at the maximum value. This should almost always be true.
        :param current_pan_position: Current pan joint state for better interpolation (only active if both joints are set).
        :param current_tilt_position: Current tilt joint state for better interpolation (only active if both joints are set).
        :return: False if the target position collides, True otherwise
        """
        rospy.logdebug("target pan/tilt: {}/{}".format(pan_position, tilt_position))

        if clip:
            pan_position, tilt_position = self.pre_clip(pan_position, tilt_position)

        # Check if we should use the better interpolation
        if current_pan_position and current_tilt_position:
            # Calculate the deltas
            delta_pan = abs(current_pan_position - pan_position)
            delta_tilt = abs(current_tilt_position - tilt_position)
            # Check which speed should be lowred to achieve better interpolation
            if delta_pan > delta_tilt:
                tilt_speed = self._calculate_lower_speed(delta_pan, delta_tilt, pan_speed)
            else:
                pan_speed = self._calculate_lower_speed(delta_tilt, delta_pan, tilt_speed)

        # Check for collision
        self.collision_checker.set_head_motors(pan_position, tilt_position)
        if self.collision_checker.check_collision():
            pan = round(math.degrees(pan_position), 2)
            tilt = round(math.degrees(tilt_position), 2)
            rospy.logwarn(f"Colliding head position: {pan}°, {tilt}°. Not moving.")
            return False
        else:
            self.pos_msg.positions = pan_position, tilt_position
            self.pos_msg.velocities = [pan_speed, tilt_speed]
            self.pos_msg.header.stamp = rospy.Time.now()
            self.position_publisher.publish(self.pos_msg)
            return True

    def pre_clip(self, pan, tilt):
        """
        Return clipped motor goals for each axis

        :param pan: The goal pan position
        :param tilt: The goal tilt position
        :return (new_pan, new_tilt): Clipped motor goals
        """
        max_pan = self.blackboard.config['max_pan']
        max_tilt = self.blackboard.config['max_tilt']
        new_pan = min(max(pan, min(max_pan)), max(max_pan))
        new_tilt = min(max(tilt, min(max_tilt)), max(max_tilt))
        return new_pan, new_tilt

    ##################
    # Head positions #
    ##################

    def joint_state_callback(self, msg):
        buf = BytesIO()
        msg.serialize(buf)
        self.collision_checker.set_joint_states(buf.getvalue())
        # head_pan = msg.position[msg.name.index('HeadPan')]
        # head_tilt = msg.position[msg.name.index('HeadTilt')]
        head_pan = msg.position[msg.name.index('neck')]
        head_tilt = msg.position[msg.name.index('head')]
        self.current_head_position = [head_pan, head_tilt]

    def get_head_position(self):
        return self.current_head_position

    #####################
    # Pattern generator #
    #####################


    def _lineAngle(self, line, line_count, min_angle, max_angle):
        """
        Converts a scanline number to an tilt angle
        """
        delta = abs(min_angle - max_angle)
        steps = delta / (line_count - 1)
        value = steps * line + min_angle
        return value

    def _calculateHorizontalAngle(self, is_right, angle_right, angle_left):
        """
        The right/left position to an pan angle
        """
        if is_right:
            return angle_right
        else:
            return angle_left

    def _interpolatedSteps(self, steps, tilt, min_pan, max_pan):
        """
        Splits a scanline in a number of dedicated steps
        """
        if steps == 0:
           return []
        steps += 1
        delta = abs(min_pan - max_pan)
        step_size = delta / float(steps)
        output_points = list()
        for i in range(1, steps):
            value = int(i * step_size + min_pan)
            point = (value, tilt)
            output_points.append(point)
        return output_points

    def generate_pattern(self, lineCount, maxHorizontalAngleLeft, maxHorizontalAngleRight, maxVerticalAngleUp, maxVerticalAngleDown, reduce_last_scanline=1, interpolation_steps=0):
        """
        :param lineCount: Number of scanlines
        :param maxHorizontalAngleLeft: maximum look left angle
        :param maxHorizontalAngleRight: maximum look right angle
        :param maxVerticalAngleUp: maximum upwards angle
        :param maxVerticalAngleDown: maximum downwards angle
        :param interpolation_steps: number of interpolation steps for each line
        :return: List of angles (Pan, Tilt)
        """
        keyframes = []
        # Init first state
        downDirection = False
        rightSide = False
        rightDirection = True
        line = lineCount - 1
        # Calculate number of keyframes
        iterations = max((2 * lineCount - 2) * 2, 2)

        for i in range(iterations):
            # Create keyframe
            currentPoint = (self._calculateHorizontalAngle(rightSide, maxHorizontalAngleRight, maxHorizontalAngleLeft),
                            self._lineAngle(line, lineCount, maxVerticalAngleDown, maxVerticalAngleUp))
            # Add keyframe
            keyframes.append(currentPoint)

            # Interpolate to next keyframe if we are moving horizontally
            if rightSide != rightDirection:
                interpolatedKeyframes = self._interpolatedSteps(interpolation_steps, currentPoint[1], maxHorizontalAngleRight, maxHorizontalAngleLeft)
                if rightDirection:
                    interpolatedKeyframes.reverse()
                keyframes.extend(interpolatedKeyframes)

            # Next state
            # Switch side
            if rightSide != rightDirection:
                rightSide = rightDirection
            # Or go up/down
            elif rightSide == rightDirection:
                rightDirection = not rightDirection
                if line in [0, lineCount - 1]:
                    downDirection = not downDirection
                if downDirection:
                    line -= 1
                else:
                    line += 1

        # Reduce the with of the last scanline if wanted.
        for index, keyframe in enumerate(keyframes):
            if keyframe[1] == maxVerticalAngleDown:
                keyframes[index] = (keyframe[0] * reduce_last_scanline, maxVerticalAngleDown)

        return keyframes
