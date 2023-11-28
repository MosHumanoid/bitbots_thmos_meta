import os
import math
import time

from controller import Robot, Node, Field

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from rosgraph_msgs.msg import Clock

from bitbots_msgs.msg import JointCommand, FootPressure

CAMERA_DIVIDER = 8  # every nth timestep an image is published, this is n


class RobotController:
    def __init__(self, ros_active=False, robot='wolfgang', do_ros_init=True, external_controller=False, base_ns='',
                 recognize=False, camera_active=True):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param do_ros_init: Whether to call rospy.init_node (only used when ros_active is True)
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_active = ros_active
        self.recognize = recognize
        self.camera_active = camera_active
        if not external_controller:
            self.robot_node = Robot()
        self.walkready = [0] * 20
        self.time = 0
        self.clock_msg = Clock()
        self.motors = []
        self.sensors = []
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.switch_coordinate_system = True
        self.is_wolfgang = False
        self.pressure_sensors = None
        
        if robot == 'thmos':
            self.is_wolfgang = False
            self.motor_names = ["neck", "head", "L_arm_1", "L_arm_2", "L_arm_3", "R_arm_1", "R_arm_2", "R_arm_3",
                                "L_leg_1", "L_leg_2", "L_leg_3", "L_leg_4", "L_leg_5", "L_leg_6",  
                                "R_leg_1", "R_leg_2", "R_leg_3", "R_leg_4", "R_leg_5", "R_leg_6"]
 
            self.external_motor_names = self.motor_names
            self.sensor_suffix = "_sensor"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            camera_name = "Camera"
            inertial_name = "Imu" 

        for motor_name in self.motor_names:
            self.motors.append(self.robot_node.getDevice(motor_name))
            self.motors[-1].enableTorqueFeedback(self.timestep)
            self.sensors.append(self.robot_node.getDevice(motor_name + self.sensor_suffix))
            self.sensors[-1].enable(self.timestep)

        self.accel = self.robot_node.getDevice(accel_name)
        self.accel.enable(self.timestep)
        self.gyro = self.robot_node.getDevice(gyro_name)
        self.gyro.enable(self.timestep)
        self.inertial = self.robot_node.getDevice(inertial_name)
        self.inertial.enable(self.timestep)
        self.camera = self.robot_node.getDevice(camera_name)
        self.camera_counter = 0
        if self.camera_active:
            self.camera.enable(self.timestep*CAMERA_DIVIDER)
        if self.recognize:
            self.camera.recognitionEnable(self.timestep)
            self.last_img_saved = 0.0
            self.img_save_dir = "/tmp/webots/images" +\
                                time.strftime("%Y-%m-%d-%H-%M-%S") +\
                                os.getenv('WEBOTS_ROBOT_NAME')
            if not os.path.exists(self.img_save_dir):
                os.makedirs(self.img_save_dir)

        # set zero points
        self.set_arms_zero()
        self.set_legs_zero()

        if self.ros_active:
            if base_ns == "":
                clock_topic = "/clock"
            else:
                clock_topic = base_ns + "clock"
            if do_ros_init:
                rospy.init_node("webots_ros_interface", argv=['clock:=' + clock_topic])
            self.clock_publisher = rospy.Publisher(clock_topic, Clock, queue_size=1)
            self.l_sole_frame = rospy.get_param("~l_sole_frame", "l_sole")
            self.r_sole_frame = rospy.get_param("~r_sole_frame", "r_sole")
            self.camera_optical_frame = rospy.get_param("~camera_optical_frame", "camera_optical_frame")
            self.head_imu_frame = rospy.get_param("~head_imu_frame", "imu_frame_2")
            self.imu_frame = rospy.get_param("~imu_frame", "imu_frame")
            self.pub_js = rospy.Publisher(base_ns + "joint_states", JointState, queue_size=1)
            self.pub_imu = rospy.Publisher(base_ns + "imu/data_raw", Imu, queue_size=1)

            self.pub_imu_head = rospy.Publisher(base_ns + "imu_head/data", Imu, queue_size=1)
            self.pub_cam = rospy.Publisher(base_ns + "usb_cam/image_raw", Image, queue_size=1)
            self.pub_cam_info = rospy.Publisher(base_ns + "camera/camera_info", CameraInfo, queue_size=1, latch=True)

            self.pub_pres_left = rospy.Publisher(base_ns + "foot_pressure_left/filtered", FootPressure, queue_size=1)
            self.pub_pres_right = rospy.Publisher(base_ns + "foot_pressure_right/filtered", FootPressure, queue_size=1)
            self.cop_l_pub_ = rospy.Publisher(base_ns + "cop_l", PointStamped, queue_size=1)
            self.cop_r_pub_ = rospy.Publisher(base_ns + "cop_r", PointStamped, queue_size=1)
            
            rospy.Subscriber(base_ns + "walking_motor_goals", JointCommand, self.command_cb, queue_size=1)
            rospy.Subscriber(base_ns + "head_motor_goals", JointCommand, self.command_head_cb)
            rospy.Subscriber(base_ns + "kick_motor_goals", JointCommand, self.command_kick_cb)
            rospy.Subscriber(base_ns + "/dynup_motor_goals", JointCommand, self.command_dynup_cb)
            
            # publish camera info once, it will be latched
            self.cam_info = CameraInfo()
            self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
            self.cam_info.header.frame_id = self.camera_optical_frame
            self.cam_info.height = self.camera.getHeight()
            self.cam_info.width = self.camera.getWidth()
            f_y = self.mat_from_fov_and_resolution(
                self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
                self.cam_info.height)
            f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
            self.cam_info.K = [f_x, 0, self.cam_info.width / 2,
                               0, f_y, self.cam_info.height / 2,
                               0, 0, 1]
            self.cam_info.P = [f_x, 0, self.cam_info.width / 2, 0,
                               0, f_y, self.cam_info.height / 2, 0,
                               0, 0, 1, 0]
            self.pub_cam_info.publish(self.cam_info)

        if robot == "op3":
            # start pose
            command = JointCommand()
            command.joint_names = ["r_sho_roll", "l_sho_roll"]
            command.positions = [-math.tau/8, math.tau/8]
            self.command_cb(command)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_ros()

    def publish_ros(self):
        self.publish_imu()
        self.publish_joint_states()
        self.publish_clock()
        if self.camera_active and self.camera_counter == 0:
            self.publish_camera()
        #self.publish_pressure()
        if self.recognize:
            self.save_recognition()
        self.camera_counter = (self.camera_counter + 1) % CAMERA_DIVIDER

    def command_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                self.motors[motor_index].setPosition(command.positions[i])
                if len(command.velocities) == 0 or command.velocities[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                if not len(command.accelerations) == 0:
                    self.motors[motor_index].setAcceleration(command.accelerations[i])

            except ValueError:
                print(f"invalid motor specified ({name})")

    def command_head_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                self.motors[motor_index].setPosition(command.positions[i])
                if len(command.velocities) == 1.57320 or command.velocities[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                if not len(command.accelerations) == 0:
                    self.motors[motor_index].setAcceleration(command.accelerations[i])

            except ValueError:
                print(f"invalid motor specified ({name})")   

    def command_kick_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                self.motors[motor_index].setPosition(command.positions[i])
                if len(command.velocities) == 0 or command.velocities[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                if not len(command.accelerations) == 0:
                    self.motors[motor_index].setAcceleration(command.accelerations[i])

            except ValueError:
                print(f"invalid motor specified ({name})") 

    def command_dynup_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                sim_bug = 1
                sim_pi = 0
                if i == 3:
                    sim_bug = -1
                    sim_pi = np.pi*0.4
                if i == 6:
                    sim_bug = -1
                    sim_pi = -np.pi*0.4
                self.motors[motor_index].setPosition(sim_bug * command.positions[i] + sim_pi)
                if len(command.velocities) == 0 or command.velocities[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                if not len(command.accelerations) == 0:
                    self.motors[motor_index].setAcceleration(command.accelerations[i])

            except ValueError:
                print(f"invalid motor specified ({name})") 

    def set_head_tilt(self, pos):
        self.motors[-1].setPosition(pos)

    def set_arms_zero(self):
        positions = [0, 0, 0, 
                     0, 0, 0]
        for i in range(2, 8):
            self.motors[i].setPosition(positions[i-2])

    def set_legs_zero(self):
        positions = [-0.05101498763939969, -0.4764168173419817, 0.006769736355572479, 0.8049079656668593, -0.45333041272529706, -0.04828855321310417, 
                    0.051024988764736795, 0.47641599257384537, -0.006771592248384993, -0.8049033115711886, 0.45332676273253447, 0.04830260921860783]
        for i in range (8,19):
            self.motors[i].setPosition(positions[i-8])

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        for i in range(len(self.sensors)):
            js.name.append(self.external_motor_names[i])
            value = self.sensors[i].getValue()
            js.position.append(value)
            js.effort.append(self.motors[i].getTorqueFeedback())
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)
        if head:
            msg.header.frame_id = self.head_imu_frame
        else:
            msg.header.frame_id = self.imu_frame

        # change order because webots has different axis

        accel_vels = self.accel.getValues()
        msg.linear_acceleration.x = accel_vels[0]
        msg.linear_acceleration.y = accel_vels[1]
        msg.linear_acceleration.z = accel_vels[2]

        gyro_vels = self.gyro.getValues()
        msg.angular_velocity.x = gyro_vels[0]
        msg.angular_velocity.y = gyro_vels[1]
        msg.angular_velocity.z = gyro_vels[2]
        
        # rpy to caculate rotate matrix
        rpy_vels = self.inertial.getRollPitchYaw()
        rpy_x = rpy_vels[0] + np.pi * 0.5
        rpy_y = -rpy_vels[1]
        rpy_z = rpy_vels[2]
        
        Rx = np.array([[1,0,0],[0,np.cos(rpy_x),-np.sin(rpy_x)],[0,np.sin(rpy_x),np.cos(rpy_x)]])
        Ry = np.array([[np.cos(rpy_y),0,np.sin(rpy_y)],[0,1,0],[-np.sin(rpy_y),0,np.cos(rpy_y)]])
        Rz = np.array([[np.cos(rpy_z),-np.sin(rpy_z),0],[np.sin(rpy_z),np.cos(rpy_z),0],[0,0,1]])
        RR = np.matmul(np.matmul(Rz , Ry) , Rx)
        RR = RR.flatten()
        msg.orientation_covariance = RR.tolist()  
        
        return msg

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg(head=False))
        if self.is_wolfgang:
            self.pub_imu_head.publish(self.get_imu_msg(head=True))

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.header.frame_id = self.camera_optical_frame
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

    def save_recognition(self):
        if self.time - self.last_img_saved < 1.0:
            return
        self.last_img_saved = self.time
        annotation = ""
        img_stamp = f"{self.time:.2f}".replace(".", "_")
        img_name = f"img_{os.getenv('WEBOTS_ROBOT_NAME')}_{img_stamp}.PNG"
        recognized_objects = self.camera.getRecognitionObjects()
        # variables for saving not in image later
        found_ball = False
        found_wolfgang = False
        for e in range(self.camera.getRecognitionNumberOfObjects()):
            model = recognized_objects[e].get_model()
            position = recognized_objects[e].get_position_on_image()
            size = recognized_objects[e].get_size_on_image()
            if model == b"soccer ball":
                found_ball = True
                vector = f"""{{"x1": {position[0] - 0.5*size[0]}, "y1": {position[1] - 0.5*size[1]}, "x2": {position[0] + 0.5*size[0]}, "y2": {position[1] + 0.5*size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "ball|"
                annotation += vector
                annotation += "\n"
            if model == b"wolfgang":
                found_wolfgang = True
                vector = f"""{{"x1": {position[0] - 0.5*size[0]}, "y1": {position[1] - 0.5*size[1]}, "x2": {position[0] + 0.5*size[0]}, "y2": {position[1] + 0.5*size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "robot|"
                annotation += vector
                annotation += "\n"
        if not found_ball:
            annotation +=  f"{img_name}|ball|not in image\n"
        if not found_wolfgang:
            annotation += f"{img_name}|robot|not in image\n"
        with open(os.path.join(self.img_save_dir, "annotations.txt"), "a") as f:
            f.write(annotation)
        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name), quality=100)

    def get_image(self):
        return self.camera.getImage()

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)
