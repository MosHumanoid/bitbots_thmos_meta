#!/usr/bin/env python3
import termios
import sys
import tty
import rospy
import socket
import pickle
import base64
import select
import time
from threading import Thread
from udp_bridge.aes_helper import AESCipher


class UdpReceiver:
    def __init__(self):
        port = rospy.get_param("udp_bridge/port")
        rospy.loginfo("Initializing udp_bridge on port " + str(port))

        self.sock = socket.socket(type=socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(1)

        self.known_senders = []  # type: list

        self.cipher = AESCipher(rospy.get_param("udp_bridge/encryption_key", None))

        self.publishers = {}

    def recv_message(self):
        """
        Receive a message from the network, process it and publish it into ROS
        """
        self.sock.settimeout(1)
        acc = bytes()
        while not rospy.is_shutdown():
            try:
                acc += self.sock.recv(10240)

                if acc[-3:] == b'\xff\xff\xff':  # our package delimiter
                    self.handle_message(acc[:-3])
                    acc = bytes()

            except socket.timeout:
                pass

    def handle_message(self, msg: bytes):
        """
        Handle a new message which came in from the socket
        """
        try:
            dec_msg = self.cipher.decrypt(msg)
            bin_msg = base64.b64decode(dec_msg)
            deserialized_msg = pickle.loads(bin_msg)

            data = deserialized_msg['data']
            topic = deserialized_msg['topic']
            hostname = deserialized_msg['hostname']

            if hostname not in self.known_senders:
                self.known_senders.append(hostname)

            self.publish(topic, data, hostname)
        except Exception as e:
            rospy.logerr('Could not deserialize received message with error {}'.format(str(e)))

    def publish(self, topic: str, msg, hostname: str):
        """
        Publish a message into ROS

        :param topic: The topic on which the message was sent on the originating host
        :param msg: The ROS message which was sent on the originating host
        :param hostname: The hostname of the originating host
        """

        # publish msg under host namespace
        namespaced_topic = "{}/{}".format(hostname, topic).replace("//", "/")

        # create a publisher object if we don't have one already
        if namespaced_topic not in self.publishers.keys():
            rospy.loginfo('Publishing new topic {}'.format(namespaced_topic))
            self.publishers[namespaced_topic] = rospy.Publisher(namespaced_topic, type(msg), tcp_nodelay=True,
                                                                queue_size=5, latch=True)

        self.publishers[namespaced_topic].publish(msg)


def validate_params() -> bool:
    result = True
    if not rospy.has_param("udp_bridge"):
        rospy.logfatal("parameter 'udp_bridge' not found")
        result = False

    if not rospy.has_param("udp_bridge/port"):
        rospy.logfatal("parameter 'udp_bridge/port' not found")
        result = False
    if not isinstance(rospy.get_param("udp_bridge/port"), int):
        rospy.logfatal("parameter 'udp_bridge/port' is not an Integer")
        result = False

    return result


def main():
    if validate_params():
        rospy.init_node("udp_bridge_receiver")
        # setup udp receiver
        receiver = UdpReceiver()

        while not rospy.is_shutdown():
            receiver.recv_message()


if __name__ == '__main__':
    main()
