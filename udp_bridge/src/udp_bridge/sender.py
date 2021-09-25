#!/usr/bin/env python3
import rospy
import rostopic
import socket
import pickle
import base64
from threading import Lock
from queue import Queue, Empty, Full

from udp_bridge.aes_helper import AESCipher


class AutoSubscriber:
    """
    A class which automatically subscribes to a topic as soon as it becomes available and buffers received messages
    in a queue.
    """

    def __init__(self, topic, queue_size):
        """
        :param topic: Topic to subscribe to
        :type topic: str
        :param queue_size: How many received messages should be buffered
        :type queue_size: int
        """
        self.topic = topic
        self.queue = Queue(queue_size)

        self.__subscriber = None        # type: rospy.Subscriber
        self.__subscribe()

    def __subscribe(self, backoff=1.0):
        """
        Try to subscribe to the set topic
        :param backoff: How long to wait until another try
        """
        data_class, _, _ = rostopic.get_topic_class(self.topic)
        if data_class is not None:
            # topic is known
            self.__subscriber = rospy.Subscriber(self.topic, data_class, self.__message_callback,
                                                 queue_size=1, tcp_nodelay=True)
            rospy.loginfo('Subscribed to topic {}'.format(self.topic))

        else:
            # topic is not yet known
            rospy.loginfo('Topic {} is not yet known. Retrying in {} seconds'.format(self.topic, int(backoff)))
            if backoff > 30:
                backoff = 30
            rospy.Timer(
                rospy.Duration(int(backoff)),
                lambda event: self.__subscribe(backoff * 1.2),
                oneshot=True
            )

    def __message_callback(self, data):
        serialized_data = base64.b64encode(pickle.dumps({
            "data": data,
            "topic": self.topic,
            "hostname": hostname
        }, pickle.HIGHEST_PROTOCOL)).decode("ASCII")
        enc_data = cipher.encrypt(serialized_data)

        try:
            self.queue.put(enc_data, block=True, timeout=0.5)
        except Full:
            rospy.logwarn_throttle(5, 'Could enqueue new message of topic {}. Queue full.'.format(self.topic))


def validate_params():
    """:rtype: bool"""
    result = True
    if not rospy.has_param("udp_bridge"):
        rospy.logfatal("parameter 'udp_bridge' not found")
        result = False

    if not rospy.has_param("udp_bridge/target_ips"):
        rospy.logfatal("parameter 'udp_bridge/target_ips' not found")
        result = False
    target_ips = rospy.get_param("udp_bridge/target_ips")
    if not isinstance(target_ips, list):
        rospy.logfatal("parameter udp_bridge/target_ips is not a list")
        result = False
    for addr in target_ips:
        try:
            socket.inet_aton(addr)
        except Exception as e:
            rospy.logfatal("Cannot parse " + str(addr) + " as IP Address")
            result = False

    if not rospy.has_param("udp_bridge/port"):
        rospy.logfatal("parameter 'udp_bridge/port' not found")
        result = False
    if not isinstance(rospy.get_param("udp_bridge/port"), int):
        rospy.logfatal("parameter 'udp_bridge/port' is not an Integer")
        result = False

    if not rospy.has_param("udp_bridge/topics"):
        rospy.logfatal("parameter 'udp_bridge/port' not found")
        result = False
    if not isinstance(rospy.get_param("udp_bridge/topics"), list):
        rospy.logfatal("parameter 'udp_bridge/topics' is not a list")
        result = False
    if len(rospy.get_param("udp_bridge/topics")) == 0:
        rospy.logwarn("parameter 'udp_bridge/topics' is an empty list")

    if not rospy.has_param('udp_bridge/sender_queue_max_size'):
        rospy.logfatal('parameter \'sender_queue_max_size\' not found')
        result = False
    if not isinstance(rospy.get_param('udp_bridge/sender_queue_max_size'), int):
        rospy.logfatal('parameter \'sender_queue_max_size\' is not an Integer')
        result = False

    if not rospy.has_param('udp_bridge/send_frequency'):
        rospy.logfatal('parameter \'send_frequency\' not found')
        result = False
    if not isinstance(rospy.get_param('udp_bridge/send_frequency'), float) \
        and not isinstance(rospy.get_param('udp_bridge/send_frequency'), int):
        rospy.logfatal('parameter \'send_frequency\' is not an Integer or Float')
        result = False

    return result


if __name__ == '__main__':
    if validate_params():
        rospy.init_node('udp_bridge_sender',log_level=rospy.INFO)

        hostname = socket.gethostname()
        cipher = AESCipher(rospy.get_param("udp_bridge/encryption_key", None))
        port = rospy.get_param("udp_bridge/port")
        freq = rospy.get_param("udp_bridge/send_frequency")
        targets = rospy.get_param('udp_bridge/target_ips')
        max_queue_size = rospy.get_param('udp_bridge/sender_queue_max_size')
        topics = rospy.get_param("udp_bridge/topics")

        sock = socket.socket(type=socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        subscribers = []

        for topic in topics:
            subscribers.append(AutoSubscriber(topic, max_queue_size))

        while not rospy.is_shutdown():
            for subscriber in subscribers:
                try:
                    data = subscriber.queue.get_nowait()

                    for target in targets:
                        try:
                            sock.sendto(data + b'\xff\xff\xff', (target, port))
                        except Exception as e:
                            rospy.logerr('Could not send data of topic {} to {} with error {}'
                                         .format(subscriber.topic, target, str(e)))

                except Empty:
                    pass

            rospy.sleep(rospy.Duration(0, int(1000000000 / freq)))

