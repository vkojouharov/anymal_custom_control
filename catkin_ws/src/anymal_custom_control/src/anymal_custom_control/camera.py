"""CameraReceiver — subscribe to ANYmal camera topics and get frames as numpy arrays."""

import threading
import time

import cv2 as _cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class CameraReceiver:
    """Subscribe to a ROS Image topic and provide frames as numpy arrays.

    Usage:
        cam = CameraReceiver('/wide_angle_camera_front/image_raw')
        cam = CameraReceiver('/wide_angle_camera_front/image_raw/compressed', compressed=True)
        time.sleep(1)  # wait for first frame
        frame = cam.get_frame()  # numpy array (H, W, 3) BGR uint8
    """

    def __init__(self, topic, compressed=False):
        """
        Args:
            topic: ROS image topic to subscribe to.
            compressed: If True, subscribe to CompressedImage (JPEG) instead of raw Image.
                        Use for lower bandwidth over the network.
        """
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = None
        self._frame_count = 0
        self._topic = topic
        self._compressed = compressed

        if compressed:
            self._sub = rospy.Subscriber(topic, CompressedImage, self._callback_compressed, queue_size=1)
        else:
            self._sub = rospy.Subscriber(topic, Image, self._callback, queue_size=1)
        rospy.loginfo("CameraReceiver: subscribed to %s%s", topic, " (compressed)" if compressed else "")

    def _callback(self, msg):
        try:
            if msg.encoding == '16UC1':
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn("CameraReceiver: failed to convert image: %s", e)
            return

        with self._lock:
            self._latest_frame = frame
            self._latest_stamp = msg.header.stamp
            self._frame_count += 1

    def _callback_compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = _cv2.imdecode(np_arr, _cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logwarn("CameraReceiver: failed to decode compressed image: %s", e)
            return

        with self._lock:
            self._latest_frame = frame
            self._latest_stamp = msg.header.stamp
            self._frame_count += 1

    def get_frame(self):
        """Get the latest camera frame as a numpy array.

        Returns:
            numpy.ndarray or None: BGR uint8 array (H, W, 3) for color topics,
                                   uint16 array (H, W) for depth topics,
                                   or None if no frame received yet.
        """
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    def get_depth(self):
        """Alias for get_frame() — use with depth image topics.

        Returns:
            numpy.ndarray or None: uint16 array (H, W) with depth in mm,
                                   or None if no frame received yet.
        """
        return self.get_frame()

    def get_frame_with_timestamp(self):
        """Get the latest frame along with its ROS timestamp.

        Returns:
            tuple: (numpy.ndarray, rospy.Time) or (None, None)
        """
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy(), self._latest_stamp
            return None, None

    def is_receiving(self):
        """Check if frames are being received.

        Returns:
            bool: True if at least one frame has been received.
        """
        with self._lock:
            return self._frame_count > 0

    @property
    def frame_count(self):
        """Number of frames received since creation."""
        with self._lock:
            return self._frame_count

    @property
    def topic(self):
        """The ROS topic this receiver is subscribed to."""
        return self._topic

    def shutdown(self):
        """Unsubscribe from the topic."""
        self._sub.unregister()
        rospy.loginfo("CameraReceiver: unsubscribed from %s", self._topic)
