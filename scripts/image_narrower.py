import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class ImageNarrower:
    """Image Narrower"""

    def __init__(self):
        rospy.init_node('image_narrower', anonymous=True)

        self.v_filter_pct = float(rospy.get_param('vertical_filter_pct', 12.5))

        image_sub = message_filters.Subscriber('in/image_raw', Image)
        info_sub = message_filters.Subscriber('in/camera_info', CameraInfo)

        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher("out/image_raw", Image, queue_size=10)

        self.camera_info_pub = rospy.Publisher(
            "out/camera_info", CameraInfo, queue_size=10)

        self.bridge = CvBridge()
        rospy.spin()

    def callback(self, image_msg, camera_info_msg):
        """Timesync callback"""
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "32FC1")

        window = int(self.v_filter_pct / 100 * image_msg.width)
        cv_image[:, :window] = 0
        cv_image[:, -window:] = 0
        msg_narrow = self.bridge.cv2_to_imgmsg(cv_image, "32FC1")
        msg_narrow.header = image_msg.header

        camera_info_msg.width = msg_narrow.width

        self.image_pub.publish(msg_narrow)
        self.camera_info_pub.publish(camera_info_msg)


if __name__ == '__main__':
    ImageNarrower()
