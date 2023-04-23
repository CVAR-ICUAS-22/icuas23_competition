import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np


ROW_1 = 0.20
ROW_2 = 0.40
mask_wide = 0.4
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


        """
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "32FC1")

        window = int(self.v_filter_pct / 100 * image_msg.width)
        # create a mask between the row 1 * height and row 2 * height
        # between this rows the value of cv_image must be the maximum value

        # print the mask to see if it is correct
        mask = np.zeros(cv_image.shape, dtype=np.uint8)
        width = int(mask_wide * image_msg.width)
        width2 = int((1 - mask_wide) * image_msg.width)
        mask[:int(ROW_1 * image_msg.height), :] = 1
        mask[int(ROW_2 * image_msg.height):, :] = 1
        # the rectangle enclosed betheen width and width2 must be 1 in order to keep the image

        top_left = (width, int(ROW_1 * image_msg.height))
        bottom_right = (width2, int(ROW_2 * image_msg.height))
        # cv2.rectangle(mask, top_left, bottom_right, (1, 1, 1), -1)
        mask[int(ROW_1 * image_msg.height):int(ROW_2 * image_msg.height), width:] = 1
        mask[int(ROW_1 * image_msg.height):int(ROW_2 * image_msg.height), width2:] = 0

        # This applies the propeller mask to the camera image
        # cv_image = cv_image * mask


        # show_mask = mask * 255
        # cv2.imshow('mask', show_mask)

        # apply the mask to the image

        # cv_image[:, :window] = 0
        # cv_image[:, -window:] = 0

        # cv2.imshow('image', cv_image)
        # cv2.waitKey(1)

        msg_narrow = self.bridge.cv2_to_imgmsg(cv_image, "32FC1")
        msg_narrow.header = image_msg.header

        camera_info_msg = camera_info_msg
        camera_info_msg.width = msg_narrow.width

        self.image_pub.publish(msg_narrow)
        self.camera_info_pub.publish(camera_info_msg)
        """


if __name__ == '__main__':
    ImageNarrower()
