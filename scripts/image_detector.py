import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import copy
import numpy as np
from geometry_msgs.msg import Polygon, Point32


class image_node():
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.image_sub = rospy.Subscriber(
            "/red/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(
            "/red/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        # public new topic
        self.image_pub = rospy.Publisher(
            "/detections/image", Image, queue_size=10)
        # publisher an array ros message
        self.bbox_detections = rospy.Publisher(
            "/detections/bbox", Polygon, queue_size=10)
        self.crack_annotated_pub = rospy.Publisher(
            "/red/crack_image_annotated", Image, queue_size=10)

        self.bridge = CvBridge()
        rospy.spin()

    def calculate_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        # compute the area of intersection rectangle
        return max(0, xB - xA + 1) * max(0, yB - yA + 1)

    def canny(self, cv_image, cv_image_gray):
        cv_blur = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
        canny = cv2.Canny(cv_blur, 150, 180)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(canny, kernel, iterations=1)

        cnt, hierarchy = cv2.findContours(
            dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(cv_image,cnt,-1,(0,0,255), 2)
        currents_detections = []
        for c in cnt:
            approx = cv2.approxPolyDP(c, 0.01*cv2.arcLength(c, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(c)
                ratio = float(w)/h
                if ratio >= 0.9 and ratio <= 1.1:
                    x, y, w, h = cv2.boundingRect(c)
                    if w > 65 and h > 65 and w < 95 and h < 95:
                        print(w, h)
                        same = False
                        for c in currents_detections:
                            iou = self.calculate_iou(c, [x, y, x + w, y + h])
                            print('iou', iou)
                            if iou > 0.5:
                                same = True
                                break

                        if not same:
                            currents_detections.append([x, y, x + w, y + h])
                            cv2.rectangle(cv_image, (x, y),
                                          (x + w, y + h), (36, 255, 12), 2)
                            self.image_pub.publish(
                                self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                            pol = Polygon()
                            pol.points.append(Point32(x, y, 0))
                            pol.points.append(Point32(x + w, y, 0))
                            pol.points.append(Point32(x + w, y + h, 0))
                            pol.points.append(Point32(x, y + h, 0))
                            self.bbox_detections.publish(pol)

        # cv2.imshow("Image canny", canny)
        # cv2.imshow("Detections", cv_image)

    def find_countours(self, cv_image, cv_image_gray):
        cnt, hierarchy = cv2.findContours(
            cv_image_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_image, cnt, -1, (0, 0, 255), 2)
        # cv2.imshow("Image findContours", cv_image)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        original_image = copy.deepcopy(cv_image)

        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.canny(original_image, cv_image_gray)

        # cv2.imshow("Image window",pvision2021
        # image)
        # cv2.waitKey(3)

    def camera_info_callback(self, msg):
        # print(msg)
        pass


if __name__ == '__main__':
    image_node()
