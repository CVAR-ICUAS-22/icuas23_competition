#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import cv_bridge

OCUPANCY_GRID_TOPIC = "/projected_map"
OCCUPANCY_IMAGE_TOPIC = "/occupancy_image"
RESIZED_OCCUPANCY_IMAGE_TOPIC = "/occupancy_image_resized"
RED_POSE_TOPIC = "/red/pose"


class OccupancyImage:
    """
    This image represents a occupancy grid as a image, the centre of the image is the (0,0) point
    in the map coordinates, the image is in the same orientation as the map, the image is in
    grayscale, 255 is free space and 0 is occupied space. We dont consider unknown space.

    width: img width in pixels
    height: img height in pixels
    resolution: pixel/metre
    """

    def __init__(self, width=100, height=100, resolution=0.5):
        self.width = int(width)
        self.height = int(height)
        self.resolution = resolution
        self.image = np.ones((self.height, self.width, 1), np.uint8) * 255
        self.pose = [0.0, 0.0]

        self.image_publisher = rospy.Publisher(
            OCCUPANCY_IMAGE_TOPIC, Image, queue_size=10)

        self.resized_image_publisher = rospy.Publisher(
            RESIZED_OCCUPANCY_IMAGE_TOPIC, Image, queue_size=10)

        self.occ_grid_sub = rospy.Subscriber(OCUPANCY_GRID_TOPIC, OccupancyGrid,
                                             self.parse_occupancy_grid)

        self.red_pose_sub = rospy.Subscriber(
            RED_POSE_TOPIC, PoseStamped, self.pose_callback)
        self.bridge = cv_bridge.CvBridge()

    def pose_callback(self, msg):
        """PoseStamped callback"""
        self.pose = [msg.pose.position.x, msg.pose.position.y]

        # debbuging draw drone pose
        # self.color_coordinates(
        #     msg.pose.position.x, msg.pose.position.y, 125)

    def color_coordinates(self, x, y, value):
        """
        Color the coordinates in the image
        x and y in meters
        """

        # considering resolution is self.resolution
        px = int(x / self.resolution)
        py = int(y / self.resolution)

        # map origin is in bottom left corner
        py = self.height - py

        # check if the coordinates are inside the image
        if self.out_of_bounds(px, py):
            # print(
            #     f"The coordinates {px} {py} are outside the image size {self.width} {self.height}")
            return

        self.image[py, px] = value

    def parse_occupancy_grid(self, occ_grid: OccupancyGrid):
        """
        Parse the occupancy grid into the image
        :param occ_grid: the occupancy grid to parse
        :return:
        """
        if self.resolution != occ_grid.info.resolution:
            raise ValueError(
                "The resolution of the occupancy grid is different from the resolution of the image")

        width = occ_grid.info.width
        height = occ_grid.info.height
        resolution = occ_grid.info.resolution
        origin_x = occ_grid.info.origin.position.x
        origin_y = occ_grid.info.origin.position.y

        # iterate over the occupancy grid
        for i in range(width):
            for j in range(height):
                # get the value of the cell
                value = occ_grid.data[i + j * width]
                x = origin_x + i * resolution  # metres
                y = origin_y + j * resolution  # metres

                if self.out_of_bounds(x, y):
                    continue

                if sqrt((x - self.pose[0])**2 + (y - self.pose[1])**2) <= self.resolution*2:
                    continue

                # if abs(self.pose[0] - x) <= self.resolution*2:
                #     continue
                # if abs(self.pose[1] - y) <= self.resolution*2:
                #     continue

                # if the value is unknown, we dont consider it
                if value == -1:
                    self.color_coordinates(x, y, 255)
                # if the value is occupied, we color the cell in black
                elif value == 100:
                    self.color_coordinates(x, y, 0)
                # if the value is free, we color the cell in white
                elif value == 0:
                    self.color_coordinates(x, y, 255)

        security_distance = 1.0 # [m]

        self.add_security_margin(security_distance)

        # self.color_coordinates(0, 0, 127) # REMOVE THIS
        # self.show_image(self.image)
        self.publish_image()

    def add_security_margin(self, security_margin):

        occ2bin_th = 100 # [0-255]
        dist2bin_th = 0.7 # [0.0-1.0]
        # security_distance = 0.5 # [m]
        # img_resolution = 0.5 # [m/pixel]

        image = self.image.astype(np.uint8)
        # cv2.imshow("frame", self.image)
        _, binary_map = cv2.threshold(image, occ2bin_th, 255, cv2.THRESH_BINARY)
        # print("Binary map type: ", type(binary_map))
        # cv2.imshow("binary_map", binary_map)
        distance_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 3)
        # cv2.imshow("distance_map", distance_map)
        dist_normalized_map = distance_map / (security_margin / self.resolution)
        # cv2.imshow("dist_normalized_map", dist_normalized_map)
        _, occupancy_map = cv2.threshold(dist_normalized_map, dist2bin_th, 255, cv2.THRESH_BINARY)
        # cv2.imshow("security_map", occupancy_map)
        self.image = occupancy_map.astype(np.uint8)

    def out_of_bounds(self, x, y):
        """check if pose is inside the map"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return False

    def publish_image(self):
        """Publish image and image_resized"""
        resized_image = cv2.resize(
            self.image, (self.width * 10, self.height * 10), interpolation=cv2.INTER_NEAREST)
        resized_image_msg = self.bridge.cv2_to_imgmsg(
            resized_image, encoding="mono8")
        resized_image_msg.header.stamp = rospy.Time.now()
        self.resized_image_publisher.publish(resized_image_msg)

        image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="mono8")
        image_msg.header.stamp = rospy.Time.now()

        self.image_publisher.publish(image_msg)

    def show_image(self, image, resize_factor=10):
        """Show image for debbuging"""
        resized_image = cv2.resize(
            image, (self.width * resize_factor, self.height * resize_factor), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("frame", resized_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User pressed q")
            exit(1)


if __name__ == '__main__':
    rospy.init_node("map_generator", anonymous=True)

    # MAP SIZE (m)
    MAP_X = 20
    MAP_Y = 50
    OCC_MAP_RESOLUTION = 0.5

    occupancy_image = OccupancyImage(width=MAP_X/OCC_MAP_RESOLUTION,
                                     height=MAP_Y/OCC_MAP_RESOLUTION,
                                     resolution=OCC_MAP_RESOLUTION)
    rospy.spin()
