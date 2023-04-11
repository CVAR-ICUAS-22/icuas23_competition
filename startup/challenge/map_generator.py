#!/usr/bin/env python
import rospy
import std_msgs
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
from dataclasses import dataclass
import tf_conversions
import cv_bridge

OCUPANCY_GRID_TOPIC = "/projected_map"
OCCUPANCY_IMAGE_TOPIC = "/occupancy_image"
RESIZED_OCCUPANCY_IMAGE_TOPIC = "/occupancy_image_resized"


class OccupancyImage():
    """
    This image represents a occupancy grid as a image, the centre of the image is the (0,0) point
    in the map coordinates, the image is in the same orientation as the map, the image is in
    grayscale, 255 is free space and 0 is occupied space. We dont consider unknown space.

    """

    def __init__(self, width=100, height=100, resolution=0.5):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.image = np.ones((height, width, 1), np.uint8) * 255
        self.image_publisher = rospy.Publisher(
            OCCUPANCY_IMAGE_TOPIC, Image, queue_size=10)
        self.resized_image_publisher = rospy.Publisher(
            RESIZED_OCCUPANCY_IMAGE_TOPIC, Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()

    def color_coordinates(self, x, y, value):
        """
        Color the coordinates in the image
        x and y in meters
        """

        # displace the coordinates considering the centre of the image is the (0,0) point
        # considering resolution is self.resolution
        px = int(x / self.resolution) + int(self.width / 2)
        py = int(y / self.resolution) + int(self.height / 2)
        # check if the coordinates are inside the image

        # we want x> to be in the upper part of the image
        py = self.height - py
        px = self.width - px

        if px < 0 or px >= self.width or py < 0 or py >= self.height:
            raise ValueError("The coordinates are outside the image")

        self.image[px, py] = value

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
                # if the value is unknown, we dont consider it
                if value == -1:
                    self.color_coordinates(
                        origin_x + i * resolution, origin_y + j * resolution, 255)
                # if the value is occupied, we color the cell in black
                elif value == 100:
                    self.color_coordinates(
                        origin_x + i * resolution, origin_y + j * resolution, 0)
                # if the value is free, we color the cell in white
                elif value == 0:
                    self.color_coordinates(
                        origin_x + i * resolution, origin_y + j * resolution, 255)

        # self.color_coordinates(0, 0, 127) # REMOVE THIS
        # self.show_image()
        self.publish_image()

    def get_image(self):
        return self.image

    def publish_image(self):

        resized_image = cv2.resize(
            self.image, (self.width * 10, self.height * 10), interpolation=cv2.INTER_NEAREST)
        resized_image_msg = self.bridge.cv2_to_imgmsg(
            resized_image, encoding="mono8")
        resized_image_msg.header.stamp = rospy.Time.now()
        self.resized_image_publisher.publish(resized_image_msg)

        image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="mono8")
        image_msg.header.stamp = rospy.Time.now()

        self.image_publisher.publish(image_msg)

    def show_image(self):
        # resize the image x10
        resized_image = cv2.resize(
            self.image, (self.width * 10, self.height * 10), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("frame", resized_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User pressed q")
            exit(1)


if __name__ == '__main__':
    rospy.init_node("pose_estimador", anonymous=True)
    occupancy_image = OccupancyImage()
    rospy.Subscriber(OCUPANCY_GRID_TOPIC, OccupancyGrid,
                     occupancy_image.parse_occupancy_grid)
    rospy.spin()
