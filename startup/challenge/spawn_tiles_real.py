#!/usr/bin/env python

import random
import subprocess
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest

from geometry_msgs.msg import Point
from icuas23_competition.msg import poi
import rospy
import numpy as np


def construct_poi(pose):
    d = 1
    r = d*1.25
    cc_x = pose[0] + d*np.cos(pose[3])
    cc_y = pose[1] + d*np.sin(pose[3])

    z_off = 2*np.random.random()-4

    # return (cc_x, cc_y, pose[2]+z_off, r)
    return (cc_x, cc_y, pose[2], r)


tile_poses = [(3.4, -2.46, 1, 2.3),
              (0.92, 1.5, 1, 3.14),
              (1.6, 0, 1, 0.8),
              (-0.36, -2.53, 1, 0.9),
              (-1.60, -2.53, 1, 2.3)]

rospy.init_node('spawner_deleter')
# delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
pub = rospy.Publisher('poi', poi, queue_size=10, latch=True)

poses = random.sample(tile_poses, 5)

command1 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_1_tiny x:=%s y:=%s z:=%s yaw:=%s" % poses[
    0]
command2 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_2_tiny x:=%s y:=%s z:=%s yaw:=%s" % poses[
    1]
command3 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_3_tiny x:=%s y:=%s z:=%s yaw:=%s" % poses[
    2]
command4 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_1_tiny x:=%s y:=%s z:=%s yaw:=%s" % poses[
    3]
command5 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_2_tiny x:=%s y:=%s z:=%s yaw:=%s" % poses[
    4]


roslaunch = subprocess.Popen(command1.split(" "), shell=False)
rospy.sleep(2)
roslaunch = subprocess.Popen(command2.split(" "), shell=False)
rospy.sleep(2)
roslaunch = subprocess.Popen(command3.split(" "), shell=False)
rospy.sleep(2)
roslaunch = subprocess.Popen(command4.split(" "), shell=False)
rospy.sleep(2)
roslaunch = subprocess.Popen(command5.split(" "), shell=False)
rospy.sleep(2)


print("Publishing pois")

PoIArray = poi()
point_list = []
for pose in poses:
    point_of_interest = construct_poi(pose)

    msg = Point()

    msg.x = point_of_interest[0]
    msg.y = point_of_interest[1]
    msg.z = point_of_interest[2]

    point_list.append(msg)

PoIArray.poi = point_list

while True:
    pub.publish(PoIArray)
    rospy.sleep(0.5)

# raw_input("Press enter to continue...")

# req = DeleteModelRequest()
# req.model_name = "crack_1"
# delete_model.call(req)
# req.model_name = "crack_2"
# delete_model.call(req)
# req.model_name = "crack_3"
# delete_model.call(req)
# req.model_name = "crack_4"
# delete_model.call(req)
# req.model_name = "crack_5"
# delete_model.call(req)
# req.model_name = "noncrack_1"
# delete_model.call(req)
# req.model_name = "noncrack_2"
# delete_model.call(req)
# req.model_name = "noncrack_3"
# delete_model.call(req)
# req.model_name = "noncrack_4"
# delete_model.call(req)
# req.model_name = "noncrack_5"
# delete_model.call(req)
# break
