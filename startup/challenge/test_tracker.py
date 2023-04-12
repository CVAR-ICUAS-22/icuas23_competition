import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped


class TestTrackerNode:
    def __init__(self):
        rospy.init_node('test_tracker', anonymous=True)
        self.challenge_sub = rospy.Subscriber(
            "/red/challenge_started", Bool, self.challenge_callback_2)

        self.tracker_pub = rospy.Publisher(
            "/red/tracker/input_pose", PoseStamped, queue_size=10)

        self.start_path_planning = rospy.ServiceProxy(
            '/red/path_planning/run', SetBool)

        rospy.spin()

    def challenge_callback_2(self, msg):
        if msg.data:
            resp = self.start_path_planning(True)
            print(resp)

    def challenge_callback(self, msg):
        if msg.data:
            print("Going to pose 10 10 5")
            ps = PoseStamped()
            ps.pose.position.x = 10.0
            ps.pose.position.y = 10.0
            ps.pose.position.z = 5.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 1.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 0.0

            self.tracker_pub.publish(ps)


if __name__ == '__main__':
    TestTrackerNode()
