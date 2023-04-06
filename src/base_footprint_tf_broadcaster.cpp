#include "geometry_msgs/PoseStamped.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const geometry_msgs::PoseStamped &msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, -msg.pose.position.z));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), "red/base_link", "red/base_link_footprint"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_broadcaster_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("red/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};