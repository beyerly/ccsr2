#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>



void laserScanCallback(sensor_msgs::LaserScan scan_msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.5) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser_frame"));
}

void imuCallback(sensor_msgs::Imu data){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 1, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_frame"));
}

void range0Callback(sensor_msgs::Range){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, -0.1, 0.3) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "range_link0"));
}

void range1Callback(sensor_msgs::Range){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0.1, 0.3) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "range_link1"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber subScan = node.subscribe("/scan", 10, &laserScanCallback);
  ros::Subscriber subImu = node.subscribe("/imu/data", 10, &imuCallback);
  ros::Subscriber subRange0 = node.subscribe("/range0", 10, &range0Callback);
  ros::Subscriber subRange1 = node.subscribe("/range1", 10, &range1Callback);
  ROS_INFO("Create TF broadcaster, broadcasting base_link->imu_frame and base_link->laser_frame");
  ros::spin();
  return 0;
};
