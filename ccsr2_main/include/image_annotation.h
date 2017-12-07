#ifndef _IMAGE_ANNOTATION_H
#define _IMAGE_ANNOTATION_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <unistd.h>
#include <string>
#include <iomanip>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "face_recognition.h"

#define PI 3.14159265359
#define SENSOR_TIMEOUT 3


struct sensorDataItem {
   std::string name_;
   std::string value_;
   std::string unit_;
};

class sensorData {
   std::vector <sensorDataItem> item_;
   int precision_;
   bool active_;
   int timer_;

public:
   sensorData();
   void addItem(std::string name, std::string unit);
   template <typename T> void update(const T a_value, int item);
   void watchDog();
   std::string getString();
   cv::Scalar getColor();
};


class ImageConverter
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   ros::Subscriber temp_sub_;
   ros::Subscriber batt_sub_;
   ros::Subscriber odom_sub_;
   ros::Subscriber pose_sub_;
   ros::Timer timer;

   ccsr2::faceRecognition face_recognition;
   std::vector <sensorData*> sensors_;

   //  ros::Publisher caminfo_pub_;


public:
   sensorData temperature_;
   sensorData current_;
   sensorData battery_state_;
   sensorData pose_;
   sensorData odom_;

   ImageConverter();
   void temperatureCallback(const sensor_msgs::Temperature& msg);
   void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
   void odomCallback(const nav_msgs::Odometry& msg);
   void batteryCallback(const sensor_msgs::BatteryState& msg);
   void addSensorData(cv::Mat& img);
   void imageCb(const sensor_msgs::ImageConstPtr& msg);
   void timerCallback(const ros::TimerEvent&);
};

#endif
