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

#include "image_annotation.h"
#include "face_recognition.h"

static const std::string OPENCV_WINDOW = "Image window";



ImageConverter::ImageConverter()
: it_(nh_)
{
   // Subscrive to input video feed and publish output video feed
   image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
   image_pub_ = it_.advertise("/image_converter/output_video", 1);
   temp_sub_ = nh_.subscribe("/temperature", 1, &ImageConverter::temperatureCallback, this);
   batt_sub_ = nh_.subscribe("/pmon", 1, &ImageConverter::batteryCallback, this);
   pose_sub_= nh_.subscribe("/amcl_pose", 1, &ImageConverter::poseCallback, this);
   odom_sub_= nh_.subscribe("/odom", 1, &ImageConverter::odomCallback, this);

   temperature_ = sensorData();
   current_ = sensorData();
   battery_state_ = sensorData();
   pose_ = sensorData();
   odom_ = sensorData();

   temperature_.addItem("Temp", "C");
   current_.addItem("Current", "A");
   battery_state_.addItem("Batt", "%");
   odom_.addItem("Speed lin", "m/s");
   odom_.addItem("Speed ang", "m/s");
   pose_.addItem("Pose x", "");
   pose_.addItem("Pose y", "");


   sensors_.push_back(&temperature_);
   sensors_.push_back(&current_);
   sensors_.push_back(&battery_state_);
   sensors_.push_back(&pose_);
   sensors_.push_back(&odom_);

   timer = nh_.createTimer(ros::Duration(1), &ImageConverter::timerCallback, this);

   // Note I have to call something from CV:: otherwise I get segmentation fault on cv_bridge call. Probably opencv library must be initialized...
   cv::Mat M(2,2, CV_8UC3, cv::Scalar(0,255,0));
   //cv::namedWindow(OPENCV_WINDOW);
}

void ImageConverter::temperatureCallback(const sensor_msgs::Temperature& msg) {
   temperature_.update(msg.temperature, 0);
};

void ImageConverter::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
   pose_.update(msg.pose.pose.position.x, 0);
   pose_.update(msg.pose.pose.position.y, 1);
};

void ImageConverter::odomCallback(const nav_msgs::Odometry& msg) {
   odom_.update(msg.twist.twist.linear.x, 0);
   odom_.update(msg.twist.twist.angular.z, 1);
};

void ImageConverter::batteryCallback(const sensor_msgs::BatteryState& msg) {
   battery_state_.update(msg.current, 0);
   current_.update((int) floor(100*msg.percentage), 0);
};

void ImageConverter::addSensorData(cv::Mat& img) {
   std::string data;
   int fontFace = cv::FONT_HERSHEY_PLAIN;
   double fontScale = 1;
   int thickness = 1;
   double roll, pitch, yaw;

   //tf::Quaternion q;
   //tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
   //tf::Matrix3x3 mat(q);
   //mat.getRPY(roll, pitch, yaw);

   cv::Point textOrg(10, 30);
   cv::Size textSize = cv::getTextSize(sensors_[0]->getString(), fontFace, fontScale, thickness, NULL);
   sensorData* item;
   for(std::vector<sensorData*>::iterator it = sensors_.begin(); it != sensors_.end(); it++){
      item = *it;
      cv::putText(img, item->getString(), textOrg, fontFace, fontScale, item->getColor(), thickness, 8);
      textOrg.y += textSize.height + 3;
   }
}


void ImageConverter::timerCallback(const ros::TimerEvent&) {
   sensorData* item;
   for(std::vector<sensorData*>::iterator it = sensors_.begin(); it != sensors_.end(); it++){
      item = *it;
      item->watchDog();
   }
}


void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

   // Somehow we're gettiung bogus image sizes the first few frames, do a quick check we have the correct size.
   if (msg->height == 480) {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
      addSensorData(cv_ptr->image);
      face_recognition.detectFaces(cv_ptr->image);

      // Update GUI Window
      //    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //    cv::waitKey(3);

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
   }
}



sensorData::sensorData() {
   precision_ = 2;
   timer_ = 0;
}

void sensorData::addItem(std::string name, std::string unit) {
   sensorDataItem it;
   it.name_ = name;
   it.unit_ = unit;
   it.value_ = "0";
   item_.push_back(it);
}


template <typename T> void sensorData::update(const T a_value, int item) {
   T value = round (a_value*(10^2))/(10^2);
   std::ostringstream out;
   out << std::setprecision(precision_) << value;
   item_[item].value_ = out.str();
   timer_ = 0;
}

void sensorData::watchDog() {

   if (timer_< SENSOR_TIMEOUT) {
      timer_++;
   }
}

std::string sensorData::getString() {
   std::string out;
   for(std::vector<sensorDataItem>::iterator it = item_.begin(); it !=  item_.end(); it++){
      out = out + it->name_ + " " + it->value_ + it->unit_ + " ";

   }
   return out;
}

cv::Scalar sensorData::getColor() {
   if (timer_ < SENSOR_TIMEOUT) {
      return cv::Scalar(0, 255, 0);
   }
   else {
      return cv::Scalar(0, 0, 255);
   }
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_converter");
   ROS_INFO("Created image_converter node");
   ImageConverter ic;
   ros::spin();
   return 0;
}
