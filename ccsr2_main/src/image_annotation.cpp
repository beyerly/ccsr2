#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
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

static const std::string OPENCV_WINDOW = "Image window";



class ImageConverter
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   ros::Subscriber temp_sub_;
   ros::Subscriber batt_sub_;
   ros::Subscriber odom_sub_;

   ccsr2::faceRecognition face_recognition;

   //  ros::Publisher caminfo_pub_;


public:
   sensor_msgs::Temperature temperature;
   sensor_msgs::BatteryState batteryState;
   nav_msgs::Odometry odom;

   ImageConverter()
   : it_(nh_)
   {
      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      temp_sub_ = nh_.subscribe("/temperature", 1, &ImageConverter::temperatureCallback, this);
      batt_sub_ = nh_.subscribe("/pmon", 1, &ImageConverter::batteryCallback, this);
      odom_sub_= nh_.subscribe("/odom", 1, &ImageConverter::odomCallback, this);;

      // Note I have to call something from CV:: otherwise I get segmentation fault on cv_bridge call. Probably opencv library must be initialized...
      cv::Mat M(2,2, CV_8UC3, cv::Scalar(0,255,0));
      //cv::namedWindow(OPENCV_WINDOW);
   }

   //~ImageConverter()
   //{
   //  cv::destroyWindow(OPENCV_WINDOW);
   //}


   void temperatureCallback(const sensor_msgs::Temperature& msg) {
      temperature = msg;
   };

   void odomCallback(const nav_msgs::Odometry& msg) {
      odom = msg;
   };

   void batteryCallback(const sensor_msgs::BatteryState& msg) {
      batteryState = msg;
   };

   void addSensorData(cv::Mat& img) {
      std::vector <std::string> sensors;
      int fontFace = cv::FONT_HERSHEY_PLAIN;
      double fontScale = 1;
      int thickness = 1;
      double roll, pitch, yaw;

      tf::Quaternion q;
      tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
      tf::Matrix3x3 mat(q);
      mat.getRPY(roll, pitch, yaw);

      sensors.push_back("Temp: " + to_string_with_precision(temperature.temperature, 2) + "C");
      sensors.push_back("Current: " + to_string_with_precision(batteryState.current, 2) + "A");
      sensors.push_back("Batt: " + std::to_string((int) floor(100*batteryState.percentage)) + "%");
      sensors.push_back("Speed lin: " + to_string_with_precision(odom.twist.twist.linear.x,  2) + "m/s ang: " + to_string_with_precision(odom.twist.twist.angular.z,  2) + "m/s");
      sensors.push_back("Odom x: " + to_string_with_precision(odom.pose.pose.position.x,  2) + " y " + to_string_with_precision(odom.pose.pose.position.y,  2) + " hd: " + std::to_string((int) floor(360*(yaw+PI)/(2*PI))) + "deg");
      cv::Point textOrg(10, 30);
      cv::Size textSize = cv::getTextSize(sensors[0], fontFace, fontScale, thickness, NULL);
      for (int i=0; i<sensors.size(); i++){
         cv::putText(img, sensors[i], textOrg, fontFace, fontScale, cv::Scalar(0, 255, 0), thickness, 8);
         textOrg.y += textSize.height + 3;
      }
   }

   template <typename T>
   std::string to_string_with_precision(const T a_value, const int n = 2) {
      T value = round (a_value*(10^2))/(10^2);
      std::ostringstream out;
       out << std::setprecision(n) << value;
       return out.str();
   }




   void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_converter");
   ROS_INFO("Created image_converter node");
   ImageConverter ic;
   ros::spin();
   return 0;
}
