#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <unistd.h>
#include <string>

#include "face_recognition.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   ros::Subscriber temp_sub_;
   ros::Subscriber batt_sub_;

   ccsr2::faceRecognition face_recognition;

   //  ros::Publisher caminfo_pub_;


public:
   sensor_msgs::Temperature temperature;
   sensor_msgs::BatteryState batteryState;

   ImageConverter()
   : it_(nh_)
   {
      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      temp_sub_ = nh_.subscribe("/temperature", 1, &ImageConverter::temperatureCallback, this);
      batt_sub_ = nh_.subscribe("/pmon", 1, &ImageConverter::batteryCallback, this);

      // Note I have to call something from CV:: otherwise I get segmentation fault on cv_bridge call. Probably opencv library must be initialized...
      cv::Mat M(2,2, CV_8UC3, cv::Scalar(0,0,255));
      //cv::namedWindow(OPENCV_WINDOW);
   }

   //~ImageConverter()
   //{
   //  cv::destroyWindow(OPENCV_WINDOW);
   //}


   void temperatureCallback(const sensor_msgs::Temperature& msg) {
      temperature = msg;
   };

   void batteryCallback(const sensor_msgs::BatteryState& msg) {
      batteryState = msg;
   };

   void addSensorData(cv::Mat& img) {
      std::vector <std::string> sensors;
      int fontFace = cv::FONT_HERSHEY_PLAIN;
      double fontScale = 1;
      int thickness = 1;

      sensors.push_back("Temp: " + std::to_string(temperature.temperature));
      sensors.push_back("current: " + std::to_string(batteryState.current));
      cv::Point textOrg(10, 30);
      cv::Size textSize = cv::getTextSize(sensors[0], fontFace, fontScale, thickness, NULL);
      for (int i=0; i<sensors.size(); i++){
         cv::putText(img, sensors[i], textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness, 8);
         textOrg.y += textSize.height + 3;
      }
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

         // Draw an example circle on the video stream
         if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

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
