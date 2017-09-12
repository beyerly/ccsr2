#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <iostream>

#include <sstream>
#include <wiringPi.h>

#define LWHEEL_A 0
#define LWHEEL_B 2
#define RWHEEL_A 3
#define RWHEEL_B 4

class wheelEncoder {
   public:
      // Need to make undateEncoder static, because wiringPi can only reference c-style callback function calls, not class members
      // This also means we need to make all private vars references static.
      // Need to make the ros node a pointer, such that it does not get constructed as global static variable before ros is initialized
      static ros::NodeHandle* n;
      static ros::Publisher* wheel_pub;
      static int wheel_count;
      static int dvdr;
      static bool setA;
      static bool setB;
      static int wheel_encoder_A_gpio;
      static int wheel_encoder_B_gpio;

      static std::string wheel;
      wheelEncoder();   
      static void updateEncoder();
};

