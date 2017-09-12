#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>

#include <sstream>


class wheelEncoderSim {
   public:
      ros::NodeHandle n;
      ros::Publisher wheel_pub;
      ros::Subscriber motor_cmd;
      int wheel_count;
      struct timeval timeLastMotorCmd;
      double lastMotorCmd;
      wheelEncoderSim();
      void run(const std_msgs::Float32& motor_cmd);
};

