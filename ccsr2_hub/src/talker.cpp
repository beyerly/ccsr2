#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <iostream>

#include <sstream>
#include <wiringPi.h>

#define LWHEEL_A 0
#define LWHEEL_B 2


class wheelEncoder {
   public:
      static ros::NodeHandle* n;
      static ros::Publisher* chatter_pub;
      static int* count;

      wheelEncoder();   
      static void updateEncoder();



};

int* wheelEncoder::count;

ros::NodeHandle* wheelEncoder::n;
ros::Publisher* wheelEncoder::chatter_pub;

wheelEncoder::wheelEncoder() {
      n = new(ros::NodeHandle);
      chatter_pub = new(ros::Publisher);
      *chatter_pub = n->advertise<std_msgs::Int16>("chatter", 1000);
      count = new(int);
      *count = 0;
      wiringPiISR (LWHEEL_A, INT_EDGE_FALLING, &updateEncoder);
      ROS_INFO("%s", "created node");

}


void wheelEncoder::updateEncoder() {
    (*wheelEncoder::count)++;

    std_msgs::Int16 msg;

    msg.data = *wheelEncoder::count;
    ROS_INFO("%d", msg.data);
    ROS_INFO("%s", "got interrupt");

    wheelEncoder::chatter_pub->publish(msg);

}





int main(int argc, char **argv)
{


  ros::init(argc, argv, "talker");

  wiringPiSetup () ;
  pinMode (LWHEEL_A, INPUT) ;
  pullUpDnControl(LWHEEL_A, PUD_UP);
  
  wheelEncoder lwheel;
  
  ros::Rate loop_rate(10);
  
  
  while (ros::ok())
  {

    
    loop_rate.sleep();

  }


  return 0;
}


