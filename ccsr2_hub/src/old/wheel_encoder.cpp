#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "ccsr2_pi_sensact_hub/wheel_encoder.h"
#include <sstream>
#include <wiringPi.h>



std::string wheelEncoder::wheel;
int wheelEncoder::wheel_count;
int wheelEncoder::dvdr;
bool wheelEncoder::setA;
bool wheelEncoder::setB;
int wheelEncoder::wheel_encoder_A_gpio;
int wheelEncoder::wheel_encoder_B_gpio;

ros::NodeHandle* wheelEncoder::n;
ros::Publisher* wheelEncoder::wheel_pub;

wheelEncoder::wheelEncoder() {
      n = new(ros::NodeHandle);
      wheel_pub = new(ros::Publisher);
      wheel_count = 128;
      setA = 0;
      setB = 0;
      dvdr = 0;
      if (ros::param::get("~wheel", wheel)) {
         *wheel_pub = n->advertise<std_msgs::Int16>(wheel, 1000);

         if (strcmp (wheel.c_str(), "rwheel")) {
            wheel_encoder_A_gpio = LWHEEL_A;
            wheel_encoder_B_gpio = LWHEEL_B;
         }
         else {
            wheel_encoder_A_gpio = RWHEEL_A;
            wheel_encoder_B_gpio = RWHEEL_B;
         }

         pinMode (wheel_encoder_A_gpio, INPUT) ;
         pinMode (wheel_encoder_B_gpio, INPUT) ;
         pullUpDnControl(wheel_encoder_A_gpio, PUD_UP);
         pullUpDnControl(wheel_encoder_B_gpio, PUD_UP);

         wiringPiISR (wheel_encoder_A_gpio, INT_EDGE_RISING, &updateEncoder);
         ROS_INFO("Created %s wheel_encoder node on GPIO pins %d (A) and %d (B)", wheel.c_str(), wheel_encoder_A_gpio, wheel_encoder_B_gpio);
      }
      else {
         ROS_ERROR("Failed to get param 'wheel'");
      }
}


void wheelEncoder::updateEncoder() {

    setB = (bool)digitalRead(wheel_encoder_B_gpio);
    if (setB) {
       wheel_count++;
    }
    else {
       wheel_count--;
    }
    dvdr++;
    if (dvdr == 100){
       ROS_INFO("%s encoder is %d", wheel.c_str(),  wheel_count/100);
       dvdr = 0;
       std_msgs::Int16 msg;

       msg.data = wheel_count;

       wheelEncoder::wheel_pub->publish(msg);

    }

}





int main(int argc, char **argv)
{


  ros::init(argc, argv, "talker");

  wiringPiSetup () ;
  
  wheelEncoder wheel;
  
  ros::Rate loop_rate(10);
  
  
  while (ros::ok())
  {
    loop_rate.sleep();
  }


  return 0;
}


