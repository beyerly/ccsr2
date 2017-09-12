#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "ccsr2_pi_sensact_hub/wheel_encoder_sim.h"
#include <sstream>
#include <math.h>
#include <sys/time.h>



wheelEncoderSim::wheelEncoderSim() {
   wheel_pub = n.advertise<std_msgs::Int16>("wheel", 1000);
   motor_cmd = n.subscribe("motor_cmd", 1000, &wheelEncoderSim::run, this);
   wheel_count = 0;
   lastMotorCmd = 0;
   struct timeval now;
   int rv = gettimeofday(&now, NULL);
   if (0 != rv)
   {
     ROS_ERROR("Invalid return from gettimeofday: %d", rv);
   }
//   timeLastMotorCmd = now.tv_usec;
//   ROS_INFO("Created wheel_encoder_sim node at time %d", timeLastMotorCmd);
}


void wheelEncoderSim::run(const std_msgs::Float32& motor_cmd) {
// motor_cmd = 10 had 13s per rev: 32k pulses./
//   ROS_INFO("wheel_count %d timeLastMotorCmd %d",  wheel_count, timeLastMotorCmd);

   struct timeval now;
   int rv = gettimeofday(&now, NULL);
   if (0 != rv)
   {
     ROS_ERROR("Invalid return from gettimeofday: %d", rv);
   }

   //ROS_INFO("sec %ld usec %ld sec %ld usec %ld", now.tv_sec , now.tv_usec, timeLastMotorCmd.tv_sec, timeLastMotorCmd.tv_usec);
   double delta;
   if (now.tv_sec == timeLastMotorCmd.tv_sec){
      delta = (double)  (now.tv_usec - timeLastMotorCmd.tv_usec)/1000000;
   }
   else{
      delta = (double)  (1000000 + (now.tv_usec - timeLastMotorCmd.tv_usec))/1000000; // sec
   }
   timeLastMotorCmd = now;
   double speed = lastMotorCmd* 246.1;   // encoder pulses/s
   lastMotorCmd = (double) motor_cmd.data;
   int pulsesElapsed = (int) ceil(delta*speed);
   wheel_count += pulsesElapsed;
   ROS_INFO("wheel %d delta %f speed %f elapsed %d", wheel_count, delta, speed, pulsesElapsed);
   //ROS_INFO("sec %ld usec %ld", now.tv_sec , now.tv_usec);

   std_msgs::Int16 msg;
   msg.data = wheel_count/10;

   wheelEncoderSim::wheel_pub.publish(msg);
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_encoder_sim");
  wheelEncoderSim wheelSim;
  ros::spin();
  return 0;
}


