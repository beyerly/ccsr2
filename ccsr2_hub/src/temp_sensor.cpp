


#include "ros/ros.h"
#include <sensor_msgs/Temperature.h>
#include "ccsr2_pi_sensact_hub/temp_sensor.h"
#include "std_msgs/Float64.h"
#include <std_srvs/SetBool.h>
#include <sstream>
#include <wiringPiI2C.h>
#include <unistd.h>




tempSensor::tempSensor() {
   temp_pub_ = n.advertise<sensor_msgs::Temperature>("temperature", 1);
   n.param<std::string>("frame_id", temp_frame_id_, "temp_link");
   enableTempSensor_srv_= n.advertiseService("enable_temp_sensor", &tempSensor::enableTempSensorCallback, this);
   enabled_ = true;
   if ((i2c_dev = wiringPiI2CSetup(TMP102_ADDR)) < 0){
      ROS_ERROR("Failed to set up I2C device on address %x", TMP102_ADDR);
   }
   else {
      ROS_INFO("Created temp_sensor node, talking on I2C device %d on address %x", i2c_dev, TMP102_ADDR);
   }
}

double tempSensor::getTemperature() {
   int temp;
   double result;

   if ((temp = wiringPiI2CReadReg16 (i2c_dev, TMP102_TEMP_REG)) < 0) {
      ROS_ERROR("Failed to read from I2C device %d on register %d", i2c_dev, TMP102_TEMP_REG);
   }
   result = (double) (((temp & 0xFF) << 8) | ((temp >> 8) & 0xFF) & 0xFFFF);
   result = (result * 0.0625)/10;
   return result;
}

void tempSensor::publishTemp() {
   if (enabled_) {
      ros::Time current_time = ros::Time::now();
      temp_msg.header.stamp = current_time;
      temp_msg.header.frame_id = temp_frame_id_;
      temp_msg.temperature  =   getTemperature(); // # Measurement of the Temperature in Degrees Celsius
      temp_pub_.publish(temp_msg);
   }
}

bool tempSensor::enableTempSensorCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
   enabled_=req.data;
   if (req.data) {
      ROS_INFO("Enabling temperature sensor");
   }
   else {
      ROS_INFO("Disabling temperature sensor");
   }
   return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "temp_sensor");
  tempSensor tsense;
  ros::Rate loop_rate(TEMP_SENS_SAMPLE_RATE);
  while (ros::ok())
  {
     loop_rate.sleep();
     tsense.publishTemp();
     ros::spinOnce();

  }
}
