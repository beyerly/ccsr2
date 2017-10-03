#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ccsr2_pi_sensact_hub/motor_driver.h"
#include "ccsr2_hub/motorAcceleration.h"
#include <sstream>


motorDriver::motorDriver() {

//   if (!ros::param::get("~no_robot_hw", noRobotHw)) {
//      ROS_ERROR("Failed to get param 'no_robot_hw'");
//   }
   noRobotHw = false;
   motorsEnabled = false;
   if (!ros::param::get("~motor", motor)) {
      ROS_ERROR("Failed to get param 'motor'");
   }
   else {
      motor_sub = n.subscribe(motor + "_cmd", 1000, &motorDriver::updateSpeed, this);
   }
   if (strcmp (motor.c_str(), "rmotor")) {
      speed_reg = MD22_I2C_REG_LSPEED;
   }
   else {
      speed_reg = MD22_I2C_REG_RSPEED;
   }
   if (noRobotHw) {
      ROS_INFO("Created %s motor_driver node, no robot HW enabled, debug only.");
   }
   else {
      if ((i2c_dev = wiringPiI2CSetup(MD22_I2C_ADDRESS)) < 0){
         ROS_ERROR("Failed to set up I2C device on address %x", MD22_I2C_ADDRESS);
      }
      else {
         if (wiringPiI2CWriteReg8 (i2c_dev, MD22_I2C_REG_MODE, MD22_I2C_MODE) < 0) {
            ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, MD22_I2C_REG_MODE);
         }
         if (wiringPiI2CWriteReg8 (i2c_dev, MD22_I2C_REG_ACCELERATION, MD22_I2C_ACCELERATION) < 0) {
            ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, MD22_I2C_REG_ACCELERATION);
         }
         ROS_INFO("Created %s motor_driver node, talking on I2C device %d on address %x", motor.c_str(), i2c_dev, MD22_I2C_ADDRESS);
      }
   }
   std::string serviceName;

   serviceName = "set" + motor + "Acceleration";
   setMotorAcceleration_srv_ = n.advertiseService(serviceName.c_str(), &motorDriver::setMotorAccelerationCallback, this);
   serviceName = "enable" + motor;
   enableMotors_srv_= n.advertiseService(serviceName.c_str(), &motorDriver::enableMotorsCallback, this);

}

void motorDriver::updateSpeed(const std_msgs::Float32& msg) {

   int motor_cmd = (int) ceil(msg.data);
   motor_cmd = -motor_cmd;
   if (-127 <= motor_cmd < 127){
      //ROS_INFO("Setting speed for %s motor to %d", motor.c_str(), motor_cmd);

      if (noRobotHw) {
         ROS_INFO("No Robot hardware. %s motor_driver would be set to %d", motor.c_str(), motor_cmd);
      }
      else {
         if (motorsEnabled) {
            if (wiringPiI2CWriteReg8 (i2c_dev, speed_reg, motor_cmd) < 0) {
               ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, speed_reg);
            }
            else {
               //ROS_INFO("%s motor_driver set to %d", motor.c_str(), motor_cmd);
            }
         }
         else {
            ROS_INFO("Motors disabled: %s motor_driver would be set to %d", motor.c_str(), motor_cmd);
         }
      }
   }
   else {
      ROS_ERROR("%d command exceeds limits of -127:127: %d", motor.c_str(), motor_cmd);
   }
}



 bool motorDriver::setMotorAccelerationCallback(ccsr2_hub::motorAcceleration::Request& req, ccsr2_hub::motorAcceleration::Response& res) {
    if (noRobotHw) {
       ROS_INFO("No Robot hardware. %s motor_driver acceleration would be set to %d", motor.c_str(), req.acceleration);
    }
    else {
       if (wiringPiI2CWriteReg8 (i2c_dev, MD22_I2C_REG_ACCELERATION, req.acceleration) < 0) {
          ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, MD22_I2C_REG_ACCELERATION);
       }
       else {
          ROS_INFO("%s motor_driver acceleration set to %d", motor.c_str(), req.acceleration);
       }
    }
    return true;
}

 bool motorDriver::enableMotorsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
    if (noRobotHw) {
       ROS_INFO("No Robot hardware. %s will remain disabled", motor.c_str());
    }
    else {
       ros::param::set("~motors_enabled", req.data);
       motorsEnabled = req.data;
       if (req.data) {
          ROS_INFO("Enabling motors for %s motor_driver", motor.c_str());
       }
       else {
          ROS_INFO("Disabling motors for %s motor_driver", motor.c_str());
       }
    }
    return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_driver");
  motorDriver mdrv;
  ros::spin();
}








