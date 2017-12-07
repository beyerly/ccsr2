/*
 * powerMonitor.cpp
 *
 *  Created on: Oct 1, 2017
 *      Author: robert
 */


#include "ros/ros.h"
#include <sensor_msgs/BatteryState.h>
#include "ccsr2_pi_sensact_hub/power_monitor.h"
#include "std_msgs/Float32.h"
#include <std_srvs/SetBool.h>
#include <sstream>
#include <wiringPiI2C.h>
#include <unistd.h>




powerMonitor::powerMonitor() {

   pmon_pub_ = n.advertise<sensor_msgs::BatteryState>("pmon", 10);
   n.param<std::string>("frame_id", pmon_frame_id_, "pmon_link");
   enablePowerMonitor_srv_= n.advertiseService("enable_pmon", &powerMonitor::enablePowerMonitorCallback, this);

   pmon_msg.design_capacity  = 0.0; // Capacity in Ah (design capacity)  (If unmeasured NaN)
   pmon_msg.power_supply_technology = 1; // The battery chemistry. Values defined above
   pmon_msg.present = 1; //           True if the battery is present
   enabled_ = false;
   /*
   pmon_msg.cell_voltage   # An array of individual cell voltages for each cell in the pack
                            # If individual voltages unknown but number of cells known set each to NaN
   pmon_msg.location          # The location into which the battery is inserted. (slot number or plug)
   pmon_msg.serial_number     # The best approximation of the battery serial number
    */
   if ((i2c_dev = wiringPiI2CSetup(INA219_ADDR)) < 0){
      ROS_ERROR("Failed to set up I2C device on address %x", INA219_ADDR);
   }
   else {
      ROS_INFO("Created powerMonitor node, talking on I2C device %d on address %x", i2c_dev, INA219_ADDR);
   }
}


int powerMonitor::updateCharge() {

   /*
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
    */
   return 1;
}

bool powerMonitor::enablePowerMonitorCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
   enabled_=req.data;
   if (req.data) {
      ROS_INFO("Enabling Power Monitor");
   }
   else {
      ROS_INFO("Disabling Power Monitor");
   }
   return true;
}


double powerMonitor::clip(double value, double max, double min){
   if (value > max) {
      return max;
   }
   else if (value < min) {
      return min;
   }
   return value;
}

double powerMonitor::getBatteryVoltage() {

   double result;
   int data;

   if ((data = wiringPiI2CReadReg16 (i2c_dev, INA219_REG_VBUS)) < 0) {
      ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, INA219_REG_VBUS);
   }
   data = (((data & 0xFF) << 5) | ((data & 0xFF00) >> 11)) & 0x1FFF;
   if(data<MIN_VBUS_RANGE) {
            ROS_INFO("Clipping out of range INA219_REG_VBUS");
            data = MIN_VBUS_RANGE;
   }
   else if (result>MAX_VBUS_RANGE) {
            ROS_INFO("Clipping out of range INA219_REG_VBUS");
            data = MAX_VBUS_RANGE;
   }
   data = (MAX_VBAT_RANGE*data)/MAX_VBUS_RANGE;   // mV
   result = ((double) data)/1000.0;
   return result;
}




double powerMonitor::getOperatingCurrent() {

   double result;
   int data;

   if ((data = wiringPiI2CReadReg16  (i2c_dev, INA219_REG_VSHUNT)) < 0) {
      ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, INA219_REG_VSHUNT);
   }

   data = (((data & 0xFF) << 8) | ((data & 0xFF00) >> 8)) & 0xFFFF;

   if(data<MIN_VSHUNT_RANGE) {
      ROS_INFO("Clipping out of range INA219_REG_VSHUNT");
      data = MIN_VSHUNT_RANGE;
   }
   else if (result>MAX_VSHUNT_RANGE) {
      ROS_INFO("Clipping out of range INA219_REG_VSHUNT");
      data = MAX_VSHUNT_RANGE;
   }
   data = MAX_I_RANGE*data/MAX_VSHUNT_RANGE;   // mA
   result = ((double) data)/1000.0;
   return result;
}




void powerMonitor::publishPower() {

   if (enabled_) {

      ros::Time current_time = ros::Time::now();


      pmon_msg.header.stamp = current_time;
      pmon_msg.header.frame_id = pmon_frame_id_;

      pmon_msg.voltage  =    getBatteryVoltage(); //      # Voltage in Volts (Mandatory)
      pmon_msg.current   =  getOperatingCurrent(); //      # Negative when discharging (A)  (If unmeasured NaN)
      pmon_msg.percentage =  clip(pmon_msg.voltage/12.0, 1, 0);  // @@ not accurate, needs fixing

      /*   float32 charge           # Current charge in Ah  (If unmeasured NaN)
   float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
       */

      pmon_pub_.publish(pmon_msg);
   }
}





int main(int argc, char **argv)
{
   ros::init(argc, argv, "power_monitor");
   powerMonitor pmon;
   ros::Rate loop_rate(PMON_SAMPLE_RATE);
   while (ros::ok())
   {
      loop_rate.sleep();
      pmon.publishPower();
      ros::spinOnce();

   }
}
