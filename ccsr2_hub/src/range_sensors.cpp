

#include "ros/ros.h"
#include <sensor_msgs/Range.h>
#include "ccsr2_pi_sensact_hub/range_sensors.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <unistd.h>
#include <string>

rangeSensors::rangeSensors() {
   int data;
   std::string frame_id;
   n.param<std::string>("frame_id", frame_id, "range_link");
   for (int s=0; s<NUM_RANGE_SENSORS; s++) {
      range_pub_[s] = n.advertise<sensor_msgs::Range>("range" + std::to_string(s), 10);
      range_frame_id_[s] = frame_id +  std::to_string(s);
   }
   if (wiringPiSetup () < 0){
      ROS_ERROR("Failed to set up GPIO");
   }
   if ((i2c_dev = wiringPiI2CSetup(ADC0_ADDR)) < 0){
      ROS_ERROR("Failed to set up I2C device on address %x", ADC0_ADDR);
   }
   ROS_INFO("Created range_sensors node with %d sonar sensors, talking on I2C device %d on address %x", NUM_RANGE_SENSORS, i2c_dev, ADC0_ADDR);
   range_msg.radiation_type = 0; //  # the type of radiation used by the sensor: ULTRASOUND
   range_msg.min_range = 0.01; //      # minimum range value [m]
   range_msg.max_range = 20; //      # maximum range value [m]


   data = (OS_0 << OS_offset) |
         (0 << MUX_offset) |
         (PGA_1x << PGA_offset) |
         (MODE_SS << MODE_offset) |
         (DR_128 << DR_offset) |
         (COMP_MODE_TRAD << COMP_MODE_offset) |
         (COMP_POLL_AL << COMP_POLL_offset) |
         (COMP_Q_disable << COMP_Q_offset);


   pinMode (RANGE_SENSOR_CHAIN_TRIGGER_GPIO, OUTPUT) ;
   digitalWrite(RANGE_SENSOR_CHAIN_TRIGGER_GPIO, 0);
   currentSensor = 0;
   startRangeSensorChain();
}

int rangeSensors::i2c16bMsbfirst(int data) {
   return (((data & 0xFF) << 8) | ((data & 0xFF00) >> 8)) & 0xFFFF;
}


void rangeSensors::startRangeSensorChain() {
   ros::Time time_stamp = ros::Time::now();
   digitalWrite(RANGE_SENSOR_CHAIN_TRIGGER_GPIO, 1);
   while ((ros::Time::now().toSec() - time_stamp.toSec()) < MB1000_MIN_TX_PULSE_WIDTH) {
   }
   digitalWrite(RANGE_SENSOR_CHAIN_TRIGGER_GPIO, 0);
}



double rangeSensors::getRange() {

   int data;
   double result;
   bool convDone=false;
   int nextSensor;

   while(!convDone){
      if ((data = wiringPiI2CReadReg16  (i2c_dev, ADC_REG_CNFG)) < 0) {
         ROS_ERROR("Failed to read from I2C device %d on register %d", i2c_dev, ADC_REG_CONV);
      }
      data = i2c16bMsbfirst(data);
      if((data & (OS_1 << OS_offset)) > 0 ){
         convDone = true;
      }
   }

   if ((data = wiringPiI2CReadReg16  (i2c_dev, ADC_REG_CONV)) < 0) {
      ROS_ERROR("Failed to read from I2C device %d on register %d", i2c_dev, ADC_REG_CONV);
   }
   data = i2c16bMsbfirst(data) >> 3;
   result = (ADC_FULL_RANGE_PGA_1x*(((double) data)/ADC_FULL_OUPUT_CODE))/MB1000_VOLT_PER_METER_AT_3V3;
   //ROS_INFO("sensor %d read %x %f", currentSensor, data, result);

   nextSensor = currentSensor + 1;
   if (nextSensor==NUM_RANGE_SENSORS) {
      nextSensor = 0;
   }


   data = (OS_1 << OS_offset) |
         (sensorIDLUT[nextSensor] << MUX_offset) |
         (PGA_1x << PGA_offset) |
         (MODE_SS << MODE_offset) |
         (DR_128 << DR_offset) |
         (COMP_MODE_TRAD << COMP_MODE_offset) |
         (COMP_POLL_AL << COMP_POLL_offset) |
         (COMP_Q_disable << COMP_Q_offset);

   data = i2c16bMsbfirst(data);


   if (wiringPiI2CWriteReg16  (i2c_dev, ADC_REG_CNFG, data) < 0) {
      ROS_ERROR("Failed to write to I2C device %d on register %d", i2c_dev, ADC_REG_CNFG);
   }

   return result;
}

void rangeSensors::publishRange() {

   double range;

   ros::Time current_time;

   current_time = ros::Time::now();

   range = getRange();

   range_msg.header.stamp = current_time;
   range_msg.header.frame_id = range_frame_id_[currentSensor];
   range_msg.range = range; //            # range data [m]
   range_msg.field_of_view  = 30; //  # the size of the arc that the distance reading is

   range_pub_[currentSensor].publish(range_msg);
   currentSensor++;
   if (currentSensor==NUM_RANGE_SENSORS) {
      startRangeSensorChain();
      currentSensor = 0;
   }

}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "range_sensors");
   rangeSensors sensors;
   ros::Rate loop_rate(RANGE_SENSOR_SAMPLE_RATE);
   while (ros::ok())
   {
      loop_rate.sleep();
      sensors.publishRange();
      ros::spinOnce();

   }
}
