/*
 * powerMonitor.h
 *
 *  Created on: Oct 1, 2017
 *      Author: robert
 */

#include <std_srvs/SetBool.h>

#define TEMP_SENS_SAMPLE_RATE 1 // Hz


// ADS1015

#define TMP102_ADDR 0x48
#define TMP102_TEMP_REG  0x0
#define TMP102_CONFIG_REG  0x1
#define TMP102_TLOW_REG  0x2
#define TMP102_THIGH_REG  0x3


class tempSensor {
   private:
      ros::NodeHandle n;
      int i2c_dev;
      bool enabled_;
      sensor_msgs::Temperature temp_msg;
      ros::ServiceServer enableTempSensor_srv_;
      std::string temp_frame_id_;
      ros::Publisher temp_pub_;
   public:
      tempSensor();
      double getTemperature();
      void publishTemp();
      bool enableTempSensorCallback(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
};


