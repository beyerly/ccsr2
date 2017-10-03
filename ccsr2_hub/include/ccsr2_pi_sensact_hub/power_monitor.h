/*
 * powerMonitor.h
 *
 *  Created on: Oct 1, 2017
 *      Author: robert
 */

#ifndef CCSR2_HUB_INCLUDE_CCSR2_PI_SENSACT_HUB_POWER_MONITOR_H_
#define CCSR2_HUB_INCLUDE_CCSR2_PI_SENSACT_HUB_POWER_MONITOR_H_


#define PMON_SAMPLE_RATE 2 // Hz


// ADS1015

#define INA219_ADDR 0x40
#define RST_offset 15
#define BRNG_offset 13
#define PG_offset 11
#define BADC_offset 7
#define CADC_offset 3
#define PMMODE_offset 0

#define RST_ASSERT 1
#define RST_DEASSERT 0
#define BRNG_32V 1
#define PG_DIV8 3
#define BADC_12b 3
#define CADC_12b 3
#define MODE_SHUNT_BUS_CONT 7

#define INA219_REG_CNFG 0
#define INA219_REG_VSHUNT 1
#define INA219_REG_VBUS 2
#define INA219_REG_PWR 3
#define INA219_REG_CUR 4
#define INA219_REG_CAL 5

#define MAX_VBUS_RANGE 8000
#define MIN_VBUS_RANGE 0
#define MAX_VBAT_RANGE 32000 // mV

#define MAX_VSHUNT_RANGE 32000
#define MIN_VSHUNT_RANGE 0
#define MAX_I_RANGE 3200 //mA

#define MIN_VMAINBATTERY 6000 // mV
#define MAX_VMAINBATTERY 8400 // mV
#define VMAINBATTERY_RANGE 2400

#define MAX_OPERATING_CURRENT 3500 // mA
#define LOW_BATTERY_LEVEL 5 // percent
#define MAX_LOW_BATTERY_EVENTS 16 // So POWER_MONITOR_PERIOD*MAX_LOW_BATTERY_EVENTS delay before alarm
#define MAX_CURRENTLIMIT_EVENTS 16 // So POWER_MONITOR_PERIOD*MAX_CURRENTLIMIT_EVENTS delay before alarm

#define CURRENT_AVG_WINDOW 8
#define VOLTAGE_AVG_WINDOW 3


class powerMonitor {
   private:
      ros::NodeHandle n;
      int i2c_dev;
      sensor_msgs::BatteryState pmon_msg;
      std::string pmon_frame_id_;
      ros::Publisher pmon_pub_;
   public:
      powerMonitor();
      int updateCharge();
      double getBatteryVoltage();
      double getOperatingCurrent();
      void publishPower();
};


#endif /* CCSR2_HUB_INCLUDE_CCSR2_PI_SENSACT_HUB_POWER_MONITOR_H_ */
