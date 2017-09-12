#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <std_srvs/SetBool.h>
#include "ccsr2_pi_sensact_hub/motorAcceleration.h"
#include <sstream>

#define MD22_I2C_ADDRESS           0x58
#define MD22_I2C_REG_MODE          0x0
#define MD22_I2C_REG_LSPEED        0x1
#define MD22_I2C_REG_RSPEED        0x2
#define MD22_I2C_REG_ACCELERATION  0x3

#define MD22_I2C_MODE              0x1
// From https://www.robot-electronics.co.uk/htm/md22tech.htm
// Mode 1 is similar to Mode 0, except that the speed registers are interpreted as signed values.
// The meaning of the speed registers is literal speeds in the range of:
//       -128 (full reverse)   0 (stop)   127 (full forward).
#define MD22_I2C_ACCELERATION     255
// The acceleration register contains the rate at which the motor board moves through the steps.
// At 0 (default) the board changes the power (accelerates) at its fastest rate, each step taking 64us.
// When the acceleration register is loaded with the Slowest setting of 255, the board will change the
// power output every 16.4ms.


class motorDriver {
   private:
      ros::NodeHandle n;
      ros::Subscriber motor_sub;
      int i2c_dev;
      std::string motor;
      bool noRobotHw;
      bool motorsEnabled;
      int speed_reg;
      ros::ServiceServer setMotorAcceleration_srv_;
      ros::ServiceServer enableMotors_srv_;
      bool setMotorAccelerationCallback(ccsr2_pi_sensact_hub::motorAcceleration::Request&, ccsr2_pi_sensact_hub::motorAcceleration::Response&);
      bool enableMotorsCallback(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

   public:
      motorDriver();
      void updateSpeed(const std_msgs::Float32& msg);
};
