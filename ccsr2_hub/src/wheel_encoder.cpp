#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "ccsr2_pi_sensact_hub/wheel_encoder.h"
#include <sstream>
#include <wiringPiSPI.h>
#include <unistd.h>

wheelEncoder::wheelEncoder() {
   if (!ros::param::get("~wheel", wheel)) {
      ROS_ERROR("Failed to get param 'wheel'");
   }
   wheel_pub = n.advertise<std_msgs::Int16>(wheel, 1000);
   std::string topicName;

   if (strcmp (wheel.c_str(), "rwheel")) {
      spiPort = LWHEELENCODER_SPI_PORT;
      topicName = "lmotor_cmd";
   }
   else {
      spiPort = RWHEELENCODER_SPI_PORT;
      topicName = "rmotor_cmd";
   }

   motor_cmd = n.subscribe(topicName, 1, &wheelEncoder::updateEncoder, this);

   if (wiringPiSPISetup (spiPort, SPI_SPEED) >= 0){
      ROS_INFO("Created %s wheel_encoder node on SPI port %d ", wheel.c_str(), spiPort);
   }
   else {
      ROS_ERROR("Failed to open SPI port %d", spiPort);
   }

   std::string serviceName;
   serviceName = "resetWheelCount";
   resetWheelCount_srv_ = n.advertiseService(serviceName.c_str(), &wheelEncoder::resetWheelCountCallback, this);

   unsigned char data[2];
   data[0] = WR_MDR0;
   data[1] = X1_QUAD | FREE_RUN | DISABLE_IDX | ASYN_IDX | CLK_DIV_1;
   if (wiringPiSPIDataRW (spiPort, data, 2) < 0){
      ROS_ERROR("Failed to write MDR0 through SPI port %d", spiPort);
   }
   resetWheelCount();
}

bool wheelEncoder::resetWheelCountCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
   resetWheelCount();
   return true;
}

void wheelEncoder::resetWheelCount(){
    ROS_INFO("%s wheel encoder reset ", wheel.c_str());
    unsigned char data[5];
//    data[0] = WR_DTR;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
//    data[4] = 3;
//
//    if (wiringPiSPIDataRW (spiPort, data, 5) < 0){
//       ROS_ERROR("Failed to write CNTR through SPI port %d", spiPort);
//    }

//
//     data[0] = RD_DTR;
//     if (wiringPiSPIDataRW (spiPort, data, 5) < 0){
//        ROS_ERROR("Failed to read to SPI port %d", spiPort);
//     }
//
//     wheel_count = buffToInteger(data);
//     ROS_INFO(" DTR %u",  wheel_count);
//
//
//
//
//    usleep(1000);
//    data[0] = LD_CNTR;
//    if (wiringPiSPIDataRW (spiPort, data, 1) < 0){
//       ROS_ERROR("Failed to read to SPI port %d", spiPort);
//    }
////
//    data[0] = RD_CNTR;
//    if (wiringPiSPIDataRW (spiPort, data, 5) < 0){
//       ROS_ERROR("Failed to read to SPI port %d", spiPort);
//    }
//
//    wheel_count = buffToInteger(data);
//    ROS_INFO(" CNTR %u",  wheel_count);


    data[0] = CLR_CNTR;
    if (wiringPiSPIDataRW (spiPort, data, 1) < 0){
       ROS_ERROR("Failed to read to SPI port %d", spiPort);
    }

    data[0] = RD_MDR0;
    if (wiringPiSPIDataRW (spiPort, data, 2) < 0){
       ROS_ERROR("Failed to read to SPI port %d", spiPort);
    }

    ROS_INFO(" MDR %u",  data[1]);
    data[0] = RD_STR;
    if (wiringPiSPIDataRW (spiPort, data, 2) < 0){
       ROS_ERROR("Failed to read to SPI port %d", spiPort);
    }

    ROS_INFO(" STR %u",  data[1]);



}


unsigned int wheelEncoder::buffToInteger(unsigned char * buffer){
   unsigned int a = (int)(buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
   return a;
}

void wheelEncoder::updateEncoder(const std_msgs::Float32& msg) {

/* We don;t do anything with the velocity messages, we just trigger odometry retrieval from
 * encoder chip when velocity messages are streaming. If not, robot is stationary anyway
 * perhaps relate poll-rate to velocity? Can polling take longer, and twist msgs back up?
 */


   unsigned char data[5];
   data[0] = RD_CNTR;
   if (wiringPiSPIDataRW (spiPort, data, 5) < 0){
      ROS_ERROR("Failed to read to SPI port %d", spiPort);
   }

   wheel_count = buffToInteger(data);
   std_msgs::Int16 whlmsg;  // @@ increase to 32b?
   whlmsg.data = (int) (0xFFFF & wheel_count) - 32768;
   ROS_INFO("%s wheel_encoder at %d %d", wheel.c_str(), wheel_count, whlmsg.data);
   wheelEncoder::wheel_pub.publish(whlmsg);
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_encoder");
  wheelEncoder wheel;
  ros::spin();
  return 0;
}


