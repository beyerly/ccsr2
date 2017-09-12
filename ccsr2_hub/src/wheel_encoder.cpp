#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "ccsr2_pi_sensact_hub/wheel_encoder.h"
#include <sstream>
#include <wiringPi.h>



wheelEncoder::wheelEncoder() {
   if (!ros::param::get("~wheel", wheel)) {
      ROS_ERROR("Failed to get param 'wheel'");
   }
   wheel_pub = n->advertise<std_msgs::Int16>(wheel, 1000);
   twist = n.subscribe('twist', 1, &wheelEncoder::updateEncoder, this);

   if (strcmp (wheel.c_str(), "rwheel")) {
      spiPort = LWHEELENCODER_SPI_PORT;
   }
   else {
      spiPort = RWHEELENCODER_SPI_PORT;
   }

   if (wiringPiSPISetup (spiPort, SPI_SPEED) >= 0){
      ROS_INFO("Created %s wheel_encoder node on SPI port %d ", wheel.c_str(), spiPort);
   }
   else {
      ROS_ERROR("Failed to open SPI port %d", spiPort);
   }

   unsigned char data[2];
   data[0] = WR_MDR0;
   data[1] = X1_QUAD | FREE_RUN | DISABLE_IDX | ASYN_IDX | CLK_DIV_1;
   if (wiringPiSPIDataRW (spiPort, data, 2)){
      ROS_ERROR("Failed to write to SPI port %d", spiPort);
   }


}

int wheelEncoder::buffToInteger(char * buffer)
    int a = (int)(buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
    return a;
}

void wheelEncoder::updateEncoder(const geometry_msgs::Twist& msg) {

/* We don;t do anything with the velocity messages, we just trigger odometry retrieval from
 * encoder chip when velocity messages are streaming. If not, robot is stationary anyway
 * perhaps relate poll-rate to velocity? Can polling take longer, and twist msgs back up?
 */


   unsigned char data[5];
   data[0] = RD_CNTR;
   if (wiringPiSPIDataRW (spiPort, data, 5)){
      ROS_ERROR("Failed to read to SPI port %d", spiPort);
   }

   wheel_count = buffToInteger(data);
   ROS_INFO("%s wheel_encoder at %d", wheel.c_str(), wheel_count);
   std_msgs::Int16 msg;  // @@ increase to 32b?
   msg.data = 0xFFFF & wheel_encoder;
   wheelEncoder::wheel_pub->publish(msg);
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_encoder");
  wiringPiSetup () ;
  wheelEncoder wheel;
  ros::spin();
  return 0;
}


