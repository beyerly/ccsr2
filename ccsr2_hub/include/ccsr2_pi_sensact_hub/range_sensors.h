

#define NUM_RANGE_SENSORS 2
#define MAX_NUM_RANGE_SENSORS 4 // max 4 ADC analog inputs. For more instantiate another range_sensors node
#define RANGE_SENSOR_CHAIN_TRIGGER_GPIO 0  // Pi GPIO
#define RANGE_SENSOR_SAMPLE_RATE 4   // Hz
#define ADC_FULL_OUPUT_CODE 4095  // 2^12-1 bits of the ADC_REG_CONV
#define ADC_FULL_RANGE_PGA_1x 4.096   // V


#define MB1000_VOLT_PER_METER_AT_3V3 0.2519685039
#define MB1000_MIN_TX_PULSE_WIDTH 0.00002  // sec

// ADS1015 A/D converter

#define ADC0_ADDR 0x49
#define OS_offset 15
#define MUX_offset 12
#define PGA_offset 9
#define MODE_offset 8
#define DR_offset 5
#define COMP_MODE_offset 4
#define COMP_POLL_offset 3
#define COMP_Q_offset 0

#define OS_1 1
#define OS_0 0
#define MUX_AIN0 4
#define MUX_AIN1 5
#define MUX_AIN2 6
#define MUX_AIN3 7
#define PGA_1x 1          // 4.096V
#define MODE_SS 1
#define MODE_CONT 0
#define DR_3300 7
#define DR_1600 4
#define DR_128 0
#define COMP_MODE_TRAD 0
#define COMP_POLL_AL 0
#define COMP_Q_disable 3

#define ADC_REG_CONV 0
#define ADC_REG_CNFG 1
#define ADC_REG_THRS_LO 2
#define ADC_REG_THRS_HI 3


class rangeSensors {
   private:
      int i2c_dev;
      ros::NodeHandle n;
      std::string range_frame_id_[NUM_RANGE_SENSORS];
      sensor_msgs::Range range_msg;
      ros::Publisher range_pub_[NUM_RANGE_SENSORS];
      int sensorIDLUT [MAX_NUM_RANGE_SENSORS] { 4, 5, 6, 7};
      int currentSensor;
   public:
      rangeSensors();
      int i2c16bMsbfirst(int data);
      void startRangeSensorChain();
      double getRange();
      void publishRange();
};

