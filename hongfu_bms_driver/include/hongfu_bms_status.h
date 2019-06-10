#ifndef __HONGFU_BMS_STATUS_HONGFU_BMS_DRIVER_H__
#define __HONGFU_BMS_STATUS_HONGFU_BMS_DRIVER_H__
#include "ros/ros.h"
#include "unistd.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "string.h"
#include "hongfu_bms_msg/HongfuStatus.h"
#include "vector"
#include "boost/format.hpp"

namespace IQR{
  class HongfuBmsStatus {

  public:
      
      uint8_t cmd_status_ = 0x03, cmd_voltage_ = 0x04, cmd_status_sum_ = 0xFD, cmd_voltage_sum_ = 0xFC;
      std::vector<uint8_t> buffer_all_, buffer_vol_;
      int looprate_;
      HongfuBmsStatus(ros::NodeHandle& nod);
      bool initPort();
      std::vector<uint8_t> dataRead(uint8_t date_type, uint8_t check_sum_write, uint16_t buffer_sum, 
          uint16_t check_sum_read, std::vector<uint8_t> buffer);
      void dataParsing(std::vector<uint8_t>& all, std::vector<uint8_t>& vol);

  private:
      
      hongfu_bms_msg::HongfuStatus hongfu_status_;
      serial::Serial bms_ser_;
      ros::Time time_now_;
      bool findpack = false; 
      std::vector<int> error_id_;  
      ros::Publisher hongfu_pub_;
      std::string port_, frame_id_, data_production_string_, path_name_, node_name_; 
      float ntf_data_[10], cell_[30];
      float voltage_, current_;
      uint8_t buffer_write_[7];
      uint16_t data_production_int_, status_protect_, version_, mos_status_;
      uint32_t status_balance_;
      int baudrate_, cell_number_, residual_capacity_, design_capacity_, cycle_index_, rsoc_,
          ntc_, ntc_number_, day_production_, month_production_, year_production_, position_;  
      void hongfuCallback();
  };
}
#endif