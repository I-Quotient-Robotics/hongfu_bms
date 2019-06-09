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
    hongfu_bms_msg::HongfuStatus hongfu_status_;
    serial::Serial bms_ser_;
    ros::Time time_now_;
    bool findpack = false;    
    std::vector<uint8_t> buffer_all_, buffer_vol_, path_node_vec_;
    int looprate_bms_;
    HongfuBmsStatus(ros::NodeHandle& nod);
    bool initPort(int argc, char *argv[]);
    std::vector<uint8_t> dataRead(float date_type, float check_sum_write, uint16_t buffer_sum, 
        uint16_t check_sum_read, std::vector<uint8_t> buffer);
    void dataParsing(std::vector<uint8_t>& all, std::vector<uint8_t>& vol);
    std::vector<int> error_id_;  

private:
    ros::Publisher hongfu_pub_;
    std::string port_bms_, hongfu_id_, data_production_string_, path_node_str_; 
    float ntf_data_[10], cell_[30];
    float voltage_, current_;
    uint8_t buffer_write_[7];
    uint16_t data_production_int_, status_protect_, version_, mos_status_;
    uint32_t status_balance_;
    int baudrate_bms_, cell_number_, residual_capacity_, design_capacity_, cycle_index_, rsoc_,
        ntc_, ntc_number_, day_production_, month_production_, year_production_;  
    void hongfuCallback();
};
}
#endif