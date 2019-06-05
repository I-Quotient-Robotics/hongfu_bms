#ifndef __HONGFU_BMS_STATUS_HONGFU_BMS_DRIVER_H__
#define __HONGFU_BMS_STATUS_HONGFU_BMS_DRIVER_H__
#include "ros/ros.h"
#include "unistd.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "string.h"
#include "hongfu_bms/Hongfustatus.h"
#include "vector"
#include "boost/format.hpp"
class HongfuBmsStatus {

public:
    hongfu_bms::Hongfustatus hongfuStatus;
    serial::Serial bms_ser_;
    ros::Time timeNow;
    bool findpack = false;    
    std::vector<uint8_t> buffer_all, buffer_vol;
    int looprate_bms;
    
    HongfuBmsStatus(ros::NodeHandle& nod);
    bool initport();
    void dataread();
    std::vector<uint8_t> dataRead(float datetype, float checksumW, uint16_t buffer_sum, uint16_t checksumR, std::vector<uint8_t> buffer);
    void dataParsing(std::vector<uint8_t>& all, std::vector<uint8_t>& vol);
    std::vector<int> error_id;
    std::string error_info[13] = {"Monomer Overvoltage Protection", "Single undervoltage protection",
        "Overvoltage protection of whole group", "Overall undervoltage protection", 
        "Charging Overtemperature Protection", "Charging cryogenic protection", 
        "Discharge Overtemperature Protection", "Discharge cryogenic protection", 
        "Charging Overcurrent Protection", "Discharge Overcurrent Protection", 
        "Short circuit protection", "Front-end IC error detection", "Software Lock-in MOS"};    

private:
    ros::Publisher hongfu_pub_;
    std::string port_bms, hongfu_id, data_production; 
    float ntfC[10], cell[30];
    float voltage, current;
    uint8_t buffer_write[7];
    uint16_t data_pro, status_protect, version, statue_mos;
    uint32_t status_balance;
    int baudrate_bms, cell_number, residual_capacity, design_capacity, cycle_index, rsoc,
        ntc, ntc_number, day_pro, month_pro, year_pro;  
    
    void hongfuCallback();


};
#endif