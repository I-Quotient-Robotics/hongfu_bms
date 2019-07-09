#include "hongfu_bms_status.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hongfu_bms_status_node");
  ros::NodeHandle private_nh("~");

  iqr::HongfuBmsStatus hongfu_bms_status(private_nh);  
  hongfu_bms_status.initPort();

  uint16_t buffer_sum = 0x00;
  std::vector<uint8_t> buffer_v;
  ros::Rate loop_rate(hongfu_bms_status.looprate_);
  while(ros::ok) {
    hongfu_bms_status.buffer_all_ = hongfu_bms_status.dataRead(
        hongfu_bms_status.cmd_status_,
        hongfu_bms_status.cmd_status_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    hongfu_bms_status.buffer_vol_ = hongfu_bms_status.dataRead(
        hongfu_bms_status.cmd_voltage_,
        hongfu_bms_status.cmd_voltage_sum_,
        buffer_sum,
        buffer_sum,
        buffer_v);

    hongfu_bms_status.dataParsing(hongfu_bms_status.buffer_all_, hongfu_bms_status.buffer_vol_);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
