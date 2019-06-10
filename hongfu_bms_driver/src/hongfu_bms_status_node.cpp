#include "hongfu_bms_status.h"

int main(int argc, char *argv[])
{

  uint16_t buffer_sum = 0x00;
  std::vector<uint8_t> bufferV;
  ros::init(argc, argv, "hongfu_bms");
  ros::NodeHandle nod("~");
  IQR::HongfuBmsStatus hongfuBmsStatus(nod);  
  hongfuBmsStatus.initPort();
  ros::Rate loop_rate(hongfuBmsStatus.looprate_);
  while(ros::ok) {
    hongfuBmsStatus.buffer_all_ = hongfuBmsStatus.dataRead(hongfuBmsStatus.cmd_status_,
      hongfuBmsStatus.cmd_status_sum_, buffer_sum, buffer_sum, bufferV);
    hongfuBmsStatus.buffer_vol_ = hongfuBmsStatus.dataRead(hongfuBmsStatus.cmd_voltage_,
      hongfuBmsStatus.cmd_voltage_sum_, buffer_sum, buffer_sum, bufferV);
    hongfuBmsStatus.dataParsing(hongfuBmsStatus.buffer_all_, hongfuBmsStatus.buffer_vol_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
