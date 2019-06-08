#include "hongfu_bms_status.h"

int main(int argc, char *argv[])
{

  uint16_t buffer_sum_all = 0x00, checksum_all = 0x00;
  int index = 0;
  std::vector<uint8_t> bufferV;
  ros::init(argc, argv, "hongfu_bms_status");
  ros::NodeHandle nod("~");
  HongfuBmsStatus hongfuBmsStatus(nod);  
  hongfuBmsStatus.initPort();
  ros::Rate loop_rate(hongfuBmsStatus.looprate_bms_);
  while(ros::ok) {
    hongfuBmsStatus.buffer_vol_.clear();
    hongfuBmsStatus.buffer_all_.clear();
    hongfuBmsStatus.hongfu_status_.NtcTem.clear();
    hongfuBmsStatus.hongfu_status_.CellVoltage.clear();
    hongfuBmsStatus.hongfu_status_.ErrorId.clear();
    hongfuBmsStatus.hongfu_status_.ErrorInfo.clear();
    hongfuBmsStatus.buffer_all_ = hongfuBmsStatus.dataRead(0x03, 0xFD, buffer_sum_all, checksum_all, bufferV);
    hongfuBmsStatus.buffer_vol_ = hongfuBmsStatus.dataRead(0x04, 0xFC, buffer_sum_all, checksum_all, bufferV);
    hongfuBmsStatus.time_now_ = ros::Time::now();
    hongfuBmsStatus.dataParsing(hongfuBmsStatus.buffer_all_, hongfuBmsStatus.buffer_vol_);
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}
