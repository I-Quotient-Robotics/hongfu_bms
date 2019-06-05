#include "hongfu_bms_status.h"

int main(int argc, char *argv[])
{

    uint16_t buffer_sumall = 0x00, checksumall = 0x00;
    int index = 0;
    std::vector<uint8_t> bufferV;
    ros::init(argc, argv, "hongfu_bms_status");
    ros::NodeHandle nod("~");
    HongfuBmsStatus hongfuBmsStatus(nod);  
    hongfuBmsStatus.initport();
    ros::Rate loop_rate(hongfuBmsStatus.looprate_bms);
    while(ros::ok) {
        hongfuBmsStatus.buffer_vol.clear();
        hongfuBmsStatus.buffer_all.clear();
        hongfuBmsStatus.hongfuStatus.NtcTem.clear();
        hongfuBmsStatus.hongfuStatus.CellVoltage.clear();
        hongfuBmsStatus.hongfuStatus.ErrorId.clear();
        hongfuBmsStatus.hongfuStatus.ErrorInfo.clear();

        try {   
            hongfuBmsStatus.buffer_all = hongfuBmsStatus.dataRead(0x03, 0xFD, buffer_sumall, checksumall, bufferV);
            hongfuBmsStatus.buffer_vol = hongfuBmsStatus.dataRead(0x04, 0xFC, buffer_sumall, checksumall, bufferV);
            hongfuBmsStatus.timeNow = ros::Time::now();
        }
        catch (serial::SerialException& e) {
            hongfuBmsStatus.bms_ser_.close();
            hongfuBmsStatus.initport();
        }
        catch (serial::IOException& e) {
            hongfuBmsStatus.bms_ser_.close();
            hongfuBmsStatus.initport();
        } 
        hongfuBmsStatus.dataParsing(hongfuBmsStatus.buffer_all, hongfuBmsStatus.buffer_vol);
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
