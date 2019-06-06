#include "hongfu_bms_status.h"

HongfuBmsStatus::HongfuBmsStatus(ros::NodeHandle& nod) {
    buffer_write[0] = 0xDD;
    buffer_write[1] = 0xA5;
    buffer_write[2] = 0x03;
    buffer_write[3] = 0x00;
    buffer_write[4] = 0xFF;
    buffer_write[5] = 0xFD;
    buffer_write[6] = 0x77;
    nod.param<std::string>("Port_bms", port_bms, "/dev/ttyUSB0");
    nod.param<std::string>("Hongfu_id", hongfu_id, "hongfu_bms");
    nod.param<int>("Looprate_bms", looprate_bms, 2);
    nod.param<int>("Baudrate_bms", baudrate_bms, 9600);
    hongfu_pub_ = nod.advertise<hongfu_bms_msg::Hongfustatus>("hongfu_bms", 1); 
}

void HongfuBmsStatus::hongfuCallback() {
}

bool HongfuBmsStatus::initport() {
    ros::Rate loop_openport(0.2);
    while(!bms_ser_.isOpen()) {
        try {
            bms_ser_.setPort(port_bms);
            bms_ser_.setBaudrate(baudrate_bms);
            serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
            bms_ser_.setTimeout(t_out);
            bms_ser_.open();
            ROS_INFO_STREAM("Serial port initialized");
        }
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port ");
            ROS_ERROR_STREAM("Try again ");
            sleep(1);    
            ROS_ERROR_STREAM("5");
            sleep(1);    
            ROS_ERROR_STREAM("4");
            sleep(1);    
            ROS_ERROR_STREAM("3");
            sleep(1);    
            ROS_ERROR_STREAM("2");
            sleep(1);    
            ROS_ERROR_STREAM("1");        
            loop_openport.sleep(); 
        }        
    }
}

void HongfuBmsStatus::dataParsing(std::vector<uint8_t>& buffer_read,std::vector<uint8_t>& buffer_vol) {

    if (buffer_read.size()!=0) {
        voltage = (buffer_read[4]<<8|buffer_read[5])/100.0;
        if (((buffer_read[6] & 0b10000000) >> 7) == 1) {
            current = ((buffer_read[6]<<8|buffer_read[7])-65535.0)/100.0;
        }
        else {
            current = (buffer_read[6]<<8|buffer_read[7])/100.0;
        }
        residual_capacity = (buffer_read[8]<<8|buffer_read[9])*10;
        design_capacity = (buffer_read[10]<<8|buffer_read[11])*10;
        cycle_index = (buffer_read[12]<<8|buffer_read[13]);
        data_pro = (buffer_read[14]<<8|buffer_read[15]);
        status_balance = (buffer_read[16]<<8|buffer_read[17]|buffer_read[18]<<8|buffer_read[19]);
        status_protect = (buffer_read[20]<<8|buffer_read[21]);
        version = buffer_read[22];
        rsoc = buffer_read[23];
        statue_mos = buffer_read[24];
        cell_number = buffer_read[25];
        ntc_number = buffer_read[26];
        for (int i = 0; i < ntc_number*2; i+=2) {
            ntfC[i/2] = ((buffer_all[27+i]<<8|buffer_all[28+i])-2731)/10.0;
        }
        day_pro = (data_pro&0x1f);
        month_pro = ((data_pro>>5)&0x0f);
        year_pro = (2000+ (data_pro>>9));
        data_production = (boost::format("%04d-%02d-%02d") % year_pro % month_pro % day_pro).str();
       
        hongfuStatus.header.stamp = timeNow;
        hongfuStatus.header.frame_id = hongfu_id;
        hongfuStatus.Voltage = voltage;
        hongfuStatus.Current = current;
        hongfuStatus.ResidualCapacity = residual_capacity;
        hongfuStatus.DesignCapacity = design_capacity;
        hongfuStatus.CycleIndex = cycle_index;
        hongfuStatus.DataProduction = data_production;
        hongfuStatus.StatusBalance = status_balance;
        hongfuStatus.StatusProtect = status_protect;
        hongfuStatus.Version = version;
        hongfuStatus.Rsoc = rsoc;
        hongfuStatus.StatueMos = statue_mos;
        hongfuStatus.CellNumber = cell_number;
        hongfuStatus.NtcNumber = ntc_number;
        for (int i = 0; i < ntc_number; ++i) {
            hongfuStatus.NtcTem.push_back(ntfC[i]);
        }
        for (int i = 0; i < 13; ++i) {
            if((status_protect & (0x0001>>i)>>i)==1){
            hongfuStatus.ErrorId.push_back(i);
            hongfuStatus.ErrorInfo.push_back(error_info[i]);  
            }
        }
    }
    /////////////****************///////////////
    if (buffer_vol.size()!=0) {   
        cell_number = buffer_vol[3]/2;
        for (int i = 0; i < cell_number*2; i+=2) {
            cell[i/2] = (buffer_vol[4+i]<<8|buffer_vol[5+i])/1000.0;
            hongfuStatus.CellVoltage.push_back(cell[i/2]);       
        }   
    }
    hongfu_pub_.publish(hongfuStatus);
}

std::vector<uint8_t> HongfuBmsStatus::dataRead(float datetype, float checksumW, uint16_t buffer_sum, uint16_t checksumR, std::vector<uint8_t> buffer) {
    int index = 0;
    buffer_write[2] = datetype;
    buffer_write[5] = checksumW;
    bms_ser_.write(buffer_write,7); 
    ros::Duration(0.1).sleep();
    if (bms_ser_.available()) {   
        bms_ser_.read(buffer, bms_ser_.available());
        while(!findpack) {
            if (buffer[index]==0xDD) {
                buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+index);
                checksumR = buffer[buffer.size()-3]<<8|buffer[buffer.size()-2];
                for (int i = 0; i < buffer.size()-5; ++i) {
                    buffer_sum += buffer[i+2];
                }
                buffer_sum = ~buffer_sum + 1;
                if (buffer_sum==checksumR) {
                    findpack = true;
                }
            }
            index += 1;
        }            
    }
    return buffer;
}