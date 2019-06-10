#include "hongfu_bms_status.h"

const std::string error_info[14] = {"Monomer Overvoltage Protection", "Single undervoltage protection",
  "Overvoltage protection of whole group", "Overall undervoltage protection", 
  "Charging Overtemperature Protection", "Charging cryogenic protection", 
  "Discharge Overtemperature Protection", "Discharge cryogenic protection", 
  "Charging Overcurrent Protection", "Discharge Overcurrent Protection", 
  "Short circuit protection", "Front-end IC error detection", "Software Lock-in MOS"};  

IQR::HongfuBmsStatus::HongfuBmsStatus(ros::NodeHandle& nod) {
  buffer_write_[0] = 0xDD;
  buffer_write_[1] = 0xA5;
  buffer_write_[3] = 0x00;
  buffer_write_[4] = 0xFF;
  buffer_write_[6] = 0x77;
  nod.param<std::string>("port_bms", port_bms_, "port_link_bms");
  nod.param<std::string>("hongfu_id", hongfu_id_, "hongfu_bms");
  nod.param<int>("looprate_bms", looprate_bms_, 2);
  nod.param<int>("baudrate_bms", baudrate_bms_, 9600);
  hongfu_pub_ = nod.advertise<hongfu_bms_msg::HongfuStatus>("hongfu_bms", 1); 
}

void IQR::HongfuBmsStatus::hongfuCallback() {
}

bool IQR::HongfuBmsStatus::initPort(char *argv[]) {
// bool IQR::HongfuBmsStatus::initPort() {
  ros::Rate loop_openport(0.2);
  std::string path_node_str_(argv[0]); 
  int position = path_node_str_.rfind('/');
  std::string ss = path_node_str_.substr(position);
  ss = ss.at(0);  
  while(!bms_ser_.isOpen()) {
    try {
      bms_ser_.setPort(port_bms_);
      bms_ser_.setBaudrate(baudrate_bms_);
      serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
      bms_ser_.setTimeout(t_out);
      bms_ser_.open();
      ROS_INFO("[%s]Serial port initialized", ss);
      ROS_INFO_STREAM("[hongfu_bms]Serial port initialized");
    }
    catch (serial::IOException& e) {
      ROS_ERROR_STREAM("[hongfu_bms]Unable to open port ");
      ROS_ERROR_STREAM("[hongfu_bms]Try again,wait 5 secs");       
      loop_openport.sleep(); 
    }        
  }
}

void IQR::HongfuBmsStatus::dataParsing(std::vector<uint8_t>& buffer_read,std::vector<uint8_t>& buffer_vol) {

  // if (!bms_ser_.isOpen())
  // {
  //   buffer_read.clear();
  //   buffer_vol.clear();
  // }
  if (buffer_read.size()!=0) {
    voltage_ = (buffer_read[4]<<8|buffer_read[5])/100.0;
    if (((buffer_read[6] & 0b10000000) >> 7) == 1) {
      current_ = ((buffer_read[6]<<8|buffer_read[7])-65535.0)/100.0;
    }
    else {
      current_ = (buffer_read[6]<<8|buffer_read[7])/100.0;
    }
    residual_capacity_ = (buffer_read[8]<<8|buffer_read[9])*10;
    design_capacity_ = (buffer_read[10]<<8|buffer_read[11])*10;
    cycle_index_ = (buffer_read[12]<<8|buffer_read[13]);
    data_production_int_ = (buffer_read[14]<<8|buffer_read[15]);
    status_balance_ = (buffer_read[16]<<8|buffer_read[17]|buffer_read[18]<<8|buffer_read[19]);
    status_protect_ = (buffer_read[20]<<8|buffer_read[21]);
    version_ = buffer_read[22];
    rsoc_ = buffer_read[23];
    mos_status_ = buffer_read[24];
    cell_number_ = buffer_read[25];
    ntc_number_ = buffer_read[26];
    for (int i = 0; i < ntc_number_*2; i+=2) {
      ntf_data_[i/2] = ((buffer_read[27+i]<<8|buffer_read[28+i])-2731)/10.0;
    }
    day_production_ = (data_production_int_&0x1f);
    month_production_ = ((data_production_int_>>5)&0x0f);
    year_production_ = (2000+ (data_production_int_>>9));
    data_production_string_ = (boost::format("%04d-%02d-%02d") % year_production_ % month_production_% 
        day_production_).str();
   
    hongfu_status_.header.stamp = time_now_;
    hongfu_status_.header.frame_id = hongfu_id_;
    hongfu_status_.Voltage = voltage_;
    hongfu_status_.Current = current_;
    hongfu_status_.ResidualCapacity = residual_capacity_;
    hongfu_status_.DesignCapacity = design_capacity_;
    hongfu_status_.CycleIndex = cycle_index_;
    hongfu_status_.DataProduction = data_production_string_;
    hongfu_status_.StatusBalance = status_balance_;
    hongfu_status_.StatusProtect = status_protect_;
    hongfu_status_.Version = version_;
    hongfu_status_.Rsoc = rsoc_;
    hongfu_status_.StatueMos = mos_status_;
    hongfu_status_.CellNumber = cell_number_;
    hongfu_status_.NtcNumber = ntc_number_;
    for (int i = 0; i < ntc_number_; ++i) {
      hongfu_status_.NtcTem.push_back(ntf_data_[i]);
    }
    for (int i = 0; i < 13; ++i) {
      if((status_protect_ & (0x0001>>i)>>i)==1){
      hongfu_status_.ErrorId.push_back(i);
      hongfu_status_.ErrorInfo.push_back(error_info[i]);  
      }
    }
  }
  /////////////****************///////////////
  if (buffer_vol.size()!=0) {   
    cell_number_ = buffer_vol[3]/2;
    for (int i = 0; i < cell_number_*2; i+=2) {
      cell_[i/2] = (buffer_vol[4+i]<<8|buffer_vol[5+i])/1000.0;
      hongfu_status_.CellVoltage.push_back(cell_[i/2]);       
    }   
  }

  if (hongfu_status_.CellVoltage.size()!=0 && hongfu_status_.NtcTem.size()!=0)
  {
    hongfu_pub_.publish(hongfu_status_);
  }
  // else if(!bms_ser_.isOpen()) {
  //   hongfu_status_.ErrorId[13] = 13;
  //   hongfu_status_.ErrorInfo[13] = "Unable to open port";
  //   hongfu_pub_.publish(hongfu_status_);
  // }
}

std::vector<uint8_t> IQR::HongfuBmsStatus::dataRead(float date_type, float checksum_write, uint16_t buffer_sum, uint16_t checksum_read, std::vector<uint8_t> buffer,
    char* argv[]) {
  int index = 0;
  buffer_write_[2] = date_type;
  buffer_write_[5] = checksum_write;
  try{
    bms_ser_.write(buffer_write_,7); 
    ros::Duration(0.1).sleep();
    if (bms_ser_.available()) {   
      bms_ser_.read(buffer, bms_ser_.available());
      while(!findpack) {
        if (buffer[index]==0xDD) {
          buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+index);
          checksum_read = buffer[buffer.size()-3]<<8|buffer[buffer.size()-2];
          for (int i = 0; i < buffer.size()-5; ++i) {
            buffer_sum += buffer[i+2];
          }
          buffer_sum = ~buffer_sum + 1;
          if (buffer_sum==checksum_read) {
            findpack = true;
          }
        }
        index += 1;
      }
    }
  }
    catch (serial::SerialException& e) {
      bms_ser_.close();
      initPort(argv);
    }
    catch (serial::IOException& e) {
      bms_ser_.close();
      initPort(argv);
    }                 
  return buffer;
}