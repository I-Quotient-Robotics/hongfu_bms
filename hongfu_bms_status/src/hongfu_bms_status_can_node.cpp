#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <hongfu_bms_msg/HongfuStatus.h>

#include <boost/format.hpp>

#define BMS_ID 0xB

enum RegisterIndex {
  kTemperature = 0x08,
  kVoltage = 0x09,
  kCurrent = 0x0A,
  kRSOC = 0x0D,
  kDesignCapacity = 0x18,
  kFullCapacity = 0x10,
  kResidualCapacity = 0x0F,
  kChargeCycle = 0x17,
  kProductionData = 0x1B,
  kVoltage7 = 0x39,
  kVoltage6 = 0x3A,
  kVoltage5 = 0x3B,
  kVoltage4 = 0x3C,
  kVoltage3 = 0x3D,
  kVoltage2 = 0x3E,
  kVoltage1 = 0x3F,
};

struct BMSStruct {
  float rsoc;
  float current;
  float temperature;
  float voltage;
  float cell_voltage[7];
  float full_capacity;
  float design_capacity;
  float residual_capacity;
  int32_t charge_cycle;
} bms_states;

void BMSDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
  boost::format key_format;

  status.add("RSOC (%)", bms_states.rsoc);
  status.add("Current (A)", bms_states.current);
  status.add("Temperature %f% (℃)", bms_states.temperature);
  status.add("Voltage (V)", bms_states.voltage);
  status.add("ResidualCapacity (mAh)", bms_states.residual_capacity);
  status.add("FullCapacity (mAh)", bms_states.full_capacity);
  status.add("DesignCapacity (mAh)", bms_states.design_capacity);
  status.add("ChargeCycle", bms_states.charge_cycle);

  for(int i=0; i<7; i++) {
    key_format = boost::format("Cell %1% voltage (V)") % i;
    status.add(key_format.str(), bms_states.cell_voltage[i]);
  }

  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

void CanReceiveCallback(const can_msgs::Frame::ConstPtr& msg) {
  uint16_t msg_id = (msg->id & 0xFFFF) >> 8;

  switch(msg_id) {
    case kTemperature: {
      bms_states.temperature = ((msg->data[1]<<8|msg->data[0])-2731)/10.0;
      // ROS_INFO("Temperature %f", bms_states.temperature);
      break;
    }
    case kVoltage: {
      bms_states.voltage = (msg->data[3]<<24|msg->data[2]<<16|msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage %f", bms_states.voltage);
      break;
    }
    case kCurrent: {
      bms_states.current = (msg->data[3]<<24|msg->data[2]<<16|msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Current %f", bms_states.current);
      break;
    }
    case kRSOC: {
      bms_states.rsoc = (msg->data[1]<<8|msg->data[0]);
      // ROS_INFO("RSOC %f", bms_states.rsoc);
      break;
    }
    case kResidualCapacity: {
      bms_states.residual_capacity = (msg->data[3]<<24|msg->data[2]<<16|msg->data[1]<<8|msg->data[0]);
      // ROS_INFO("ResidualCapacity %f", bms_states.residual_capacity);
      break;
    }
    case kDesignCapacity: {
      bms_states.design_capacity = (msg->data[3]<<24|msg->data[2]<<16|msg->data[1]<<8|msg->data[0]);
      // ROS_INFO("DesignCapacity %f", bms_states.design_capacity);
      break;
    }
    case kFullCapacity: {
      bms_states.full_capacity = (msg->data[3]<<24|msg->data[2]<<16|msg->data[1]<<8|msg->data[0]);
      // ROS_INFO("FullCapacity %f", bms_states.full_capacity);
      break;
    }
    case kChargeCycle: {
      bms_states.charge_cycle = (msg->data[1]<<8|msg->data[0]);
      // ROS_INFO("ChargeCycle %d", bms_states.charge_cycle);
      break;
    }
    case kVoltage7: {
      bms_states.cell_voltage[6] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage7 %f", bms_states.cell_voltage[6]);
      break;
    }
    case kVoltage6: {
      bms_states.cell_voltage[5] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage6 %f", bms_states.cell_voltage[5]);
      break;
    }
    case kVoltage5: {
      bms_states.cell_voltage[4] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage5 %f", bms_states.cell_voltage[4]);
      break;
    }
    case kVoltage4: {
      bms_states.cell_voltage[3] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage4 %f", bms_states.cell_voltage[3]);
      break;
    }
    case kVoltage3: {
      bms_states.cell_voltage[2] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage3 %f", bms_states.cell_voltage[2]);
      break;
    }
    case kVoltage2: {
      bms_states.cell_voltage[1] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage2 %f", bms_states.cell_voltage[1]);
      break;
    }
    case kVoltage1: {
      bms_states.cell_voltage[0] = (msg->data[1]<<8|msg->data[0])/1000.0;
      // ROS_INFO("Voltage1 %f", bms_states.cell_voltage[0]);
      break;
    }
  }
  // ROS_INFO("Receive can msg, id 0x%02X, length %d, data 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
  //           msg_id, msg->dlc, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hongfu_bms_status_can_node");

  int can_id;
  std::string send_topic, receive_topic;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param<int>("can_id", can_id, 0x09);
  private_nh.param<std::string>("can_send_topic", send_topic, "sent_messages");
  private_nh.param<std::string>("can_receive_topic", receive_topic, "received_messages");

  ros::Publisher can_pub = nh.advertise<can_msgs::Frame>(send_topic, 1000);
  ros::Subscriber can_sub = nh.subscribe<can_msgs::Frame>(receive_topic, 100, CanReceiveCallback);

  diagnostic_updater::Updater diagnostic_updater;
  diagnostic_updater.setHardwareID("hongfu_bms");
  diagnostic_updater.add("BMS", BMSDiagnostic);

  can_msgs::Frame frame;
  frame.is_rtr = 0;
  frame.is_extended = 1;
  frame.is_error = 0;
  frame.dlc = 0;

  ROS_INFO("Start...");
  ros::Rate loop_rate(1);
  while(ros::ok()) {
    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kTemperature<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kCurrent<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kRSOC<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kFullCapacity<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kResidualCapacity<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kDesignCapacity<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kChargeCycle<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage7<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage6<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage5<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage4<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage3<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage2<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    frame.header.stamp = ros::Time::now();
    frame.id = (BMS_ID<<17) | (0x01<<16) | (kVoltage1<<8);
    can_pub.publish(frame);
    ros::WallDuration(0.1).sleep();

    diagnostic_updater.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
