# iqr_hongfu_bms
### Battery Management System BMS
## use
Read battery-related information returned by BMS，You can configure device serial password, baud rate and frame rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error 
bits of information and the error content.
## Equipment type
Hongfu Power Lithium Battery
## environment
Ubuntu16.04\
ROS-kinetic\
## node info
Topic name of the publishing message is bms，the message type is Hongfustatus.msg.Three node parameters,\
serial port name:Port_bms,Default value：/dev/ttyUSB0;baudrate name:Baudrate_bms,Default value:9600;looprate name:Looprate_bms,Default value:2;frame_id:Hongfu_id,Default value:hongfu_bms;
## install
`git clone https://github.com/I-Quotient-Robotics/iqr_hongfu_bms`\
\
`git check hongfu_bms_driver`\
\
`rosdep install hongfu_bms_driver --ignore-src`
## rules
`sudo cp iqr_hongfu_bms/hongfu_bms_driver/udev/10-hongfu-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d`
## run
`cd workspace`\
\
`catkin_make`\
\
`roslaunch hongfu_bms_bringup hongfu_bms_status.launch`


