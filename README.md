# iqr_hongfu_bms
### Battery Management System BMS
## use
Read battery-related information returned by BMS，You can configure device serial password, baud rate and frame rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error 
bits of information and the error content.
## Equipment type
Hongfu Power Lithium Battery\
[name](https://github.com/I-Quotient-Robotics/iqr_hongfu_bms/blob/master/type_pic/144283718.jpg)

## environment
Ubuntu16.04\
ROS-kinetic
## node info
* Topic name: bms
* message type: Hongfustatus.msg
* Three node parameters(param,name,default)
  * serial port,Port_bms,/dev/ttyUSB0
  * baudrate,Baudrate_bms,9600
  * looprate,Looprate_bms,2
  * frame_id,Hongfu_id,hongfu_bms
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
`source /devel/setup.bash`\
\
`roslaunch hongfu_bms_bringup hongfu_bms_status.launch`


