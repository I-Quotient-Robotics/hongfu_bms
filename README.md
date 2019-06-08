# iqr_hongfu_bms
### Battery Management System BMS
## use
Read battery-related information returned by BMS，You can configure serial port name, baudrate and loop rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error bits of information and the error content.
## equipment type
Hongfu Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/iqr_hongfu_bms/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/iqr_hongfu_bms/blob/master/type_pic/60348685.jpg)

## environment
Ubuntu16.04\
ROS-kinetic
## node parameter
|param|name|defaule value|
|--|--|--|
|topic name|bms|
|message type|HongfuStatus|
|serial port|port_bms|/dev/ttyUSB0|
|baudrate|baudrate_bms|9600|
|looprate|looprate_bms|2|
|frame_id|hongfu_id|hongfu_bms|
## install
`git clone https://github.com/I-Quotient-Robotics/iqr_hongfu_bms`\
\
`cd iqr_hongfu_bms`\
\
`rosdep install hongfu_bms_driver --ignore-src`\
\
`rosdep install hongfu_bms_msg`
## rules
`sudo cp hongfu_bms_driver/udev/10-hongfu-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d`\
\
`sudo udevadm control--reload-rules && udevadm trigger`
## run
`cd workspace`\
\
`catkin_make`\
\
`source /devel/setup.bash`\
\
`roslaunch hongfu_bms_bringup hongfu_bms_status.launch`


