# OBU_AV_APP

This folder aims to collect any development based on OBU usage

# Requirements 

Python 3.11 

ROS2 humble

Matlab 2024b or higher

Commsignia APIs (need license)

Novatel RTK GPS (need license)

# RTK Usage 
Once you are in the docker, follow the following procedures to activate the RTK GPS:

1. Open firefox, check the bookmark on the top of the browser page, click the one named Knovatel, and check if the error is in centimeter level, if it is, proceed from step 2, if not turn off the RTK switch, named powerpak7, on center control pannel and wait for 10s to proceed from step 2.

2. Launch the RTK node using the following command in new docker terminal: ros2 launch novatel_oem7_driver oem7_net.launch.py oem7_ip_addr:=192.168.100.201 oem7_port:=3005

3. Once that line has been ran, it's expected to see that the RTK initialization running through. That means knovatel messages are expected to be subscribed, and you can start running your code associate with the RTK GPS. 

Example: 
[`RTK_code.py`](RTK_code.py) incldues details of collecting RTK GPS and synchronization with NAV messages.

After runnning [`RTK_code.py`](RTK_code.py), a data folder will be generated within the same directory you ran the code, and use [`plotter.m`](plotter.m) to verify the data that [`RTK_code.py`](RTK_code.py) recorded. 

