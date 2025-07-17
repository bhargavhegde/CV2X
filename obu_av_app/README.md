# OBU_AV_APP

This folder contains CV2X-based applications that can run on the Lincoln MKZ platform

## Required by all applications

Python 3.11 \
ROS2 humble \
Commsignia APIs (need license)

## RTK GPS

### Additional Requirements
Novatel RTK GPS (need license) \
Matlab 2024b or higher (for plotter)


### Instructions
TODO by @haosongx
 - add some screenshots for the instructions.
 - improve the api so that RTK GPS may be acquired by other applications through calling a module/function.

#### Steps to run test and visualized RTK GPS on Lincoln MKZ platform:

1. Launch a container for [ub-cavas ros2](https://github.com/ub-cavas/ub-lincoln-docker/blob/main/docker/ros2.dockerfile) docker, use the following steps to activate the RTK GPS:

2. Open firefox, check the bookmark on the top of the browser page, click the one named `Knovatel`, and check if the error is in centimeter level, if it is, proceed from step 3; if not turn off the RTK switch, named `powerpak7`, on center control panel and wait for 10s to proceed from step 3.

3. Launch the RTK node using the following command in new docker terminal: 

```
ros2 launch novatel_oem7_driver oem7_net.launch.py oem7_ip_addr:=192.168.100.201 oem7_port:=3005
```

4. Once that line has been running, it's expected to see that the RTK initialization running through. 
That means Novatel messages are expected to be subscribed, and you can start running your code associate with the RTK GPS. 

#### Instructions on how to incorporate RTK GPS data into your application:
TODO by @haosongx

#### Related files: 
[`RTK_code.py`](RTK_code.py) includes details of collecting RTK GPS and synchronization with NAV messages. After running [`RTK_code.py`](RTK_code.py), a data folder will be generated within the same directory of [`RTK_code.py`](RTK_code.py). \
[`plotter.m`](plotter.m) is used to verify the data that [`RTK_code.py`](RTK_code.py) recorded. 

## CV2X based car-following

TODO by @haosongx

## CV2X based crack-detection

TODO by @harshbhargava123
