# OBU_AV_APP

This folder aims to collect any development based on OBU usage

# Requirements 

Python 3.11 

ROS2 humble

Matlab 2024b or higher

Commsignia APIs (need license)

Novatel RTK GPS (need license)

# Usage 
[`RTK_code.py`](RTK_code.py) incldues details of collecting RTK GPS and synchronization with NAV messages.

After runnning [`RTK_code.py`](RTK_code.py), a data folder will be generated within the same directory you ran the code, and use [`plotter.m`](plotter.m) to verify the data that [`RTK_code.py`](RTK_code.py) recorded. 

