## Recorder
This directory contains the code for the Recorder module, which is basic data recorder that can be used with any Commsignia devices.

### Setup to use Commsignia Devices
Please refer to [Commsignia](https://buffalo.box.com/s/uoe7r7khazz3pa8peidn9etjti41ds6p) folder for details 
- C-V2X protocol and standards
- Commsignia C-V2X devices documentation 
- SDKs to setup Commsignia APIs. 

Python interface are used primarily. 
Both C and python interfaces shares the same functionalities to V2X stack. C interface has extension to Commsignia safety services at the application level.

It is recommended to build a virtual environment for Python API usages. E.g., on Mac OS (M1 chip also works).
```
    python3 -m venv path/to/venv
    source path/to/venv/bin/activate
    python3 -m pip install xyz
    python3 -m pip install -r requirements.txt
```

The Python API requires Python 3.7 or newer version. 
We recommend using Python 3.9 or 3.10.
3.9 is used for CCR and 3.10 is used now in docker container on the Lincoln MKZ platform.

Please add necessary packages to the [requirements.txt](requirements.txt) file.

To install the package, 
```
pip3 install $whl_dist
```
`whl_dist` is the built distribution. The current sdk version in use is `Unplugged-RT-y20.39.4-b205116-pythonsdk.tar.xz`. [link to whl file](https://buffalo.box.com/s/5izm06ax343tg5rsto15pw6ao2u4zse4)

### Communication setup instructions

TODO: add instructions on how to setup the communication between Commsignia devices and the Recorder module.

For now one can consult [README OBU](../RSU_OBU_API/README_OBU.md) for the OBU side and [README RSU](../RSU_OBU_API/README_RSU.md) for the RSU side.

### File Description

[synced_recorder.py](synced_recorder.py) Main script to run the Recorder with the Commsignia devices. \
[plotter.py](plotter.py) Script for plotting the recorded data, useful for visualizing the results right after data recording. \
[offline_synchronization.py](offline_synchronization.py) Script for offline synchronization of recorded data, developing the synchronization algorithm. \
[test_synchronization.py](test_synchronization.py) Unit tests for the synchronization algorithm. \
[utility.py](utility.py) Utility functions used in the Recorder module. \
[device_dictionary.json](device_dictionary.json) JSON file containing the device ID dictionary for all the Commsignia devices at SEAS Lab.

### Usage

Steps to use the Recorder:

1. Following the instructions in [README.md](../README.md), set up the environment. Only need to do this once on a new machine.

2. In the environment, run the following command to start the Recorder:
   ```bash
   python synced_recorder.py
   ```
   A set of .txt files will appear in the same directory of synced_recorder.py, corresponding to the data recorded from the Commsignia devices. The files will be start with the prefix corresponding to the YYYYMMDDHHMMSS timestamp of the recording start time. Possible files include:
    - `nav.txt`: Contains GPS data.
    - `fac.txt`: Contains received BSM from other vehicles.
    - `sync.txt`: Contains synced entry of V2X messages at each time stamp.
    - `ctrl.txt`: Contains debugging information.
