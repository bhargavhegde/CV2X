## Recorder
This directory contains the code for the Recorder module, which is basic data recorder that can be used with any Commsignia devices.

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
