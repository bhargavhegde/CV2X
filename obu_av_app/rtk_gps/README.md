# Process of Using Swift RTK GPS to Localize the Crack

This document explains the procedures for using Swift RTK GPS.

## Requirements

- Swift Console v4.019 (current firmware)
- Internet sharing (UB campus Wi-Fi does not allow this, so hotspot is used)

## Swift RTK Usage
Remark: The following steps are for using Swift RTK GPS on the SEAS Lab Macbook. If using a different operating system, the steps may vary. The Linux Internet sharing setup is more complex. 

1. Connect Macbook or running machine to the internet. If in test, hotspot will be used.

2. Open terminal, and check if internet connection is stable. Type `ping google.com` to check the response.

3. Connect Swift device's Ethernet cable to Macbook. Meanwhile, connect the power cable to the mobile power source.

   ![Swift RTK Device Setup](Images/Swift%20RTK%20device%20Small.png)

4. Once connected, use `ifconfig` to confirm that the bridge is shown in the terminal.

5. Open another terminal, ping `192.168.2._`, most likely the last digit is 2, but needs to try from 2 to 6 until seeing a connection.

6. Once connection is confirmed, open the Swift Console application.

7. Type your Swift device host IP, choose TCP/IP, and click connect.

   ![Swift Console Interface](Images/Swift%20Console.png)

8. Click **Solution** on the left column and observe the GPS location signal. If it's working, you should expect to see green dots steady, and the corresponding range of all parameters should be reasonably close to the ones in the screenshot attached.

   ![Swift GPS Signal](Images/Swift%20GPS%20fixed.png)

9. If step 8 is not working, repeat steps 1-8 until it works. If nothing works, consider restarting the machine and doing steps 1-8 again.


## Python APIs for Logging / Checking Data

Please consider using the following [tool](https://github.com/swift-nav/libsbp) to convert collected SBP-JSON files. It can be installed using pip:

```bash
pip3 install sbp
```
The tool can then be invoked using the following command:

```bash
python3 -m sbp2json <sbp.bin
```

Note that for the gps location, please use all 9 digits after the decimal points to get the centimeter-level accuracy. Also it is always a good idea to average the GPS location over a period of time to get a more stable result.

With the console one can log `sbp.json` files. [static_point_sample.sbp.json](static_point_sample.sbp.json) is an example of such a file. It corresponds to a fixed location. [static_point_sample_console_view.png](static_point_sample_console_view.png) is a screenshot of the console view during the test.

[Read_RTK_gps](Read_RTK_gps.py) is a script to read the logged data from Swift RTK GPS. It can be run using the following command:


```bash
python3 Read_RTK_gps.py
```
The script will generate a `csv` file and a `json` file with the GPS data. The `csv` file can be used for further analysis or visualization. 


[simple.py](simple.py) is a simple script to log the GPS data from Swift RTK GPS. It can be run using the following command:

```bash
python3 simple.py
```
This should parse the data in real time. It requires setting up the interface differently which is not explored yet.
