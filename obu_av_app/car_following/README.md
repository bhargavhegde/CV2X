# CV2X based vehicle planning and control
This repo contains source codes for car-following control.

The codes are based on Commsignia V2X Remote Python SDK. The SDK is a python package to access low level functionality of the Commsignia V2X stack. 

## Required by all applications

Python 3.10 \
ROS2 humble 

### Additional Requirements
Dataspeed DBW control (need license) \
Commsignia APIs (need license) \
Matlab 2024b or higher (for plotter) 

The Python API requires the package requires python 3.7 or newer version, we recommend using Python 3.9.
Please add necessary packages to the [requirements.txt](requirements.txt) file.

To installing the package, `pip3 install $whl_dist` (`whl_dist` is the built distribution).
The current sdk version in use is `Unplugged-RT-y20.39.4-b205116-pythonsdk.tar.xz`. 

### Instructions
To run ACC control, please following the procedures after starting-up the vehicle properly: 
1. Ping 192.168.3.54 in terminal, make sure OBU is on and able to be subscribed. 
2. Checkout this repo to Lincoln code, we usually use `Desktop/seaslab/cv2x`.
3. Go to Downloads/OBU_CV2X, and ./ docker_start_new.sh to start the docker.  
4. Open a new terminal using same image (changing every time you spawn the docker), type `ros2 launch ds_dbw_can dbw.launch.xml`, you will see DBW status from there. Note that if other packages are needed (e.g., camera, lidar etc., other command would be needed, see instruction at [ub-cavas](https://github.com/ub-cavas/ub-lincoln-docker?tab=readme-ov-file#individual-packages)).
5. Run the code [ACC_control](ACC_control.py) to start the code, please note that starting the code doesn't mean that the control is engaged.
6. If code run successfully, the real-time headway should be printing in terminal, that means the code is logging. 
7. Once there is a safe distance between Leading vehicle and ego vehicle, and read to engage; press both ok bottoms on the steering wheel. You are expected to hear beeping. 
8. Optional: you can also disengage during the test and reengage back, the logging won't stop. If disengaged is triggered by the driver, you should also hear a beeping sound. 
9. Once you are done with the test, you can ctrl+c the code to shut down the algorithm. 
10. After running [ACC_control](ACC_control.py), you should expect to see a data folder within car_following folder, you can then use [speed_log](speed_log.m) to check the test result. A python version of this plotter will be developed for convenience. 

### TODOs
1. Stopping performance, the vehicle will creep and jittering between move and stop to try to achieve the desired distance. Such performance may not be desirable, and is not safe.
2. Car following parameters (alpha, beta, kappa) are subject to fine tunning. Also logic of approaching vehicle from far away is subject to optimizations.
