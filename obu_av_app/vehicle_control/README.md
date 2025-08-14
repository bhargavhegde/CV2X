# CV2X-based Vehicle Planning and Control

This folder contains source code for different vehicle longitudinal controllers.

The code is based on the Commsignia V2X Remote Python SDK. The SDK is a Python package that provides access to low-level functionality of the Commsignia V2X stack.

## Required by All Applications

- Python 3.10  
- ROS2 Humble

### Additional Requirements

- Dataspeed DBW control (requires license)  
- Commsignia APIs (requires license)  
- Matlab 2024b or higher (for plotter)

The Python API requires Python 3.7 or newer; we recommend using Python 3.10 to match the required version above.    
Please add necessary packages to the [requirements.txt](requirements.txt) file.

To install the package, run: `pip3 install $whl_dist` (`whl_dist` is the built distribution).  
The current SDK version in use is `Unplugged-RT-y20.39.4-b205116-pythonsdk.tar.xz`.

### Instructions

To run any vehicle control, you need to start the vehicle properly by following the steps below:

1. Ping `192.168.3.54` in the terminal to make sure the OBU is on and can be subscribed to.
2. Check out this repo to the Lincoln code; we usually use `Desktop/seaslab/cv2x`.
3. Go to `Downloads/OBU_CV2X`, and run `./docker_start_new.sh` to start the Docker container.
4. Open a new terminal using the same image (it changes every time you spawn the Docker container), and type `ros2 launch ds_dbw_can dbw.launch.xml`. You will see the DBW status from there. Note that if other packages are needed (e.g., camera, lidar, etc.), other commands may be needed. See the instructions at [ub-cavas](https://github.com/ub-cavas/ub-lincoln-docker?tab=readme-ov-file#individual-packages).

Once the vehicle has been started properly, you can further control the vehicle's longitudinal speed using different controllers.  

To run speed tracking control, please follow the procedures below:
1. Run the code [run_control_example](run_control_example.py) to start. Please note that starting the code does not mean that the control is engaged.
2. If the code is running successfully, the real-time target speed and vehicle speed should be printed in the terminal; moreover, the speed error and corresponding integral error should also be updating in the terminal to monitor the working status of the PI speed control.
3. Please note that the speed tracking does not consider any traffic in front; thus, you can engage/disengage whenever you feel comfortable.
4. Once you are done with the test, you can press `Ctrl+C` to shut down the algorithm.
5. After running [run_control_example](run_control_example.py), you should see a `data` folder within the `vehicle_control` folder. You can then use [speed_log](speed_log.m) to check the test result. A Python version of this plotter will be developed for convenience.

To run ACC control, please follow the procedures below:

1. Run the code [ACC_control](ACC_control.py) to start. Please note that starting the code does not mean that the control is engaged.
2. If the code runs successfully, the real-time headway should be printed in the terminal, indicating that the code is logging.
3. Once there is a safe distance between the leading vehicle and the ego vehicle, and you are ready to engage, press both OK buttons on the steering wheel. You should hear a beep.
4. Optional: You can also disengage during the test and re-engage later; the logging won't stop. If disengagement is triggered by the driver, you should also hear a beep.
5. Once you are done with the test, you can press `Ctrl+C` to shut down the algorithm.
6. After running [ACC_control](ACC_control.py), you should see a `data` folder within the `car_following` folder. You can then use [speed_log](speed_log.m) to check the test result. A Python version of this plotter will be developed for convenience.

### TODOs

For speed tracking:
1. Coordinate with [cv2x_crack_detection](../cv2x_crack_detection.py) to subscribe to CV2X callbacks more efficiently.

For ACC:
1. Stopping performance: The vehicle will creep and jitter between moving and stopping to try to achieve the desired distance. Such performance may not be desirable and is not safe.
2. Car-following parameters (alpha, beta, kappa) are subject to fine-tuning. Also, the logic for approaching a vehicle from far away is subject to optimization.

For plotter:
1. Plotter needs to be more optimized, such that it can visualize the results for all kinds of controller. A python version of plotter needs to be developed soon. 
