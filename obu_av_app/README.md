# Applications for Connected Autonomous Vehicle with OBU

This folder contains CV2X-based applications that can run on the Lincoln MKZ platform

## Required by all applications

Python 3.10 \
ROS2 humble \
Commsignia APIs (need license)

Using Docker by following the instruction [here](https://github.com/CHELabUB/crack_detection/tree/master/Docker)is highly recommended. The following instruction assumes usage of the docker. 

## RTK GPS

See [RTK GPS](RTK_GPS/README.md) for details on how to set up the RTK GPS.

## CV2X based car-following

See [CV2X based car-following](CV2X_Car_Following/README.md) for details on how to set up the CV2X-based car-following application.

## CV2X based crack-detection

Step to run the crack-detection application on Lincoln MKZ platform:
1. check this repo as it is to the Lincoln MKZ platform
2. run `docker_run_lincoln.sh` to start the docker container. 
   ```bash
   ./docker_run_lincoln.sh
   ```
   This will start a Docker container with the necessary environment for running the crack-detection application. The launched container will have the name `cv2x`.

3. Open another terminal, type `docker ps` you should see the docker container running. 
Then run the bash command to enter the container:
   ```bash
   docker exec -it cv2x /bin/bash
   ```
   [docker_bash.sh](docker_bash.sh) is a script that can be used to enter the Docker container.
   You can also use the command above directly in the terminal.
   Both of these steps will give you a shell terminal inside the Docker container.

4. Direct to the obu_av_app directory, and use the command to launch the pipeline:

   ```bash
      cd /ws/cv2x/obu_av_app
   ```

   ```bash
    # DBW:
    ros2 launch launch/all_launch.py
   ```
    This launch command will launch the necessary drivers from [ub-cavas](https://github.com/ub-cavas/ub-lincoln-docker?tab=readme-ov-file#in-container-commands), along with the detection pipeline from a single ROS2 launch. 


## Troubleshooting Tips

Remark 1: To fulfill the crack detection resolution requirement, the current pipeline will only start collecting image when the crack is within the range threshold.


## Offline Tuning
To be written by @HaosongXXXD. 
For example 
- how [heatmap.png](heatmap.png) is generated and used for tuning.
- what's the usage for [offline_tuning.py](offline_tunning.py)
