#!/bin/bash
# This bash script is used to launch a Docker container for the CV2X application on the Lincoln MKZ platform.
# For explanation of the tag, check official docker documentation:
# april27_testing:latest is the name of the docker image. 
# It will be updated soon.
# The docker container will have a fixed name cv2x.
# The --rm flag will remove the container after it is stopped.
# The --privileged flag is used to give the container access to the host's devices.

docker run -it --network host --runtime=nvidia --gpus all --privileged \
  -v /home/lincolncavas23/Desktop/seaslab:/mnt \
  --name=cv2x --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  april27_testing:latest
