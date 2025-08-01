#!/bin/bash
# This script is used to run the CV2X Docker container on the Lincoln AV platform.

# define your container name, we use cv2x usually
container_name=cv2x
# define the path to your workspace
# this folder will appear as /ws in the container
workspace_path=
# define the image name, we use cv2x:v1 usually
image_name=

docker run --gpus all -it --rm \
  -v $workspace_path:/ws \
  --name=$container_name \
  $image_name
