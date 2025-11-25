# ZED 2i Ground Truth Measurement Tools

This repository contains two Python applications and a Docker-based development environment for obtaining **ground-truth geometric measurements** from real-world structures using the **ZED 2i stereo camera**.

The tools allow you to:

* Measure the **length** between any two points in the scene using 3D point cloud data.
* Inspect **depth** at any pixel in real time.
* Run everything inside a fully configured **ZED SDK + CUDA Docker environment**.

---

## Repository Structure

```
.
├── docker_command.txt         # Text file having the docker run command
├── zed_length.py              # Click-based 3D distance measurement tool
├── zed_depth.py               # Depth-at-cursor visualization tool
└── README.md
```

---

# 1. Overview

These scripts provide ground truth measurements useful for:

* Crack detection validation
* Structural/Concrete condition assessment
* Depth-based computer vision research
* Robotics and 3D reconstruction tasks

Both scripts rely on `pyzed.sl`, OpenCV, and the ZED SDK running inside a Docker container.

---

# 2. Requirements

### Hardware

* ZED 2i camera
* NVIDIA GPU with CUDA support

### Software

* Docker
* NVIDIA Container Toolkit
* Linux host with X11 support (Ubuntu/WSL2 tested)

---

# 3. Running the Docker Environment

### Enable X11 display forwarding

```bash
xhost +local:root
```

### Run the ZED Docker container

```bash
docker run -it --rm \
    --gpus all \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/bus/usb \
    --device /dev/video0 \
    --device /dev/video1 \
    --device /dev/video2 \
    --device /dev/video3 \
    -v /dev:/dev \
    -v /path/to/your/workspace:/workspace \
    stereolabs/zed:4.2-gl-devel-cuda11.4-ubuntu20.04
```

### Install required packages inside the container

```bash
apt-get update && apt-get install -y usbutils libglib2.0-0 libgtk2.0-dev
```

You can now run the scripts from `/workspace`.

---

# 4. Usage

## 4.1 Distance Measurement Tool — `zed_length.py`

This tool lets you click **two points** in the camera feed to compute their **true 3D Euclidean distance (in cm)**.

### Run:

```bash
python3 zed_length.py
```

### Features

* Live camera feed
* Click to select points; last two selections are used
* (x, y) → (X, Y, Z) mapping printed to console
* Final distance displayed on frame

### Example Output

```
Point selected: Pixel (421, 210) → 3D [12.3, -4.5, 87.1]
Distance: 15.62 cm
```

---

## 4.2 Depth Mapping Tool — `zed_depth.py`

Hover over any pixel to display its **depth value (in cm)**.

### Run:

```bash
python3 zed_depth.py
```

### Features

* Displays RGB feed
* Shows depth near the cursor
* Green = valid depth, red = invalid
* Automatic bounding to keep the label on-screen

---

# 5. Troubleshooting

### ZED camera not detected

Check USB and video devices:

```bash
lsusb
ls /dev/video*
```

Ensure devices are included using `--device` flags in Docker.

### No depth returned

This can occur due to:

* Overexposed or low-light surfaces
* Textureless objects
* Objects too close (< minimum depth range)

Consider adjusting lighting or depth mode.

### X11 errors in WSL

Set display:

```bash
export DISPLAY=$(hostname).local:0
```

Run an X server such as VcXsrv or Xming.

---

# 6. Citation

If using these tools for publications:

**"Ground truth measurement tools developed using the ZED 2i stereo camera (2025)."**

---

# 7. License

MIT License (modify if needed).

---

# 8. Contact

For issues, improvements, or collaboration, open a GitHub issue or contact the project maintainer.