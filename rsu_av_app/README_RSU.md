# RSU Crack Detection — User Guide

This document explains how to set up and run the **RSU script (`rsu_api.py`)** on a computer connected to a Commsignia RSU. The script supports sending crack-detection requests to the OBU, including the **Area of Interest (AOI)** defined by simple **GPS latitude and longitude coordinates**.

---

## 1. Prerequisites

- **Python 3** installed.  
- **Commsignia Python SDK** installed:  
  ```bash
  pip install pycmssdk
  ```
- **RSU hardware** powered on and connected to the same network as your computer.  
- This folder should contain:
  ```
  ├─ rsu_api.py
  ├─ rsu_parameters.json
  └─ README.md   (this file)
  ```

---

## 2. Configuration (`rsu_parameters.json`)

All runtime settings are defined in the JSON file. Example:

```json
{
  "V2X_STACK_IP": "192.168.0.54",
  "send_image_psid": 200,
  "receive_confirm_psid": 300,
  "receive_feedback_psid": 400,
  "receive_ready_psid": 201,

  "chunk_size": 1024,
  "cycle_timeout": 2,
  "ready_timeout": 1,

  "image_path": "frame_1750704066_18.png",

  "crack_info": "Test-1",
  "gps_lat": 43.002831897548944,
  "gps_lon": -78.78671042296428,

  "logging_level": 0,
  "send_image": true
}
```

### Key Fields

- **V2X_STACK_IP** — RSU IP address (computer must reach this).  
- **PSIDs** — protocol identifiers for sending/receiving messages.  
- **chunk_size** — how many bytes per chunk when sending an image.  
- **cycle_timeout / ready_timeout** — timeouts for communication.  
- **image_path** — full path of PNG image to send.  
- **send_image** —  
  - `true`: send image chunks + metadata.  
  - `false`: send metadata only.  
- **crack_info** — optional label/test ID.  
- **gps_lat / gps_lon** — GPS coordinates of AOI (this is what defines the detection region).  


A few examples GPS coordinates of AOIs
```
Point near the garage
"gps_lat": 43.002831897548944,
"gps_lon": -78.78671042296428,


Service center road crack
Updtaed based on 4th sept 2025 testing
"gps_lat": 42.992588111025974
"gps_lon": -78.79378366607423



Pilot site road crack
Eastbound start
"gps_lat": 42.97801200961085
"gps_lon": -78.76423024453611
Eastbound end
"gps_lat": 42.97799925436498
"gps_lon": -78.76392044940569

Westbound start
"gps_lat": 42.97813759958253
"gps_lon": -78.76395867088281
Westbound end
"gps_lat": 42.97813759958253
"gps_lon": -78.76426779546101
```
---

## 3. AOI Definition (GPS-Based)

The **only AOI information required** is latitude and longitude.  

- Enter your AOI GPS location in `gps_lat` and `gps_lon`.  
- These values are sent with every RSU transmission.  
- The OBU will receive them and pass them into the **crack detection pipeline**, which will use the coordinates to target the AOI.  

> No polygon/rectangle drawing or manual AOI definitions are needed.  

---

## 4. Running the RSU

1. Power on the RSU and connect your computer to the RSU network.  
2. Edit `rsu_parameters.json` with the correct:  
   - IP address (`V2X_STACK_IP`)  
   - AOI coordinates (`gps_lat`, `gps_lon`)  
   - Image path (`image_path`)  
3. Open a terminal in the folder and run:
   ```bash
   python rsu_api.py
   ```
4. The script will:  
   - Wait for the OBU **ready signal**.  
   - Send image chunks (if `send_image=true`) and metadata (lat, lon, crack_info).  
   - Receive feedback PNG/TXT from the OBU.  
   - Save feedback files and logs locally.  

---

## 5. Outputs

- **Logs** → stored under `rsu_logs_v4/`  
  - `rsu_sent_#.txt` — sent data per cycle.  
  - `rsu_received_#.txt` — received feedback details.
  - `rsu_params_#.txt` — log parameters details used for sending.  

- **Feedback** → stored under `rsu_received_feedback_v4/`  
  - Reconstructed PNG files from OBU.  
  - TXT feedback metadata files.  

---

## 6. Quick Example

1. Edit `rsu_parameters.json`:  
   ```json
   "crack_info": "BridgeTest",
   "gps_lat": 43.0028319,
   "gps_lon": -78.7867104,
   "send_image": true
   ```
2. Run:
   ```bash
   python rsu_api.py
   ```
3. Console output will show:  
   - RSU waiting for OBU ready.  
   - RSU sending chunks with lat/lon info.  
   - RSU receiving feedback image and text.  
4. Check `rsu_received_feedback_v1/` for results.  

---

## 7. Troubleshooting

- **RSU stuck on “waiting for ready”**  
  - Ensure OBU is powered and broadcasting ready signal.  
  - Check `receive_ready_psid` matches OBU config.  

- **No feedback received**  
  - Verify `receive_feedback_psid`.  
  - Increase `cycle_timeout` if OBU is slow.  

- **File not found errors**  
  - Make sure `image_path` points to a valid PNG file.  

---

## 8. Summary

- RSU sends **GPS AOI coordinates (lat, lon)** + optional crack info.  
- OBU uses these coordinates for crack detection pipeline.  
- Results are logged and saved automatically.  
