
#  OBU_API.py — On-Board Unit V2X Communication Script

This script implements the **On-Board Unit (OBU)** logic for a V2X (Vehicle-to-Everything) communication system using Commsignia’s SDK. It handles the reception of image chunks from a Roadside Unit (RSU), sends confirmation upon successful reception, and returns feedback (segmented image + text) to the RSU.

---


## How to Use This Script

### 1. Install Required SDK

Before running the script, you need to install the **Commsignia Python SDK**:

```bash
pip install pycmssdk
```

> *Ensure that the SDK is installed in the same environment where you're running the scripts.*

---

### 2. Setup Configuration

- Make sure a valid `parameters.json` file exists in the same directory with correct PSIDs and V2X stack IP.

- For **OBU**, ensure feedback files are available at:
  ```
  latest/masked_image_latest.png
  latest/text_file.txt
  ```

---

### 3. Running the Script

```bash
python OBU_API.py
```

- Broadcasts readiness status
- Receives image chunks from RSU
- Sends confirmation and feedback
- Logs all activity in `obu_logs/`

---

### 4. Integration Guidelines

- OBU and RSU must be on the same V2X network (configured in `parameters.json`).
- Ensure **Commsignia devices** are powered and linked properly.
- Log and received images are stored automatically.

---
## Purpose of the Script

The `OBU_API.py` script performs the following:

1. Continuously broadcasts a "ready" signal to notify RSU it is available.
2. Listens for image data sent in chunks from RSU.
3. Reassembles the full image once all chunks are received.
4. Sends a confirmation back to RSU.
5. Sends feedback image (segmented output) and text (detection result).
6. Logs all major actions, durations, hashes, and transmission status.

---

## Breakdown of the Script

### 1. **Configuration & Imports**
- Loads settings from `parameters.json`.
- Uses `pycmssdk` for V2X message handling.
- Paths to feedback image and text files are hardcoded and read from disk.

### 2. **Directory Management**
- Auto-creates:
  - `obu_logs/`  for CSV logs
  - `obu_received_images/` for saving received images from RSU

### 3. **State Variables**
- Tracks image chunks, total expected, hash, crack info, etc.
- Uses `ready_flag` to control when the OBU is broadcasting availability.

---

## Image Reception Logic

### `handle_chunk(...)`
- Processes each incoming image chunk.
- Stores it with order index.
- Once all chunks are received:
  - Reconstructs the image.
  - Verifies hash match.
  - Saves the image.
  - Sends confirmation and feedback to RSU.
  - Logs the whole cycle info.

---

##  Feedback & Confirmation

### `send_confirmation(...)`
- Sends a small message saying image was successfully received.

### `send_feedback(...)`
- Reads two feedback files:
  - A PNG segmentation result
  - A TXT file with crack details
- Encodes and splits them into chunks.
- Sends each chunk to RSU with metadata.
- Logs how many chunks were sent and when.

---

## Ready Signal

###  `send_ready_loop()`
- Continuously sends a broadcast message like:
  ```json
  { "obu_id": "OBU_XYZ", "status": "ready" }
  ```
- Paused during image processing to avoid conflict.

---

## Logging Format

Each test cycle (`Test1`, `Test2`, ...) generates:

- `TestX_chunks_received.csv`: All received chunks and metadata
- `TestX_feedback_sent.csv`: Count of feedback chunks sent and timestamp
- `TestX_cycle_summary.csv`: Total duration, hash match status, crack info

---

##  Important Notes

- The feedback files (`masked_image_latest.png` and `text_file.txt`) must exist in the `latest/` directory before each cycle starts.
- The script pauses broadcasting "ready" while it is processing or sending data.
- The system is fully event-driven and continuous.

---

## Requirements

- Python 3.x
- `pycmssdk` from Commsignia
- Commsignia hardware and network setup
- A valid `parameters.json` with IPs and PSIDs

---

## Example Workflow

1. OBU sends “ready” status
2. RSU sends image chunks
3. OBU reconstructs image
4. OBU sends confirmation
5. OBU sends back feedback image and text
6. Logging is done automatically
7. Cycle continues to the next round

