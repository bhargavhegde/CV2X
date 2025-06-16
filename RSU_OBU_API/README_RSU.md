
# RSU_API.py — Roadside Unit V2X Communication Script

This script implements the **Roadside Unit (RSU)** logic for a V2X (Vehicle-to-Everything) communication system using Commsignia’s SDK. It is designed for real-time image transmission, feedback reception, and performance logging for infrastructure crack detection using connected vehicles.

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

- Make sure a valid `parameters.json` file exists in the same directory:
  ```json
  {
    "V2X_STACK_IP": "192.168.1.54",
    "RECEIVE_IMAGE_PSID": 200,
    "SEND_CONFIRM_PSID": 300,
    "SEND_FEEDBACK_PSID": 400,
    "SEND_READY_PSID": 201,
    "CHUNK_SIZE": 1024,
    "CYCLE_TIMEOUT": 1.5,
    "READY_TIMEOUT": 1
  }
  ```

- For **RSU**, place the input image at:
  ```
  latest/crack_img_1.png
  ```

---

### 3. Running the Script

```bash
python RSU_API.py
```

- Waits for OBU readiness
- Sends image in chunks to OBU
- Receives confirmation and feedback
- Logs activity in `rsu_logs/`

---

### 4. Integration Guidelines

- RSU and OBU must use the same V2X IP (as configured in `parameters.json`).
- Ensure **Commsignia hardware is powered on** and connected.
- Logs and received feedback will be saved automatically to their respective folders.

---
## Purpose of the Script

The `RSU_API.py` script does the following:

1. Waits for an OBU (On-Board Unit) to broadcast a **ready** signal.
2. Sends a **crack image** (in chunks) along with hardcoded metadata to the OBU.
3. Receives:
   - **Confirmation** that image was received.
   - **Feedback image and text** processed by OBU.
4. Logs all events and timestamps to help analyze communication delays and system performance.
5. Runs in continuous cycles and handles timeout, image hash matching, and feedback integrity.

---

## Breakdown of the Script

### 1. **Configuration & Imports**
- Loads standard libraries (`os`, `time`, `json`, etc.)
- Loads V2X communication functions from `pycmssdk`.
- Reads all settings (IP, PSIDs, timeouts) from a JSON file (`parameters.json`).

### 2. **Directories**
- Automatically creates folders:
  - `rsu_logs/`  to save CSV logs of each test.
  - `rsu_received_feedback/` to save received image and text feedback from OBU.

### 3. **State Variables**
- Uses variables like:
  - `feedback_chunks`, `feedback_total_chunks` to store incoming data
  - `ready_received`  a flag that waits for OBU readiness
  - `cycle_counter`  tracks which test cycle we’re on

---

##  Image Sending Logic

### `chunk_image(image_path)`
- Opens the image, base64-encodes it, and splits it into chunks of `CHUNK_SIZE`.
- Computes a SHA-256 hash of the image string (used later for verification).

### `send_chunks(...)`
- Sends each chunk to the OBU with metadata like:
  - image ID
  - total chunks
  - image hash
  - crack description
- Logs each chunk's send time to a CSV file.

---

## Feedback Handling

###  `send_detection_request(buffer, tag)`
- Collects feedback (either a PNG image or a TXT message) in chunks.
- Reassembles and decodes it.
- Saves the file in `rsu_received_feedback/`.
- Logs reception time and compares hash for PNG file to ensure it wasn't corrupted.

###  `handle_confirmation(...)`
- Receives confirmation from OBU that the image was fully received.
- Logs how long the full send+confirm cycle took.

---

## Readiness and Cycle Control

### `handle_ready(...)`
- Listens for a message from OBU like: `{ "status": "ready" }`
- Triggers `ready_received` flag to start a new image transmission cycle.

### `main()`
- The core loop:
  1. Waits for `ready` signal.
  2. Sends image if OBU is ready.
  3. Waits for feedback or timeout.
  4. Moves to the next cycle.
- Skips the cycle if no ready signal is received within the timeout.

---

##  Logging Format

Each cycle (`Test1`, `Test2`, ...) generates:

- `TestX_chunks_sent.csv`: Chunk number, timestamp, hash, etc.
- `TestX_feedback_received.csv`: Feedback chunk reception logs.
- `TestX_cycle_summary.csv`: Overall success status, feedback hash match, time taken, etc.

---

## Important Notes

- Image path and crack info are hardcoded (can be updated).
- Handles both image and text feedback in parallel.
- Skips cycles automatically when no OBU is in range — no crash or hang.
- Uses timestamps in milliseconds for precise logging and analysis.

---

## Requirements

- Python 3.x
- `pycmssdk` from Commsignia
- A valid `parameters.json` file with proper V2X stack IP and PSIDs
- Commsignia hardware and firmware set up

---

## Example Workflow

1. RSU waits → OBU sends "ready"
2. RSU sends crack image in chunks
3. OBU replies: confirmation + segmented image + text
4. RSU logs everything (chunk info, hash match, total time)
5. Moves to the next test cycle

