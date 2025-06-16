# File: RSU_API.py
# Description: Roadside Unit (RSU) API for V2X communication, handling image chunk transmission and feedback reception.
# Author: Rishabh Shukla
# Date: June 12, 2025

# === Standard Library Imports ===
import os
import time
import base64
import json
import csv
import logging
import hashlib
from collections import defaultdict
from datetime import datetime
from threading import Event

# Importing Parameters
with open('parameters.json', 'r') as f:
    config = json.load(f)

# === Commsignia Imports ===
from pycmssdk import (
    create_cms_api, WsmpSendData, WsmpTxHdrInfo, RadioTxParams, MacAddr,
    SecDot2TxInfo, SecDot2TxSignInfo, SignMethod
)

# === Configuration Section ===
# Network and Protocol Settings
# === Configuration Section ===
# Network and Protocol Settings
V2X_STACK_IP = config['V2X_STACK_IP']
RECEIVE_IMAGE_PSID = config['RECEIVE_IMAGE_PSID']
SEND_CONFIRM_PSID = config['SEND_CONFIRM_PSID']
SEND_FEEDBACK_PSID = config['SEND_FEEDBACK_PSID']
SEND_READY_PSID = config['SEND_READY_PSID']
CHUNK_SIZE = config['CHUNK_SIZE']
CYCLE_TIMEOUT = config['CYCLE_TIMEOUT']
READY_TIMEOUT = config['READY_TIMEOUT']

# File Paths and Data
IMAGE_PATH = r"latest/crack_img_1.png"  # will be updated depending upon file location(where images are being saved)
CRACK_INFO = "Crack Detected - Type A - Severity: High"

# === Directory Management ===
def make_unique_dir(base):
    """Create a unique directory with a numbered suffix if it already exists."""
    if not os.path.exists(base):
        os.makedirs(base)
        return base
    i = 1
    while True:
        new_path = f"{base}_{i}"
        if not os.path.exists(new_path):
            os.makedirs(new_path)
            return new_path
        i += 1

# Initialize log and received feedback directories
RSU_LOG_DIR = make_unique_dir("rsu_logs")
RECEIVED_FB_DIR = make_unique_dir("rsu_received_feedback")

# === State Management ===
feedback_chunks = defaultdict(dict)  # Store received feedback chunks per image ID
feedback_total_chunks = {}  # Total feedback chunks expected per image
feedback_start_time = {}  # Start time of feedback reception
ready_received = Event()  # Flag for OBU readiness
cycle_counter = 1  # Current cycle number
send_start_time = 0  # Global timestamp for send-confirmation timing

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# === Utility Functions ===
def timestamp():
    """Return current timestamp in milliseconds."""
    return int(time.time() * 1000)

def log_csv(path, headers, row):
    """Log data to a CSV file with headers if new file."""
    new_file = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(headers)
        writer.writerow(row)

def compute_sha256_base64(b64_str):
    """Compute SHA-256 hash of a base64-encoded string."""
    return hashlib.sha256(b64_str.encode('utf-8')).hexdigest()

def chunk_image(image_path):
    """Chunk an image into base64-encoded segments with computed hash."""
    with open(image_path, "rb") as f:
        b64_data = base64.b64encode(f.read()).decode("utf-8")
    hash_val = compute_sha256_base64(b64_data)
    chunks = [b64_data[i:i+CHUNK_SIZE] for i in range(0, len(b64_data), CHUNK_SIZE)]
    return chunks, hash_val

# === Communication Functions ===
def send_chunks(api, chunks, image_id, image_hash, tag):
    """Send image chunks to OBU with logging."""
    send_data = WsmpSendData(
        radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
        wsmp_hdr=WsmpTxHdrInfo(psid=RECEIVE_IMAGE_PSID),
        security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, RECEIVE_IMAGE_PSID))
    )
    for i, chunk in enumerate(chunks):
        payload = {
            "image_id": image_id,
            "chunk_number": i,
            "total_chunks": len(chunks),
            "data": chunk,
            "image_hash": image_hash,
            "crack_info": CRACK_INFO
        }
        api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        logging.info(f"[RSU] Sent chunk {i+1}/{len(chunks)} for {image_id}")
        log_csv(os.path.join(RSU_LOG_DIR, f"{tag}_chunks_sent.csv"),
                ["image_id", "chunk_number", "total_chunks", "send_time", "image_hash"],
                [image_id, i, len(chunks), timestamp(), image_hash])
        time.sleep(0.005)

def send_detection_request(buffer, tag):
    """Process received feedback chunks and save files."""
    try:
        msg = json.loads(buffer.decode("utf-8"))
        image_id = msg["image_id"]
        chunk_num = msg["chunk_number"]
        total = msg["total_chunks"]
        data = msg["data"]
        file_type = msg["file_type"]
        feedback_hash = msg.get("feedback_hash", None)

        if image_id not in feedback_start_time:
            feedback_start_time[image_id] = timestamp()
            feedback_total_chunks[image_id] = total

        feedback_chunks[image_id][chunk_num] = data

        log_csv(os.path.join(RSU_LOG_DIR, f"{tag}_feedback_received.csv"),
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type"],
                [image_id, chunk_num, total, timestamp(), file_type])

        if len(feedback_chunks[image_id]) == total:
            output = ''.join(feedback_chunks[image_id][i] for i in range(total))
            ext = "png" if file_type == "png" else "txt"
            filepath = os.path.join(RECEIVED_FB_DIR, f"{file_type}_{timestamp()}.{ext}")

            with open(filepath, "wb" if file_type == "png" else "w") as f:
                if file_type == "png":
                    f.write(base64.b64decode(output))
                else:
                    f.write(base64.b64decode(output).decode("utf-8"))
            logging.info(f"[RSU] Detection request {file_type} saved: {filepath}")

            if file_type == "png" and feedback_hash:
                rsu_computed_hash = compute_sha256_base64(output)
                hash_match = (feedback_hash == rsu_computed_hash)
                log_csv(os.path.join(RSU_LOG_DIR, f"{tag}_cycle_summary.csv"),
                        ["image_id", "feedback_hash_received", "feedback_hash_computed", "hash_match", "recv_time"],
                        [image_id, feedback_hash, rsu_computed_hash, hash_match, timestamp()])
    except Exception as e:
        logging.error(f"[RSU] Error processing detection request: {e}")

def handle_confirmation(buffer, image_id, tag):
    """Handle confirmation from OBU and log cycle duration."""
    global send_start_time
    try:
        msg = json.loads(buffer.decode("utf-8"))
        if msg.get("image_received") and msg.get("image_id") == image_id:
            duration = timestamp() - send_start_time
            log_csv(os.path.join(RSU_LOG_DIR, f"{tag}_cycle_summary.csv"),
                    ["image_id", "chunks_sent", "start_time", "end_time", "duration_ms", "crack_info"],
                    [image_id, len(chunk_image(IMAGE_PATH)[0]), send_start_time, timestamp(), duration, CRACK_INFO])
            logging.info(f"[RSU] Image {image_id} confirmed by OBU")
    except Exception as e:
        logging.error(f"[RSU] Confirm error: {e}")

def handle_ready(buffer):
    """Handle OBU readiness signal."""
    try:
        msg = json.loads(buffer.decode("utf-8"))
        if msg.get("status") == "ready":
            ready_received.set()
            logging.info("[RSU] OBU ready received")
    except Exception:
        pass

# === Main Execution ===
def main():
    """Main loop to send images and handle feedback/confirmations."""
    global cycle_counter
    with create_cms_api(host=V2X_STACK_IP) as api:
        api.wsmp_rx_subscribe(SEND_CONFIRM_PSID, lambda _, __, b: send_detection_request(b, f"Test{cycle_counter}"))
        api.wsmp_rx_subscribe(SEND_CONFIRM_PSID, lambda _, __, b: handle_confirmation(b, f"img_{cycle_counter}", f"Test{cycle_counter}"))
        api.wsmp_rx_subscribe(SEND_READY_PSID, lambda _, __, b: handle_ready(b))

        while True:
            image_id = f"img_{cycle_counter}"
            tag = f"Test{cycle_counter}"
            ready_received.clear()
            time.sleep(0.05)

            logging.info(f"[RSU] Waiting for OBU readiness before {tag}")
            start_wait = time.time()
            while not ready_received.is_set() and (time.time() - start_wait < READY_TIMEOUT):
                time.sleep(0.05)

            if not ready_received.is_set():
                logging.warning("[RSU] No ready signal. Skipping this cycle.")
                continue

            chunks, image_hash = chunk_image(IMAGE_PATH)
            send_chunks(api, chunks, image_id, image_hash, tag)
            send_start_time = timestamp()

            timeout_time = time.time() + CYCLE_TIMEOUT
            while time.time() < timeout_time:
                time.sleep(0.05)

            logging.info(f"[RSU] Cycle {cycle_counter} complete\n")
            cycle_counter += 1

            time.sleep(0.5)  # Cooldown to prevent cycle overlap

if __name__ == "__main__":
    main()
