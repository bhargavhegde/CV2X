# File: OBU_API.py
# Description: On-Board Unit (OBU) API for V2X communication, handling image chunk reception, confirmation, and feedback.
# Author: Rishabh Shukla
# Date: June 13, 2025

# === Standard Library Imports ===
import os
import time
import json
import base64
import csv
import logging
import hashlib
from datetime import datetime
from collections import defaultdict
from threading import Event, Thread

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
V2X_STACK_IP = config['V2X_STACK_IP']
RECEIVE_IMAGE_PSID = config['RECEIVE_IMAGE_PSID']
SEND_CONFIRM_PSID = config['SEND_CONFIRM_PSID']
SEND_FEEDBACK_PSID = config['SEND_FEEDBACK_PSID']
SEND_READY_PSID = config['SEND_READY_PSID']
CHUNK_SIZE = config['CHUNK_SIZE']
CYCLE_TIMEOUT = config['CYCLE_TIMEOUT']
READY_TIMEOUT = config['READY_TIMEOUT']

# File Paths for Feedback
FEEDBACK_IMAGE_PATH = r"latest/masked_image_latest.png" # will be updated depending upon file location(where images are being saved)
FEEDBACK_TEXT_PATH = r"latest/text_file.txt"  # will be updated depending upon file location(where txt are being saved)

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

# Initialize log and received image directories
OBU_LOG_DIR = make_unique_dir("obu_logs")
RECEIVED_IMG_DIR = make_unique_dir("obu_received_images")

# === State Management ===
image_chunks = defaultdict(dict)  # Store received chunks per image ID
image_chunk_total = {}  # Total chunks expected per image
image_start_time = {}  # Start time of image reception
image_crack_info = {}  # Crack information per image
image_hash_expected = {}  # Expected hash value per image
cycle_counter = 1  # Current cycle number
ready_flag = Event()  # Flag for controlling ready broadcast

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

# === Communication Functions ===
def send_ready_loop():
    """Continuously broadcast OBU readiness status in a separate thread."""
    def loop():
        with create_cms_api(host=V2X_STACK_IP) as api:
            packet = json.dumps({"obu_id": "OBU_XYZ", "status": "ready"}).encode("utf-8")
            send_data = WsmpSendData(
                radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
                wsmp_hdr=WsmpTxHdrInfo(psid=SEND_READY_PSID),
                security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, SEND_READY_PSID))
            )
            while True:
                if not ready_flag.is_set():
                    api.wsmp_send(send_data, buffer=packet)
                    logging.info("[OBU] Broadcasting ready...")
                time.sleep(READY_TIMEOUT)
    Thread(target=loop, daemon=True).start()

def save_image(image_id, b64_data, tag, hash_val):
    """Save received image data to a file."""
    filename = f"{image_id}_{hash_val}.png"
    path = os.path.join(RECEIVED_IMG_DIR, filename)
    with open(path, "wb") as f:
        f.write(base64.b64decode(b64_data))
    logging.info(f"[OBU] Image saved: {path}")
    return path

def send_confirmation(api, image_id):
    """Send confirmation of image reception to RSU."""
    payload = {"image_received": True, "image_id": image_id}
    send_data = WsmpSendData(
        radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
        wsmp_hdr=WsmpTxHdrInfo(psid=SEND_CONFIRM_PSID),
        security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, SEND_CONFIRM_PSID))
    )
    api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
    logging.info(f"[OBU] Sent confirmation for {image_id}")

def encode_feedback_file(path):
    """Encode a feedback file (image or text) into base64."""
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

def send_feedback(api, image_id, tag):
    """Send feedback (image and text) to RSU in chunks."""
    img_data = encode_feedback_file(FEEDBACK_IMAGE_PATH)
    txt_data = encode_feedback_file(FEEDBACK_TEXT_PATH)

    feedback_img_chunks = [img_data[i:i+CHUNK_SIZE] for i in range(0, len(img_data), CHUNK_SIZE)]
    feedback_txt_chunks = [txt_data[i:i+CHUNK_SIZE] for i in range(0, len(txt_data), CHUNK_SIZE)]

    feedback_hash = compute_sha256_base64(img_data)

    send_data = WsmpSendData(
        radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
        wsmp_hdr=WsmpTxHdrInfo(psid=SEND_FEEDBACK_PSID),
        security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, SEND_FEEDBACK_PSID))
    )

    for idx, chunk in enumerate(feedback_img_chunks):
        payload = {
            "image_id": image_id,
            "chunk_number": idx,
            "total_chunks": len(feedback_img_chunks),
            "data": chunk,
            "file_type": "png",
            "feedback_hash": feedback_hash
        }
        api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        logging.info(f"[OBU] Sent image feedback chunk {idx+1}/{len(feedback_img_chunks)}")
        time.sleep(0.005)

    for idx, chunk in enumerate(feedback_txt_chunks):
        payload = {
            "image_id": image_id,
            "chunk_number": idx,
            "total_chunks": len(feedback_txt_chunks),
            "data": chunk,
            "file_type": "txt"
        }
        api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        logging.info(f"[OBU] Sent text feedback chunk {idx+1}/{len(feedback_txt_chunks)}")
        time.sleep(0.005)

    log_csv(os.path.join(OBU_LOG_DIR, f"{tag}_feedback_sent.csv"),
            ["image_id", "img_chunks_sent", "txt_chunks_sent", "send_time"],
            [image_id, len(feedback_img_chunks), len(feedback_txt_chunks), timestamp()])

def handle_chunk(buffer, tag):
    """Process incoming image chunks, assemble, confirm, and send feedback."""
    try:
        msg = json.loads(buffer.decode("utf-8"))
        image_id = msg["image_id"]
        chunk_num = msg["chunk_number"]
        total = msg["total_chunks"]
        data = msg["data"]
        crack_info = msg.get("crack_info", "N/A")
        hash_val = msg.get("image_hash", "N/A")

        if image_id not in image_start_time:
            image_start_time[image_id] = timestamp()
            image_chunk_total[image_id] = total
            image_crack_info[image_id] = crack_info
            image_hash_expected[image_id] = hash_val

        image_chunks[image_id][chunk_num] = data

        log_csv(os.path.join(OBU_LOG_DIR, f"{tag}_chunks_received.csv"),
                ["image_id", "chunk_number", "total_chunks", "recv_time", "image_hash"],
                [image_id, chunk_num, total, timestamp(), hash_val])

        if len(image_chunks[image_id]) == total:
            ordered = ''.join(image_chunks[image_id][i] for i in range(total))
            hash_computed = compute_sha256_base64(ordered)
            hash_match = hash_computed == image_hash_expected[image_id]

            saved_path = save_image(image_id, ordered, tag, hash_computed)

            ready_flag.set()  # Pause ready broadcast during processing

            with create_cms_api(host=V2X_STACK_IP) as api:
                send_confirmation(api, image_id)
                send_feedback(api, image_id, tag)

            duration = timestamp() - image_start_time[image_id]
            log_csv(os.path.join(OBU_LOG_DIR, f"{tag}_cycle_summary.csv"),
                    ["image_id", "chunks_received", "start_time", "end_time", "duration_ms", "crack_info", "expected_hash", "computed_hash", "hash_match"],
                    [image_id, total, image_start_time[image_id], timestamp(), duration, image_crack_info[image_id], image_hash_expected[image_id], hash_computed, hash_match])

            image_chunks.pop(image_id)
            image_chunk_total.pop(image_id)
            image_start_time.pop(image_id)
            image_crack_info.pop(image_id)
            image_hash_expected.pop(image_id)
            ready_flag.clear()

            logging.info(f"[OBU] Full image received: {image_id} ({total} chunks)")
            logging.info("[OBU] Ready for next cycle...")
    except Exception as e:
        logging.error(f"[OBU] Error in chunk handler: {e}")

# === Main Execution ===
def main():
    """Main loop to subscribe to image reception and manage cycles."""
    global cycle_counter
    send_ready_loop()
    with create_cms_api(host=V2X_STACK_IP) as api:
        def callback(_, __, buffer):
            handle_chunk(buffer, f"Test{cycle_counter}")
        api.wsmp_rx_subscribe(RECEIVE_IMAGE_PSID, callback)

        while True:
            tag = f"Test{cycle_counter}"
            logging.info(f"[OBU] Starting {tag}")
            time.sleep(CYCLE_TIMEOUT)
            cycle_counter += 1

if __name__ == "__main__":
    main()
