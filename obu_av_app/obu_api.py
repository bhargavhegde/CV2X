#! /usr/bin/env python3

"""
Description: On-Board Unit (OBU) API for V2X communication,
handling image chunk reception, confirmation, and feedback.
Author: Rishabh Shukla
Date: June 27, 2025
"""

# Standard library imports
import base64
import csv
import json
import logging
import os
import queue
import threading
import time
from collections import defaultdict
from datetime import datetime
from threading import Event, Thread

# Third-party imports
# CV2X imports
from pycmssdk import ( # noqa: F401
    MacAddr,
    RadioTxParams,
    SecDot2TxInfo,
    SecDot2TxSignInfo,
    SignMethod,
    WsmpSendData,
    WsmpTxHdrInfo,
    create_cms_api
)

# Local imports
# None

# Global Variables for getting GPS
# global gps_crack_lat, gps_crack_lon

gps_crack_lat = 500
gps_crack_lon = 500
lock = threading.Lock()


class OBUHandler:
    def __init__(self, image_queue, test_num): #, image_queue, gps_crack_lat, gps_crack_lon
        # Configuration
        self.v2x_stack_ip = "192.168.3.54"
        self.receive_image_psid = 200
        self.send_confirm_psid = 300
        self.send_feedback_psid = 400
        self.send_ready_psid = 201
        self.chunk_size = 1024
        self.ready_interval = 0.05
        self.cycle_timeout = 2
        self.feedback_image_path = r"data/latest/picture.png"
        

        self.image_chunks = defaultdict(dict)
        self.image_chunk_total = {}
        self.cycle_counter = 1
        self.ready_flag = Event()

        self.obu_log_dir = "data/obu_logs"
        self.received_img_dir = "data/obu_received_images"
        os.makedirs(self.obu_log_dir, exist_ok=True)
        os.makedirs(self.received_img_dir, exist_ok=True)

        self.obu_received_log = os.path.join(self.obu_log_dir, self.get_next_log_filename("obu_received"))
        self.obu_sent_log = os.path.join(self.obu_log_dir, self.get_next_log_filename("obu_sent"))

        # Shared variables
        self.image_queue = image_queue
        self.test_num = test_num
        # self.crack_lat = gps_crack_lat
        # self.crack_lon = gps_crack_lon

        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    def get_next_log_filename(self, base_name):
        i = 1
        while os.path.exists(f"{base_name}_{i}.csv"):
            i += 1
        return f"{base_name}_{i}.csv"

    def timestamp(self):
        return int(time.time() * 1000)

    def write_csv(self, path, headers, row):
        file_exists = os.path.exists(path)
        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(headers)
            writer.writerow(row)

    def send_ready_loop(self):
        def loop():
            with create_cms_api(host=self.v2x_stack_ip) as api:
                packet = json.dumps(
                    {"obu_id": "OBU_XYZ", "status": "ready"}
                ).encode("utf-8")
                send_data = WsmpSendData(
                    radio=RadioTxParams(
                        interface_id=1,
                        dest_address=MacAddr(0xFF,) * 6,
                        datarate=6,
                        tx_power=20,
                        expiry_time=1000
                    ),
                    wsmp_hdr=WsmpTxHdrInfo(psid=self.send_ready_psid),
                    security=SecDot2TxInfo(
                        sign_info=SecDot2TxSignInfo(
                            SignMethod.SIGN_METH_SIGN_CERT,
                            self.send_ready_psid
                        )
                    )
                )
                while True:
                    if not self.ready_flag.is_set():
                        api.wsmp_send(send_data, buffer=packet)
                        logging.info("[OBU] Broadcasting ready...")
                    time.sleep(self.ready_interval)
        Thread(target=loop, daemon=True).start()

    def save_image(self, image_id, b64_data):
        try:
            parts = image_id.split("_")
            cycle = parts[1] if len(parts) > 1 else "X"
            send_time = parts[2] if len(parts) > 2 else str(self.timestamp())

            log_version = os.path.basename(self.obu_received_log).split("_")[-1].split(".")[0]
            log_folder = f"log{log_version}"

            test_folder = f"Test{cycle}"
            test_dir = os.path.join(self.received_img_dir, log_folder, test_folder)
            os.makedirs(test_dir, exist_ok=True)

            filename = f"img_{cycle}_{send_time}.png"
            path = os.path.join(test_dir, filename)
            with open(path, "wb") as f:
                f.write(base64.b64decode(b64_data))
            logging.info(f"[OBU] Image saved: {path}")
            return path
        except Exception as e:
            logging.error(f"[OBU] Error saving image: {e}")
            return ""

    def send_confirmation(self, api, image_id):
        payload = {"image_received": True, "image_id": image_id}
        send_data = WsmpSendData(
            radio=RadioTxParams(
                interface_id=1,
                dest_address=MacAddr(0xFF,) * 6,
                datarate=6,
                tx_power=20,
                expiry_time=1000
            ),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.send_confirm_psid),
            security=SecDot2TxInfo(
                sign_info=SecDot2TxSignInfo(
                    SignMethod.SIGN_METH_SIGN_CERT,
                    self.send_confirm_psid
                )
            )
        )
        api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        logging.info(f"[OBU] Sent confirmation for {image_id}")

    def encode_feedback_file(self, path):
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")

    def send_feedback(self, api, image_id, send_type):
        img_path = (
            f"data/Test_{self.test_num}/Image_Folder/masks/"
            + str(self.image_queue.get())
        )
        print(
            "Current Image: ", img_path,
            "------------------------------------------------------------------------------"
        )
        img_data = self.encode_feedback_file(img_path)

        feedback_img_chunks = [
            img_data[i:i + self.chunk_size]
            for i in range(0, len(img_data), self.chunk_size)
        ]

        send_data = WsmpSendData(
            radio=RadioTxParams(
                interface_id=1,
                dest_address=MacAddr(0xFF,) * 6,
                datarate=6,
                tx_power=20,
                expiry_time=1000
            ),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.send_feedback_psid),
            security=SecDot2TxInfo(
                sign_info=SecDot2TxSignInfo(
                    SignMethod.SIGN_METH_SIGN_CERT,
                    self.send_feedback_psid
                )
            )
        )

        for idx, chunk in enumerate(feedback_img_chunks):
            payload = {
                "image_id": image_id,
                "chunk_number": idx,
                "total_chunks": len(feedback_img_chunks),
                "data": chunk,
                "file_type": "png"
            }
            api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
            self.write_csv(
                self.obu_sent_log,
                ["image_id", "chunk_number", "total_chunks", "send_time", "file_type"],
                [image_id, idx, len(feedback_img_chunks), self.timestamp(), "png"]
            )
            logging.info(
                f"[OBU] Sent feedback PNG chunk {idx + 1}/{len(feedback_img_chunks)}"
            )
            time.sleep(0.01)

        feedback_done_payload = {
            "type": send_type,
            "image_id": image_id,
            "feedback_send_time": self.timestamp()
        }
        api.wsmp_send(send_data, buffer=json.dumps(feedback_done_payload).encode("utf-8"))
        logging.info(f"[OBU] Sent feedback_complete for {image_id}")
        logging.info(
            f"[OBU] Feedback send complete: {len(feedback_img_chunks)} PNG chunks."
        )

    def handle_chunk(self, buffer):
        # print("Handle Chunk is working...")
        try:
            msg = json.loads(buffer.decode("utf-8"))

            global gps_crack_lat, gps_crack_lon

            if msg.get("type") == "meta_only":
                image_id = msg["image_id"]
                crack_info = msg.get("crack_info", "")
                gps_crack_lat = msg.get("lat")
                gps_crack_lon = msg.get("lon")
                # Log a row with chunk_number marked as META_ONLY to keep CSV schema stable
                self.write_csv(
                    self.obu_received_log,
                    ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                    [image_id, "META_ONLY", "", self.timestamp(), "", msg.get("send_time", ""), crack_info]
                )
                logging.info(
                    f"[OBU] Meta-only detection request received for {image_id} (lat={gps_crack_lat}, lon={gps_crack_lon})"
                )
                # Send confirmation and then send feedback queue (reuses existing path)
                with create_cms_api(host=self.v2x_stack_ip) as api:
                    self.send_confirmation(api, image_id)
                    if not self.image_queue.empty():
                        send_type = "feedback_complete"
                        if self.image_queue.qsize() == 1:
                            send_type = "queue_complete"
                        self.send_feedback(api, image_id, send_type)
                    else:
                        logging.info("[OBU] Meta-only request received, but feedback queue is empty.")
                self.ready_flag.clear()
                print("Returning from here----------------------------------------")
                return

            if msg.get("type") == "send_complete":
                image_id = msg["image_id"]
                send_time = msg["send_complete_time"]
                self.write_csv(
                    self.obu_received_log,
                    ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                    [image_id, "RSU_DONE", "", self.timestamp(), "", send_time, ""]
                )
                logging.info(f"[OBU] Logged RSU send_complete for {image_id}")
                return

            image_id = msg["image_id"]
            chunk_num = msg["chunk_number"]
            total = msg["total_chunks"]
            data = msg["data"]
            
            # Getting GPS Location of the target crack
            # global gps_crack_lat, gps_crack_lon
            gps_crack_lat = float(msg["lat"])
            gps_crack_lon = float(msg["lon"])
            
            print('lat from RSU-----------------------------------------------', gps_crack_lat)
            print('lon from RSU-----------------------------------------------', gps_crack_lon)
            crack_info = msg.get("crack_info", "")

            # print(f"Lat: {gps_crack_lat} Lon: {gps_crack_lon} ----------------------------------------------------")
            if image_id not in self.image_chunk_total:
                self.image_chunk_total[image_id] = total

            self.image_chunks[image_id][chunk_num] = data

            self.write_csv(
                self.obu_received_log,
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                [image_id, chunk_num, total, self.timestamp(), "image", "", crack_info]
            )

            if len(self.image_chunks[image_id]) == total:
                ordered = ''.join(self.image_chunks[image_id][i] for i in range(total))
                self.save_image(image_id, ordered)

                self.ready_flag.set()

                with create_cms_api(host=self.v2x_stack_ip) as api:
                    self.send_confirmation(api, image_id)
                    if not self.image_queue.empty():
                        send_type = "feedback_complete"
                        if self.image_queue.qsize() == 1:
                            send_type = "queue_complete"
                        self.send_feedback(api, image_id, send_type)
                        #time.sleep(1.0)
                    else:
                        print("Queue is empty!!!")

                self.image_chunks.pop(image_id)
                self.image_chunk_total.pop(image_id)
                self.ready_flag.clear()

                logging.info(f"[OBU] Image fully received: {image_id}")

        except Exception as e:
            logging.error(f"[OBU] Error handling chunk: {e}")


# Standalone runner that can be imported
def run_obu(image_queue, test_num): #image_queue, gps_crack_lat, gps_crack_lon
    '''
    The main function that handles
    1. (TODO) receiving detection request (including GPS location and images if any)
    2. (TODO) sending status updates
    3. sending detection result queue (referred as feedback in this module)
    4. (TODO) Should we put the BSM publishing here too? Or this will just take api as input?
    '''
    handler = OBUHandler(image_queue, test_num) # image_queue, gps_crack_lat, gps_crack_lon
    handler.send_ready_loop()
    # TODO: [CRH Question] Should we use the API here or pass it as an argument?
    with create_cms_api(host=handler.v2x_stack_ip) as api:
        def callback(_, __, buffer):
            handler.handle_chunk(buffer)
        api.wsmp_rx_subscribe(handler.receive_image_psid, callback)

        while True:
            print("Image Queue Size: ", image_queue.qsize())
            logging.info(f"[OBU] Starting cycle {handler.cycle_counter}")
            time.sleep(handler.cycle_timeout)
            handler.cycle_counter += 1


def get_data_obu():
    '''
    Get crack GPS data from OBU.
    '''
    # print('lat :', gps_crack_lat)
    # print('lon :', gps_crack_lon)
    with lock:
        # return 500, 500
        return gps_crack_lat, gps_crack_lon


# Optional direct execution
if __name__ == "__main__":
    image_queue = queue.Queue()
    image_queue.put("latest/picture.png")
    image_queue.put("latest/picture1.png")
    image_queue.put("latest/picture2.png")
    run_obu(image_queue, 0)
