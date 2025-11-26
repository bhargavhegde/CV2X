"""
RSU_API.py — Roadside Unit (RSU) handler for CV2X crack-detection pipeline

Module responsibilities:
  1) Load runtime parameters from rsu_parameters.json.
  2) Send either (A) chunked base64 image + metadata or (B) metadata-only payloads to OBU via WSMP.
  3) Receive feedback files (PNG/TXT) as chunked base64 and reconstruct/save to disk.
  4) Track timing segments and save TXT/CSV logs.
  5) Passively log BSM (Facilities) messages to TXT for analysis.

Key runtime flow (per cycle):
  • Wait for OBU "ready" broadcast (with timeout).
  • Depending on send_image flag, either send chunks or meta-only.
  • Await feedback queue completion or timeout.
  • Log timing breakdown and advance to next cycle.

Parameters (from rsu_parameters.json):
  V2X_STACK_IP         — IP address of Commsignia stack (CMS) host.
  send_image_psid      — PSID for RSU→OBU crack image/meta WSMP frames.
  receive_confirm_psid — PSID to receive OBU confirmation that it received crack image.
  receive_feedback_psid— PSID to receive OBU feedback (chunked PNG/TXT + completion signals).
  receive_ready_psid   — PSID to receive OBU readiness beacons (status="ready").
  chunk_size           — Base64 character window per chunk when sending or receiving.
  cycle_timeout        — Max seconds to wait for queue_complete after sending.
  ready_timeout        — Max seconds to wait for OBU ready before skipping cycle.
  image_path           — Absolute path to source PNG for chunked send mode.
  crack_info           — String metadata included with every chunk/meta-only payload.
  gps_lat/gps_lon      — GPS coordinates embedded in payload alongside crack_info.
  logging_level        — Reserved (currently unused; logging config via basicConfig below).
  send_image           — Boolean. True: send PNG chunks + meta. False: send meta-only payload.

"""


# Standard Library Imports
import os
import time
import base64
import json
import csv
import logging
from collections import defaultdict
from threading import Event


# Commsignia (pycmssdk) Imports
from pycmssdk import (
    create_cms_api,
    WsmpSendData, WsmpTxHdrInfo,
    RadioTxParams, MacAddr,
    asn1_decode,
    SecDot2TxInfo, SecDot2TxSignInfo,
    SignMethod, Asn1Type, FacMsgType,
)


class RSUHandler:
    """High-level coordinator for RSU runtime.

    Bootstraps configuration, folders, log files; manages send/receive cycle;
    subscribes to Facilities (BSM) and WSMP feedback; reconstructs feedback files;
    """

    def __init__(self, config_path="rsu_parameters.json"):
        """Initialize configuration, directories, file handles, and in-memory state."""
        # === CONFIGURATION ===
        def load_parameters(path):
            """Load JSON parameters from disk. Adjust default path per deployment."""
            with open(path, 'r') as f:
                return json.load(f)

        config = load_parameters(config_path)
        self._config_raw = config  # keep raw dict for parameter snapshot TXT

        # --- Required config fields (fail fast if missing keys) ---
        self.V2X_STACK_IP = config["V2X_STACK_IP"]
        self.SEND_IMAGE_PSID = config["send_image_psid"]
        self.RECEIVE_CONFIRM_PSID = config["receive_confirm_psid"]
        self.RECEIVE_FEEDBACK_PSID = config["receive_feedback_psid"]
        self.RECEIVE_READY_PSID = config["receive_ready_psid"]
        self.CHUNK_SIZE = config["chunk_size"]
        self.CYCLE_TIMEOUT = config["cycle_timeout"]
        self.READY_TIMEOUT = config["ready_timeout"]
        self.IMAGE_PATH = config["image_path"]
        self.CRACK_INFO = config["crack_info"]
        self.GPS_LAT = config.get("gps_lat")
        self.GPS_LON = config.get("gps_lon")

        # === MODE FLAG (JSON-driven) ===
        # True → send image chunks + meta. False → send metadata only.
        self.SEND_IMAGE = config.get("send_image", True)

        # === DIRECTORY SETUP ===
        self.RSU_LOG_DIR = "rsu_logs_v4"
        self.RECEIVED_FB_DIR = "rsu_received_feedback_v4"
        os.makedirs(self.RSU_LOG_DIR, exist_ok=True)
        os.makedirs(self.RECEIVED_FB_DIR, exist_ok=True)

        # Rotating file names (TXT/CSV) for each run.
        self.rsu_sent_log = self.get_next_log_filename("rsu_sent", "txt")
        self.rsu_received_log = self.get_next_log_filename("rsu_received", "csv")
        # NEW: queue-level / feedback receive log (TXT instead of CSV)
        self.rsu_feedback_received_txt = self.get_next_log_filename("rsu_feedback_queue", "txt")

        

        # NEW: queue-level / feedback receive log
        self.rsu_feedback_received_log = self.get_next_log_filename("rsu_feedback_queue", "csv")

        # === PARAM SNAPSHOT (TXT only) ===
        self.rsu_params_txt = self.get_next_log_filename("rsu_params", "txt")
        self._write_param_snapshot()

        # === BSM LOGGING: TXT ONLY ===
        self.bsm_txt_path = self.get_next_log_filename("rsu_received_bsm", "txt")

        # === In-memory receive buffers and timing ===
        # feedback_chunks[image_id][file_type][chunk_idx] -> base64 str
        self.feedback_chunks = defaultdict(lambda: defaultdict(dict))
        # counter of how many feedback files we've reconstructed per (image_id, file_type)
        self.feedback_file_counters = defaultdict(lambda: defaultdict(int))

        self.feedback_total_chunks = {}
        self.feedback_start_time = {}
        self.ready_received = Event()
        self.queue_complete_received = Event()
        self.cycle_counter = 1
        self.send_start_time = 0
        self.image_id = ""
        self.tag = ""
        self.prev_cycle_start_time = None
        self.prev_cycle_end_time = None
        self.cycle_timing = {}

        # Basic logging config; adjust level as needed.
        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

   
    # Small Utilities
    def timestamp(self) -> int:
        """Return current epoch time in milliseconds as int."""
        return int(time.time() * 1000)

    def get_next_log_filename(self, base_name: str, extension: str = "csv") -> str:
        """Generate next numbered log filename inside RSU_LOG_DIR to avoid overwrite."""
        i = 1
        while os.path.exists(os.path.join(self.RSU_LOG_DIR, f"{base_name}_{i}.{extension}")):
            i += 1
        return os.path.join(self.RSU_LOG_DIR, f"{base_name}_{i}.{extension}")
      
    def _parse_image_id(self, image_id: str):
        """Extract cycle and send_time components from image_id like 'img_<cycle>_<send_time>'."""
        parts = image_id.split("_")
        cycle = parts[1] if len(parts) > 1 else "X"
        send_time = parts[2] if len(parts) > 2 else str(self.timestamp())
        return cycle, send_time


    def _reconstruct_gen_epoch_ms_from_secmark(self, recv_ms: int, sec_mark_ms):
        """Reconstruct approximate generation epoch (ms) from secMark using RSU received time."""
        if sec_mark_ms is None:
            return None
        try:
            sec_mark_ms = int(sec_mark_ms)
        except Exception:
            return None
        base_minute_ms = recv_ms - (recv_ms % 60000)  # start of this minute
        candidate = base_minute_ms + sec_mark_ms
        if candidate - recv_ms > 10000:
            candidate -= 60000
        elif recv_ms - candidate > 50000:
            candidate += 60000
        return candidate

    def write_csv(self, path: str, headers: list, row: list) -> None:
        """Append a row to a CSV file; create header if file is new."""
        file_exists = os.path.exists(path)
        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(headers)
            writer.writerow(row)

    def write_txt_line(self, path: str, fields: list, values: list) -> None:
        """Append a single human-readable line: key1=val1, key2=val2, ..."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        line = ", ".join(f"{k}={v}" for k, v in zip(fields, values)) + "\n"
        with open(path, "a", encoding="utf-8") as f:
            f.write(line)

    def _write_param_snapshot(self) -> None:
        """Write a TXT snapshot of the loaded JSON parameters for provenance."""
        try:
            with open(self.rsu_params_txt, "w", encoding="utf-8") as f:
                f.write("=== RSU Parameter Snapshot ===\n")
                f.write(f"created_at_ms: {self.timestamp()}\n\n")
                for k in sorted(self._config_raw.keys()):
                    v = self._config_raw[k]
                    f.write(f"{k}: {v}\n")
            logging.info(f"[RSU] Parameter snapshot saved: {self.rsu_params_txt}")
        except Exception as e:
            logging.error(f"[RSU] Failed to write parameter snapshot: {e}")

    def write_feedback_txt(self, fields: list, values: list) -> None:
        """Append feedback receive events in TXT format: 'key=value, key=value'."""
        line = ", ".join(f"{k}={v}" for k, v in zip(fields, values)) + "\n"
        with open(self.rsu_feedback_received_txt, "a", encoding="utf-8") as f:
            f.write(line)


   
    # Outbound Path (RSU → OBU)
    def chunk_image(self, image_path: str, image_id: str) -> list:
        """Read PNG from disk, base64-encode, and split into fixed-size chunks."""
        chunk_start = self.timestamp()
        with open(image_path, "rb") as f:
            b64_data = base64.b64encode(f.read()).decode("utf-8")
        chunks = [b64_data[i:i+self.CHUNK_SIZE] for i in range(0, len(b64_data), self.CHUNK_SIZE)]
        chunk_end = self.timestamp()
        self.cycle_timing[image_id]["rsu_chunk_time"] = chunk_end - chunk_start
        return chunks

    def send_chunks(self, api, chunks: list, image_id: str) -> None:
        """Transmit base64 chunks to OBU with metadata, then signal send_complete."""
        self.send_start_time = self.timestamp()
        self.cycle_timing[image_id]["rsu_sending_start_time"] = self.send_start_time

        send_data = WsmpSendData(
            radio=RadioTxParams(
                interface_id=1,
                dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),  # broadcast
                datarate=6,
                tx_power=20,
                expiry_time=1000,
            ),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.SEND_IMAGE_PSID),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.SEND_IMAGE_PSID)),
        )

        for i, chunk in enumerate(chunks):
            payload = {
                "image_id": image_id,
                "chunk_number": i,
                "total_chunks": len(chunks),
                "data": chunk,
                "crack_info": self.CRACK_INFO,
                "lat": self.GPS_LAT,
                "lon": self.GPS_LON,
            }
            api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
            logging.info(f"[RSU] Sent chunk {i+1}/{len(chunks)} for {image_id}")

            self.write_txt_line(
                self.rsu_sent_log,
                ["image_id", "chunk_number", "total_chunks", "send_time", "crack_info", "lat", "lon"],
                [image_id, i, len(chunks), self.timestamp(), self.CRACK_INFO, self.GPS_LAT, self.GPS_LON],
            )
            time.sleep(0.005)

        sending_end = self.timestamp()
        self.cycle_timing[image_id]["rsu_sending_time"] = sending_end - self.send_start_time

        complete_payload = {"type": "send_complete", "image_id": image_id, "send_complete_time": sending_end}
        api.wsmp_send(send_data, buffer=json.dumps(complete_payload).encode("utf-8"))

        self.write_txt_line(
            self.rsu_sent_log,
            ["image_id", "chunk_number", "total_chunks", "send_time", "crack_info", "lat", "lon"],
            [image_id, "COMPLETE", len(chunks), sending_end, self.CRACK_INFO, self.GPS_LAT, self.GPS_LON],
        )

        self.cycle_timing[image_id]["rsu_send_complete_time"] = self.timestamp() - sending_end

    def send_meta_only(self, api, image_id: str) -> None:
        """Send metadata-only payload to OBU (no PNG chunks)."""
        send_data = WsmpSendData(
            radio=RadioTxParams(
                interface_id=1,
                dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),
                datarate=6,
                tx_power=20,
                expiry_time=1000
            ),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.SEND_IMAGE_PSID),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.SEND_IMAGE_PSID)),
        )
        send_ts = self.timestamp()
        payload = {
            "type": "meta_only",
            "image_id": image_id,
            "crack_info": self.CRACK_INFO,
            "lat": self.GPS_LAT,
            "lon": self.GPS_LON,
            "send_time": send_ts,
        }
        api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        logging.info(
            f"[RSU] Sent META_ONLY for {image_id} "
            f"(lat={self.GPS_LAT}, lon={self.GPS_LON}, crack='{self.CRACK_INFO}')"
        )

        self.write_txt_line(
            self.rsu_sent_log,
            ["image_id", "type", "time", "crack_info", "lat", "lon"],
            [image_id, "META_ONLY", send_ts, self.CRACK_INFO, self.GPS_LAT, self.GPS_LON],
        )
        self.write_txt_line(
            self.rsu_sent_log,
            ["image_id", "type", "time", "crack_info", "lat", "lon"],
            [image_id, "COMPLETE_META", self.timestamp(), self.CRACK_INFO, self.GPS_LAT, self.GPS_LON],
        )


    # Inbound Path (OBU → RSU)
    def send_detection_request(self, buffer: bytes, tag: str) -> None:
        """Unified handler for feedback channel (chunks, feedback_complete, queue_complete)."""
        try:
            msg = json.loads(buffer.decode("utf-8"))

            # ---- QUEUE COMPLETE ----
            if msg.get("type") == "queue_complete":
                logging.info("[RSU] Received queue_complete from OBU.")
                image_id = msg.get("image_id") or self.image_id
                recv_time = self.timestamp()

                # derive queue_id (TestN) from image_id if possible
                cycle, _ = self._parse_image_id(image_id)
                queue_id = f"Test{cycle}"


                # log queue completion to dedicated feedback-queue log
                self.write_feedback_txt(
                    ["queue_id", "image_id", "file_type", "file_name", "queue_type", "recv_time_ms", "index_in_queue"],
                    [queue_id, image_id, "", "QUEUE_COMPLETE", "queue_complete", recv_time, ""]
                    )


                if image_id:
                    self.feedback_chunks.pop(image_id, None)
                    self.feedback_start_time.pop(image_id, None)
                    self.feedback_file_counters.pop(image_id, None)
                self.queue_complete_received.set()
                return

            # ---- FEEDBACK COMPLETE (per image_id) ----
            if msg.get("type") == "feedback_complete":
                image_id = msg.get("image_id", "")
                recv_time = self.timestamp()
                feedback_sent_time = msg.get("feedback_send_time", "")
                if image_id not in self.cycle_timing:
                    image_id = self.image_id
                start_ts = self.cycle_timing.get(image_id, {}).get("rsu_cycle_start_time", recv_time)
                total_cycle_time = recv_time - start_ts
                self.cycle_timing.setdefault(image_id, {})["total_cycle_log_time"] = total_cycle_time
                self.write_csv(
                    self.rsu_received_log,
                    ["image_id", "chunk_number", "recv_time", "file_type",
                     "feedback_sent_time", "total_cycle_log_time"],
                    [image_id, "FEEDBACK_DONE", recv_time, "", feedback_sent_time, total_cycle_time],
                )
                logging.info(f"[RSU] Received feedback_complete for {image_id}")
                return

            # ---- CHUNKED FEEDBACK (PNG/TXT) ----
            image_id = msg["image_id"]
            chunk_num = msg["chunk_number"]
            total = msg["total_chunks"]
            data = msg["data"]
            file_type = msg["file_type"]  # "png" or "txt"
            file_name_from_msg = msg.get("file_name")  # optional (OBU may not send yet)

            if image_id not in self.feedback_start_time:
                self.feedback_start_time[image_id] = self.timestamp()
                base = self.cycle_timing.get(image_id, {}).get(
                    "rsu_sending_start_time", self.feedback_start_time[image_id]
                )
                self.cycle_timing.setdefault(image_id, {})["rsu_sending_start_time"] = base
                self.cycle_timing[image_id]["rsu_feedback_delay"] = (
                    self.feedback_start_time[image_id] - base
                )

            self.feedback_chunks[image_id][file_type][chunk_num] = data

            # When all chunks for this file_type are present: reconstruct + save
            if len(self.feedback_chunks[image_id][file_type]) == total:
                output = ''.join(
                    self.feedback_chunks[image_id][file_type][i] for i in range(total)
                )
                ext = "png" if file_type == "png" else "txt"

                cycle, send_time = self._parse_image_id(image_id)

                log_version = os.path.basename(self.rsu_received_log).split("_")[-1].split(".")[0]
                test_dir = os.path.join(
                    self.RECEIVED_FB_DIR, f"log{log_version}", f"Test{cycle}"
                )
                os.makedirs(test_dir, exist_ok=True)

                idx = self.feedback_file_counters[image_id][file_type] + 1
                self.feedback_file_counters[image_id][file_type] = idx

                if file_name_from_msg:
                    base_name = os.path.splitext(os.path.basename(file_name_from_msg))[0]
                    filename = f"{base_name}.{ext}"
                else:
                    filename = f"{file_type}_{cycle}_{idx}.{ext}"

                filepath = os.path.join(test_dir, filename)

                log_start = self.timestamp()
                with open(filepath, "wb" if file_type == "png" else "w") as f:
                    if file_type == "png":
                        f.write(base64.b64decode(output))
                    else:
                        f.write(base64.b64decode(output).decode("utf-8"))
                log_end = self.timestamp()
                self.cycle_timing.setdefault(image_id, {})["rsu_logging_time"] = (
                    log_end - log_start
                )

                logging.info(
                    f"[RSU] Saved feedback {file_type} for {image_id} "
                    f"as {filepath} (image index {idx})"
                )

                # NEW: log per-file receive event into rsu_feedback_queue_#.csv
                recv_time = log_end
                queue_id = f"Test{cycle}"
                queue_type = f"feedback_{file_type}"
                self.write_feedback_txt(
                    ["queue_id", "image_id", "file_type", "file_name", "queue_type", "recv_time_ms", "index_in_queue"],
                    [queue_id, image_id, file_type, filename, queue_type, recv_time, idx]
                    )


                # Clear chunks for this (image_id, file_type) so next file in queue is clean
                self.feedback_chunks[image_id][file_type].clear()

        except Exception as e:
            logging.error(f"[RSU] Feedback error: {e}")

    def handle_ready(self, buffer: bytes) -> None:
        """Handle OBU readiness message: set Event so main loop may proceed."""
        try:
            msg = json.loads(buffer.decode("utf-8"))
            if msg.get("status") == "ready":
                self.ready_received.set()
                logging.info("[RSU] OBU ready received")
        except Exception:
            pass

    def handle_confirmation(self, buffer: bytes, image_id: str) -> None:
        """Handle OBU confirmation that it has received the crack image."""
        try:
            msg = json.loads(buffer.decode("utf-8"))
            if msg.get("image_received") and msg.get("image_id") == image_id:
                logging.info(f"[RSU] Image {image_id} confirmed by OBU")
        except Exception as e:
            logging.error(f"[RSU] Confirm error: {e}")

 
    # Facilities (BSM) Logging

    def log_bsm_txt(self, line: str) -> None:
        """Append a raw, human-readable BSM line to rsu_received_bsm_#.txt."""
        try:
            with open(self.bsm_txt_path, "a", encoding="utf-8") as f:
                f.write(line.rstrip() + "\n")
        except Exception as e:
            logging.error(f"BSM TXT write error: {e}")

    def handle_fac_bsm(self, *args) -> None:
        """Facilities BSM callback that tolerates SDK signature variants."""
        ts = time.time()

        if len(args) == 3:
            _, _, buffer = args
        elif len(args) == 2:
            _, buffer = args
        else:
            return

        raw = buffer if isinstance(buffer, (bytes, bytearray)) else b""
        if not raw:
            return

        try:
            mf = asn1_decode(raw, Asn1Type.US_MESSAGE_FRAME)
            cd = mf["value"][1]["coreData"]

            temp_id     = cd.get("id", b"")
            temp_id     = temp_id.hex() if isinstance(temp_id, (bytes, bytearray)) else temp_id
            lat_deg     = cd.get("lat")
            lon_deg     = cd.get("long")
            speed_mps   = (cd.get("speed")   / 50.0) if isinstance(cd.get("speed"), (int, float)) else None
            heading_deg = (cd.get("heading") / 80.0) if isinstance(cd.get("heading"), (int, float)) else None

            secMark_gen_time = cd.get("secMark", None)
            BSM_recv_time = int(ts * 1000)

            timestamp_gen_time = None
            try:
                timestamp_gen_time = cd.get("timeStamp", None)
                if timestamp_gen_time is None and isinstance(mf.get("value", []), (list, tuple)) and len(mf["value"]) > 1:
                    timestamp_gen_time = mf["value"][1].get("timeStamp", None)
            except Exception:
                timestamp_gen_time = None
            if timestamp_gen_time is None:
                timestamp_gen_time = self._reconstruct_gen_epoch_ms_from_secmark(BSM_recv_time, secMark_gen_time)

            self.log_bsm_txt(
                f"{ts:.3f} FAC temp_id={temp_id} lat={lat_deg} lon={lon_deg} "
                f"speed={speed_mps} heading={heading_deg} "
                f"timestamp_gen_time={timestamp_gen_time} secMark_gen_time={secMark_gen_time} "
                f"BSM_recv_time={BSM_recv_time}"
            )

        except Exception:
            return


    # Main Runtime Loop
    def run(self) -> None:
        """Entry point: establish CMS connection, subscribe to channels, and loop cycles."""
        with create_cms_api(host=self.V2X_STACK_IP) as api:
            try:
                if hasattr(api, "fac_subscribe"):
                    api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, self.handle_fac_bsm)
                    logging.info("[RSU] Subscribed to Facilities BSM via fac_subscribe(FAC_MSG_US_BSM).")
                else:
                    raise AttributeError("SDK has no fac_subscribe; add fallback if needed.")
            except Exception as e:
                logging.warning(f"[RSU] Facilities BSM subscribe failed: {e}")

            api.wsmp_rx_subscribe(
                self.RECEIVE_FEEDBACK_PSID,
                lambda _, __, b: self.send_detection_request(b, self.tag)
            )
            api.wsmp_rx_subscribe(
                self.RECEIVE_CONFIRM_PSID,
                lambda _, __, b: self.handle_confirmation(b, self.image_id)
            )
            api.wsmp_rx_subscribe(
                self.RECEIVE_READY_PSID,
                lambda _, __, b: self.handle_ready(b)
            )

            while True:
                cycle_timestamp = self.timestamp()
                self.image_id = f"img_{self.cycle_counter}_{cycle_timestamp}"
                self.tag = f"Test{self.cycle_counter}"

                self.ready_received.clear()
                self.queue_complete_received.clear()
                cycle_start_time = self.timestamp()
                self.cycle_timing[self.image_id] = {"rsu_cycle_start_time": cycle_start_time}

                self.prev_cycle_start_time = cycle_start_time

                logging.info(f"[RSU] Waiting for OBU readiness before {self.tag}")
                wait_start_ts = self.timestamp()
                start_wait = time.time()
                while (
                    not self.ready_received.is_set()
                    and (time.time() - start_wait < self.READY_TIMEOUT)
                ):
                    time.sleep(0.01)
                self.cycle_timing[self.image_id]["rsu_wait_ready"] = (
                    self.timestamp() - wait_start_ts
                )

                if not self.ready_received.is_set():
                    logging.warning("[RSU] No ready signal. Skipping this cycle.")
                    continue

                if self.SEND_IMAGE:
                    chunks = self.chunk_image(self.IMAGE_PATH, self.image_id)
                    self.send_chunks(api, chunks, self.image_id)
                else:
                    self.send_meta_only(api, self.image_id)

                self.prev_cycle_end_time = self.timestamp()

                wait_start = time.time()
                while (
                    not self.queue_complete_received.is_set()
                    and (time.time() - wait_start < self.CYCLE_TIMEOUT)
                ):
                    time.sleep(0.05)

                logging.info(f"[RSU] Cycle {self.cycle_counter} complete\n")
                self.cycle_counter += 1
                time.sleep(0.1)


if __name__ == "__main__":
    RSUHandler().run()
