#!/usr/bin/env python3

"""
OBU API Node

1) RSU → OBU: receive lat/lon and publish on /obu/gps.
2) OBU → RSU: when RSU asks (meta_only / send_complete), send ONE image from folder
   as chunks and then send feedback_complete + queue_complete (per image / per cycle).
3) OBU NAV/FAC: log own NAV and received US BSM/FAC using ROS time (ms).
4) Logging structure:
   - Root:   <directory of parameters.json> / OBU_LOG_DIR  (default OBU_LOG_DIR='obu_logs')
   - Per run: one folder per date + index, e.g.:
       obu_logs/obu_log_20251116_1/
       obu_logs/obu_log_20251116_2/
       obu_logs/obu_log_20251117_1/
   - Inside each run folder (all structured, comma-separated TXT with header):
       obu_sent.txt
       obu_received.txt
       obu_nav.txt
       obu_fac.txt
       nav_fac_debug/fac_raw.txt   (JSON-per-line debug of full FAC coreData)
       obu_received_images/        (RSU images reconstructed from chunks if sending flag is true from RSU end)

   - All timestamps (send_time, recv_time, nav_gen/recv, fac_gen/recv) use ROS time (ms),
     except NAV generation time which uses nav.timestamp when available.
"""

import base64
import csv
import json
import logging
import os
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from queue import Queue, Empty
from collections import defaultdict  # NEW: for RSU image chunk buffers

from pycmssdk import (  # type: ignore
    create_cms_api,
    MacAddr,
    RadioTxParams,
    SecDot2TxInfo,
    SecDot2TxSignInfo,
    SignMethod,
    WsmpTxHdrInfo,
    WsmpSendData,
    Asn1Type,
    FacMsgType,
    FacNotifData,
    asn1_decode,
)


# ---------------------- small helpers ----------------------
def wait_for_directory(folder_path: str, check_interval: float = 0.5, timeout: Optional[float] = None) -> bool:
    start = time.time()
    while not os.path.isdir(folder_path):
        if timeout is not None and (time.time() - start) > timeout:
            logging.error(f"[OBU] Timeout ({timeout}s) waiting for directory: {folder_path}")
            return False
        time.sleep(check_interval)
    return True


def wait_for_file(file_path: str, check_interval: float = 0.5, timeout: Optional[float] = None) -> bool:
    folder = os.path.dirname(file_path) or "."
    if not wait_for_directory(folder, check_interval, timeout):
        return False
    start = time.time()
    while not os.path.isfile(file_path):
        if timeout is not None and (time.time() - start) > timeout:
            logging.error(f"[OBU] Timeout ({timeout}s) waiting for file: {file_path}")
            return False
        time.sleep(check_interval)
    return True


def b64_of_file(path: str) -> str:
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def integrate_time(timestamp_ms: int, hearing_secmark: int) -> int:
    """
      Integrate BSM secMark into an absolute millisecond timestamp.

      secMark is the millisecond offset within the current minute (0–59999).
      This function aligns secMark with the most recent ROS time to produce a
      consistent absolute timestamp.
    """
    floored_min = timestamp_ms // 60000
    remainder_ms = timestamp_ms % 60000
    if remainder_ms >= hearing_secmark:
        time_integrated = hearing_secmark + floored_min * 60000
    else:
        time_integrated = hearing_secmark + (floored_min - 1) * 60000
    return time_integrated


def generate_fac_time(timestamp_ms: int, secmark: int) -> int:
    """Approximate FAC generation time using secMark and ROS-time-based timestamp."""
    time_generated = integrate_time(timestamp_ms, secmark)
    if abs(time_generated - timestamp_ms) > 50000:
        time_generated = time_generated + 60000
        if time_generated > timestamp_ms:
            time_generated = timestamp_ms
    return time_generated


class OBUNode(Node):
    def __init__(self):
        super().__init__("obu_node")

        # ----- load parameters (JSON) -----
        params_path = os.path.join(os.path.dirname(__file__), "obu_parameters.json")  # vehicle will use relative path
        self.declare_parameter("params_path", params_path)

        params_path = self.get_parameter("params_path").get_parameter_value().string_value
        self.params = self._load_params(params_path)

        # Base folder where params live – use this to anchor logs so paths are stable
        self.params_dir = os.path.dirname(os.path.abspath(params_path))

        # Core WSMP / pipeline params
        self.v2x_stack_ip = self.params.get("V2X_STACK_IP")
        self.rx_psid = self.params.get("RECEIVE_IMAGE_PSID")
        self.psid_confirm = self.params.get("SEND_CONFIRM_PSID")
        self.psid_feedback = self.params.get("SEND_FEEDBACK_PSID")
        self.psid_ready = self.params.get("SEND_READY_PSID")

        self.chunk_size = int(self.params.get("CHUNK_SIZE"))
        self.ready_interval = float(self.params.get("READY_INTERVAL"))
        self.cycle_timeout = float(self.params.get("CYCLE_TIMEOUT_SEC"))
        self.declare_parameter("wsmp_drain_limit", 50)
        self.wsmp_drain_limit = int(self.get_parameter("wsmp_drain_limit").value)

        # waits + filters
        self.folder_wait_timeout = self.params.get("FOLDER_WAIT_TIMEOUT", None)
        self.file_wait_timeout = self.params.get("FILE_WAIT_TIMEOUT", None)
        self.image_exts: List[str] = list(self.params.get("IMAGE_EXTS", [".png", ".jpg", ".jpeg"]))

        # ROS topics for GPS output + base_dir/test_number
        self.gps_topic = self.params.get("GPS_TOPIC", "/obu/gps")
        self.test_number_topic = self.params.get("TEST_NUMBER_TOPIC", "/test_number")
        self.base_dir_topic = self.params.get("BASE_DIR_TOPIC", "/obu/base_dir")

        # test folders for reading feedback images (if using subscribed folders)
        self.test_base_root = self.params.get("TEST_BASE_ROOT", "")
        self.use_subscribed_folder = bool(self.params.get("USE_SUBSCRIBED_FOLDER", False))

        # ----- logging root + per-run folder -----
        log_root_name = self.params.get("OBU_LOG_DIR", "obu_logs")
        self.log_root = os.path.join(self.params_dir, log_root_name)
        os.makedirs(self.log_root, exist_ok=True)

        self.run_dir = os.path.join(self.log_root, self._next_run_name())
        os.makedirs(self.run_dir, exist_ok=True)

        self.log_dir = self.run_dir

        # Per-run structured log files
        self.sent_log = os.path.join(self.log_dir, "obu_sent.txt")
        self.recv_log = os.path.join(self.log_dir, "obu_received.txt")
        self.obu_nav_log = os.path.join(self.log_dir, "obu_nav.txt")
        self.obu_fac_log = os.path.join(self.log_dir, "obu_fac.txt")

        # FAC debug folder inside this run directory
        self.nav_fac_debug_dir = os.path.join(self.log_dir, "nav_fac_debug")
        os.makedirs(self.nav_fac_debug_dir, exist_ok=True)
        self.fac_debug_file = os.path.join(self.nav_fac_debug_dir, "fac_raw.txt")

        # Directory + buffers for RSU images reconstructed on OBU side
        self.rsu_image_dir = os.path.join(self.log_dir, "obu_received_images")
        os.makedirs(self.rsu_image_dir, exist_ok=True)

        # rsu_chunks[image_id][chunk_number] = base64 data str (from RSU)
        self.rsu_chunks = defaultdict(dict)
        self.rsu_chunk_totals = {}
        self.rsu_file_types = {}

        # Track which image_ids already got feedback (avoid duplicate send)
        self.feedback_sent_for = set()

        # Remember RSU request if it arrives before base_dir is known
        self.pending_image_request_id: Optional[str] = None

        # For offline/online modes, remember which files we have already sent
        self.offline_sent_files = set()
        self.online_sent_files = set()

        # ----- node state -----
        self.gps_lat: float = 500.0
        self.gps_lon: float = 500.0

        self.obu_lat: float = 0.0
        self.obu_lon: float = 0.0
        self.obu_alt: float = 0.0
        self.obu_heading: float = 0.0
        self.obu_speed: float = 0.0
        self.obu_sats: float = 0.0

        # folder path for feedback images:
        # <TEST_BASE_ROOT>/<test_num>/detection_folder/mask OR IMAGE_FOLDER from params
        self.base_dir: Optional[str] = self.params.get("IMAGE_FOLDER") or None

        # inbound RSU message queue
        self.wsmp_in: Queue = Queue()

        # ----- ROS pubs/subs/timers -----
        self.pub_gps = self.create_publisher(Float64MultiArray, self.gps_topic, 10)
        self.pub_obu_nav = self.create_publisher(Float64MultiArray, "/obu/nav", 10)

        self.sub_test_number = self.create_subscription(String, self.test_number_topic, self._on_test_number, 10)
        self.sub_base_dir = self.create_subscription(String, self.base_dir_topic, self._on_base_dir, 10)

        self.gps_pub_hz = float(self.params.get("GPS_PUB_HZ", 2.0))
        self.create_timer(1.0 / max(self.gps_pub_hz, 0.1), self._on_gps_timer)
        self.create_timer(self.ready_interval, self._on_ready_timer)
        self.create_timer(0.02, self._process_wsmp_queue)

        # ----- WSMP API -----
        self.api = create_cms_api(host=self.v2x_stack_ip)
        self.api.__enter__()
        self.api.wsmp_rx_subscribe(self.rx_psid, self._wsmp_rx_callback)
        self.api.nav_subscribe(self._on_nav)
        self.api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, self._on_fac)

        self.get_logger().info(f"OBU node started. Logs in: {self.run_dir}")

    # ---------------- ROS callbacks ----------------
    def _on_gps_timer(self):
        msg = Float64MultiArray()
        msg.data = [float(self.gps_lat), float(self.gps_lon)]
        self.pub_gps.publish(msg)

    def _on_ready_timer(self):
        payload = json.dumps({"obu_id": "OBU_XYZ", "status": "ready"}).encode("utf-8")
        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.psid_ready),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.psid_ready)),
        )
        self.api.wsmp_send(send_data, buffer=payload)

    def _on_base_dir(self, msg: String):
        """Handle dynamic IMAGE folder updates and possibly send pending feedback."""
        if not self.use_subscribed_folder:
            self.get_logger().debug("[OBU] ignoring /obu/base_dir because USE_SUBSCRIBED_FOLDER is false")
            return
        path = msg.data.strip()
        if path:
            self.base_dir = path
            self.get_logger().info(f"[OBU] IMAGE folder updated via /obu/base_dir: {self.base_dir}")
            # If RSU already requested an image, send it now
            if self.pending_image_request_id is not None:
                img_id = self.pending_image_request_id
                self.get_logger().info(
                    f"[OBU] Pending RSU request {img_id} detected after /obu/base_dir; starting send."
                )
                try:
                    self._send_folder_all(img_id)
                    self.feedback_sent_for.add(img_id)
                except Exception as e:
                    self.get_logger().error(f"[OBU] send_folder_all failed after /obu/base_dir: {e}")
                self.pending_image_request_id = None

    def _on_test_number(self, msg: String):
        """Build IMAGE folder based on /test_number and possibly send pending feedback."""
        if not self.use_subscribed_folder:
            self.get_logger().debug("[OBU] ignoring /test_number because USE_SUBSCRIBED_FOLDER is false")
            return

        test_num = msg.data.strip()
        if not test_num:
            return

        if not self.test_base_root:
            self.get_logger().error("TEST_BASE_ROOT is empty; cannot build folder from /test_number.")
            return

        # Vehicle side uses: data/Test_<test_num>/
        folder_name = f"Test_{test_num}"
        full_path = os.path.join(
            self.test_base_root,
            folder_name,
            "detection_folder",   # must match vehicle-side directory structure
            "mask",
        )

        self.base_dir = full_path
        self.get_logger().info(f"[OBU] Built IMAGE folder from /test_number: {self.base_dir}")

        # If RSU already requested an image, send it now
        if self.pending_image_request_id is not None:
            img_id = self.pending_image_request_id
            self.get_logger().info(
                f"[OBU] Pending RSU request {img_id} detected after /test_number; starting send."
            )
            try:
                self._send_folder_all(img_id)
                self.feedback_sent_for.add(img_id)
            except Exception as e:
                self.get_logger().error(f"[OBU] send_folder_all failed after /test_number: {e}")
            self.pending_image_request_id = None

    # ---------------- WSMP bridge ----------------
    def _wsmp_rx_callback(self, _if, _hdr, buffer: bytes):
        self.wsmp_in.put(buffer)

    def _process_wsmp_queue(self):
        drained = 0
        limit = self.wsmp_drain_limit
        while drained < limit:
            try:
                buf = self.wsmp_in.get_nowait()
            except Empty:
                break
            drained += 1
            self._handle_rsu_message(buf)

    # ---------------- NAV / FAC handlers ----------------
    def _ros_time_ms(self) -> int:
        """Return current ROS2 time in milliseconds (used for all timestamps)."""
        now = self.get_clock().now()
        return int(now.nanoseconds / 1_000_000)

    def _on_nav(self, nav) -> None:
        """Handle OBU's own NAV data (GNSS). Log to TXT (CSV-style) and publish /obu/nav."""
        try:
            recv_ms = self._ros_time_ms()

            nav_lat = float(getattr(nav, "latitude", 0)) / 10e6
            nav_lon = float(getattr(nav, "longitude", 0)) / 10e6
            nav_alt = float(getattr(nav, "altitude", 0)) / 10e3
            nav_heading = float(getattr(nav, "heading", 0))
            nav_speed = float(getattr(nav, "speed", 0)) / 1000
            nav_sats = float(getattr(nav, "number_of_used_satellites", 0))

            self.obu_lat = nav_lat
            self.obu_lon = nav_lon
            self.obu_alt = nav_alt
            self.obu_heading = nav_heading
            self.obu_speed = nav_speed
            self.obu_sats = nav_sats

            # Use NAV message's own timestamp when available; fall back to recv_ms
            nav_gen_time = int(getattr(nav, "timestamp", recv_ms))

            self._csv(
                self.obu_nav_log,
                [
                    "nav_recv_time_ms",
                    "nav_gen_time_ms",
                    "latitude",
                    "longitude",
                    "altitude",
                    "heading",
                    "speed",
                    "satellites",
                ],
                [
                    nav_gen_time,
                    recv_ms,
                    nav_lat,
                    nav_lon,
                    nav_alt,
                    nav_heading,
                    nav_speed,
                    nav_sats,
                ],
            )

            msg = Float64MultiArray()
            msg.data = [
                float(nav_gen_time),
                float(recv_ms),
                nav_lat,
                nav_lon,
                nav_alt,
                nav_heading,
                nav_speed,
                nav_sats,
            ]
            self.pub_obu_nav.publish(msg)

        except Exception as e:
            self.get_logger().error(f"NAV callback error: {e}")

    def _on_fac(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        """Handle FAC US_BSM messages: log compact fields + debug coreData."""
        try:
            recv_ms = self._ros_time_ms()
            decoded = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
            core = decoded["value"][1]["coreData"]

            hearing_id = core["id"].hex()
            msg_cnt = core["msgCnt"]
            sec_mark = core["secMark"]
            fac_lat = core["lat"] / 10e6
            fac_lon = core["long"] / 10e6
            elev = core["elev"] / 10e3
            speed = core["speed"] / 50
            heading = core["heading"]
            angle = core["angle"]

            fac_gen = generate_fac_time(recv_ms, sec_mark)

            self._csv(
                self.obu_fac_log,
                [
                    "OBU_ID",
                    "msgCnt",
                    "secMark",
                    "latitude",
                    "longitude",
                    "elevation",
                    "speed",
                    "heading",
                    "angle",
                    "fac_gen_time_ms",
                    "fac_recv_time_ms",
                ],
                [
                    hearing_id,
                    msg_cnt,
                    sec_mark,
                    fac_lat,
                    fac_lon,
                    elev,
                    speed,
                    heading,
                    angle,
                    fac_gen,
                    recv_ms,
                ],
            )

            # debug full coreData (raw JSON per line)
            try:
                debug_entry = {
                    "fac_recv_time_ms": recv_ms,
                    "fac_core": core,
                }
                with open(self.fac_debug_file, "a") as f:
                    f.write(json.dumps(debug_entry, default=str) + "\n")
            except Exception as e:
                self.get_logger().error(f"FAC debug logging failed: {e}")

        except Exception as e:
            self.get_logger().error(f"FAC callback error: {e}")

    # ---------------- message handlers ----------------
    def _handle_rsu_message(self, buffer: bytes):
        """Handle RSU→OBU WSMP payloads: GPS updates, meta_only/send_complete, and chunked RSU images."""
        try:
            msg = json.loads(buffer.decode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"WSMP decode error: {e}")
            return

        # --- 1) update GPS from message that carries coords ---
        try:
            lat_val = msg.get("gps_lat", msg.get("lat"))
            lon_val = msg.get("gps_lon", msg.get("lon"))

            if lat_val is not None and lon_val is not None:
                self.gps_lat = float(lat_val)
                self.gps_lon = float(lon_val)
                self.get_logger().info(
                    f"[OBU] Updated RSU GPS to lat={self.gps_lat}, lon={self.gps_lon}"
                )
        except Exception as e:
            self.get_logger().error(f"[OBU] Failed to parse GPS from RSU message: {e}")

        mtype = msg.get("type", "")

        # --- 2) meta_only / send_complete: start of new RSU→OBU→RSU cycle ---
        if mtype in ("meta_only", "send_complete"):
            image_id = msg.get("image_id", f"img_{self.ts()}")

            # log RSU meta/send_complete in obu_received.txt
            self._csv(
                self.recv_log,
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                [
                    image_id,
                    mtype.upper(),
                    "",
                    self.ts(),
                    "",
                    msg.get("send_time", ""),
                    msg.get("crack_info", ""),
                ],
            )

            # Only respond once per image_id
            if image_id in self.feedback_sent_for:
                self.get_logger().info(
                    f"[OBU] feedback already sent for {image_id}, ignoring duplicate {mtype}"
                )
                return

            # Send confirmation immediately
            try:
                self._send_confirmation(image_id)
            except Exception as e:
                self.get_logger().error(f"[OBU] confirm failed: {e}")

            # If folder is known, send one image now; otherwise remember request and wait
            if self.base_dir:
                try:
                    self._send_folder_all(image_id)
                    self.feedback_sent_for.add(image_id)
                except Exception as e:
                    self.get_logger().error(f"[OBU] send_folder_all failed: {e}")
            else:
                self.pending_image_request_id = image_id
                self.get_logger().warning(
                    "[OBU] Folder not set yet; will send feedback once /test_number or /obu/base_dir arrives."
                )

        # --- 3) Chunked RSU image (RSU→OBU) ---
        elif "image_id" in msg and "chunk_number" in msg and "data" in msg:
            # Log raw chunk reception
            self._csv(
                self.recv_log,
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                [
                    msg.get("image_id", ""),
                    msg.get("chunk_number", ""),
                    msg.get("total_chunks", ""),
                    self.ts(),  # ROS time
                    msg.get("file_type", ""),
                    msg.get("send_time", ""),
                    msg.get("crack_info", ""),
                ],
            )

            # Buffer chunks and reconstruct RSU image when complete
            image_id = msg.get("image_id", "")
            chunk_no = msg.get("chunk_number")
            total = msg.get("total_chunks")
            data = msg.get("data")
            file_type = msg.get("file_type", "png")

            if image_id and chunk_no is not None and total is not None and data is not None:
                try:
                    chunk_no = int(chunk_no)
                    total = int(total)
                except ValueError:
                    return

                self.rsu_file_types.setdefault(image_id, file_type)
                self.rsu_chunk_totals.setdefault(image_id, total)
                self.rsu_chunks[image_id][chunk_no] = data

                # If we have all chunks for this image_id → reconstruct file
                if len(self.rsu_chunks[image_id]) == total and total > 0:
                    try:
                        # NOTE: RSU chunk_number is assumed 1..total
                        joined_b64 = "".join(self.rsu_chunks[image_id][i] for i in range(1, total + 1))
                    except KeyError:
                        # Missing chunk in sequence
                        return

                    parts = image_id.split("_")
                    cycle = parts[1] if len(parts) > 1 else "X"
                    test_dir = os.path.join(self.rsu_image_dir, f"Test{cycle}")
                    os.makedirs(test_dir, exist_ok=True)

                    ts_str = str(self.ts())
                    ext = "png" if file_type == "png" else "txt"
                    filename = f"rsu_{image_id}_{ts_str}.{ext}"
                    fpath = os.path.join(test_dir, filename)

                    try:
                        if file_type == "png":
                            with open(fpath, "wb") as f:
                                f.write(base64.b64decode(joined_b64.encode("utf-8")))
                        else:
                            with open(fpath, "w", encoding="utf-8") as f:
                                f.write(base64.b64decode(joined_b64.encode("utf-8")).decode("utf-8"))

                        self.get_logger().info(
                            f"[OBU] Saved RSU {file_type} for {image_id} to {fpath}"
                        )

                        # completion marker in obu_received.txt
                        self._csv(
                            self.recv_log,
                            ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                            [
                                image_id,
                                "IMAGE_RECV_COMPLETE",
                                total,
                                self.ts(),
                                file_type,
                                msg.get("send_time", ""),
                                msg.get("crack_info", ""),
                            ],
                        )
                    except Exception as e:
                        self.get_logger().error(
                            f"[OBU] Failed to reconstruct/save RSU image for {image_id}: {e}"
                        )

                    # Clear buffers
                    self.rsu_chunks[image_id].clear()
                    self.rsu_chunk_totals.pop(image_id, None)
                    self.rsu_file_types.pop(image_id, None)

                    # Fallback: if send_complete/meta_only somehow never arrived, still send feedback once
                    if image_id not in self.feedback_sent_for:
                        self.get_logger().info(
                            f"[OBU] All RSU chunks received for {image_id}, triggering feedback (fallback)."
                        )
                        try:
                            self._send_confirmation(image_id)
                        except Exception as e:
                            self.get_logger().error(f"[OBU] confirm failed in fallback: {e}")

                        if self.base_dir:
                            try:
                                self._send_folder_all(image_id)
                                self.feedback_sent_for.add(image_id)
                            except Exception as e:
                                self.get_logger().error(f"[OBU] send_folder_all failed in fallback: {e}")
                        else:
                            self.pending_image_request_id = image_id
                            self.get_logger().warning(
                                "[OBU] Folder not set yet (fallback); will send feedback once /test_number or /obu/base_dir arrives."
                            )

        # --- 4) Anything else ---
        else:
            self._csv(
                self.recv_log,
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                ["", "", "", self.ts(), "unknown", "", msg.get("crack_info", "")],
            )

    # ---------------- sending helpers ----------------
    def _send_confirmation(self, image_id: str):
        payload = {"image_received": True, "image_id": image_id}
        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.psid_confirm),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.psid_confirm)),
        )
        self.api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))
        self.get_logger().info(f"Sent confirmation for {image_id}")

    def _send_one_image(self, image_id: str, fp: str, send_data: WsmpSendData):
        """Send one PNG image to RSU as chunks, and log send_time in ROS ms (no path column)."""
        if not wait_for_file(fp, timeout=self.file_wait_timeout):
            self.get_logger().error(f"Skip after file timeout: {fp}")
            return
        data = b64_of_file(fp)
        chunks = [data[i:i + self.chunk_size] for i in range(0, len(data), self.chunk_size)]
        for idx, ch in enumerate(chunks):
            payload = {
                "image_id": image_id,
                "chunk_number": idx,
                "total_chunks": len(chunks),
                "data": ch,
                "file_type": "png"
            }
            self.api.wsmp_send(send_data, buffer=json.dumps(payload).encode("utf-8"))

            send_time_ros = self.ts()  # ROS time
            self._csv(
                self.sent_log,
                ["image_id", "chunk_number", "total_chunks", "send_time", "file_type"],
                [os.path.basename(fp), idx, len(chunks), send_time_ros, "png"],
            )
            time.sleep(0.01)
        self.get_logger().info(f"Sent {os.path.basename(fp)} in {len(chunks)} chunks")

    def _send_folder_all(self, image_id: str):
        """
        For each RSU request (per cycle), send exactly ONE image from the folder.

        Offline mode (USE_SUBSCRIBED_FOLDER=false):
            - send the next unsent image from IMAGE_FOLDER (based on filename sort).
        Online mode:
            - watch subscribed folder and send the first new image within CYCLE_TIMEOUT_SEC.
        After sending one image (or none if unavailable), OBU sends:
            - feedback_complete
            - queue_complete
        """
        folder = self.base_dir
        if not folder:
            self.get_logger().warning("[OBU] No folder set, cannot send feedback image.")
            return

        if not wait_for_directory(folder, timeout=self.folder_wait_timeout):
            self.get_logger().error(f"[OBU] Folder does not exist: {folder}")
            return

        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.psid_feedback),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.psid_feedback)),
        )

        # -------- OFFLINE MODE: fixed IMAGE_FOLDER, next unsent file per cycle --------
        if not self.use_subscribed_folder:
            files = sorted(
                os.path.join(folder, f)
                for f in os.listdir(folder)
                if os.path.isfile(os.path.join(folder, f))
                and os.path.splitext(f)[1].lower() in [ext.lower() for ext in self.image_exts]
            )
            available = [fp for fp in files if fp not in self.offline_sent_files]

            if not available:
                self.get_logger().warning(f"[OBU] No new images in folder: {folder}")
            else:
                fp = available[0]
                self.offline_sent_files.add(fp)
                self.get_logger().info(f"[OBU] Offline mode: sending single image {fp}")
                self._send_one_image(image_id, fp, send_data)

        # -------- ONLINE MODE: watch folder and send first new file per cycle --------
        else:
            sent_files = self.online_sent_files
            start_wall = time.time()
            self.get_logger().info(
                f"[OBU] Online mode: watching {folder} up to {self.cycle_timeout:.1f}s for one new image"
            )

            while True:
                try:
                    all_files = sorted(
                        os.path.join(folder, f)
                        for f in os.listdir(folder)
                        if os.path.isfile(os.path.join(folder, f))
                        and os.path.splitext(f)[1].lower() in [ext.lower() for ext in self.image_exts]
                    )
                except FileNotFoundError:
                    self.get_logger().warning(f"[OBU] Folder disappeared: {folder}")
                    break

                new_files = [fp for fp in all_files if fp not in sent_files]

                if new_files:
                    fp = new_files[0]
                    self.get_logger().info(f"[OBU] Found new image in {folder}: {fp}")
                    self._send_one_image(image_id, fp, send_data)
                    sent_files.add(fp)
                    break

                elapsed = time.time() - start_wall
                if elapsed >= self.cycle_timeout:
                    self.get_logger().info(
                        f"[OBU] Reached cycle timeout ({elapsed:.1f}s); "
                        f"no new image found for this cycle."
                    )
                    break

                time.sleep(0.5)

        # -------- Send feedback_complete + queue_complete to close the cycle --------
        feedback_send_time = self.ts()

        done_feedback = {
            "type": "feedback_complete",
            "image_id": image_id,
            "feedback_send_time": feedback_send_time,
        }
        self.api.wsmp_send(send_data, buffer=json.dumps(done_feedback).encode("utf-8"))
        self.get_logger().info(f"[OBU] Sent feedback_complete for {image_id}")

        done_queue = {
            "type": "queue_complete",
            "image_id": image_id,
            "feedback_send_time": feedback_send_time,
        }
        self.api.wsmp_send(send_data, buffer=json.dumps(done_queue).encode("utf-8"))
        self.get_logger().info(f"[OBU] Sent queue_complete for {image_id}")

    # ---------------- util + naming + shutdown ----------------
    def ts(self) -> int:
        """Unified timestamp helper: returns ROS time in ms (used for all logs)."""
        return self._ros_time_ms()

    def _csv(self, path: str, headers: List[str], row: List):
        """
        Append one structured row to a TXT log.

        - If the file does not exist, write a single header row first.
        - Rows are written as comma-separated values so they can still
          be loaded easily with pandas/csv, even though the extension is .txt.
        """
        exists = os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(headers)
            w.writerow(row)

    def _load_params(self, path: str) -> dict:
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load params from {path}: {e}")
            return {}

    def _next_run_name(self) -> str:
        """
        Compute next per-run name under log_root:
        - Look for folders named 'obu_log_<YYYYMMDD>_<index>'
        - Next index = max(existing) + 1, else 1
        """
        date_tag = time.strftime("%Y%m%d")
        prefix = f"obu_log_{date_tag}_"
        existing_indices: List[int] = []

        try:
            for name in os.listdir(self.log_root):
                if not name.startswith(prefix):
                    continue
                # format: obu_log_<date>_<idx>
                parts = name.split("_")
                if len(parts) < 4:
                    continue
                try:
                    idx = int(parts[-1])
                    existing_indices.append(idx)
                except ValueError:
                    continue
        except FileNotFoundError:
            pass

        next_idx = (max(existing_indices) + 1) if existing_indices else 1
        return f"obu_log_{date_tag}_{next_idx}"

    def destroy_node(self):
        """Cleanly close CMS API before destroying the node."""
        try:
            if hasattr(self, "api") and self.api is not None:
                try:
                    self.api.__exit__(None, None, None)
                    self.get_logger().info("CMS API closed cleanly.")
                except Exception as e:
                    self.get_logger().error(f"Error while closing CMS API: {e}")
        finally:
            super().destroy_node()


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    rclpy.init()
    node = OBUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down OBU node...")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
