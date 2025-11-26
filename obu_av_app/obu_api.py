#!/usr/bin/env python3

"""
OBU API Node

1) RSU → OBU: receive lat/lon and publish on /obu/gps.
2) OBU → RSU: when RSU asks (meta_only / send_complete), send images from folder
   as chunks and then send queue_complete.
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
   - All timestamps (send_time, recv_time, nav_gen/recv, fac_gen/recv) use ROS time (ms),
     except NAV generation time which uses nav.timestamp when available.

     For Param file need to update the Test_base_root to vehicle directory 
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
        params_path = os.path.join(os.path.dirname(__file__), "obu_parameters.json")# will change to relative path on vehicle
        self.declare_parameter("params_path", params_path)

        params_path = self.get_parameter("params_path").get_parameter_value().string_value
        self.params = self._load_params(params_path)

        # Base folder where params live – use this to anchor logs so paths are stable
        self.params_dir = os.path.dirname(os.path.abspath(params_path))

        # Core WSMP / pipeline params
        self.v2x_stack_ip   = self.params.get("V2X_STACK_IP")
        self.rx_psid        = self.params.get("RECEIVE_IMAGE_PSID")
        self.psid_confirm   = self.params.get("SEND_CONFIRM_PSID")
        self.psid_feedback  = self.params.get("SEND_FEEDBACK_PSID")
        self.psid_ready     = self.params.get("SEND_READY_PSID")

        self.chunk_size     = int(self.params.get("CHUNK_SIZE"))
        self.ready_interval = float(self.params.get("READY_INTERVAL"))
        self.cycle_timeout  = float(self.params.get("CYCLE_TIMEOUT_SEC"))
        self.declare_parameter("wsmp_drain_limit", 50)
        self.wsmp_drain_limit = int(self.get_parameter("wsmp_drain_limit").value)

        # waits + filters
        self.folder_wait_timeout = self.params.get("FOLDER_WAIT_TIMEOUT", None)
        self.file_wait_timeout   = self.params.get("FILE_WAIT_TIMEOUT", None)
        self.image_exts: List[str] = list(self.params.get("IMAGE_EXTS", [".png", ".jpg", ".jpeg"]))

        # ROS topics for GPS output + base_dir/test_number
        self.gps_topic         = self.params.get("GPS_TOPIC", "/obu/gps")
        self.test_number_topic = self.params.get("TEST_NUMBER_TOPIC", "/test_number")
        self.base_dir_topic    = self.params.get("BASE_DIR_TOPIC", "/obu/base_dir")

        # test folders for reading feedback images (if using subscribed folders)
        self.test_base_root        = self.params.get("TEST_BASE_ROOT", "")
        self.use_subscribed_folder = bool(self.params.get("USE_SUBSCRIBED_FOLDER", False))

        # ----- logging root + per-run folder -----
        # Stable root under the params directory
        log_root_name = self.params.get("OBU_LOG_DIR", "obu_logs")
        self.log_root = os.path.join(self.params_dir, log_root_name)
        os.makedirs(self.log_root, exist_ok=True)

        # Per-run directory name: obu_log_YYYYMMDD_index (robust index calculation)
        self.run_dir = os.path.join(self.log_root, self._next_run_name())
        os.makedirs(self.run_dir, exist_ok=True)

        # For convenience, use run_dir as log_dir
        self.log_dir = self.run_dir

        # Per-run structured log files (TXT with header, comma-separated)
        self.sent_log     = os.path.join(self.log_dir, "obu_sent.txt")
        self.recv_log     = os.path.join(self.log_dir, "obu_received.txt")
        self.obu_nav_log  = os.path.join(self.log_dir, "obu_nav.txt")
        self.obu_fac_log  = os.path.join(self.log_dir, "obu_fac.txt")

        # FAC debug folder inside this run directory
        self.nav_fac_debug_dir = os.path.join(self.log_dir, "nav_fac_debug")
        os.makedirs(self.nav_fac_debug_dir, exist_ok=True)
        self.fac_debug_file = os.path.join(self.nav_fac_debug_dir, "fac_raw.txt")

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
        if not self.use_subscribed_folder:
            self.get_logger().debug("[OBU] ignoring /obu/base_dir because USE_SUBSCRIBED_FOLDER is false")
            return
        path = msg.data.strip()
        if path:
            self.base_dir = path
            self.get_logger().info(f"[OBU] IMAGE folder updated via /obu/base_dir: {self.base_dir}")

    def _on_test_number(self, msg: String):
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
            "detection_folder",   # or "detection_result" – must match vehicle code
            "mask",
        )

        self.base_dir = full_path
        self.get_logger().info(f"[OBU] Built IMAGE folder from /test_number: {self.base_dir}")

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
                    "nav_gen_time_ms",
                    "nav_recv_time_ms",
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
        try:
            msg = json.loads(buffer.decode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"WSMP decode error: {e}")
            return

        # --- 1) update GPS from message that carries coords ---
        try:
            # Support both styles: gps_lat/gps_lon OR lat/lon
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

        # --- 2) Now handle message type as before (NO early return!!) ---
        mtype = msg.get("type", "")

        if mtype in ("meta_only", "send_complete"):
            # Meta-only / send_complete receive log – recv_time is ROS time
            self._csv(
                self.recv_log,
                ["image_id", "chunk_number", "total_chunks", "recv_time", "file_type", "rsu_send_time", "crack_info"],
                [
                    msg.get("image_id", ""),
                    mtype.upper(),
                    "",
                    self.ts(),  # ROS time
                    "",
                    msg.get("send_time", ""),
                    msg.get("crack_info", ""),
                ],
            )

            try:
                self._send_confirmation(msg.get("image_id", f"img_{self.ts()}"))
            except Exception as e:
                self.get_logger().error(f"confirm failed: {e}")

            if self.base_dir:
                try:
                    self._send_folder_all(msg.get("image_id", f"img_{self.ts()}"))
                except Exception as e:
                    self.get_logger().error(f"send_folder_all failed: {e}")
            else:
                self.get_logger().warning("Folder not set yet; (waiting for /test_number or /obu/base_dir).")

        elif "image_id" in msg and "chunk_number" in msg and "data" in msg:
            # Chunk receive log – recv_time is ROS time
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
                [image_id, idx, len(chunks), send_time_ros, "png"],
            )
            time.sleep(0.01)
        self.get_logger().info(f"Sent {os.path.basename(fp)} in {len(chunks)} chunks")

    def _send_folder_all(self, image_id: str):
        folder = self.base_dir
        if not folder:
            self.get_logger().warning("No folder set, cannot send.")
            return

        if not wait_for_directory(folder, timeout=self.folder_wait_timeout):
            return

        files = sorted(
            os.path.join(folder, f)
            for f in os.listdir(folder)
            if os.path.isfile(os.path.join(folder, f))
            and os.path.splitext(f)[1].lower() in [ext.lower() for ext in self.image_exts]
        )
        if not files:
            self.get_logger().warning(f"No images in folder: {folder}")
            return

        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF,) * 6, datarate=6, tx_power=20, expiry_time=1000),
            wsmp_hdr=WsmpTxHdrInfo(psid=self.psid_feedback),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(SignMethod.SIGN_METH_SIGN_CERT, self.psid_feedback)),
        )

        for fp in files:
            self._send_one_image(image_id, fp, send_data)

        done = {"type": "queue_complete", "image_id": image_id, "feedback_send_time": self.ts()}
        self.api.wsmp_send(send_data, buffer=json.dumps(done).encode("utf-8"))
        self.get_logger().info(f"Sent queue_complete for {image_id}")

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
