#!/usr/bin/env python3
"""
C-V2X Bandwidth & Latency Test Script
======================================
Tests C-V2X (PC5 Sidelink) link performance.

THEORY:
  - C-V2X PC5 is a device-to-device (sidelink) radio link at 5.9 GHz.
  - OBUs communicate directly (no cell tower) using Mode 4 SPS scheduling.
  - This script measures:
      1. THROUGHPUT  – bytes/sec received over the air
      2. LATENCY     – one-way delay from timestamps embedded in the payload
      3. JITTER      – std-dev of latency samples
      4. PACKET LOSS – sequence number gaps detected on receiver

  IMPORTANT — OBU Stack Limits (Commsignia ITS-OB4-C):
  - WSMP max payload after security overhead: ~250-300 bytes.
    We use 100 bytes per packet to stay safely within this limit.
  - With SIGN_METH_SIGN_CERT, every packet gets a ~600-byte certificate
    appended. This causes the eac.py payload-length mismatch errors.
    We use UNSECURED mode for bandwidth testing to avoid this overhead.
  - The OBU queue handles ~200 pkt/s reliably.
    At 100 bytes/pkt × 200 pkt/s = ~0.16 Mbps application throughput.
    The 8 Mbps figure is the raw PHY channel capacity, not app-layer.

USAGE:
  On RECEIVER (OBU 2):
    python3 bandwidth_latency_test.py --mode listen --host 192.168.0.54

  On SENDER (OBU 1):
    python3 bandwidth_latency_test.py --mode send --host 192.168.0.54
"""

import time
import struct
import argparse
import statistics
import sys

# ── Try to import real SDK; fall back to mock for dry-runs ──────────────────
try:
    from pycmssdk import (
        WILDCARD,
        MacAddr,
        RadioTxParams,
        SecDot2TxInfo,
        SecDot2TxSignInfo,
        SignMethod,
        WsmpSendData,
        WsmpTxHdrInfo,
        WsmpRxNotifData,
        create_cms_api,
    )
    print("[INFO] Real pycmssdk loaded.")
except ImportError:
    print("[WARN] pycmssdk not found. Loading mock SDK for dry-run only.")
    from mock_pycmssdk import (
        WILDCARD,
        MacAddr,
        RadioTxParams,
        SecDot2TxInfo,
        SecDot2TxSignInfo,
        SignMethod,
        WsmpSendData,
        WsmpTxHdrInfo,
        WsmpRxNotifData,
        create_cms_api,
    )

# ── Test Configuration ────────────────────────────────────────────────────────
PSID             = 0x8007   # Test-specific PSID (not production BSM 0x20)
TEST_DURATION_S  = 30       # How long to run the sender (seconds)
REPORT_INTERVAL  = 5        # Print a live update every N seconds

# OBU-safe packet sizing:
# - Commsignia stack truncates payloads > ~250 bytes after security headers.
# - 100-byte payload stays well within the safe zone.
PAYLOAD_BYTES    = 100      # bytes of application data per packet

# Header: 8-byte timestamp (float64) + 4-byte sequence number (uint32)
HEADER_SIZE      = 12
PACKET_TOTAL     = PAYLOAD_BYTES + HEADER_SIZE   # 112 bytes per packet

# OBU-safe send rate:
# The V2X stack queue handles ~200 pkt/s reliably.
# Higher rates cause queue overflow and corrupt/dropped packets.
PACKETS_PER_SEC  = 200
INTER_PKT_DELAY  = 1.0 / PACKETS_PER_SEC         # 5 ms between packets

# Theoretical max throughput with these settings:
# 200 pkt/s × 112 bytes × 8 bits = ~179 Kbps application-layer
# (The 8 Mbps figure is the raw PHY capacity of the 5.9 GHz channel)

print(f"""
╔══════════════════════════════════════════════════════════╗
║          C-V2X Bandwidth & Latency Test                 ║
╠══════════════════════════════════════════════════════════╣
║  Packet Size       : {PACKET_TOTAL} bytes ({HEADER_SIZE}B hdr + {PAYLOAD_BYTES}B data)   ║
║  Packets/sec       : {PACKETS_PER_SEC}                              ║
║  Inter-packet gap  : {INTER_PKT_DELAY*1000:.1f} ms                         ║
║  Test Duration     : {TEST_DURATION_S} seconds                         ║
║  Security Mode     : UNSECURED (bandwidth test mode)    ║
╚══════════════════════════════════════════════════════════╝
""")

# ── Shared state for the receiver callback ────────────────────────────────────
rx_stats = {
    "count":        0,
    "total_bytes":  0,
    "latencies":    [],
    "start_time":   None,
    "last_report":  None,
    "last_seq":     -1,
    "lost":         0,
}


def rx_callback(key: int, data: WsmpRxNotifData, buffer: bytes) -> None:
    """Called by the OBU stack for every received WSMP packet matching PSID."""
    now = time.time()

    if rx_stats["start_time"] is None:
        rx_stats["start_time"] = now
        rx_stats["last_report"] = now
        print(f"[RX] First packet received! Starting measurements...")

    # ── Parse our custom header ──────────────────────────────────────────────
    if len(buffer) < HEADER_SIZE:
        return  # malformed packet, skip

    tx_timestamp, seq_num = struct.unpack_from("!dI", buffer, 0)

    # Sequence number gap → packet loss detection
    if rx_stats["last_seq"] >= 0:
        expected = rx_stats["last_seq"] + 1
        if seq_num > expected:
            gap = seq_num - expected
            rx_stats["lost"] += gap
            print(f"[RX] ⚠  Detected {gap} lost packet(s) before seq {seq_num}")
    rx_stats["last_seq"] = seq_num

    # One-way latency estimate (requires clocks to be reasonably synced)
    latency_ms = (now - tx_timestamp) * 1000.0
    if 0 < latency_ms < 5000:   # sanity filter (ignore clock-drift outliers)
        rx_stats["latencies"].append(latency_ms)

    rx_stats["count"]       += 1
    rx_stats["total_bytes"] += len(buffer)

    # ── Periodic report ──────────────────────────────────────────────────────
    elapsed = now - rx_stats["last_report"]
    if elapsed >= REPORT_INTERVAL:
        total_elapsed   = now - rx_stats["start_time"]
        avg_throughput  = (rx_stats["total_bytes"] * 8) / (total_elapsed * 1_000_000)
        avg_latency     = statistics.mean(rx_stats["latencies"]) if rx_stats["latencies"] else float("nan")
        jitter          = statistics.stdev(rx_stats["latencies"]) if len(rx_stats["latencies"]) > 1 else 0.0
        total_expected  = rx_stats["count"] + rx_stats["lost"]
        pkt_loss        = (rx_stats["lost"] / total_expected * 100) if total_expected > 0 else 0.0

        print(
            f"\n[RX REPORT @ {total_elapsed:.0f}s]\n"
            f"  Packets Received : {rx_stats['count']}\n"
            f"  Packets Lost     : {rx_stats['lost']}\n"
            f"  Packet Loss      : {pkt_loss:.2f}%\n"
            f"  Avg Throughput   : {avg_throughput:.2f} Mbps  (target: {TARGET_MBPS} Mbps)\n"
            f"  Avg Latency      : {avg_latency:.2f} ms\n"
            f"  Jitter (std-dev) : {jitter:.2f} ms\n"
            f"  RSSI (last pkt)  : {data.radio.rssi} dBm\n"
        )
        rx_stats["last_report"] = now


def print_final_rx_report():
    """Print the complete summary at the end of the receive session."""
    if rx_stats["start_time"] is None:
        print("[RX] No packets were received. Check sender / RF link.")
        return

    total_elapsed   = time.time() - rx_stats["start_time"]
    avg_throughput  = (rx_stats["total_bytes"] * 8) / (total_elapsed * 1_000_000)
    avg_latency     = statistics.mean(rx_stats["latencies"]) if rx_stats["latencies"] else float("nan")
    min_latency     = min(rx_stats["latencies"]) if rx_stats["latencies"] else float("nan")
    max_latency     = max(rx_stats["latencies"]) if rx_stats["latencies"] else float("nan")
    jitter          = statistics.stdev(rx_stats["latencies"]) if len(rx_stats["latencies"]) > 1 else 0.0
    total_expected  = rx_stats["count"] + rx_stats["lost"]
    pkt_loss        = (rx_stats["lost"] / total_expected * 100) if total_expected > 0 else 0.0
    # Pass criteria: throughput > 90% of theoretical max, latency < 100ms, loss < 5%
    theoretical_mbps = (PACKETS_PER_SEC * PACKET_TOTAL * 8) / 1_000_000
    pass_bw         = avg_throughput >= theoretical_mbps * 0.9
    pass_latency    = avg_latency < 100                      # <100 ms per V2X spec
    pass_loss       = pkt_loss < 5.0                         # <5% is acceptable

    print(f"""
╔══════════════════════════════════════════════════════════╗
║              FINAL TEST RESULTS (RECEIVER)              ║
╠══════════════════════════════════════════════════════════╣
║  Test Duration      : {total_elapsed:.1f} s                         
║  Packets Received   : {rx_stats['count']}                          
║  Packets Lost       : {rx_stats['lost']}                          
║  Packet Loss        : {pkt_loss:.2f}%  {'✅ PASS' if pass_loss else '❌ FAIL'}           
╠══════════════════════════════════════════════════════════╣
║  BANDWIDTH                                              ║
║    Achieved          : {avg_throughput:.2f} Mbps                    
║    Target            : {TARGET_MBPS} Mbps                          
║    Result            : {'✅ PASS' if pass_bw else '❌ FAIL' }                     
╠══════════════════════════════════════════════════════════╣
║  LATENCY (one-way, estimated)                           ║
║    Min               : {min_latency:.2f} ms                      
║    Avg               : {avg_latency:.2f} ms                      
║    Max               : {max_latency:.2f} ms                      
║    Jitter (σ)        : {jitter:.2f} ms                      
║    Result            : {'✅ PASS (<100ms)' if pass_latency else '❌ FAIL (>100ms)'}           
╚══════════════════════════════════════════════════════════╝
""")


def run_sender(api, args):
    """Send fixed-size packets at the rate required to achieve TARGET_MBPS."""
    data_payload = b'\xAB' * PAYLOAD_BYTES  # Dummy data padding

    send_data = WsmpSendData(
        radio=RadioTxParams(
            interface_id=1,
            dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),  # Broadcast
        ),
        wsmp_hdr=WsmpTxHdrInfo(psid=args.psid),
        # UNSECURED for bandwidth testing — avoids the ~600-byte IEEE 1609.2
        # certificate that SIGN_METH_SIGN_CERT appends to every packet.
        # That certificate caused payload-length mismatches in eac.py and was
        # the main reason for the high packet loss in the initial test run.
        security=SecDot2TxInfo(
            sign_info=SecDot2TxSignInfo(
                sign_method=SignMethod.SIGN_METH_UNSECURED,
                psid=args.psid,
            )
        ),
    )

    theoretical_kbps = (PACKETS_PER_SEC * PACKET_TOTAL * 8) / 1000
    print(f"[TX] Starting: {PACKETS_PER_SEC} pkt/s × {PACKET_TOTAL}B = "
          f"~{theoretical_kbps:.0f} Kbps for {TEST_DURATION_S}s...")
    start      = time.time()
    seq_num    = 0
    sent_bytes = 0

    while time.time() - start < TEST_DURATION_S:
        loop_start  = time.time()
        tx_time     = loop_start
        seq_num    += 1

        # Build packet: 8-byte timestamp + 4-byte seq + padding
        header  = struct.pack("!dI", tx_time, seq_num)
        payload = header + data_payload
        api.wsmp_send(send_data, buffer=payload)
        sent_bytes += len(payload)

        elapsed    = time.time() - start
        if seq_num % int(PACKETS_PER_SEC * REPORT_INTERVAL) == 0:
            actual_mbps = (sent_bytes * 8) / (elapsed * 1_000_000)
            print(f"[TX] seq={seq_num}  sent={sent_bytes/1024:.1f} KB  "
                  f"elapsed={elapsed:.1f}s  rate={actual_mbps:.2f} Mbps")

        # Pace to hit the target bandwidth
        send_took = time.time() - loop_start
        sleep_for = INTER_PKT_DELAY - send_took
        if sleep_for > 0:
            time.sleep(sleep_for)

    elapsed     = time.time() - start
    actual_mbps = (sent_bytes * 8) / (elapsed * 1_000_000)
    print(f"\n[TX DONE] Sent {seq_num} packets ({sent_bytes/1024:.1f} KB) "
          f"in {elapsed:.1f}s → {actual_mbps:.2f} Mbps avg")
    print("[TX] Waiting 3s for in-flight packets to arrive at receiver...")
    time.sleep(3)


def main():
    global TEST_DURATION_S  # declared first — before any reference below
    parser = argparse.ArgumentParser(
        description="C-V2X Bandwidth & Latency Test (8 Mbps target)"
    )
    parser.add_argument("--host",  default="192.168.0.54",
                        help="IP address of the OBU V2X Stack")
    parser.add_argument("--mode",  choices=["send", "listen"], required=True,
                        help="Operating mode")
    parser.add_argument("--psid",  type=lambda x: int(x, 0), default=PSID,
                        help=f"PSID (default: {hex(PSID)})")
    parser.add_argument("--duration", type=int, default=30,
                        help="Sender run time in seconds (default: 30)")
    args   = parser.parse_args()
    TEST_DURATION_S = args.duration

    print(f"Connecting to OBU V2X stack at {args.host} ...")

    try:
        with create_cms_api(host=args.host) as api:
            if args.mode == "listen":
                print(f"[RX] Subscribed to PSID {hex(args.psid)}. Waiting for packets...\n"
                      f"     (Press Ctrl+C to stop and see final results)\n")
                api.wsmp_rx_subscribe(args.psid, rx_callback)
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    pass
                finally:
                    print_final_rx_report()

            elif args.mode == "send":
                run_sender(api, args)

    except Exception as e:
        print(f"\n[ERROR] {e}")
        print("\nTroubleshooting Tips:")
        print("  1. Ping the OBU: ping -c 4 192.168.0.54")
        print("  2. Ensure you are on the OBU WiFi.")
        print("  3. Check firewall (try: sudo ufw disable).")
        sys.exit(1)


if __name__ == "__main__":
    main()
