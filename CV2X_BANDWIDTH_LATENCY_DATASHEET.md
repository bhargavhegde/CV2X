# C-V2X PC5 Link Performance Datasheet
## Bandwidth & Latency Test — 8 Mbps Target

| Field            | Value                                  |
|------------------|----------------------------------------|
| **Project**      | Real-Time V2X Perception Pipeline      |
| **Test Date**    | February 24, 2026                      |
| **Author**       | Bhargav Hegde                          |
| **Hardware**     | Commsignia ITS-OB4-C (×2)             |
| **Software**     | `bandwidth_latency_test.py` (custom)   |
| **Standard**     | 3GPP C-V2X PC5 (LTE-V2X Sidelink)    |
| **Frequency**    | 5.9 GHz ITS Band                       |

---

## 1. Hardware Under Test

### Commsignia ITS-OB4-C (C-V2X Version)

| Specification       | Value                                          |
|---------------------|------------------------------------------------|
| **Product**         | ITS-OB4-C On-Board Unit                       |
| **V2X Standard**    | C-V2X / 3GPP Rel-14 (LTE-V2X Mode 4)         |
| **V2X Chipset**     | Qualcomm® MDM9150 C-V2X                       |
| **CPU**             | 800 MHz NXP i.MX 6                            |
| **RAM**             | 2 GB DDR3 SDRAM                               |
| **Storage**         | 4 GB eMMC + Dual micro SD                     |
| **Operating System**| Linux                                         |
| **Ethernet**        | 10/100/1000 Mbps                              |
| **Wi-Fi**           | Dual-band 802.11a/b/g/n (Management)          |
| **GNSS**            | Advanced GNSS (GPS/GLONASS)                   |
| **V2X Frequency**   | 5.855 – 5.925 GHz (5.9 GHz ITS band)         |
| **Channel BW**      | 10 MHz / 20 MHz (configurable)                |
| **Temp. Range**     | −40 °C to +65 °C                             |
| **SDK**             | `pycmssdk` (Python SDK from Commsignia)       |

> **Datasheet Source:** [commsignia.com/products](https://www.commsignia.com/products/its-ob4/)
> Contact Commsignia directly at `info@commsignia.com` for the full official PDF datasheet.

---

## 2. C-V2X PC5 Technology Overview

### What is C-V2X PC5?

C-V2X (Cellular Vehicle-to-Everything) running on the **PC5 interface** (also called Sidelink or Direct Communication Mode 4) is a **device-to-device** radio technology. Unlike regular cellular, it does **not** go through a cell tower. Packets travel directly from OBU to OBU through the air.

```
  OBU 1 (Sender)                        OBU 2 (Receiver)
  ┌───────────────┐   5.9 GHz PC5      ┌───────────────┐
  │ Python Script │ ─── Air Link ────▶ │ Python Script │
  │ pycmssdk API  │   (direct, no BS)  │ pycmssdk API  │
  │ WSMP/WAVE     │                    │ WSMP/WAVE     │
  │ V2X Stack     │                    │ V2X Stack     │
  │ Qualcomm 9150 │                    │ Qualcomm 9150 │
  └───────────────┘                    └───────────────┘
```

### Key Terminology

| Term         | Meaning                                                             |
|--------------|---------------------------------------------------------------------|
| **PC5**      | 3GPP name for the V2X sidelink (direct) interface                  |
| **Mode 4**   | Autonomous resource selection using semi-persistent scheduling (SPS)|
| **PSID**     | Provider Service Identifier – tags packets by application type      |
| **WSMP**     | Wave Short Message Protocol – lightweight V2X packet format         |
| **SPS**      | Semi-Persistent Scheduling – PC5 resource allocation mechanism      |
| **RSSI**     | Received Signal Strength Indicator (dBm) – measures signal quality  |

---

## 3. Theoretical Throughput Limits

| Channel BW | Modulation / MCS     | Peak PHY Rate | Practical App Rate  |
|-----------|----------------------|---------------|---------------------|
| 10 MHz    | QPSK ½ (MCS 3)       | ~7 Mbps       | ~5–6 Mbps           |
| 10 MHz    | QAM-16 (MCS 7)       | ~14 Mbps      | ~10–12 Mbps         |
| 20 MHz    | QAM-16 (MCS 9)       | ~27 Mbps      | ~20 Mbps            |
| 10 MHz    | **Target: 8 Mbps**   | ✅ Achievable  | **Tested here**      |

> The **8 Mbps** target sits between QPSK and QAM-16 at 10 MHz BW — this is a realistic
> operating point for the ITS-OB4-C under good RF conditions (short range, line-of-sight).

---

## 4. Test Methodology

### 4.1 Setup

```
  Laptop 1                              Laptop 2
  ├─ Connected to OBU 1 WiFi           ├─ Connected to OBU 2 WiFi
  │   (e.g., 192.168.0.125)            │   (e.g., 192.168.0.174)
  │                                    │
  └─ Runs: bandwidth_latency_test.py   └─ Runs: bandwidth_latency_test.py
           --mode send                          --mode listen
           --host 192.168.0.54                  --host 192.168.0.54
```

### 4.2 Test Script: `bandwidth_latency_test.py`

**Packet Structure (each WSMP payload):**

```
 ┌──────────────┬──────────────┬────────────────────────┐
 │  TX Timestamp│  Seq Number  │  Padded Data           │
 │  (8 bytes)   │  (4 bytes)   │  (1012 bytes)          │
 │  float64     │  uint32      │  0xAB × 1012           │
 └──────────────┴──────────────┴────────────────────────┘
 Total: 1024 bytes per packet
```

**Sender Rate Calculation:**

```
Target Bandwidth = 8 Mbps = 8,000,000 bits/sec
Packet Size      = 1024 bytes = 8192 bits
Packets/sec      = 8,000,000 / 8192 ≈ 976.6 pkt/s
Inter-pkt Gap    = 1 / 976.6 ≈ 1.02 ms
```

### 4.3 Commands

**Step 1: Start Receiver (OBU 2 side) first:**
```bash
python3 bandwidth_latency_test.py --mode listen --host 192.168.0.54
```

**Step 2: Start Sender (OBU 1 side):**
```bash
python3 bandwidth_latency_test.py --mode send --host 192.168.0.54
```

**Step 3: Stop receiver with Ctrl+C** → Final report prints automatically.

### 4.4 Metrics Measured

| Metric          | How Measured                                             |
|-----------------|----------------------------------------------------------|
| **Throughput**  | `(total bytes received × 8) / elapsed time`             |
| **Latency**     | `rx_timestamp − tx_timestamp` (embedded in payload)      |
| **Jitter**      | Standard deviation of latency samples                   |
| **Packet Loss** | Sequence number gaps detected on receiver               |

---

## 5. Expected Results (Reference Values from Literature)

These are the expected/target values based on C-V2X PC5 specifications and published benchmarks:

| Metric              | Expected / Target Value          | Standard Reference           |
|---------------------|----------------------------------|------------------------------|
| Throughput          | ≥ **7.2 Mbps** (90% of 8 Mbps)  | Test requirement             |
| Avg Latency         | **< 50 ms** (stationary lab)     | 3GPP TS 22.186               |
| Max Latency         | **< 100 ms**                     | ETSI TR 102 638              |
| Jitter (σ)          | **< 10 ms**                      | 5GAA performance benchmarks  |
| Packet Loss         | **< 1%**                         | V2X safety requirements      |
| RSSI (typical lab)  | **−40 to −65 dBm**               | Commsignia device behavior   |

---

## 6. Test Results (Fill in after running the test)

> **Instructions:** Run the test, press Ctrl+C on the receiver when the sender finishes.
> Copy the FINAL TEST RESULTS box output into the table below.

| Metric              | Measured Value | Pass/Fail |
|---------------------|----------------|-----------|
| Test Duration (s)   |                |           |
| Packets Sent        |                |           |
| Packets Received    |                |           |
| Packet Loss (%)     |                | ✅ / ❌   |
| Avg Throughput (Mbps)|               | ✅ / ❌   |
| Avg Latency (ms)    |                | ✅ / ❌   |
| Min Latency (ms)    |                |           |
| Max Latency (ms)    |                |           |
| Jitter σ (ms)       |                |           |
| RSSI (dBm)          |                |           |

### Pass/Fail Criteria
- ✅ **PASS** Throughput: Achieved ≥ 7.2 Mbps (≥90% of 8 Mbps target)
- ✅ **PASS** Latency:    Avg latency < 100 ms
- ✅ **PASS** Packet Loss: Lost < 1% of packets

---

## 7. Where to Find the Official Datasheet

| Resource                         | Link / Location                                           |
|----------------------------------|-----------------------------------------------------------|
| **Commsignia Product Page**      | https://www.commsignia.com/products/its-ob4/             |
| **Email (request PDF datasheet)**| info@commsignia.com                                       |
| **Qualcomm MDM9150 C-V2X Chipset**| https://www.qualcomm.com/products/technology/modems/cseries/9150 |
| **3GPP C-V2X Spec**              | 3GPP TS 36.321, TS 36.331, TS 22.186                     |
| **ETSI ITS Standards**           | https://www.etsi.org/committee/its                        |
| **5GAA White Papers**            | https://5gaa.org/news/whitepapers/                        |

---

## 8. Comparison: C-V2X vs DSRC (for context)

| Property           | C-V2X PC5             | DSRC (802.11p)         |
|--------------------|-----------------------|------------------------|
| Standard           | 3GPP Rel-14+          | IEEE 802.11p / WAVE    |
| Frequency          | 5.9 GHz               | 5.9 GHz                |
| Channel BW         | 10–20 MHz             | 10 MHz                 |
| Max PHY Rate       | 27 Mbps               | 27 Mbps                |
| Typical App Rate   | 8–15 Mbps             | 6–12 Mbps              |
| Latency            | ~25–50 ms             | ~20–40 ms              |
| Network Infra      | Not needed (Mode 4)   | Not needed             |
| Range (open road)  | 300–500 m             | 200–350 m              |
| Coverage (density) | Better in high density| Better in low density  |

---

*This document is part of the V2X Perception Pipeline project.*
*Generated: February 24, 2026 | Commsignia ITS-OB4-C hardware*
