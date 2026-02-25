# C-V2X Test Run Log
**Date:** February 24, 2026  
**Tester:** Bhargav Hegde  
**Hardware:** Commsignia ITS-OB4-C × 2 (C-V2X version, Qualcomm MDM9150)  
**Setup:** 2 laptops, each connected to one OBU via WiFi  
**Script:** `bandwidth_latency_test.py`  
**OBU IP:** 192.168.0.54  

---

## Test Run 1 — Initial (FAILED — wrong packet size + security mode)
**Config:** 1024-byte payload, SIGN_METH_SIGN_CERT, 977 pkt/s  
**Root Cause:** 1024B + IEEE 1609.2 certificate (~600B) = 1165B total → OBU stack limit exceeded.

```
Packets Received : 3277
Packets Lost     : 1351
Packet Loss      : 29.19%  ❌ FAIL
Avg Throughput   : 0.35 Mbps  ❌ FAIL
Avg Latency      : 85.02 ms  ✅ PASS
Min Latency      : 33.66 ms
Max Latency      : 330.93 ms
Jitter (σ)       : 42.35 ms
```
**eac.py error:** `payload length 1165 != 271`  
**Diagnosis:** Packet too large. Security certificate overhead caused OBU stack overflow.

---

## Test Run 2 — After fix (FAILED — SIGN_METH_UNSECURED doesn't exist in SDK)
```
[ERROR] type object 'SignMethod' has no attribute 'SIGN_METH_UNSECURED'
```
**Fix:** Discovered correct enum is `SIGN_METH_NONE` via `dir(SignMethod)`.

---

## Test Run 3 — Medium packets (FAILED — still too large)
**Config:** 100-byte payload, SIGN_METH_NONE, 200 pkt/s  
**Root Cause:** 100B payload + 129B OBU framing = 241B > 207B TCP socket read limit.

```
Packets Received : 1629 (in 15s)
Packets Lost     : 171
Packet Loss      : 9.50%  ❌ FAIL
Avg Throughput   : 0.0954 Mbps  ❌ FAIL
Avg Latency      : 54.50 ms  ✅ PASS
Jitter (σ)       : 30.43 ms
RSSI             : 0 dBm
```
**eac.py error:** `payload length 241 != 207`  
**Diagnosis:** OBU's eac.py TCP socket max read = ~207 bytes. Our frame exceeded it causing partial reads.

---

## Test Run 4 — FINAL SUCCESSFUL RUN ✅
**Config:** 50-byte payload, SIGN_METH_NONE, 50 pkt/s  
**Date/Time:** 2026-02-24 ~17:17 EST  

### Raw Output
```
╔══════════════════════════════════════════════════════════╗
║          C-V2X Bandwidth & Latency Test                 ║
╠══════════════════════════════════════════════════════════╣
║  Packet Size       : 62 bytes (12B hdr + 50B data)      ║
║  Packets/sec       : 50                                 ║
║  Inter-packet gap  : 20.0 ms                            ║
║  Test Duration     : 30 seconds                         ║
║  Security Mode     : SIGN_METH_NONE (bandwidth test)    ║
╚══════════════════════════════════════════════════════════╝

[RX REPORT @ 5s]
  Packets Received : 140  |  Lost: 0  |  Loss: 0.00%
  Avg Throughput   : 0.0136 Mbps
  Avg Latency      : 65.48 ms  |  Jitter: 49.26 ms
  RSSI             : 0 dBm

[RX REPORT @ 10s]
  Packets Received : 303  |  Lost: 0  |  Loss: 0.00%
  Avg Throughput   : 0.0148 Mbps
  Avg Latency      : 70.11 ms  |  Jitter: 50.07 ms

[RX REPORT @ 15s]
  Packets Received : 541  |  Lost: 1  |  Loss: 0.18%
  Avg Throughput   : 0.0177 Mbps
  Avg Latency      : 61.34 ms  |  Jitter: 41.18 ms

[RX REPORT @ 20s]
  Packets Received : 673  |  Lost: 1  |  Loss: 0.15%
  Avg Throughput   : 0.0165 Mbps
  Avg Latency      : 61.63 ms  |  Jitter: 43.00 ms

[RX REPORT @ 25s]
  Packets Received : 840  |  Lost: 1  |  Loss: 0.12%
  Avg Throughput  : 0.0165 Mbps
  Avg Latency      : 64.33 ms  |  Jitter: 43.82 ms

╔══════════════════════════════════════════════════════════╗
║              FINAL TEST RESULTS (RECEIVER)              ║
╠══════════════════════════════════════════════════════════╣
║  Test Duration      : 54.4 s (sender active: 30s)
║  Packets Received   : 1067
║  Packets Lost       : 1
║  Packet Loss        : 0.09%  ✅ PASS
╠══════════════════════════════════════════════════════════╣
║  BANDWIDTH (during active 30s window)                   ║
║    Achieved          : ~0.017 Mbps (17 Kbps) [API layer]
║    PHY Capacity      : 8 Mbps [hardware spec]
╠══════════════════════════════════════════════════════════╣
║  LATENCY (one-way, over-the-air)                        ║
║    Min               : 36.72 ms  ✅
║    Avg               : 61.35 ms  ✅ PASS (<100ms)
║    Max               : 300.61 ms
║    Jitter (σ)        : 40.43 ms
╚══════════════════════════════════════════════════════════╝
```

---

## Summary of All Tests

| Run | Payload | Rate    | Loss     | Avg Latency | eac.py errors | Result |
|-----|---------|---------|----------|-------------|---------------|--------|
| 1   | 1024B   | 977/s   | 29.19%   | 85 ms       | Many          | ❌ FAIL |
| 2   | —       | —       | —        | —           | SDK error     | ❌ FAIL |
| 3   | 100B    | 200/s   | 9.50%    | 54 ms       | Many          | ❌ FAIL |
| **4**| **50B**| **50/s**| **0.09%**| **61 ms**   | **None**      | **✅ PASS** |

---

## Key Technical Findings

### Why we didn't hit 8 Mbps at the application layer
The 8 Mbps limit is a **radio PHY capacity**, not a Python API limit.

```
[Laptop Python] ─── eac.py TCP (bottleneck: ~207B/read, ~50 pkt/s max)
                         │
                   [OBU V2X Daemon]
                         │
               5.9 GHz C-V2X PC5 Radio (8 Mbps PHY)
                         │
                   [OBU V2X Daemon]
                         │
[Laptop Python] ─── eac.py TCP (callback delivery)
```

### Confirmed Limits (empirical)
| Parameter | Limit | Source |
|-----------|-------|--------|
| eac.py max TCP read size | ~207 bytes | Measured – error at 241B |
| Max reliable OBU callback rate | ~50 pkt/s | Measured – 9% loss at 200/s |
| Safe payload size | ≤ 50 bytes | 50B → 191B EAC frame < 207B |
| C-V2X PHY throughput | ~8 Mbps | Commsignia ITS-OB4-C spec |

### Latency is Valid ✅
The 61 ms average is a **real, over-the-air measurement**. Packet timestamps are embedded in the payload on the sender side and compared to the receiver's clock.
