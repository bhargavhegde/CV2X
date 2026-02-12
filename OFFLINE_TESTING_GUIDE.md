# Offline OBU Testing Guide
**Follow these steps once you connect to the OBU WiFi.**

### **1. Physical Setup**
*   Power on both OBUs.
*   Connect your computer to the WiFi of **OBU 1**.
*   If you have a second computer, connect it to the WiFi of **OBU 2**.

### **2. Verify Connection (Ping Test)**
Before running any scripts, verify you can "see" the V2X stack:
```bash
ping -c 4 192.168.0.54
```
*   **Success**: You see `64 bytes from 192.168.0.54: icmp_seq=1 ttl=64 time=...`
*   **Failure**: If it fails, try `ping -c 4 192.168.1.54`. Note which IP works.

### **3. Run the Peer-to-Peer Test**

#### **On RECEIVER (OBU 2)**
Run this script first so it's ready to catch incoming messages:
```bash
python3 p2p_test.py --mode listen --host 192.168.0.54
```
*   It will sit and wait. You should see: `Listening for messages on PSID 0x42...`

#### **On SENDER (OBU 1)**
Run this on the computer connected to the first OBU:
```bash
python3 p2p_test.py --mode send --host 192.168.0.54
```
*   You will see: `[SEND] Sending: OBU_PING_1`, then `OBU_PING_2`, etc.

### **4. Verify Reception**
Check the screen of the **Receiver (OBU 2)**. You should see:
```text
[RECEIVE] ------ Message Received Over-The-Air ------
PSID: 0x42
From (MAC): [Some MAC Address]
RSSI: -45 dBm
Payload: OBU_PING_1
```

### **5. Troubleshooting Tips (If it doesn't work)**
*   **No WiFi Connection**: Your OS might disconnect from the OBU because it has "No Internet". Go to your WiFi settings and tell it to **"Connect Anyway"** or **"Switch to Mobile Data: No"**.
*   **Wrong IP**: The script defaults to `192.168.1.54` (common). If your OBU is `192.168.0.54`, run with `--host 192.168.0.54`.
*   **Firewall**: Ensure your computer's firewall is not blocking incoming UDP/TCP traffic from the OBU.

### **6. Running the Main Project Script (OBU_API.py)**
Once you've verified raw data is flowing with `p2p_test.py`, you can try the main project logic:
1.  **Check Config**: Open `RSU_OBU_API/parameters.json` and ensure `V2X_STACK_IP` matches your OBU (likely `192.168.0.54`).
2.  **Run**:
    ```bash
    python3 RSU_OBU_API/OBU_API.py
    ```
*This script is more complex: it handles image chunks, readiness signals, and logging. Refer to `RSU_OBU_API/README_OBU.md` for full details.*

### **7. Using Only One Computer?**
If you have only one computer: Use OBU 1 to send and OBU 2 to receive. You will need to switch between WiFi networks.
*Recommendation: Use two computers or a phone/laptop combo if possible to see the live data exchange.*
