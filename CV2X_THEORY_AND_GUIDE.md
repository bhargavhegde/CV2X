# C-V2X Testing: Theory & Practical Guide

You have everything you need to start testing. The existing script `p2p_test.py` in your directory is perfect for this purpose.

## 1. Theory of Operation

### The Data Flow
When you run the Python scripts, the data flows through several layers:

1.  **Application Layer (Python)**:
    -   **Sender**: Constructs a **WSMP (Wave Short Message Protocol)** packet. This is the standard protocol for V2X safety messages.
    -   **Receiver**: Subscribes to specific **PSIDs (Provider Service Identifiers)** to interest in certain types of messages.

2.  **V2X Stack (On the OBU)**:
    -   The Python script talks to the OBU's V2X software stack over the network (likely TCP/IP).
    -   The stack handles signing (security), encoding, and scheduling.

3.  **Radio Layer (C-V2X PC5)**:
    -   The OBU broadcasts the message directly to other nearby OBUs using the **PC5 interface** (Direct Communication / Sidelink). This does *not* go through a cell tower or WiFi; it's device-to-device.

4.  **Air Interface**:
    -   The signal travels through the air at 5.9 GHz.

5.  **Receiver Stack**:
    -   OBU 2 hears the signal, decodes it, verifies the signature, and matches the PSID.

6.  **Application Layer (Python on Receiver)**:
    -   If the PSID matches (we use `0x42` or `66`), the stack sends the data back to your Python script via the callback function.

### Code Explanation (`p2p_test.py`)

**Sending (OBU 1)**:
```python
# Create a specialized V2X radio packet
send_data = WsmpSendData(
    # Broadcast to everyone (FF:FF:FF:FF:FF:FF)
    radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, ...)),
    # PSID 0x42 (66) identifies the service/application type
    wsmp_hdr=WsmpTxHdrInfo(psid=0x42),
    ...
)
# Send the raw bytes "OBU_PING_1"
api.wsmp_send(send_data, buffer=msg.encode('utf-8'))
```

**Receiving (OBU 2)**:
```python
# Tell the OBU: "Send me any received packets with PSID 0x42"
api.wsmp_rx_subscribe(0x42, rx_callback)

# This function triggers whenever a packet arrives
def rx_callback(key, data, buffer):
    print(f"Received from {data.radio.source_address}: {buffer.decode()}")
```

---

## 2. Step-by-Step Offline Testing Guide

Since you have two stationary OBUs (one on network `...125`, one on `...174`), here is exactly what to do.

> **Note:** You will lose internet access during this test. Save these instructions or keep this page open.

### Phase 1: Prepare the RECEIVER (OBU 2 - 174)

1.  **Connect** your computer to the WiFi network ending in **174**.
    *   *Tip:* If your OS complains "No Internet", ensure you stay connected.
2.  **Open a Terminal** and navigate to your folder:
    ```bash
    cd CV2X
    ```
3.  **Run the Receiver Script**:
    ```bash
    python3 p2p_test.py --mode listen
    ```
    *   *Note:* The script now defaults to `192.168.1.54` (the IP shown in your settings). It will tell you "Listening for messages ...".
4.  **Wait**: Leave this terminal running. It is now waiting for packets from the air.

### Phase 2: Start the SENDER (OBU 1 - 125)

1.  **Switch WiFi**: On the second computer, connect to the network ending in **125**.
2.  **Open a Terminal** and run:
    ```bash
    python3 p2p_test.py --mode send
    ```
3.  **Observe**:
    *   You should see: `[SEND] Sending: OBU_PING_1` ... `OBU_PING_2` etc.
    *   The OBU is now broadcasting these messages into the air.

### Phase 3: Verify Reception

Look at the **Receiver** computer. You should see:
```text
[RECEIVE] ------ Message Received Over-The-Air ------
PSID: 0x42
From (MAC): [Some MAC Address]
RSSI: -45 dBm
Payload: OBU_PING_1
```

---

## 3. Checklist Before You Go Offline

- [ ] **File**: Verify `p2p_test.py` is present.
- [ ] **Library**: Verify `pycmssdk` is installed or `mock_pycmssdk.py` is in the folder.
- [ ] **IP Address**: Know the IP of your OBUs (confirmed as `192.168.1.54` in your settings).
- [ ] **Command**: Remember `python3 p2p_test.py --mode listen` for the receiver side.

If you don't see packets:
1.  Check the **IP address** (ping `192.168.1.54`).
2.  Check the **Firewall** (disable it temporarily if needed).
3.  Ensure both OBUs have antennas connected!
