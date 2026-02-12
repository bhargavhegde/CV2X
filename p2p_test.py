#!/usr/bin/env python3
import time
import argparse
import sys
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

def rx_callback(key: int, data: WsmpRxNotifData, buffer: bytes) -> None:
    print("\n[RECEIVE] ------ Message Received Over-The-Air ------")
    print(f"PSID: {hex(key)}")
    print(f"From (MAC): {data.radio.source_address}")
    print(f"RSSI: {data.radio.rssi} dBm")
    try:
        print(f"Payload: {buffer.decode('utf-8')}")
    except:
        print(f"Payload (Hex): {buffer.hex()}")
    print("--------------------------------------------------")

def main():
    parser = argparse.ArgumentParser(description="OBU Peer-to-Peer Test Tool")
    parser.add_argument("--host", default="192.168.1.54", help="IP address of the OBU V2X Stack")
    parser.add_argument("--mode", choices=["send", "listen"], required=True, help="Operating mode")
    parser.add_argument("--psid", type=int, default=0x42, help="PSID to use for transmission/reception")
    args = parser.parse_args()

    print(f"Connecting to OBU V2X stack at {args.host}...")
    
    try:
        with create_cms_api(host=args.host) as api:
            if args.mode == "listen":
                print(f"Listening for messages on PSID {hex(args.psid)}...")
                api.wsmp_rx_subscribe(args.psid, rx_callback)
                # Keep alive
                while True:
                    time.sleep(1)
            
            elif args.mode == "send":
                print(f"Broadcasting messages on PSID {hex(args.psid)} every 1 second...")
                send_data = WsmpSendData(
                    radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)),
                    wsmp_hdr=WsmpTxHdrInfo(psid=args.psid),
                    security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(sign_method=SignMethod.SIGN_METH_SIGN_CERT, psid=args.psid)),
                )
                
                count = 0
                while True:
                    count += 1
                    msg = f"OBU_PING_{count}"
                    print(f"[SEND] Sending: {msg}")
                    api.wsmp_send(send_data, buffer=msg.encode('utf-8'))
                    time.sleep(1)

    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting Tips:")
        print(1. "Ensure you are connected to the OBU WiFi.")
        print(2. "Ping the OBU IP (try 192.168.0.54 or 192.168.1.54).")
        print(3. "Ensure no firewall is blocking the connection.")

if __name__ == "__main__":
    main()
