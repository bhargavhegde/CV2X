#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time
from typing import Any

from pycmssdk import WILDCARD, WsmpRxNotifData, create_cms_api


def print_wsmp_rx_notif_data(data: WsmpRxNotifData):
    print('Timestamp:', data.radio.timestamp)
    print('Interface ID:', data.radio.interface_id)
    print('Source address:', data.radio.source_address)
    print('Destination address:', data.radio.dest_address)
    print('Datarate:', data.radio.datarate)
    print('User priority:', data.radio.user_prio)
    print('RSSI:', data.radio.rssi)
    print('PSID:', hex(data.wsmp_hdr.psid))
    print('Security verify result:', data.security.verify_result)


def wsmp_rx_callback(key: int, data: Any, buffer: bytes) -> None:
    print("------ Received WSMP Rx notification ------")
    print(f"PSID: {hex(key)}")
    print_wsmp_rx_notif_data(data)
    print("-------------------------------------------")


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        api.wsmp_rx_subscribe(WILDCARD, wsmp_rx_callback)
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
