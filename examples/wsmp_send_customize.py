#!/usr/bin/env python3
#
# Copyright (C) CHELabUB - All Rights Reserved.
# Contact: chaozheh@buffalo.edu

import time
from typing import Any

try:
    from pycmssdk import (  # noqa: F401
        WILDCARD,
        MacAddr,
        RadioTxParams,
        SecDot2TxInfo,
        SecDot2TxSignInfo,
        SignMethod,
        WsmpSendData,
        WsmpTxHdrInfo,
        WsmpTxNotifData,
        create_cms_api,
    )
    print("Successfully loaded real pycmssdk.")
except ImportError:
    import sys
    import os
    # Add parent directory to path to import mock_pycmssdk
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from mock_pycmssdk import (  # noqa: F401
        WILDCARD,
        MacAddr,
        RadioTxParams,
        SecDot2TxInfo,
        SecDot2TxSignInfo,
        SignMethod,
        WsmpSendData,
        WsmpTxHdrInfo,
        WsmpTxNotifData,
        create_cms_api,
    )
    print("Loaded MOCK pycmssdk for local experimentation.")


def print_wsmp_tx_notif_data(data: WsmpTxNotifData):
    print('Interface ID:', data.radio.interface_id)
    print('Destination address:', data.radio.dest_address)
    print('Datarate:', data.radio.datarate)
    print('Tx power:', data.radio.tx_power)
    print('User priority:', data.radio.user_prio)
    print('Expire time:', data.radio.expiry_time)
    print('SPS Channel Index:', data.radio.sps_index)
    print('PSID:', hex(data.wsmp_hdr.psid))
    print('Security sign method:', data.security.sign_info.sign_method)
    print('PSID in security notif:', hex(data.security.sign_info.psid))
    print('SSP length:', data.security.sign_info.ssp.length)


def wsmp_tx_callback(key: int, data: Any, buffer: bytes) -> None:
    print("------ Received WSMP Tx notification ------")
    print('Unix Timestamp:', time.time())
    print(f"PSID: {hex(key)}")
    print(f"Raw message Tx: {buffer}")
    # print_wsmp_tx_notif_data(data)
    print("-------------------------------------------")


def main() -> None:

    with create_cms_api(host="192.168.0.54") as api:
        # [TODO] figure out the restrictions of PSID definition
        # 0x20 is for BSM
        psid = 66
        psid_hex = 0x42
        # only get tx notice for the specified PSID
        api.wsmp_tx_subscribe(psid_hex, wsmp_tx_callback)
        # api.wsmp_tx_subscribe(WILDCARD, wsmp_tx_callback)
        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54)),
            wsmp_hdr=WsmpTxHdrInfo(psid=psid),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(sign_method=SignMethod.SIGN_METH_SIGN_CERT, psid=psid)),
        )
        id = api.sti_get_station_id()
        print("{}, type {}".format(id, type(id)))
        id_decode = ""
        for v in id.data.id:
            if v > 0:
                id_decode = id_decode + hex(v)[2:]
        # print("ID decoded to {} from {}".format(id_decode, id))

        while True:
            # continuously sending
            # [TODO] figure out the buffer format
            # (it is in bytes but what's the restrictions)
            raw_msg = id_decode + " " + str(time.time())
            api.wsmp_send(send_data, buffer=raw_msg.encode())
            time.sleep(1)


if __name__ == "__main__":
    main()
