#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time
from typing import Any

from pycmssdk import (
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
    print(f"PSID: {hex(key)}")
    print_wsmp_tx_notif_data(data)
    print("-------------------------------------------")


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        api.wsmp_tx_subscribe(WILDCARD, wsmp_tx_callback)
        psid = 130
        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)),
            wsmp_hdr=WsmpTxHdrInfo(psid=psid),
            security=SecDot2TxInfo(sign_info=SecDot2TxSignInfo(sign_method=SignMethod.SIGN_METH_SIGN_CERT, psid=psid)),
        )

        api.wsmp_send(send_data, buffer=b"\x01\x02\x03\x04")
        time.sleep(1)


if __name__ == "__main__":
    main()
