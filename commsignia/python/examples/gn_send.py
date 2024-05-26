#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import (
    WILDCARD,
    BtpParams,
    GnDestArea,
    GnDestAreaType,
    GnHdrInfo,
    GnSendData,
    GnSendMethod,
    GnTxNotifData,
    GnTxParams,
    MacAddr,
    RadioTxParams,
    create_cms_api,
)


def print_gn_notif_data(data: GnTxNotifData):
    print('Interface ID:', data.radio.interface_id)
    print('Destination address:', data.radio.dest_address)
    print('Datarate:', data.radio.datarate)
    print('Tx power:', data.radio.tx_power)
    print('User priority:', data.radio.user_prio)
    print('Expire time:', data.radio.expiry_time)
    print('SPS Channel Index:', data.radio.sps_index)
    print('BTP destination port in notification notif:', data.gn_params.btp_params.btp_port)
    print('Security sign method:', data.security.sign_info.sign_method)
    print('PSID in security notif:', hex(data.security.sign_info.psid))
    print('SSP length:', data.security.sign_info.ssp.length)


def gn_tx_callback(key: int, data: GnTxNotifData, buffer: bytes) -> None:
    print("------ Detected Geonet message send -------")
    print(f"BTP port: {hex(key)}")
    print_gn_notif_data(data)
    print("-------------------------------------------")


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        api.gn_tx_subscribe(WILDCARD, gn_tx_callback)
        send_data = GnSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)),
            gn_params=GnTxParams(
                gn_hdr=GnHdrInfo(
                    method=GnSendMethod.GN_SEND_METHOD_GBC,
                    dest_area=GnDestArea(
                        type=GnDestAreaType.GN_DEST_AREA_TYPE_CIRCLE,
                    ),
                ),
                btp_params=BtpParams(
                    btp_port=1235,
                ),
            ),
        )

        api.gn_send(send_data, buffer=b"\x01\x02\x03\x04")
        time.sleep(1)


if __name__ == "__main__":
    main()
