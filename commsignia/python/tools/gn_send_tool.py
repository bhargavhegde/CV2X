#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2023

import os
import sys
import time
from argparse import ArgumentParser, RawTextHelpFormatter

from pycmssdk import (
    WILDCARD,
    BtpParams,
    BtpPort,
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

filename = os.path.basename(__file__)
description = f"""
Commsignia Geonetworking send tool. Triggers sending GN packets with custom payload via the remote API.

Example for sending a CAM packet:
\techo \"<CAM packet>\" | {filename} --btpPort 2001
Example for sending a DENM packet:
\techo \"<CAM packet>\" | {filename} --btpPort 2002
Example for sending an IVI packet:
\techo \"<IVI packet>\" | {filename} --btpPort 2006
"""


def print_gn_notif_data(data: GnTxNotifData):
    print('Interface ID:', data.radio.interface_id)
    print('Destination address:', data.radio.dest_address)
    print('Datarate:', data.radio.datarate)
    print('Tx power:', data.radio.tx_power)
    print('User priority:', data.radio.user_prio)
    print('Expire time:', data.radio.expiry_time)
    print('SPS Channel Index:', data.radio.sps_index)
    print('BTP destination port in notification:', data.gn_params.btp_params.btp_port)
    print('Security sign method:', data.security.sign_info.sign_method)
    print('PSID in security notif:', hex(data.security.sign_info.psid))
    print('SSP length:', data.security.sign_info.ssp.length)


def gn_tx_callback(key: int, data: GnTxNotifData, buffer: bytes) -> None:
    # pylint: disable=unused-argument
    print("------ Detected Geonet message send -------")
    print(f"BTP port: {hex(key)}")
    print_gn_notif_data(data)
    print("-------------------------------------------")


def main() -> None:
    parser = ArgumentParser(
        description=description,
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument(
        "--ip", type=str, default="127.0.0.1", help="External Api Server (EAS) IP address [default: 127.0.0.1]"
    )
    parser.add_argument("--port", type=int, default=7942, help="External Api Server (EAS) port [default: 7942]")
    parser.add_argument(
        "--btpPort",
        type=BtpPort,
        default=2001,
        help="BTP port (default: 2001 [CAM])",
    )
    args = parser.parse_args()

    for line in sys.stdin:
        if line.rstrip() == 'Exit':
            break
        hex_string = line.rstrip()
        payload = bytearray.fromhex(hex_string)

    with create_cms_api(host=args.ip, port=args.port) as api:
        api.gn_tx_subscribe(WILDCARD, gn_tx_callback)
        send_data = GnSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)),
            gn_params=GnTxParams(
                gn_hdr=GnHdrInfo(
                    method=GnSendMethod.GN_SEND_METHOD_GBC,
                    dest_area=GnDestArea(type=GnDestAreaType.GN_DEST_AREA_TYPE_CIRCLE, dist_a=500000),
                ),
                btp_params=BtpParams(
                    btp_port=args.btpPort,
                ),
            ),
        )

        api.gn_send(send_data, buffer=payload)
        time.sleep(1)


if __name__ == "__main__":
    main()
