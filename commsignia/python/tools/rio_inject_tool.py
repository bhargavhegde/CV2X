#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2023

import os
import sys
from argparse import ArgumentParser, RawTextHelpFormatter

from pycmssdk import MacAddr, RadioRxParams, RioPacket, RioRxNotifData, create_cms_api

filename = os.path.basename(__file__)
description = f"""
Commsignia radio inject tool. Injects radio packets directly into the stack via the remote API.

Example for injecting a CAM packet:
\techo \"<CAM packet>\" | {filename} --packetType RIO_GNP_PACKET
Example for injecting an US BSM packet:
\techo \"<US BSM packet>\" | {filename} --packetType RIO_WSMP_PACKET
Example for injecting a CN BSM packet:
\techo \"<CN BSM packet>\" | {filename} --packetType RIO_DSMP_PACKET
Example for injecting an IPV6 packet:
\techo \"<IPV6 packet>\" | {filename} --packetType RIO_IPV6_PACKET
"""

# Custom function to convert the string argument to an enum value
def packettype_arg(value):
    try:
        return RioPacket[value]
    except KeyError:
        print(f"Invalid packetType: {value}, using default RIO_GNP_PACKET")
        return RioPacket.RIO_GNP_PACKET


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
        "--packetType",
        type=packettype_arg,
        choices=RioPacket,
        default=RioPacket.RIO_GNP_PACKET,
        help="Radio packet type (default: RIO_GNP_PACKET)",
    )
    parser.add_argument("--interface", type=int, default=1, help="Radio interface (default: 1)")
    args = parser.parse_args()

    rio_rx_data = RioRxNotifData(
        packet_type=args.packetType,
        radio_params=RadioRxParams(
            interface_id=args.interface,
            dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF),
        ),
    )

    with create_cms_api(host=args.ip, port=args.port) as api:
        for line in sys.stdin:
            if line.rstrip() == 'Exit':
                break
            hex_string = line.rstrip()
            rio_data = bytearray.fromhex(hex_string)
            api.rio_inject(rio_rx_data, rio_data)


if __name__ == "__main__":
    main()
