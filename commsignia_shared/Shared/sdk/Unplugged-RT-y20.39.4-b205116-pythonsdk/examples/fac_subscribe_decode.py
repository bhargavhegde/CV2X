#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api


class FacRxCtx:
    def __init__(self):
        self.rx_counter = {}

    def __call__(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        if data.type not in self.rx_counter.keys():
            self.rx_counter[data.type] = 0
        self.rx_counter[data.type] += 1
        print("---- Received FAC notification ---")
        print(self.rx_counter)
        print("------- Decoded BSM data ---------")
        decoded_message = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
        print("Latitude:", decoded_message["value"][1]["coreData"]["lat"])
        print("Longitude:", decoded_message["value"][1]["coreData"]["long"])
        print("Elevation:", decoded_message["value"][1]["coreData"]["elev"])
        print("----------------------------------")


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, FacRxCtx())
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
