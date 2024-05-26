#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import WILDCARD, FacNotifData, create_cms_api


def print_fac_notif_data(data: FacNotifData):
    print('Type:', data.type)
    print('Source address:', data.radio.source_address)
    print('Verify result:', data.security.verify_result)


class FacRxCtx:
    def __init__(self):
        self.rx_counter = {}

    def __call__(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        if data.type not in self.rx_counter.keys():
            self.rx_counter[data.type] = 0
        self.rx_counter[data.type] += 1
        print("---- Received FAC notification ----")
        print(self.rx_counter)
        print_fac_notif_data(data)
        print("-----------------------------------")


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        api.fac_subscribe(WILDCARD, FacRxCtx())
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
