#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import create_cms_api
from pycmssdk.typedef import CamTxState


def main() -> None:

    with create_cms_api(host='127.0.0.1') as api:
        api.cam_tx_enable(CamTxState(enable=True))
        time.sleep(5)
        api.cam_tx_enable(CamTxState(enable=False))


if __name__ == '__main__':
    main()
