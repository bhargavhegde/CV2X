#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import (
    Asn1Type,
    MacAddr,
    RadioTxParams,
    SecDot2TxInfo,
    SecDot2TxSignInfo,
    SignMethod,
    WsmpSendData,
    WsmpTxHdrInfo,
    asn1_encode,
    create_cms_api,
)

TIM = {
    'messageId': 31,
    'value': (
        'TravelerInformation',
        {
            'msgCnt': 1,
            'timeStamp': 381239,
            'dataFrames': [
                {
                    'notUsed': 0,
                    'frameType': 'advisory',
                    'msgId': (
                        'roadSignID',
                        {
                            'position': {
                                'lat': 304003637,
                                'long': -842325784,
                            },
                            'viewAngle': (
                                65535,
                                16,
                            ),
                            'mutcdCode': 'warning',
                        },
                    ),
                    'startTime': 381239,
                    'durationTime': 2,
                    'priority': 3,
                    'notUsed1': 0,
                    'regions': [
                        {
                            'name': '0',
                            'anchor': {
                                'lat': 304003637,
                                'long': -842325784,
                            },
                            'directionality': 'forward',
                            'description': (
                                'path',
                                {
                                    'offset': (
                                        'xy',
                                        (
                                            'nodes',
                                            [
                                                {
                                                    'delta': (
                                                        'node-LatLon',
                                                        {
                                                            'lon': -842329625,
                                                            'lat': 304003851,
                                                        },
                                                    ),
                                                    'attributes': {
                                                        'dElevation': 10,
                                                    },
                                                },
                                                {
                                                    'delta': (
                                                        'node-LatLon',
                                                        {
                                                            'lon': -842325225,
                                                            'lat': 304004129,
                                                        },
                                                    ),
                                                    'attributes': {
                                                        'dElevation': -10,
                                                    },
                                                },
                                                {
                                                    'delta': (
                                                        'node-LatLon',
                                                        {
                                                            'lon': -842325722,
                                                            'lat': 304003151,
                                                        },
                                                    ),
                                                },
                                                {
                                                    'delta': (
                                                        'node-LatLon',
                                                        {
                                                            'lon': -842328418,
                                                            'lat': 304003133,
                                                        },
                                                    ),
                                                },
                                            ],
                                        ),
                                    ),
                                },
                            ),
                        },
                    ],
                    'notUsed2': 0,
                    'notUsed3': 0,
                    'content': (
                        'advisory',
                        [
                            {
                                'item': (
                                    'itis',
                                    1793,
                                ),
                            },
                            {
                                'item': (
                                    'text',
                                    'Wrong Way Detection!!!',
                                ),
                            },
                        ],
                    ),
                },
            ],
        },
    ),
}


def main() -> None:

    with create_cms_api(host="127.0.0.1") as api:
        tim_psid = 131
        send_data = WsmpSendData(
            radio=RadioTxParams(interface_id=1, dest_address=MacAddr(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)),
            wsmp_hdr=WsmpTxHdrInfo(psid=tim_psid),
            security=SecDot2TxInfo(
                sign_info=SecDot2TxSignInfo(sign_method=SignMethod.SIGN_METH_SIGN_CERT, psid=tim_psid)
            ),
        )

        encoded_tim = asn1_encode(TIM, Asn1Type.US_MESSAGE_FRAME)

        while True:
            api.wsmp_send(send_data, encoded_tim)
            print("TIM message sent")
            time.sleep(1)


if __name__ == "__main__":
    main()
