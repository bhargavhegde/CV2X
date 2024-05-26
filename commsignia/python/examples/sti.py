#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import time

from pycmssdk import (
    StiItem,
    StiNotifData,
    StiSetItems,
    StiType,
    StiTypeList,
    create_cms_api,
)


def print_sti_notif_data(data: StiNotifData):
    for item in data.items:
        print('Type: ', item.type)
        print('Value: ', item.value)


def sti_callback(data: StiNotifData):
    print('-- Received Station info notification --')
    print_sti_notif_data(data)
    print('----------------------------------------')


def main() -> None:
    with create_cms_api(host='127.0.0.1') as api:
        api.sti_subscribe(sti_callback)
        api.sti_set(
            StiSetItems(
                items=(
                    StiItem(type=StiType.STI_STEERING_WHEEL_ANGLE, value=12),
                    StiItem(type=StiType.STI_VEHICLE_LENGTH, value=4000),
                )
            )
        )

        time.sleep(2)

        response = api.sti_get(
            StiTypeList(
                types=(
                    StiType.STI_VEHICLE_LENGTH,
                    StiType.STI_STEERING_WHEEL_ANGLE,
                )
            )
        )

        print('-- Received Station info get response --')
        print_sti_notif_data(response.data)
        print('----------------------------------------')


if __name__ == "__main__":
    main()
