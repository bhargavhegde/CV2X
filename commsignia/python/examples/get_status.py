#!/usr/bin/env python3
# Copyright Commsignia Ltd., 2021, All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.

import datetime

from pycmssdk import (
    StatCounterRef,
    StatCounterRefs,
    StatModuleId,
    StatWsmpCounterId,
    create_cms_api,
)


def main() -> None:

    with create_cms_api(host='127.0.0.1') as api:
        response = api.stat_get_device_status()
        print("Device unique ID:", response.data.device_data.unique_id)
        print("CMS Stack version:", response.data.device_data.version_info)
        stack_uptime = datetime.timedelta(seconds=response.data.misc_data.stack_uptime_ms / 1000.0)
        print("CMS Stack uptime:", stack_uptime)
        timediff = datetime.timedelta(seconds=response.data.misc_data.system_uptime_ms / 1000.0)
        print("System uptime:", timediff)
        current_time = (datetime.datetime.fromtimestamp(response.data.misc_data.timestamp_ms / 1000.0),)
        print("System time:", current_time[0])

        stat_in = StatCounterRefs(
            refs=(
                StatCounterRef(
                    StatModuleId.STAT_MODULE_ID_FAC_TX_US_BSM, StatWsmpCounterId.STAT_WSMP_COUNTER_ID_TX_PACKET
                ),
                StatCounterRef(
                    StatModuleId.STAT_MODULE_ID_FAC_RX_US_BSM, StatWsmpCounterId.STAT_WSMP_COUNTER_ID_RX_PACKET
                ),
            )
        )

        stat_response = api.stat_get(stat_in)
        print("BSM messages sent:", stat_response.data.counters[0].value)
        print("BSM messages received:", stat_response.data.counters[1].value)


if __name__ == '__main__':
    main()
