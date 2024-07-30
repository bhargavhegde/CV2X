#!/usr/bin/env python3
#
# Copyright (C) CHELabUB - All Rights Reserved.
# Contact: chaozheh@buffalo.edu

import datetime
import time
from typing import Any

from pycmssdk import (
    NavFix,
    NavNotifData,
    NavSource,
    create_cms_api,
)

from pycmssdk import (
    Asn1Type,
    FacMsgType,
    FacNotifData,
    asn1_decode,
)

from pycmssdk import (  # noqa: F401
    WILDCARD,
    WsmpRxNotifData
)

from pycmssdk import (  # noqa: F401
    StiItem,
    StiNotifData,
    StiType,
    StiTypeList,
)

# (TODO) figure out the message PSID definition protocol
# 0x20 is the BSM id
# hex of 66
FILTERPSID = 0x42
# hex of 155
# FILTERPSID = 0x9B


# (TODO) make this part more modular
def get_local_time_suffix():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


SUFFIX = get_local_time_suffix()


def print_to_file(file, line):
    with open(file, "a+") as f:
        f.write(line)


def print_wsmp_rx_notif_data(data: WsmpRxNotifData):
    print('Timestamp:', data.radio.timestamp)
    print('Interface ID:', data.radio.interface_id)
    print('Source address:', data.radio.source_address)
    print('Destination address:', data.radio.dest_address)
    print('Datarate:', data.radio.datarate)
    print('User priority:', data.radio.user_prio)
    print('RSSI:', data.radio.rssi)
    print('PSID:', hex(data.wsmp_hdr.psid))
    print('Security verify result:', data.security.verify_result)
    # print('data', data)


class CAMDecode:
    def __init__(self):
        self.rx_counter = {}

    def __call__(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        if data.type not in self.rx_counter.keys():
            self.rx_counter[data.type] = 0
        self.rx_counter[data.type] += 1
        # print("---- Received FAC notification ---")
        # print(self.rx_counter)
        # print("------- Decoded CAM data ---------")
        decoded_message = asn1_decode(buffer, Asn1Type.EU_CAM)
        print(decoded_message)
        # print_to_file("fac_" + SUFFIX + ".txt", fac_msg)
        # print('---- Printed FAC to file ----')
        # print("----------------------------------")


class FacRxCtx:
    def __init__(self):
        self.rx_counter = {}

    def __call__(self, key: int, data: FacNotifData, buffer: bytes) -> None:
        if data.type not in self.rx_counter.keys():
            self.rx_counter[data.type] = 0
        self.rx_counter[data.type] += 1
        # print("---- Received FAC notification ---")
        # print(self.rx_counter)
        # print("------- Decoded BSM data ---------")
        decoded_message = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
        # print(decoded_message)
        # station id is of string type in hex, and it can be set for each devices.
        # if Pseudonymity module is on, then station id could be changing.
        station_id = decoded_message["value"][1]["coreData"]["id"].hex()
        print("FAC info From {}, speed {}".format(station_id,
                                                  decoded_message["value"][1]["coreData"]["speed"] / 100))

        # print("Cnt:", decoded_message["value"][1]["coreData"]["msgCnt"])
        # print("secMark:", decoded_message["value"][1]["coreData"]["secMark"])
        # print("id:", station_id)
        # print("Latitude:", decoded_message["value"][1]["coreData"]["lat"] / 10e6)
        # print("Longitude:", decoded_message["value"][1]["coreData"]["long"] / 10e6)
        # print("Elevation:", decoded_message["value"][1]["coreData"]["elev"] / 10e3)
        # print("Speed:", decoded_message["value"][1]["coreData"]["speed"] / 100)

        fac_msg = [str(decoded_message["value"][1]["coreData"]["msgCnt"]),
                   str(decoded_message["value"][1]["coreData"]["secMark"]),
                   station_id,
                   str(decoded_message["value"][1]["coreData"]["lat"] / 10e6),
                   str(decoded_message["value"][1]["coreData"]["long"] / 10e6),
                   str(decoded_message["value"][1]["coreData"]["elev"] / 10e3),
                   str(decoded_message["value"][1]["coreData"]["speed"] / 100),
                   str(decoded_message["value"][1]["coreData"]["heading"]),
                   str(decoded_message["value"][1]["coreData"]["angle"] / 100), ]
        fac_msg = ",".join(fac_msg) + " \n"
        print_to_file("fac_" + SUFFIX + ".txt", fac_msg)
        # print('---- Printed FAC to file ----')
        # print("----------------------------------")


def print_nav_fix(data: NavFix):
    print("{} {}".format(data.timestamp, data.speed / 100))
    # print('Nav fix is valid:', data.is_valid)
    # print('Timestamp:', data.timestamp)
    # print('Latitude:', data.latitude / 10e6)
    # print('Longitude:', data.longitude/10e6)
    # print('Altitude:', data.altitude / 10e3)
    # print('Confidence ellipse for latitude angle:', data.pce_semi_major)
    # print('Confidence ellipse for longitude angle:', data.pce_semi_minor)
    # print('Confidence ellipse orientation:', data.pce_orientation)
    # print('Heading:', data.heading)
    # print('Speed:', data.speed / 100)
    # print('Drive direction:', data.drive_direction)
    # print('Number of satellites:', data.number_of_used_satellites)
    msg = [str(data.is_valid), str(data.timestamp),
           str(data.latitude / 10e6), str(data.longitude / 10e6), str(data.altitude / 10e3),
           str(data.heading), str(data.speed / 100)]
    msg = ",".join(msg) + " \n"
    return msg


def nav_callback(data: NavNotifData):
    # print('---- Received NAV notification ----')
    msg = print_nav_fix(data)
    print_to_file("nav_" + SUFFIX + ".txt", msg)
    # print('---- Printed NAV to file ----')
    # print('-----------------------------------')


def print_sti_notif_data(data: StiNotifData):
    msg = []
    for item in data.items:
        # print('Time: ', time.time())
        # print('Type: ', item.type)
        # print('Value: ', item.value)
        msg.extend([str(item.type), str(item.value)])
    return msg


def sti_callback(data: StiNotifData):
    # print('-- Received Station info notification --')
    msg = [str(round(time.time(), 3))]
    msg_content = print_sti_notif_data(data)
    print("sti notification called {} {}".format(msg, msg_content))
    msg.extend(msg_content)
    msg = ",".join(msg) + " \n"
    print_to_file("sti_notif_" + SUFFIX + ".txt", msg)
    # print('---- Printed sti to file ----')
    # print('----------------------------------------')


def sti_param_record(api):
    response = api.sti_get(
        StiTypeList(
            types=(
                StiType.STI_VEHICLE_LENGTH,
                StiType.STI_VEHICLE_ROLE,
            )
        )
    )
    msg = [str(round(time.time(), 3))]
    msg_content = print_sti_notif_data(response.data)
    msg.extend(msg_content)
    msg = ",".join(msg) + " \n"
    print_to_file("sti_get_" + SUFFIX + ".txt", msg)
    msg = ",".join(msg) + " \n"
    print_to_file("sti_notif_" + SUFFIX + ".txt", msg)
    # print('---- Printed sti to file ----')


def wsmp_rx_callback(key: int, data: Any, buffer: bytes) -> None:
    # print("------ Received WSMP Rx notification ------")
    print(f"Rx msg with PSID: {hex(key)}")
    msg = [str(round(time.time(), 3))]
    # (TODO) proper decoding
    msg_content = buffer.decode()
    msg.append(msg_content)
    msg = ",".join(msg) + " \n"
    # print_wsmp_rx_notif_data(data)
    # print raw messages
    # print("raw messages {}".format(buffer))
    # If it is a standard messages, such as the TIM example
    # decoded_msg = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
    # print(decoded_msg)
    print_to_file("wsmp_custom_" + SUFFIX + ".txt", msg)
    # print("-------------------------------------------")


def main() -> None:
    with create_cms_api(host='192.168.0.54') as api:
        # through subscribe
        api.nav_subscribe(nav_callback)

        # the following only run once
        source_data = api.nav_get_source()
        try:
            source = source_data.data.source
            source_name = "unknown"
            if source == NavSource.NAV_SOURCE_MANUAL:
                source_name = "manual"
            elif source == NavSource.NAV_SOURCE_REAL:
                source_name = "GNSS"
            elif source == NavSource.NAV_SOURCE_GPSD:
                source_name = "GPSD"
            print("navigation is set by {}".format(source_name))
        except Exception as e:
            print("Encounter error {}".format(e))
            print("Bad gps source data {}".format(source_data))

        api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, FacRxCtx())

        # # The EU CAM message cannot be transmitted for now
        # api.fac_subscribe(FacMsgType.FAC_MSG_EU_CAM, CAMDecode())

        # api.wsmp_rx_subscribe(WILDCARD, wsmp_rx_callback)
        api.wsmp_rx_subscribe(FILTERPSID, wsmp_rx_callback)

        # # check sti id decoding, should be the same as set in the browser
        # id = api.sti_get_station_id()
        # print("{}, type {}".format(id, type(id)))
        # id_decode = ""
        # for v in id.data.id:
        #     if v > 0:
        #         id_decode = id_decode + hex(v)[2:]
        # print("ID decoded to {} from {}".format(id_decode, id))

        # The the call back will only be called if there is a change/update in an STI parameter.
        # Note that by default at least two messages acceleration and yaw rate will be continuously
        # printing because these two are automatically calculated every time the NAV module update speed
        # and acceleration. The auto calculation can be turned off using the web interface.
        api.sti_subscribe(sti_callback)

        while True:
            # record sti information on a fix schedule
            time.sleep(.1)
            sti_param_record(api)


if __name__ == "__main__":
    main()
