#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2021-2023

import time

from pycmssdk import (
    NavDriveDirection,
    NavFix,
    NavNotifData,
    NavSetManual,
    NavSource,
    UtcTimestampMs,
    create_cms_api,
)


def print_nav_fix(data: NavFix):
    print('Nav fix is valid:', data.is_valid)
    print('Timestamp:', data.timestamp)
    print('Latitude:', data.latitude)
    print('Longitude:', data.longitude)
    print('Altitude:', data.altitude)
    print('Confidence ellipse for latitude angle:', data.pce_semi_major)
    print('Confidence ellipse for longitude angle:', data.pce_semi_minor)
    print('Confidence ellipse orientation:', data.pce_orientation)
    print('Heading:', data.heading)
    print('Speed:', data.speed)
    print('Drive direction:', data.drive_direction)
    print('Number of satellites:', data.number_of_used_satellites)


def nav_callback(data: NavNotifData):
    print('---- Received NAV notification ----')
    print_nav_fix(data)
    print('-----------------------------------')


def main() -> None:
    with create_cms_api(host='127.0.0.1') as api:
        api.nav_subscribe(nav_callback)
        # Create a nav data to set
        # e.g. set the position of the Commsignia office with 1.5m accuracy:
        # - latitude: 47.4754593°
        # - longitude: 19.0582323°
        # - altitude: 104m
        # - heading: 90°
        # - speed: 18 m/s
        api.nav_set_manual(
            NavSetManual(
                nav_fix=(
                    NavFix(
                        is_valid=True,
                        timestamp=UtcTimestampMs(),
                        latitude=474754593,
                        longitude=190582323,
                        altitude=104000,
                        pce_semi_major=1500,
                        pce_semi_minor=1500,
                        pce_orientation=250,
                        heading=90000,
                        speed=18000,
                        drive_direction=NavDriveDirection.NAV_DRIVE_DIR_FORWARD,
                        number_of_used_satellites=0,
                    )
                ),
                auto_update=False,
            )
        )

        time.sleep(2)

        response = api.nav_get_fix()

        print('-- Received NAV get fix response --')
        print_nav_fix(response.data)
        print('-----------------------------------')

        source = api.nav_get_source()
        print("navigation is set by ", end="")
        if source == NavSource.NAV_SOURCE_MANUAL:
            print("manual")
        elif source == NavSource.NAV_SOURCE_REAL:
            print("GNSS")
        elif source == NavSource.NAV_SOURCE_GPSD:
            print("GPSD")


if __name__ == "__main__":
    main()
