#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2022

import time
from argparse import ArgumentParser, RawTextHelpFormatter
from datetime import datetime

from geographiclib.constants import Constants
from geographiclib.geodesic import Geodesic

from pycmssdk import (
    ALTITUDE_NA,
    HEADING_CONFIDENCE_NA,
    HEADING_NA,
    LATITUDE_NA,
    LENGTH_NA,
    LONGITUDE_NA,
    SPEED_NA,
    UTC_TIMESTAMP_NA,
    Altitude,
    Heading,
    HeadingConfidence,
    Latitude,
    Length,
    Longitude,
    NavDriveDirection,
    NavFix,
    NavSetManual,
    Speed,
    UtcTimestampMs,
    create_cms_api,
)


def parse_drive_direction(value: str) -> NavDriveDirection:
    try:
        return NavDriveDirection[value]
    except KeyError:
        pass

    return NavDriveDirection(int(value))


def parse_timestamp(value: str) -> UtcTimestampMs:
    if value == "NA":
        return UTC_TIMESTAMP_NA

    if value == "NOW":
        return round(datetime.utcnow().timestamp() * 1000)

    return int(value)


def int_or_na(value: int, na_value: int) -> str:
    return "NA" if value == na_value else str(value)


def print_nav_fix(fix: NavFix) -> None:
    print(f"is_valid={fix.is_valid}")
    print(f"timestamp={int_or_na(fix.timestamp, UTC_TIMESTAMP_NA)} [ms]")
    print(f"leap_seconds={fix.leap_seconds} [s]")
    print(f"latitude={int_or_na(fix.latitude, LATITUDE_NA)} [0.1 udeg]".format())
    print(f"longitude={int_or_na(fix.longitude, LONGITUDE_NA)} [0.1 udeg]".format())
    print(f"altitude={int_or_na(fix.altitude, ALTITUDE_NA)} [mm]")
    print(f"altitude_confidence={int_or_na(fix.altitude_confidence, ALTITUDE_NA)} [mm]")
    print(f"pce_semi_major={int_or_na(fix.pce_semi_major, LENGTH_NA)} [mm]")
    print(f"pce_semi_minor={int_or_na(fix.pce_semi_minor, LENGTH_NA)} [mm]")
    print(f"pce_orientation={int_or_na(fix.pce_orientation, HEADING_NA)} [0.001 degree]")
    print(f"heading={int_or_na(fix.heading, HEADING_NA)}  [0.001 degree]")
    print(f"heading_confidence={int_or_na(fix.heading_confidence, HEADING_NA)} [0.001 degree]")
    print(f"speed={int_or_na(fix.speed, SPEED_NA)} [mm/s]")
    print(f"speed_confidence={int_or_na(fix.speed_confidence, SPEED_NA)} [mm/s]")
    print(f"drive_direction={fix.drive_direction.name}")
    print(f"number_of_used_satellites={fix.number_of_used_satellites}")


def get_drive_direction_values() -> str:
    return "\n".join(f"\t{dd.name} (value: {dd.value})" for dd in NavDriveDirection)


def mm_per_sec_to_m_per_sec(speed_mm_per_sec: int) -> float:
    return speed_mm_per_sec / 1000


def ns_to_ms(ns: int) -> float:
    return ns * 1e-6


def ns_to_sec(ns: int) -> float:
    return ns * 1e-9


def wait_sec(time_to_wait_sec: float) -> float:
    prev_time_ns = time.time_ns()
    time.sleep(time_to_wait_sec)
    return ns_to_sec(time.time_ns() - prev_time_ns)


def calc_distance(speed_m_per_sec: float, time_sec: float) -> float:
    return speed_m_per_sec * time_sec


def mdeg_to_degree(udeg: float) -> float:
    return udeg * 1e-3


def tenth_udeg_to_degree(tenth_udeg: float) -> float:
    return tenth_udeg * 1e-7


def degree_to_tenth_udeg(degree: float) -> int:
    return int(degree * 1e7)


def move(fix: NavFix, delta_t: float) -> (int, int):
    distance = calc_distance(mm_per_sec_to_m_per_sec(fix.speed), delta_t)

    geod = Geodesic(Constants.WGS84_a, Constants.WGS84_f)
    d = geod.Direct(
        tenth_udeg_to_degree(fix.latitude), tenth_udeg_to_degree(fix.longitude), mdeg_to_degree(fix.heading), distance
    )
    return degree_to_tenth_udeg(d['lat2']), degree_to_tenth_udeg(d['lon2'])


def main() -> None:
    parser = ArgumentParser(
        description="Commsignia NAV tool. "
        "Inspects and manipulates values in NAV via the remote API.\n\n"
        "SET MODE: [default] (e.g. nav_tool.py --latitude 110000000 --longitude 220000000)\n"
        "\tThe tool performs a full NAV fix update with the given arguments. Omitted values are set to NA.\n\n"
        "GET MODE: (e.g. nav_tool.py --get)\n"
        "\tThe tool gets the NAV fix and prints it to the console. No value is altered.",
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument(
        "--ip", type=str, default="127.0.0.1", help="External Api Server (EAS) IP address [default: 127.0.0.1]"
    )
    parser.add_argument("--port", type=int, default=7942, help="External Api Server (EAS) port [default: 7942]")
    parser.add_argument("--get", action="store_true", help="Get mode [default: False]")
    parser.add_argument("--valid", action="store_true", help="NAV Fix is valid [default: False]")
    parser.add_argument(
        "--timestamp",
        type=str,
        default="NA",
        help="UTC timestamp (use NOW to set to current system time) [ms] [default: NA]",
    )
    parser.add_argument(
        "--leap-seconds",
        type=int,
        default=0,
        help="Leap seconds, e.g. the difference between UTC and TAI [s] [default: 0]",
    )
    parser.add_argument(
        "--latitude", type=Latitude, default=LATITUDE_NA, help="latitude angle [0.1 udeg] [default: NA]"
    )
    parser.add_argument(
        "--longitude", type=Longitude, default=LONGITUDE_NA, help="longitude angle [0.1 udeg] [default: NA]"
    )
    parser.add_argument(
        "--altitude", type=Altitude, default=ALTITUDE_NA, help="altitude above WGS ellipsoid [mm] [default: NA]"
    )
    parser.add_argument(
        "--altitude-confidence", type=Altitude, default=ALTITUDE_NA, help="altitude confidence [mm] [default: NA]"
    )
    parser.add_argument(
        "--pce-semi-major",
        type=Length,
        default=LENGTH_NA,
        help="length of the position confidence ellipse's major axis [mm] [default: NA]",
    )
    parser.add_argument(
        "--pce-semi-minor",
        type=Length,
        default=LENGTH_NA,
        help="length of the position confidence ellipse's minor axis [mm] [default: NA]",
    )
    parser.add_argument(
        "--pce-orientation",
        type=Heading,
        default=HEADING_NA,
        help="orientation of the position confidence ellipse (absolute value with North reference) [0.001 degree] [default: NA]",
    )
    parser.add_argument(
        "--heading",
        type=Heading,
        default=HEADING_NA,
        help="heading (absolute value with North reference) [0.001 degree] [default: NA]",
    )
    parser.add_argument(
        "--heading-confidence",
        type=HeadingConfidence,
        default=HEADING_CONFIDENCE_NA,
        help="heading confidence [0.001 degree] [default: NA]",
    )
    parser.add_argument("--speed", type=Speed, default=SPEED_NA, help="speed [mm/s] [default: NA]")
    parser.add_argument(
        "--speed-confidence", type=Speed, default=SPEED_NA, help="speed confidence [mm/s] [default: NA]"
    )
    parser.add_argument(
        "--drive-direction",
        type=str,
        default=NavDriveDirection.NAV_DRIVE_DIR_UNAVAILABLE.name,
        help="drive direction [enum] [default: NAV_DRIVE_DIR_UNAVAILABLE]\n"
        f"Possible values:\n{get_drive_direction_values()}",
    )
    parser.add_argument(
        "--number-of-used-satellites", type=int, default=0, help="number of used satellites (0-255) [default: 0]"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=0,
        help="timeout in milliseconds until the navigation is set then quit [default: 0]",
    )
    args = parser.parse_args()

    start_time_ms = ns_to_ms(time.time_ns())

    with create_cms_api(host=args.ip, port=args.port) as api:
        if not args.get:
            nav_fix = NavFix(
                is_valid=args.valid,
                timestamp=parse_timestamp(args.timestamp),
                leap_seconds=args.leap_seconds,
                latitude=args.latitude,
                longitude=args.longitude,
                altitude=args.altitude,
                altitude_confidence=args.altitude_confidence,
                pce_semi_major=args.pce_semi_major,
                pce_semi_minor=args.pce_semi_minor,
                pce_orientation=args.pce_orientation,
                heading=args.heading,
                heading_confidence=args.heading_confidence,
                speed=args.speed,
                speed_confidence=args.speed_confidence,
                drive_direction=parse_drive_direction(args.drive_direction),
                number_of_used_satellites=args.number_of_used_satellites,
            )
            while True:
                api.nav_set_manual(NavSetManual(auto_update=False, nav_fix=nav_fix))
                print_nav_fix(api.nav_get_fix().data)
                if (args.timeout != 0) and (ns_to_ms(time.time_ns()) >= (start_time_ms + args.timeout)):
                    break
                delta_time_sec = wait_sec(0.1)
                nav_fix.latitude, nav_fix.longitude = move(nav_fix, delta_time_sec)


if __name__ == "__main__":
    main()
