#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2022

"""Tool for triggering DENM via remote API."""

import time
from argparse import ArgumentParser, RawTextHelpFormatter

from pycmssdk import (
    DenmAutoFlags,
    DenmCreate,
    DenmManagementData,
    DenmTermination,
    GnTrafficClass,
    asn1_wrapper,
    create_cms_api,
)

DENM_TEMPLATE = {
    "management": {
        "actionID": {
            "originatingStationID": 0,
            "sequenceNumber": 0,
        },
        "detectionTime": 0,
        "referenceTime": 0,
        "eventPosition": {
            "latitude": 0,
            "longitude": 0,
            "positionConfidenceEllipse": {
                "semiMajorConfidence": 0,
                "semiMinorConfidence": 0,
                "semiMajorOrientation": 0,
            },
            "altitude": {"altitudeValue": 0, "altitudeConfidence": "alt-000-01"},
        },
        "relevanceDistance": "lessThan1000m",
        "stationType": 0,
    },
    "situation": {
        "eventType": {
            "causeCode": 0,
            "subCauseCode": 0,
        },
        "informationQuality": 0,
    },
}


def parse_args():
    """Parse command line arguments."""

    parser = ArgumentParser(
        description="Commsignia DENM trigger tool. "
        "Triggers and terminates DENM sending via the remote API.\n"
        "\n"
        "Example for triggering an EEBL DENM: \n"
        "\tsti_tool.py --causeCode 99 --subCauseCode 1 --repetitionInterval 100 --repetitionDuration 2000"
        "# Trigger DENM sending for 2 seconds every 100 ms\n",
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument("--causeCode", type=int, required=True, help="Cause code")
    parser.add_argument("--subCauseCode", type=int, default=0, help="Sub cause code [default: 0]")
    parser.add_argument(
        "--ip", type=str, default="127.0.0.1", help="External Api Server (EAS) IP address [default: 127.0.0.1]"
    )
    parser.add_argument("--port", type=int, default=7942, help="External Api Server (EAS) port [default: 7942]")
    parser.add_argument(
        "--repetitionDuration", type=int, default=2000, help="Repeat DENM until N ms elapses [default: 2000]"
    )
    parser.add_argument("--repetitionInterval", type=int, default=1000, help="Repeat DENM every N ms [default: 1000]")
    parser.add_argument(
        "--validityDuration", type=int, default=2, help="DENM validity duration in seconds [default: 2]"
    )
    parser.add_argument("--informationQuality", type=int, default=1, help="Information quality [default: 1]")
    return parser.parse_args()


def ms_to_sec(ms: int) -> float:
    """Convert milliseconds to seconds."""

    return ms * 1e-3


def create_encoded_denm(args):
    """Create and encode DENM message."""

    asn1_denm_encoder = asn1_wrapper.DENMNoPduCodec
    denm = DENM_TEMPLATE
    denm["management"]["validityDuration"] = args.validityDuration
    denm["situation"]["eventType"]["causeCode"] = args.causeCode
    denm["situation"]["eventType"]["subCauseCode"] = args.subCauseCode
    denm["situation"]["informationQuality"] = args.informationQuality

    return asn1_denm_encoder.encode(denm)


def terminate_after_repetition_duration(args, api, action_id):
    """
    Terminate DENM after repetition duration expires. If the DENM is no longer valid
    at the time repetition duration expires, skip DENM termination.
    """
    repetition_duration = ms_to_sec(args.repetitionDuration)
    if args.validityDuration > repetition_duration:
        time.sleep(repetition_duration)
        api.denm_terminate(DenmTermination(action_id=action_id))


def send_denm_trigger(args, api, encoded_denm):
    """Send DENM trigger via the remote API."""
    response = api.denm_create_or_update(
        data=DenmCreate(
            auto_flags=DenmAutoFlags(
                auto_calc_action_id=True,
                auto_set_detection_time=True,
                auto_set_event_position=True,
                auto_set_event_trace=True,
                auto_set_destination_area=True,
            ),
            management=DenmManagementData(
                gn_hop_limit=2,
                repetition_duration=args.repetitionDuration,
                repetition_interval=args.repetitionInterval,
                gn_traffic_class=GnTrafficClass(tcid=2),
            ),
        ),
        buffer=encoded_denm,
    )
    action_id = response.data
    assert action_id is not None

    return action_id


def main():
    """
    Entry point of DENM triggering tool.

    :param args:
        Command line arguments.
    """

    args = parse_args()
    encoded_denm = create_encoded_denm(args)

    with create_cms_api(host=args.ip, port=args.port) as api:
        action_id = send_denm_trigger(args, api, encoded_denm)
        terminate_after_repetition_duration(args, api, action_id)


if __name__ == "__main__":
    main()
