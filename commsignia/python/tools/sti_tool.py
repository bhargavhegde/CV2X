#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2022

import time
from argparse import ArgumentParser, RawTextHelpFormatter
from sys import argv

from pycmssdk import (
    STI_VALUE_NA,
    STI_VALUE_OOR_MAX,
    StiAreaType,
    StiAuxBrakes,
    StiBeltBuckleStatus,
    StiDangerousGood,
    StiDoorState,
    StiFuelType,
    StiItem,
    StiLanePos,
    StiLightbar,
    StiPhysicalRoadSeparation,
    StiRoadClass,
    StiSetItems,
    StiSiren,
    StiState,
    StiStationType,
    StiTransmissionState,
    StiTransmissionType,
    StiTristate,
    StiType,
    StiTypeList,
    StiValue,
    StiVehicleLengthConf,
    StiVehicleRole,
    StiWeatherPrecipSituation,
    StiWiperState,
    create_cms_api,
)

STI_ENUM_MAP = {
    StiType.STI_ABS: StiTristate,
    StiType.STI_ADAPTIVE_CRUISE_CONTROL: StiState,
    StiType.STI_AREA_TYPE: StiAreaType,
    StiType.STI_AUTOMATIC_EMERGENCY_BRAKE: StiState,
    StiType.STI_AUX_BRAKES: StiAuxBrakes,
    StiType.STI_BELT_BUCKLE_ROW1_DRIVER: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW1_MIDDLE: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW1_PASSENGER: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW2_DRIVER: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW2_MIDDLE: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW2_PASSENGER: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW3_DRIVER: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW3_MIDDLE: StiBeltBuckleStatus,
    StiType.STI_BELT_BUCKLE_ROW3_PASSENGER: StiBeltBuckleStatus,
    StiType.STI_COLLISION_WARNING: StiState,
    StiType.STI_CRUISE_CONTROL: StiState,
    StiType.STI_DANGEROUS_GOODS: StiDangerousGood,
    StiType.STI_DOOR_STATE_BONNET: StiDoorState,
    StiType.STI_DOOR_STATE_FRONT_LEFT: StiDoorState,
    StiType.STI_DOOR_STATE_FRONT_RIGHT: StiDoorState,
    StiType.STI_DOOR_STATE_REAR_LEFT: StiDoorState,
    StiType.STI_DOOR_STATE_REAR_RIGHT: StiDoorState,
    StiType.STI_DOOR_STATE_TRUNK: StiDoorState,
    StiType.STI_EMBARKATION_STATUS: StiState,
    StiType.STI_EMERGENCY_BRAKE: StiState,
    StiType.STI_EV_AIRBAG_DEPLOYED: StiState,
    StiType.STI_EV_DISABLED_VEHICLE: StiState,
    StiType.STI_EV_FLAT_TIRE: StiState,
    StiType.STI_EV_STOP_LINE_VIOLATED: StiState,
    StiType.STI_EXT_LIGHT_AUTO_LIGHT_CONTROL: StiState,
    StiType.STI_EXT_LIGHT_DAYTIME_RUNNING: StiState,
    StiType.STI_EXT_LIGHT_FOG: StiState,
    StiType.STI_EXT_LIGHT_HAZARD_LIGHT: StiState,
    StiType.STI_EXT_LIGHT_HIGHBEAM_HEAD: StiState,
    StiType.STI_EXT_LIGHT_LEFT_TURN_SIGNAL: StiState,
    StiType.STI_EXT_LIGHT_LOWBEAM_HEAD: StiState,
    StiType.STI_EXT_LIGHT_PARKING: StiState,
    StiType.STI_EXT_LIGHT_REVERSE: StiState,
    StiType.STI_EXT_LIGHT_RIGHT_TURN_SIGNAL: StiState,
    StiType.STI_FUEL_TYPE: StiFuelType,
    StiType.STI_IGNITION: StiState,
    StiType.STI_LANE_POSITION: StiLanePos,
    StiType.STI_LIGHTBAR: StiLightbar,
    StiType.STI_PHYSICAL_ROAD_SEPARATION: StiPhysicalRoadSeparation,
    StiType.STI_RED_WARNING_ACTIVE: StiState,
    StiType.STI_REVERSIBLE_OCCUPANT_RESTRAINT_SYSTEM: StiState,
    StiType.STI_ROAD_CLASS: StiRoadClass,
    StiType.STI_SIREN: StiSiren,
    StiType.STI_SPEED_LIMITER: StiState,
    StiType.STI_STABILITY_CONTROL_STATUS: StiTristate,
    StiType.STI_STATION_TYPE: StiStationType,
    StiType.STI_TRACTION_CONTROL_STATUS: StiTristate,
    StiType.STI_TRANSMISSION_STATE: StiTransmissionState,
    StiType.STI_TRANSMISSION_TYPE: StiTransmissionType,
    StiType.STI_VEHICLE_LENGTH_CONF: StiVehicleLengthConf,
    StiType.STI_VEHICLE_ROLE: StiVehicleRole,
    StiType.STI_WEATHER_PRECIP_SITUATON: StiWeatherPrecipSituation,
    StiType.STI_WIPER_STATE_FRONT: StiWiperState,
    StiType.STI_WIPER_STATE_REAR: StiWiperState,
    StiType.STI_MANUAL_ECALL: StiState,
    StiType.STI_LOW_SEVERITY_CRASH: StiState,
    StiType.STI_PEDESTRIAN_COLLISION: StiState,
    StiType.STI_HIGH_SEVERITY_CRASH: StiState,
}

STI_UNIT_MAP = {
    StiType.STI_ACCELERATOR_PEDAL: "[parts-per-thousand]",
    StiType.STI_BRAKE_PEDAL: "[parts-per-thousand]",
    StiType.STI_BRAKE_STATUS_LEFT_FRONT: "[parts-per-thousand]",
    StiType.STI_BRAKE_STATUS_LEFT_REAR: "[parts-per-thousand]",
    StiType.STI_BRAKE_STATUS_RIGHT_FRONT: "[parts-per-thousand]",
    StiType.STI_BRAKE_STATUS_RIGHT_REAR: "[parts-per-thousand]",
    StiType.STI_BUMPER_HEIGHT_FRONT: "[mm]",
    StiType.STI_BUMPER_HEIGHT_REAR: "[mm]",
    StiType.STI_DRIVING_WHEEL_ANGLE: "[0.001 degree]",
    StiType.STI_LAT_ACC_CONF: "[mm/s^2]",
    StiType.STI_LAT_ACC: "[mm/s^2]",
    StiType.STI_LONG_ACC_CONF: "[mm/s^2]",
    StiType.STI_LONG_ACC: "[mm/s^2]",
    StiType.STI_RAIN_SENSOR: "[parts-per-thousand]",
    StiType.STI_STEERING_WHEEL_ANGLE_CONF: "[0.001 degree]",
    StiType.STI_STEERING_WHEEL_ANGLE_RATE: "[0.001 degree/second]",
    StiType.STI_STEERING_WHEEL_ANGLE: "[0.001 degree] CCW positive",
    StiType.STI_TRAILER_WEIGHT: "[g]",
    StiType.STI_VEHICLE_HEIGHT: "[mm]",
    StiType.STI_VEHICLE_LENGTH: "[mm]",
    StiType.STI_VEHICLE_MASS: "[gram]",
    StiType.STI_VEHICLE_WIDTH: "[mm]",
    StiType.STI_VERT_ACC_CONF: "[mm/s^2]",
    StiType.STI_VERT_ACC: "[mm/s^2]",
    StiType.STI_WEATHER_AIR_PRESSURE: "[Pascal]",
    StiType.STI_WEATHER_AIR_TEMP: "[0.1C°]",
    StiType.STI_WEATHER_COEF_FRICTION: "[parts-per-thousand]",
    StiType.STI_WEATHER_RAIN_RATE: "[0.1 g/s/m^2]",
    StiType.STI_WEATHER_SOLAR_RADIATION: "[J/m^2]",
    StiType.STI_WIPER_RATE_FRONT: "[sweeps/minute]",
    StiType.STI_WIPER_RATE_REAR: "[sweeps/minute]",
    StiType.STI_YAW_RATE_CONF: "[0.001 degree/second]",
    StiType.STI_YAW_RATE: "[0.001 degree/second] CCW positive",
    StiType.STI_PROJECT_00: "[varying]",
    StiType.STI_PROJECT_01: "[varying]",
    StiType.STI_PROJECT_02: "[varying]",
    StiType.STI_PROJECT_03: "[varying]",
    StiType.STI_PROJECT_04: "[varying]",
    StiType.STI_PROJECT_05: "[varying]",
    StiType.STI_PROJECT_06: "[varying]",
    StiType.STI_PROJECT_07: "[varying]",
    StiType.STI_PROJECT_08: "[varying]",
    StiType.STI_PROJECT_09: "[varying]",
    StiType.STI_PROJECT_10: "[varying]",
    StiType.STI_PROJECT_11: "[varying]",
    StiType.STI_PROJECT_12: "[varying]",
    StiType.STI_PROJECT_13: "[varying]",
    StiType.STI_PROJECT_14: "[varying]",
    StiType.STI_PROJECT_15: "[varying]",
    StiType.STI_PROJECT_16: "[varying]",
    StiType.STI_PROJECT_17: "[varying]",
    StiType.STI_PROJECT_18: "[varying]",
    StiType.STI_PROJECT_19: "[varying]",
    StiType.STI_PROJECT_20: "[varying]",
    StiType.STI_PROJECT_21: "[varying]",
    StiType.STI_PROJECT_22: "[varying]",
    StiType.STI_PROJECT_23: "[varying]",
    StiType.STI_PROJECT_24: "[varying]",
    StiType.STI_PROJECT_25: "[varying]",
    StiType.STI_PROJECT_26: "[varying]",
    StiType.STI_PROJECT_27: "[varying]",
    StiType.STI_PROJECT_28: "[varying]",
    StiType.STI_PROJECT_29: "[varying]",
    StiType.STI_PROJECT_30: "[varying]",
    StiType.STI_PROJECT_31: "[varying]",
}

STI_VALUE_OOR_MIN = STI_VALUE_NA + 1

STI_SPECIAL_VALUES = {
    STI_VALUE_NA: "STI_VALUE_NA",
    STI_VALUE_OOR_MAX: "STI_VALUE_OOR_MAX",
    STI_VALUE_OOR_MIN: "STI_VALUE_OOR_MIN",
}

STI_SPECIAL_VALUES_BY_NAME = {v: k for k, v in STI_SPECIAL_VALUES.items()}


def format_sti_value(sti_type: StiType, sti_value: StiValue) -> str:
    """Converts an STI value to string."""
    if sti_value in STI_SPECIAL_VALUES:
        return STI_SPECIAL_VALUES[sti_value]

    try:
        return str(STI_ENUM_MAP[sti_type](sti_value).name)
    except (KeyError, ValueError):
        pass

    return str(sti_value)


def parse_sti_type(sti_type_string: str) -> StiType:
    """Parses an STI type from string."""
    try:
        return StiType[sti_type_string]
    except KeyError:
        pass

    return StiType(int(sti_type_string))


def parse_sti_value(sti_type: StiType, sti_value_string: str) -> StiValue:
    """Parses an STI value from string."""
    try:
        return STI_SPECIAL_VALUES_BY_NAME[sti_value_string]
    except KeyError:
        pass
    try:
        return STI_ENUM_MAP[sti_type][sti_value_string]
    except (KeyError, ValueError):
        pass

    return StiValue(int(sti_value_string))


def print_list() -> None:
    """Lists all possible STI types and values."""
    print("Available STI values:")
    for value in StiType:
        if value == StiType.STI_TYPE_LAST:
            continue
        sti_value_type = "unknown"
        if value in STI_ENUM_MAP:
            sti_value_type = "enum"
        elif value in STI_UNIT_MAP:
            sti_value_type = "integer"

        sti_unit = STI_UNIT_MAP.get(value, "")

        print(f"\t{value.name} (value: {value.value}) (type: {sti_value_type}) {sti_unit}")
        if sti_value_type == "enum":
            for enum_value in STI_ENUM_MAP[value]:
                print(f"\t\t{enum_value.name} (value: {enum_value.value})")
    print("\nSpecial values:")
    for (int_value, str_value) in STI_SPECIAL_VALUES.items():
        print(f"\t{str_value} (value: {int_value})")


def main() -> None:
    if "-l" in argv[1:] or "--list" in argv[1:]:
        print_list()
        return

    parser = ArgumentParser(
        description="Commsignia STI tool. "
        "Inspects and manipulates values in STI via the remote API.\n"
        "\n"
        "Examples for setting a value: \n"
        "\tsti_tool.py STI_LONG_ACC 1000       # Setting the acceleration to 1000 mm/s^2\n"
        "\tsti_tool.py 20 1                    # Setting the emergency brake to ON\n"
        "\tsti_tool.py STI_ABS STI_VALUE_NA    # Setting ABS to NA\n"
        "\n"
        "Example for getting a value:\n"
        "\tsti_tool.py STI_LONG_ACC            # Getting the longitudinal acceleration value\n"
        "\tsti_tool.py 20                      # Getting the emergency brake status\n",
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument("type", type=str, help="STI type (as an integer or enum")
    parser.add_argument(
        "value",
        type=str,
        default=None,
        nargs="?",
        help="STI value (as an integer or enum) [default: get mode]",
    )
    parser.add_argument(
        "--ip", type=str, default="127.0.0.1", help="External Api Server (EAS) IP address [default: 127.0.0.1]"
    )
    parser.add_argument("--port", type=int, default=7942, help="External Api Server (EAS) port [default: 7942]")
    parser.add_argument("-l", "--list", action='store_true', help="Lists all possible STI types and values")
    parser.add_argument("--repeatInterval", type=int, default=0, help="Repeat STI set/get every N ms [default: 0]")
    args = parser.parse_args()

    with create_cms_api(host=args.ip, port=args.port) as api:
        sti_type = parse_sti_type(args.type)
        sti_value = STI_VALUE_NA

        while True:
            if args.value is not None:
                sti_value = parse_sti_value(sti_type, args.value)
                api.sti_set(StiSetItems(items=(StiItem(type=sti_type, value=sti_value),)))
            else:
                result = api.sti_get(StiTypeList(types=(sti_type,)))
                sti_value = result.data.items[0].value

            formatted_value = format_sti_value(sti_type, sti_value)
            print(f"{sti_type.name}={formatted_value}")

            if args.repeatInterval == 0:
                break
            time.sleep(args.repeatInterval / 1000)


if __name__ == "__main__":
    main()
