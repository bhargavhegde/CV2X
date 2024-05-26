#!/usr/bin/env python3
#
# Copyright (C) Commsignia Ltd. - All Rights Reserved.
# Unauthorised copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Date 2023

from pycmssdk import PsyTriggerParams, PsyTriggerType, create_cms_api

FORCED_TRIGGER = PsyTriggerParams(type=PsyTriggerType(PsyTriggerType.PSY_TRIGGER_FORCED))


def main() -> None:

    with create_cms_api(host='127.0.0.1') as api:
        api.psy_trigger(FORCED_TRIGGER)


if __name__ == '__main__':
    main()
