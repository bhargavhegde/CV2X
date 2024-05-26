/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cn_bsm.h>

/** @file
@brief Override the event flags in transmitted BSM packets and force predefined values
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    cms_cn_bsm_event_flag_control_t control = {
        .enable = true,
        .flags = {0},
    };

    control.flags.antilock_braking_system = true;
    control.flags.air_bag_deployed = true;
    control.flags.traction_control = true;
    control.flags.wipers_changed = true;

    error = error || cms_cn_bsm_force_event_flags(&session, &control, NULL);

    if(!error) {
        printf("BSM event flags are overridden successfully\n");
    } else {
        printf("Failed to override BSM event flags\n");
    }

    /* Wait some time */
    printf("Wait for 1 second\n");
    sleep(1);

    control.enable = false;
    error = error || cms_cn_bsm_force_event_flags(&session, &control, NULL);

    if(!error) {
        printf("Successfully stopped overriding of BSM event flags\n");
    } else {
        printf("Failed to stop overriding BSM event flags\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
