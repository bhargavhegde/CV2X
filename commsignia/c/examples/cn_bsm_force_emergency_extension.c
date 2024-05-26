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
@brief Force emergency extension in sent YD BSM messages.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Fill emergency extension for a garbage truck as an example */
    cms_cn_bsm_emergency_ext_control_t control = {
        .enable = true,
        .emergency_ext = {
            .lightbar_use = CMS_CN_BSM_LIGHTBAR_YELLOW_CAUTION_LIGHTS,
            .siren_use = CMS_CN_BSM_SIREN_NOT_IN_USE,
            .response_type = CMS_CN_BSM_RESPONSE_TYPE_STOP_AND_GO_MOVEMENT,
        },
    };

    /* Start forcing of the emergency extension */
    error = error || cms_cn_bsm_force_emergency_ext(&session, &control, NULL);
    if(error) {
        printf("Failed to start forcing the CN BSM emergency extension\n");
    } else {
        printf("Successfully started forcing the CN BSM emergency extension\n");
    }
    /* Wait for a while */
    printf("Waiting for 1 second\n");
    sleep(1);


    /* Stop forcing of the emergency extension */
    control.enable = false;
    error = error || cms_cn_bsm_force_emergency_ext(&session, &control, NULL);
    if(error) {
        printf("Failed to stop forcing the CN BSM emergency extension\n");
    } else {
        printf("Successfully stopped forcing the CN BSM emergency extension\n");
    }


    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
