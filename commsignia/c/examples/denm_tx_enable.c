/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/denm.h>

/** @file
@brief Enable DENM Tx
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    cms_denm_tx_state_t state = {
        .enable = true
    };

    /* Enable DENM Tx */
    printf("Requesting DENM Tx module %s...\n", (state.enable == true) ? "enable" : "disable");
    error = error || cms_denm_tx_enable(&session, &state, NULL);
    if(error) {
        printf("Failed.\n");
    }

    if(!error) {
        printf("Getting is DENM Tx module enabled...\n");
        error = cms_denm_tx_get_enabled(&session, NULL, &state);
        if(error) {
            printf("Failed.\n");
        } else {
            printf("DENM Tx module is %s.\n\n", (state.enable == true) ? "enabled" : "disabled");
        }
    }

    if(!error) {
        /* Wait some time */
        sleep(3);

        /* Disable DENM Tx */
        state.enable = false;

        printf("Requesting DENM Tx module %s...\n", (state.enable == true) ? "enable" : "disable");
        error = cms_denm_tx_enable(&session, &state, NULL);
        if(error) {
            printf("Failed.\n");
        }
    }

    if(!error) {
        printf("Getting is DENM Tx module enabled...\n");
        error = cms_denm_tx_get_enabled(&session, NULL, &state);
        if(error) {
            printf("Failed.\n");
        } else {
            printf("DENM Tx module is %s.\n\n", (state.enable == true) ? "enabled" : "disabled");
        }
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
