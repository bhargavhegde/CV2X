/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/bsm.h>

/** @file
@brief Enable BSM sending
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    cms_bsm_tx_state_t bsm_tx_enable = {
        .enable = true      /* BSM send enable */
    };

    /* Enable BSM sending */
    error = error || cms_bsm_tx_enable(&session, &bsm_tx_enable, NULL);

    if(error) {
        printf("Unable to %s BSM sending\n", (bsm_tx_enable.enable == true) ? "enable" : "disable");
    } else {
        printf("BSM Tx successfully %s.\n", (bsm_tx_enable.enable == true) ? "enabled" : "disabled");
    }

    /* Wait some time */
    sleep(1);

    /* Disable BSM sending */
    bsm_tx_enable.enable = false;
    error = error || cms_bsm_tx_enable(&session, &bsm_tx_enable, NULL);

    if(error) {
        printf("Unable to %s BSM sending\n", (bsm_tx_enable.enable == true) ? "enable" : "disable");
    } else {
        printf("BSM Tx successfully %s.\n", (bsm_tx_enable.enable == true) ? "enabled" : "disabled");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

