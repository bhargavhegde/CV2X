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
@brief Enable YD BSM sending
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    cms_cn_bsm_tx_state_t cn_bsm_tx_state = {
        .enable = true      /* YD BSM send enable */
    };

    /* Enable YD BSM sending */
    error = error || cms_cn_bsm_tx_enable(&session, &cn_bsm_tx_state, NULL);

    if(!error) {
        printf("BSM tx successfully %s\n", cn_bsm_tx_state.enable ? "enabled" : "disabled");
    } else {
        printf("Unable to %s BSM sending\n", cn_bsm_tx_state.enable ? "enable" : "disable");
    }

    /* Wait some time */
    sleep(1);

    /* Disable YD BSM sending */
    cn_bsm_tx_state.enable = false;
    error = error || cms_cn_bsm_tx_enable(&session, &cn_bsm_tx_state, NULL);

    if(!error) {
        printf("BSM tx successfully %s\n", cn_bsm_tx_state.enable ? "enabled" : "disabled");
    } else {
        printf("Unable to %s BSM sending\n", cn_bsm_tx_state.enable ? "enable" : "disable");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
