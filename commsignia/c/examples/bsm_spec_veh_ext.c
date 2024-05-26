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
#include <cms_v2x/bsm.h>

/** @file
@brief Set/reset BSM Special Vehicle Extension
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* UPER encoded BSM Special Vehicle Extension data */
    uint8_t payload_ext[] = {0x40, 0x49, 0x00};

    cms_buffer_view_t bsm_ext_encoded = {
        .data = payload_ext,
        .length = sizeof(payload_ext)
    };

    /* Set BSM Special Vehicle Extension */
    error = error || cms_bsm_set_spec_veh_extension(&session, NULL, bsm_ext_encoded, NULL);

    if(error) {
        printf("Unable to set BSM Special Vehicle Extension\n");
    } else {
        printf("BSM Special Vehicle Extension set successful\n");
    }

    /* Wait some time */
    sleep(1);

    /* Reset BSM Special Vehicle Extension  */
    error = error || cms_bsm_reset_spec_veh_extension(&session, NULL, NULL);

    if(error) {
        printf("Unable to reset BSM Special Vehicle Extension\n");
    } else {
        printf("BSM Special Vehicle Extension reset successful\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

