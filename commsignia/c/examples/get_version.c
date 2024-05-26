/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#include <stdio.h>
#include <stdbool.h>

#include <cms_v2x/api.h>
#include <cms_v2x/stat.h>

/**
@defgroup ex Examples
*/

/** @file
@brief Get the version of the running V2X stack.
@ingroup ex
*/


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);
    if(error) {
        printf("Unable to connect to the V2X stack\n");
    } else {

        /* Get status information, including currently running version */
        cms_stat_device_status_t data = {0};
        error = cms_stat_get_device_status(&session, NULL, &data);
        if(error) {
            /* An error happened during the API call */
            printf("Unable to retrieve device status.\n");
        } else {
            printf("The version of the running stack: %s\n", data.device_data.version_info);
        }

        /* Close connection */
        cms_api_disconnect(&session);
    }

    /* Clean up */
    cms_api_clean();

    return error;
}

