/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#include <stdbool.h>
#include <stdio.h>

#include <cms_v2x/api.h>
#include <cms_v2x/psy.h>

/** @file
@brief Trigger pseudonymity certificate change.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);
    cms_psy_trigger_params_t params = {.type = CMS_PSY_TRIGGER_FORCED};
    error = error || cms_psy_trigger(&session, &params, NULL);
    if(error) {
        printf("Unable to trigger pseudonymity certificate change\n");
    } else {
        printf("Triggered pseudonymity certificate change\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
