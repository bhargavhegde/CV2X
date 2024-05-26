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
#include <cms_v2x/licence.h>

/** @file
@brief Get the enabled software components.
@ingroup ex
*/


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* You can enable verbose logging with uncommenting the next lines */
    /* cms_api_init(); */
    /* cms_api_set_logging(CMS_API_LOG_INFO); */

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Get licence data */
    cms_licence_data_t data = {0};
    error = error || cms_get_licence_data(&session, NULL, &data);

    if(!error) {
        printf("Upl licence flags:\n");
        printf("  Upl modules: %016llx\n", (unsigned long long)data.modules);
        printf("  Cff flags: %016llx\n", (unsigned long long)data.cff_flags);
        printf("  App flags: %016llx\n", (unsigned long long)data.app_flags);
    } else {
        printf("Unable to get UPL licence flags\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

