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
#include <cms_v2x/sdsm.h>

/** @file
@brief Delete a single perceived object from the temporary object storage.
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set input parameters */
    cms_sdsm_delete_object_t object = {0};
    object.id = 1U;

    /* Delete object */
    error = error || cms_sdsm_delete_object(&session, &object, NULL);

    /* Check result */
    if(error) {
        printf("Unable to delete object with ID: %hu\n", (unsigned short)object.id);
    } else {
        printf("Object (%hu) successfully deleted.\n", (unsigned short)object.id);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
