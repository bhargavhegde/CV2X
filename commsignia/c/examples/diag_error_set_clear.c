/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/diag_error.h>

/** @file
@brief Set and clear error in Diagnostic module.
@ingroup ex
*/


typedef enum project_error_code_t {
    PROJECT_ERROR_CODE_INTERNAL_ERROR = 0,
    PROJECT_ERROR_CODE_INVALID_ARGUMENT,
    /* ... */
} project_error_code_t;


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set an error in the diag module */
    cms_diag_error_set_t error_set = {
        .error_id.code = PROJECT_ERROR_CODE_INTERNAL_ERROR,
        .error_id.group = CMS_DIAG_ERROR_GROUP_PROJECT_02
    };

    error = error || cms_diag_error_set(&session,
                                        &error_set,
                                        NULL);
    if(error) {
        printf("Unable to set diag error\n");
    } else {
        printf("An error is set for group %u with code %u\n", error_set.error_id.group, error_set.error_id.code);
    }

    /* Clear the previously set error */
    cms_diag_error_clear_t error_clear = {
        .error_id.code = error_set.error_id.code,
        .error_id.group = error_set.error_id.group
    };

    const unsigned int delay_us = 100000;
    usleep(delay_us);

    error = error || cms_diag_error_clear(&session, &error_clear, NULL);

    if(error) {
        printf("Unable to clear diag error\n");
    } else {
        printf("An error is cleared for group %u with code %u\n", error_clear.error_id.group, error_clear.error_id.code);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

