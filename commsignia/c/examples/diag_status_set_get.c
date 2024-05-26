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
#include <cms_v2x/diag_status.h>

/** @file
@brief Set and get status in Diagnostic module.
@ingroup ex
*/


typedef enum project_status_code_t {
    PROJECT_STATUS_CODE_INPUT_STATUS = 0,
    PROJECT_STATUS_CODE_OUTPUT_STATUS,
    /* ... */
} project_status_code_t;


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set a status in the diag module */
    const cms_diag_status_id_t STATUS_ID = {
        .code = PROJECT_STATUS_CODE_INPUT_STATUS,
        .group = CMS_DIAG_STATUS_GROUP_PROJECT_02
    };

    const cms_diag_status_value_t STATUS_VALUE = 42;

    cms_diag_status_set_t status_set = {
        .id = STATUS_ID,
        .value = STATUS_VALUE
    };

    error = error || cms_diag_status_set(&session,
                                         &status_set,
                                         NULL);
    if(error) {
        printf("Unable to set diag status\n");
    } else {
        printf("A status is set for group %u with code %u and value %u\n",
               status_set.id.group,
               status_set.id.code,
               status_set.value);
    }

    /* Get the previously set status */
    cms_diag_status_id_list_t status_id_list = {
        .length = 1,
        .items[0] = STATUS_ID
    };

    cms_diag_status_list_t status_list = {0};

    const unsigned int delay_us = 100000;
    usleep(delay_us);

    error = error || cms_diag_status_get(&session, &status_id_list, &status_list);

    if(error) {
        printf("Unable to get diag status\n");
    } else {
        printf("The status value of group %u with code %u is %u\n",
               status_list.items[0].id.group,
               status_list.items[0].id.code,
               status_list.items->value);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

