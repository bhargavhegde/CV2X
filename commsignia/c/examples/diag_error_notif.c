/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/diag_error.h>

/** @file
@brief Subscribe to Diagnostic module error notifications.
@ingroup ex
*/

static void diag_error_notif_cb(const cms_diag_error_notif_data_t* data, void* ctx)
{
    (void)ctx;

    if(data != NULL) {
        if(true == data->stat.state) {
            printf("Diag error is set for group %u with code %u\n", data->id.group, data->id.code);
        } else {
            printf("Diag error is cleared for group %u with code %u\n", data->id.group, data->id.code);
        }
    }
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Subscribe for notifications */
    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_diag_error_subscribe(&session,
                                              &diag_error_notif_cb,
                                              NULL,
                                              &subs_id);
    if(error) {
        printf("Unable to subscribe for Diag error notifications\n");
    } else {

        printf("Subscription successful, enter to exit.\n");

        bool finish = false;
        while(!finish) {
            int ch = getchar();
            finish = (ch == '\n');
        }

        cms_diag_error_unsubscribe(&session, subs_id);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

