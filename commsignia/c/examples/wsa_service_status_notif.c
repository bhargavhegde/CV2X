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
#include <cms_v2x/wsa.h>

/** @file
@brief Subscribe to WAVE Service Advertisement status notifications.
@ingroup ex
*/

static void wsa_service_status_notify_cb(cms_psid_t psid,
                                         const cms_wsa_service_status_notif_data_t* data,
                                         cms_buffer_view_t msg,
                                         void* ctx)
{
    (void)msg;
    (void)ctx;

    if(data != NULL) {
        bool active = data->active;
        if(active) {
            printf("Service with psid (0x%08X) is available\n", psid);
        } else {
            printf("Service with psid (0x%08X) is unavailable\n", psid);
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

    /* Subscribe for WSA service status notifications */
    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    cms_psid_t psid = 0x82UL;
    error = error || cms_wsa_service_status_subscribe(&session,
                                                      psid,
                                                      &wsa_service_status_notify_cb,
                                                      NULL,
                                                      &subs_id);
    if(error) {
        printf("Unable to subscribe for WSA service status notifications\n");
    } else {
        printf("Subscription successful, enter to exit.\n");

        bool finish = false;
        while(!finish) {
            int ch = getchar();
            finish = (ch == '\n');
        }

        cms_wsa_service_status_unsubscribe(&session, subs_id);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

