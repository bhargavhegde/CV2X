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
#include <cms_v2x/sti.h>

/** @file
@brief Subscribe to Station Info notifications.
@ingroup ex
*/

static void sti_notif_cb(const cms_sti_notif_data_t* data, void* ctx)
{
    (void)ctx;

    if(data != NULL) {
        for(uint16_t i = 0; i < data->length; ++i) {
            cms_sti_type_t key = data->items[i].type;
            cms_sti_value_t value = data->items[i].value;
            printf("STI key %d changed to %lld\n", (int)key, (unsigned long long)value);
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
    error = error || cms_sti_subscribe(&session,
                                       &sti_notif_cb,
                                       NULL,
                                       &subs_id);
    if(error) {
        printf("Unable to subscribe for STI notifications\n");
    } else {

        printf("Subscription successful, enter to exit.\n");

        bool finish = false;
        while(!finish) {
            int ch = getchar();
            finish = (ch == '\n');
        }

        cms_sti_unsubscribe(&session, subs_id);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

