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
#include <cms_v2x/nav.h>

/** @file
@brief Subscribe to navigation data.
@ingroup ex
*/

/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t rx_cnt;
} notif_ctx_t;

static bool l_connected = false;

/* Notification callback */
static void nav_notif_cb(const cms_nav_notif_data_t* notif, void* ctx)
{
    if((NULL == notif) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        ++notif_ctx->rx_cnt;
        printf("Notification %u received\n", notif_ctx->rx_cnt);
    }
}

void disconnect_cb(void* context)
{
    (void)context;
    printf("Disconnected\n");
    l_connected = false;
}

bool connect_and_subscribe(cms_session_t* session,
                           const char* host,
                           notif_ctx_t* notif_ctx,
                           cms_subs_id_t* subs_id)
{
    /* Connect to the host */
    bool error = cms_api_connect(session, host, 2U, &disconnect_cb, NULL);

    /* Subscribe to nav messages */
    error = error || cms_nav_subscribe(session,
                                       &nav_notif_cb,
                                       notif_ctx,
                                       subs_id);

    if(!error) {
        printf("Connected to API server\n");
    }

    return error;
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create sessions */
    cms_session_t session = cms_get_session();

    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    notif_ctx_t notif_ctx = {
        .rx_cnt = 0U
    };

    l_connected = !connect_and_subscribe(&session, host, &notif_ctx, &subs_id);

    /* Wait some messages on the subscription */
    static const uint32_t EXIT_ON_RX_COUNT = 50UL;
    while(notif_ctx.rx_cnt < EXIT_ON_RX_COUNT) {
        if(!l_connected) {
            printf("Reconnecting...\n");
            l_connected = !connect_and_subscribe(&session, host, &notif_ctx, &subs_id);
        }

        sleep(1);
    }

    /* Unsubscribe and disconnect */
    if(l_connected) {
        (void)cms_nav_unsubscribe(&session, subs_id);

        /* Close connection and cleanup */
        cms_api_disconnect(&session);
    }

    cms_api_clean();

    return 0;
}
