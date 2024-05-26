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

/* Context type for the disconnect callback */
typedef struct connect_ctx_t {
    uint64_t client_id;
    bool is_connected;
} connect_ctx_t;

/* Context type for the notification callback */
typedef struct notif_ctx {
    uint64_t client_id;
    uint32_t rx_cnt;
} notif_ctx_t;


/* Notification callback */
static void nav_notif_cb(const cms_nav_notif_data_t* notif, void* ctx)
{
    if((NULL == notif) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {
        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        ++notif_ctx->rx_cnt;
        printf("Notification %u received from client %llu\n",
               (unsigned)notif_ctx->rx_cnt,
               (unsigned long long)notif_ctx->client_id);
    }
}

/* Disconnect callback */
void disconnect_cb(void* context)
{
    connect_ctx_t* conn_ctx = (connect_ctx_t*)context;
    conn_ctx->is_connected = false;
    printf("DISCONNECTED (client ID=%llu)\n",
           (unsigned long long)conn_ctx->client_id);
}

bool connect_and_subscribe(cms_session_t* session, const char* host, connect_ctx_t* conn_ctx, notif_ctx_t* notif_ctx,
                           cms_subs_id_t* subs_id)
{
    /* Connect to the host */
    bool error = cms_api_connect(session, host, 2U, &disconnect_cb, conn_ctx);

    /* Subscribe to nav messages */
    error = error || cms_nav_subscribe(session,
                                       &nav_notif_cb,
                                       notif_ctx,
                                       subs_id);
    if(!error) {
        conn_ctx->is_connected = true;
        printf("Connected to API server on %s\n", host);
    } else {
        printf("Failed to connect to %s\n", host);
    }
    return error;
}


int main(int argc, char* argv[])
{
    const char* host1 = (argc > 1) ? argv[1] : "127.0.0.1";
    const char* host2 = (argc > 2) ? argv[2] : "127.0.0.1";

    /* Create sessions */
    cms_session_t session1 = cms_get_session();
    cms_session_t session2 = cms_get_session();

    /* Initialize contexts */
    connect_ctx_t conn_ctx1 = {
        .client_id = session1.client_id,
        .is_connected = false
    };
    connect_ctx_t conn_ctx2 = {
        .client_id = session2.client_id,
        .is_connected = false
    };

    notif_ctx_t notif_ctx1 = {
        .client_id = session1.client_id,
        .rx_cnt = 0U
    };
    notif_ctx_t notif_ctx2 = {
        .client_id = session2.client_id,
        .rx_cnt = 0U
    };

    cms_subs_id_t subs_id1 = CMS_SUBS_ID_INVALID;
    cms_subs_id_t subs_id2 = CMS_SUBS_ID_INVALID;

    connect_and_subscribe(&session1, host1, &conn_ctx1, &notif_ctx1, &subs_id1);
    connect_and_subscribe(&session2, host2, &conn_ctx2, &notif_ctx2, &subs_id2);

    /* Wait some messages on the subscription */
    static const uint32_t EXIT_ON_RX_COUNT = 50UL;
    while((notif_ctx1.rx_cnt < EXIT_ON_RX_COUNT) || (notif_ctx2.rx_cnt < EXIT_ON_RX_COUNT)) {
        if(!conn_ctx1.is_connected) {
            printf("Reconnecting... %s\n", host1);
            connect_and_subscribe(&session1, host1, &conn_ctx1, &notif_ctx1, &subs_id1);
        }

        if(!conn_ctx2.is_connected) {
            printf("Reconnecting... %s\n", host2);
            connect_and_subscribe(&session2, host2, &conn_ctx2, &notif_ctx2, &subs_id2);
        }

        sleep(1);
    }

    /* Unsubscribe and disconnect */
    if(conn_ctx1.is_connected) {
        (void)cms_nav_unsubscribe(&session1, subs_id1);

        /* Close connection and cleanup */
        cms_api_disconnect(&session1);
    }
    if(conn_ctx2.is_connected) {
        (void)cms_nav_unsubscribe(&session2, subs_id2);

        /* Close connection and cleanup */
        cms_api_disconnect(&session2);
    }

    cms_api_clean();

    return 0;
}
