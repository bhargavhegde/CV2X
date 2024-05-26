/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/fac_types.h>
#include <cms_v2x/fac_subscribe.h>

/** @file
@brief Subscribe to facility messages.
@ingroup ex
*/


/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t cnt;
} notif_ctx_t;

/* Notification callback to print received message details */
static void fac_notif_cb(cms_fac_msg_type_t type,
                         const cms_fac_notif_data_t* notif,
                         cms_buffer_view_t msg,
                         void* ctx)
{
    if((NULL == notif) || (NULL == msg.data) || (0UL == msg.length) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;

        printf("Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("Receive counter: %lu\n", (unsigned long)notif_ctx->cnt);
        printf("Type: %d\n", type);
        printf("Timestamp: %llu [ms]\n",
               (unsigned long long)notif->radio.timestamp);
        printf("Source address: %02X:%02X:%02X:%02X:%02X:%02X\n",
               notif->radio.source_address[0],
               notif->radio.source_address[1],
               notif->radio.source_address[2],
               notif->radio.source_address[3],
               notif->radio.source_address[4],
               notif->radio.source_address[5]);
        printf("Verification result: %s (%lu)\n",
               cms_sec_verify_result_to_string(notif->security.verify_result),
               (unsigned long)notif->security.verify_result);
        printf("Data:\n");

        for(uint32_t i = 0UL; i < msg.length; ++i) {
            printf("%02X", msg.data[i]);
        }

        printf("\n");
        printf("=====================================================\n");

        ++notif_ctx->cnt;
    }
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Create a context for the subscription callback */
    notif_ctx_t bsm_ctx = {
        .param = 1U,        /* just a random parameter */
        .cnt = 0U
    };

    /* Subscribe to BSMs */
    cms_subs_id_t bsm_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_fac_subscribe(&session,
                                       (cms_fac_subscribed_msg_t)CMS_FAC_MSG_US_BSM,
                                       fac_notif_cb,
                                       &bsm_ctx,
                                       &bsm_subs_id);
    if(error) {
        printf("Unable to subscribe to BSMs\n");
    }

    /* Create a context for the subscription callback */
    notif_ctx_t wildcard_ctx = {
        .param = 2U,        /* just a random parameter */
        .cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t wildcard_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_fac_subscribe(&session,
                                       CMS_FAC_SUBSCRIBE_ALL,
                                       &fac_notif_cb,
                                       &wildcard_ctx,
                                       &wildcard_subs_id);
    if(error) {
        printf("Unable to subscribe to all facility notifications\n");

    } else {

        /* Wait some messages on the wildcard subscription */
        static const uint32_t EXIT_ON_RECV_COUNT = 50UL;
        while(wildcard_ctx.cnt < EXIT_ON_RECV_COUNT) {
            sleep(1);
        }
    }

    /* Unsubscribe */
    error = error || cms_fac_unsubscribe(&session, bsm_subs_id);
    error = error || cms_fac_unsubscribe(&session, wildcard_subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

