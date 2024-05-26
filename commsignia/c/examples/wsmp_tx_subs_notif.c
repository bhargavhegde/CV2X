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
#include <cms_v2x/wsmp.h>

/** @file
@brief Subscribes to transmitted WSMP messages.
@ingroup ex
*/


/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t cnt;
} notif_ctx_t;


/* Notification callback to print received message details */
static void wsmp_notif_cb(cms_psid_t psid,
                          const cms_wsmp_tx_notif_data_t* notif,
                          cms_buffer_view_t msg,
                          void* ctx)
{
    if((NULL == notif) || (NULL == msg.data) || (0UL == msg.length) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        printf("Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("Tx counter: %lu\n", (unsigned long)notif_ctx->cnt);
        printf("PSID key: %lu\n", (unsigned long)psid);
        printf("Datarate: %u [kbps]\n", notif->radio.datarate);
        printf("Destination address: %02X:%02X:%02X:%02X:%02X:%02X\n",
               notif->radio.dest_address[0],
               notif->radio.dest_address[1],
               notif->radio.dest_address[2],
               notif->radio.dest_address[3],
               notif->radio.dest_address[4],
               notif->radio.dest_address[5]);
        printf("Interface ID: %lu\n", (unsigned long)notif->radio.interface_id);
        printf("Tx Power: %d [dBm]\n", notif->radio.tx_power);
        printf("User Priority: %u\n", (unsigned)notif->radio.user_prio);
        printf("Expiry Time: %u [ms]\n", (unsigned)notif->radio.expiry_time);
        printf("SPS Channeld Index: %lu\n", (unsigned long)notif->radio.sps_index);

        printf("PSID: %lu\n", (unsigned long)notif->wsmp_hdr.psid);
        printf("Datarate WSMP extension: %spresent\n",
               notif->wsmp_hdr.datarate ? "" : "not ");
        printf("TxPower in WSMP extension: %spresent\n",
               notif->wsmp_hdr.tx_power ? "" : "not ");
        printf("Channel ID in WSMP extension: %spresent\n",
               notif->wsmp_hdr.channel_id ? "" : "not ");

        const cms_sec_dot2_tx_sign_info_t* sign_info = &notif->security.sign_info;
        printf("Signing method: %s\n",
               (CMS_SIGN_METH_NONE == sign_info->sign_method) ? "none" :
               (CMS_SIGN_METH_SIGN == sign_info->sign_method) ? "sign" :
               (CMS_SIGN_METH_SIGN_CERT == sign_info->sign_method) ? "sign, include certificate" :
               "UNKNOWN");
        printf("PSID in security notif: %lu (0x%lX)\n",
               (unsigned long)sign_info->psid, (unsigned long)sign_info->psid);
        printf("SSP length: %lu\n", (unsigned long)sign_info->ssp.length);
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
    notif_ctx_t filtered_ctx = {
        .param = 1U,        /* just a random parameter */
        .cnt = 0U
    };

    /* Subscribe to a specific PSID */
    static const uint64_t FILTER_PSID = 0x20UL;
    cms_subs_id_t filtered_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_wsmp_tx_subscribe(&session,
                                           FILTER_PSID,
                                           &wsmp_notif_cb,
                                           &filtered_ctx,
                                           &filtered_subs_id);
    if(error) {
        printf("Unable to subscribe to PSID 0x%llx\n", (unsigned long long)FILTER_PSID);
    }

    /* Create a context for the subscription callback */
    notif_ctx_t wildcard_ctx = {
        .param = 2U,        /* just a random parameter */
        .cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t wildcard_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_wsmp_tx_subscribe(&session,
                                           CMS_WSMP_SUBSCRIBE_WILDCARD,
                                           &wsmp_notif_cb,
                                           &wildcard_ctx,
                                           &wildcard_subs_id);
    if(error) {
        printf("Unable to subscribe to all WSMP notifications\n");
    } else {

        /* Wait some messages on the wildcard subscription */
        static const uint32_t EXIT_ON_TX_COUNT = 50UL;
        while(wildcard_ctx.cnt < EXIT_ON_TX_COUNT) {
            sleep(1);
        }
    }

    /* Unsubscribe */
    error = error || cms_wsmp_tx_unsubscribe(&session, filtered_subs_id);
    error = error || cms_wsmp_tx_unsubscribe(&session, wildcard_subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

