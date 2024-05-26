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
#include <cms_v2x/gn.h>

/** @file
@brief Subscribe to transmitted GeoNet messages.
@ingroup ex
*/

/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t cnt;
} notif_ctx_t;

/* Helper function to print hex buffer */
static inline void print_hex_buff(const uint8_t* buff, uint32_t length)
{
    if(NULL != buff) {
        for(uint32_t i = 0UL; i < length; ++i) {
            printf("%02X", buff[i]);
        }
        printf("\n");
    }
}

/* Notification callback to print received message details */
static void gn_tx_notif_cb(cms_btp_port_t btp_port,
                           const cms_gn_tx_notif_data_t* notif,
                           cms_buffer_view_t msg,
                           void* ctx)
{
    if((NULL == notif) || (NULL == msg.data) || (0UL == msg.length) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;

        printf("Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("Tx counter: %lu\n", (unsigned long)notif_ctx->cnt);
        printf("Datarate: %u [kbps]\n", notif->radio.datarate);
        printf("Destination address: %02X:%02X:%02X:%02X:%02X:%02X\n",
               notif->radio.dest_address[0],
               notif->radio.dest_address[1],
               notif->radio.dest_address[2],
               notif->radio.dest_address[3],
               notif->radio.dest_address[4],
               notif->radio.dest_address[5]);
        printf("BTP destination port in notification notif 0x%04x \n", btp_port);
        printf("BTP destination port 0x%04x \n", notif->gn_params.btp_params.btp_port);
        printf("Interface ID: %lu\n", (unsigned long)notif->radio.interface_id);
        printf("Tx Power: %d [dBm]\n", notif->radio.tx_power);
        printf("User Priority: %u\n", (unsigned)notif->radio.user_prio);
        printf("Expiry Time: %u [ms]\n", (unsigned)notif->radio.expiry_time);
        printf("SPS Channel Index: %lu\n", (unsigned long)notif->radio.sps_index);

        const cms_sec_dot2_tx_sign_info_t* sign_info = &notif->security.sign_info;
        printf("Signing method: %s\n",
               (CMS_SIGN_METH_NONE == sign_info->sign_method) ? "none" :
               (CMS_SIGN_METH_SIGN == sign_info->sign_method) ? "sign" :
               (CMS_SIGN_METH_SIGN_CERT == sign_info->sign_method) ? "sign, include certificate" :
               "UNKNOWN");
        printf("PSID in security notif: %lu (0x%lX)\n",
               (unsigned long)sign_info->psid, (unsigned long)sign_info->psid);
        if(sign_info->ssp.length > 0UL) {
            printf("SSP: ");
            print_hex_buff(sign_info->ssp.ssp_field, sign_info->ssp.length);
        }
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
    notif_ctx_t ctx = {
        .param = 3U,
        .cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_gn_tx_subscribe(&session,
                                         CMS_GN_SUBSCRIBE_WILDCARD,
                                         &gn_tx_notif_cb,
                                         &ctx,
                                         &subs_id);
    if(error) {
        printf("Unable to subscribe to GeoNet Tx notifications\n");
    } else {

        /* Wait some messages on the subscription */
        static const uint32_t EXIT_ON_TX_COUNT = 50UL;
        while(ctx.cnt < EXIT_ON_TX_COUNT) {
            sleep(1);
        }
    }

    /* Unsubscribe */
    error = error || cms_gn_tx_unsubscribe(&session, subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

