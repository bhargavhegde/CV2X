/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/radio.h>

/** @file
@brief Subscribe to transmitted Radio messages.
@ingroup ex
*/


/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t cnt;
} notif_ctx_t;


/* Notification callback to print transmitted message details */
static void rio_notif_cb(cms_rio_packet_t packet_type,
                         const cms_rio_tx_notif_data_t* notif,
                         cms_buffer_view_t msg,
                         void* ctx)
{
    if((NULL == notif) || (NULL == msg.data) || (0UL == msg.length) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        printf("Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("Transmit counter: %lu\n", (unsigned long)notif_ctx->cnt);

        printf("Packet type: %u\n", packet_type);

        printf("Interface ID: %lu\n", (unsigned long)notif->radio_params.interface_id);
        printf("Destination address: %02X:%02X:%02X:%02X:%02X:%02X\n",
               notif->radio_params.dest_address[0],
               notif->radio_params.dest_address[1],
               notif->radio_params.dest_address[2],
               notif->radio_params.dest_address[3],
               notif->radio_params.dest_address[4],
               notif->radio_params.dest_address[5]);
        printf("Datarate: %u [kbps]\n", notif->radio_params.datarate);
        printf("User Priority: %u\n", notif->radio_params.user_prio);
        printf("Tx Power: %d\n", notif->radio_params.tx_power);
        printf("Expiry time: %u\n", notif->radio_params.expiry_time);
        printf("SPS index: %u\n", notif->radio_params.sps_index);

        (void)printf("Payload: ");
        for(uint8_t i = 0; i < msg.length; ++i) {
            (void)printf("%02X", msg.data[i] & 0xFF);
        }
        (void)printf("\n");
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

    static const cms_rio_subscribed_packet_t FILTER_PACKET = (cms_rio_subscribed_packet_t)CMS_RIO_GNP_PACKET;
    cms_subs_id_t filtered_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_rio_tx_subscribe(&session,
                                          FILTER_PACKET,
                                          &rio_notif_cb,
                                          &filtered_ctx,
                                          &filtered_subs_id);

    if(error) {
        printf("Unable to subscribe to radio tx notifications for packet type: %u\n", FILTER_PACKET);
    }

    /* Create a context for the subscription callback */
    notif_ctx_t all_packet_ctx = {
        .param = 2U,        /* just a random parameter */
        .cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t all_packet_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_rio_tx_subscribe(&session,
                                          CMS_RIO_SUBSCRIBE_ALL,
                                          &rio_notif_cb,
                                          &all_packet_ctx,
                                          &all_packet_subs_id);
    if(error) {
        printf("Unable to subscribe to all radio tx notifications\n");
    } else {

        /* Wait some messages on the wildcard subscription */
        static const uint32_t EXIT_ON_TRANSMIT_COUNT = 50UL;
        while(all_packet_ctx.cnt < EXIT_ON_TRANSMIT_COUNT) {
            sleep(1);
        }
    }

    /* Unsubscribe */
    error = error || cms_rio_tx_unsubscribe(&session, filtered_subs_id);
    error = error || cms_rio_tx_unsubscribe(&session, all_packet_subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
