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
#include <cms_v2x/nav.h>

/** @file
@brief Subscribe to navigation data.
@ingroup ex
*/


/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t rx_cnt;
} notif_ctx_t;


/* Notification callback to print received message details */
static void nav_notif_cb(const cms_nav_notif_data_t* notif, void* ctx)
{
    if((NULL == notif) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        printf("Notification information:\n");
        printf("    Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("    Rx counter: %lu\n", (unsigned long)notif_ctx->rx_cnt);
        printf("\n");

        printf("Received information:\n");
        printf("    Valid: %s\n", notif->is_valid ? "Yes" : "No");
        printf("    Timestamp: %llu\n", (unsigned long long)notif->timestamp);
        printf("    Leap seconds: %llu\n", (unsigned long long)notif->leap_seconds);
        printf("\n");

        printf("Location information:\n");
        printf("    Latitude [0.1 udeg]: %lu\n", (unsigned long)notif->latitude);
        printf("    Longitude [0.1 udeg]: %lu\n", (unsigned long)notif->longitude);
        printf("    Altitude [mm]: %lu \n", (unsigned long)notif->altitude);
        printf("    Used satellites: %u\n", (unsigned int)notif->number_of_used_satellites);
        printf("\n");

        printf("Location confidence:\n");
        printf("    Altitude [mm]: %lu\n", (unsigned long)notif->altitude_confidence);
        printf("    Ellipse major axis [mm]: %lu\n", (unsigned long)notif->pce_semi_major);
        printf("    Ellipse minor axis [mm]: %lu\n", (unsigned long)notif->pce_semi_minor);
        printf("    Ellipse orientation [0.001 deg]: %lu\n", (unsigned long)notif->pce_orientation);
        printf("\n");

        printf("Heading:\n");
        printf("    Heading [0.001 degree]: %lu\n", (unsigned long)notif->heading);
        printf("    Heading confidence [0.001 degree]: %lu\n", (unsigned long)notif->heading_confidence);
        printf("\n");

        printf("Motion:\n");
        printf("    Speed [mm/s]: %lu\n", (unsigned long)notif->speed);
        printf("    Speed confidence [mm/s]: %lu\n", (unsigned long)notif->speed_confidence);
        printf("    Direction: %s\n", (notif->drive_direction == CMS_NAV_DRIVE_DIR_FORWARD) ? "Forward" :
               (notif->drive_direction == CMS_NAV_DRIVE_DIR_BACKWARD) ? "Backward" : "N/A");
        printf("=====================================================\n");

        ++notif_ctx->rx_cnt;
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
        .param = 55U,        /* just a random parameter */
        .rx_cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_nav_subscribe(&session,
                                       &nav_notif_cb,
                                       &ctx,
                                       &subs_id);
    if(error) {
        printf("Unable to subscribe to nav notifications\n");
    } else {

        /* Wait some messages on the subscription */
        static const uint32_t EXIT_ON_RX_COUNT = 50UL;
        while(ctx.rx_cnt < EXIT_ON_RX_COUNT) {
            sleep(1);
        }
    }

    /* Unsubscribe */
    error = error || cms_nav_unsubscribe(&session, subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

