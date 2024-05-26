/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdatomic.h>

#include <cms_v2x/api.h>
#include <cms_v2x/wdt.h>

/** @file
@brief Subscribe to watchdog data.
@ingroup ex
*/

/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t rx_cnt;
} notif_ctx_t;

static atomic_bool connected = false;
static atomic_uint_least64_t last_notif_time = 0;
static const uint64_t max_time_diff_ms = 2000;
static atomic_bool wdt_status_ok = true;

/** Get current system time */
static uint64_t get_curr_time_ms(void)
{
    uint64_t result = 0U;

    /* get current time at entry */
    struct timespec now = {0};
    if(timespec_get(&now, TIME_UTC) != TIME_UTC) {
        result = (uint64_t)(now.tv_sec) * (uint64_t)1000ULL;
        result += (uint64_t)(now.tv_nsec) / (uint64_t)1000000ULL;
    }

    return result;
}

/* Notification callback */
static void wdt_notif_cb(const cms_wdt_notif_data_t* notif, void* ctx)
{
    (void)ctx;
    if((NULL == notif) || (notif->status == CMS_WDT_STATUS_THREAD_STUCK)) {
        printf("Thread timeout in V2X SW stack, please restart the V2X SW stack\n");
        wdt_status_ok = false;
    } else {
        last_notif_time = get_curr_time_ms();
    }
}

void disconnect_cb(void* context)
{
    (void)context;
    printf("Disconnected\n");
    connected = false;
}

bool connect_and_subscribe(cms_session_t* session,
                           const char* host,
                           notif_ctx_t* notif_ctx,
                           cms_subs_id_t* subs_id)
{
    /* Connect to the host */
    bool error = cms_api_connect(session, host, 2U, &disconnect_cb, NULL);

    /* Subscribe to wdt */
    error = error || cms_wdt_subscribe(session,
                                       &wdt_notif_cb,
                                       notif_ctx,
                                       subs_id);

    if(!error) {
        printf("Connected to Commsignia V2x SW Stack\n");
    } else {
        printf("Unable to connect to Commsignia V2x SW Stack\n");
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

    connected = !connect_and_subscribe(&session, host, &notif_ctx, &subs_id);

    last_notif_time = get_curr_time_ms();
    while(connected && wdt_status_ok) {
        sleep(1);
        uint64_t curr_time = get_curr_time_ms();
        if(last_notif_time + max_time_diff_ms < curr_time) {
            printf("Thread timeout in v2x sw stack or unable to connect to v2x sw stack\n");
            printf("Please restart the v2x sw stack\n");
            wdt_status_ok = false;
        } else {
            printf("V2X SW Stack is running\n");
        }
    }

    /* Unsubscribe and disconnect */
    if(connected) {
        (void)cms_wdt_unsubscribe(&session, subs_id);

        /* Close connection and cleanup */
        cms_api_disconnect(&session);
    }

    cms_api_clean();

    return 0;
}
