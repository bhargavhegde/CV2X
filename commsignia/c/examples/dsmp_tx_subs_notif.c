/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/dsmp.h>

/** @file
@brief Subscribes to transmitted DSMP messages.
@ingroup ex
*/

typedef enum context_t {
    CONTEXT_FILTER_AID = 1,
    CONTEXT_ALL_AID
} context_t;

/* Context type for the notification callback */
typedef struct notif_ctx {
    context_t param;
    uint32_t tx_cnt;
} notif_ctx_t;

static void print_byte_array(const uint8_t* buf, uint32_t len);
static void print_mac_address(const cms_mac_addr_t mac_address);
static void print_ipv6_address(const cms_ipv6_addr_t ip_address);
static void print_radio_params(const cms_radio_tx_params_t* radio);
static void print_dsmp_params(const cms_dsmp_hdr_info_t* dsmp_hdr);

/* Notification callback to print received message details */
static void dsmp_tx_notif_cb(cms_psid_t aid,
                             const cms_dsmp_tx_notif_data_t* notif,
                             cms_buffer_view_t msg,
                             void* ctx)
{
    if((NULL == notif) || (NULL == msg.data) || (0UL == msg.length) || (NULL == ctx)) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else {

        notif_ctx_t* notif_ctx = (notif_ctx_t*)ctx;
        printf("Context: %lu\n", (unsigned long)notif_ctx->param);
        printf("Tx counter: %lu\n", (unsigned long)notif_ctx->tx_cnt);
        printf("AID key: %lu\n", (unsigned long)aid);

        print_radio_params(&notif->radio);
        print_dsmp_params(&notif->dsmp_hdr);

        printf("=====================================================\n");

        ++notif_ctx->tx_cnt;
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
        .param = CONTEXT_FILTER_AID,        /* just a random parameter */
        .tx_cnt = 0U
    };

    /* Subscribe to a specific AID */
    static const uint64_t FILTER_AID = 0x11UL;
    cms_subs_id_t filtered_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_dsmp_tx_subscribe(&session,
                                           FILTER_AID,
                                           &dsmp_tx_notif_cb,
                                           &filtered_ctx,
                                           &filtered_subs_id);
    if(error) {
        printf("Unable to subscribe to AID 0x%llx\n", (unsigned long long)FILTER_AID);
    }

    /* Create a context for the subscription callback */
    notif_ctx_t wildcard_ctx = {
        .param = CONTEXT_ALL_AID,        /* just a random parameter */
        .tx_cnt = 0U
    };

    /* Subscribe to all messages */
    cms_subs_id_t wildcard_subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_dsmp_tx_subscribe(&session,
                                           CMS_DSMP_SUBSCRIBE_WILDCARD,
                                           &dsmp_tx_notif_cb,
                                           &wildcard_ctx,
                                           &wildcard_subs_id);
    if(!error) {
        /* Wait some messages on the wildcard subscription */
        static const uint32_t EXIT_ON_TX_COUNT = 50UL;
        while(wildcard_ctx.tx_cnt < EXIT_ON_TX_COUNT) {
            sleep(1);
        }
    } else {
        printf("Unable to subscribe to all DSMP notifications\n");
    }

    /* Unsubscribe */
    error = error || cms_dsmp_tx_unsubscribe(&session, filtered_subs_id);
    error = error || cms_dsmp_tx_unsubscribe(&session, wildcard_subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

/* Print out the given radio header info */
void print_radio_params(const cms_radio_tx_params_t* radio)
{
    printf("Datarate: %hu [kbps]\n", (unsigned short)radio->datarate);
    printf("Destination address: ");
    print_mac_address(radio->dest_address);
    printf("Interface ID: %lu\n", (unsigned long)radio->interface_id);
    printf("Tx Power: %hi [dBm]\n", (short int)radio->tx_power);
    printf("User Priority: %hhu\n", (unsigned char)radio->user_prio);
    printf("Expiry Time: %hu [ms]\n", (unsigned short)radio->expiry_time);
    printf("SPS channel index: %lu\n", (unsigned long)radio->sps_index);
}

/* Print out the given DSMP header info */
void print_dsmp_params(const cms_dsmp_hdr_info_t* dsmp_hdr)
{
    printf("AID: %lu\n", (unsigned long)dsmp_hdr->aid);
    if(dsmp_hdr->extension.location_2d_option) {
        printf("2D location in DSMP extension: ");
        print_byte_array(dsmp_hdr->extension.location_2d,
                         CMS_DSMP_EXT_2D_LOC_LENGTH);
    }
    if(dsmp_hdr->extension.location_3d_option) {
        printf("3D location in DSMP extendsion: ");
        print_byte_array(dsmp_hdr->extension.location_3d,
                         CMS_DSMP_EXT_3D_LOC_LENGTH);
    }
    if(dsmp_hdr->extension.advertiser_identifier_option) {
        printf("Advertiser identifier in DSMP extension: ");
        print_byte_array(dsmp_hdr->extension.advertiser_identifier.data,
                         dsmp_hdr->extension.advertiser_identifier.length);
    }
    if(dsmp_hdr->extension.provider_srv_ctx_option) {
        printf("Provider service context in DSMP extension: ");
        print_byte_array(dsmp_hdr->extension.provider_service_ctx.data,
                         dsmp_hdr->extension.provider_service_ctx.length);
    }
    if(dsmp_hdr->extension.ipv6_addr_option) {
        printf("IPv6 address in DSMP extension: ");
        print_ipv6_address(dsmp_hdr->extension.ipv6_addr);
    }
    if(dsmp_hdr->extension.service_port_option) {
        printf("Service port in DSMP extension: %u\n", dsmp_hdr->extension.service_port);
    }
    if(dsmp_hdr->extension.provider_mac_addr_option) {
        printf("Provider MAC address in DSMP extension: ");
        print_mac_address(dsmp_hdr->extension.provider_mac_addr);
    }
    if(dsmp_hdr->extension.broadcast_freq_option) {
        printf("Broadcast frequency in DSMP extension: %hhu\n", (unsigned char)dsmp_hdr->extension.broadcast_freq);
    }
    if(dsmp_hdr->extension.area_str_option) {
        printf("Area string in DSMP extension: ");
        print_byte_array(dsmp_hdr->extension.area_str,
                         CMS_DSMP_EXT_AREA_STRING_LENGTH);
    }
    if(dsmp_hdr->extension.dsa_rcpi_treshold_option) {
        printf("DSA RCPI threshold in DSMP extension: %hhu\n", (unsigned char)dsmp_hdr->extension.dsa_rcpi_treshold);
    }
    if(dsmp_hdr->extension.dsa_cnt_treshold_option) {
        printf("DSA count threshold in DSMP extension: %hhu\n", (unsigned char)dsmp_hdr->extension.dsa_cnt_treshold);
    }
    if(dsmp_hdr->extension.dsa_cnt_treshold_interval_option) {
        printf("DSA count threshold in DSMP extension: %hhu\n", (unsigned char)dsmp_hdr->extension.dsa_cnt_treshold_interval);
    }
}

void print_mac_address(const cms_mac_addr_t mac_address)
{
    printf("%.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
           mac_address[0],
           mac_address[1],
           mac_address[2],
           mac_address[3],
           mac_address[4],
           mac_address[5]);
}

void print_byte_array(const uint8_t* buf, uint32_t len)
{
    for(uint32_t i = 0; i < len; ++i) {
        printf("%02X", buf[i]);
    }
    printf("\n");
}

void print_ipv6_address(const cms_ipv6_addr_t ip_address)
{
    printf("%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
           ip_address[0], ip_address[1],
           ip_address[2], ip_address[3],
           ip_address[4], ip_address[5],
           ip_address[6], ip_address[7],
           ip_address[8], ip_address[9],
           ip_address[10], ip_address[11],
           ip_address[12], ip_address[13],
           ip_address[14], ip_address[15]);
}
