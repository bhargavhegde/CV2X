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

#include <asn1/v2x_eu_asn.h>
#include <asn1/v2x_us_asn.h>
#include <asn1defs.h>

enum { ASN_NAME_LENGTH = 64U };

/** @file
@brief Subscribe to facility messages.
@ingroup ex
*/


typedef struct {
    cms_fac_msg_type_t cms_type;
    char name[ASN_NAME_LENGTH];
    const ASN1CType* asn_type;
} asn_type_t;

/** Supported types to translate */
static const asn_type_t asn_types[] = {
    {
        .cms_type = CMS_FAC_MSG_EU_CAM,
        .name = "EU_CAM",
        .asn_type = asn1_type_EU_CAM
    },
    {
        .cms_type = CMS_FAC_MSG_EU_DENM,
        .name = "EU_DENM",
        .asn_type = asn1_type_EU_DENM
    },
    {
        .cms_type = CMS_FAC_MSG_EU_MAP,
        .name = "EU_MAP",
        .asn_type = asn1_type_EU_MAP
    },
    {
        .cms_type = CMS_FAC_MSG_EU_SPAT,
        .name = "EU_SPAT",
        .asn_type = asn1_type_EU_SPAT
    },
    {
        .cms_type = CMS_FAC_MSG_EU_IVI,
        .name = "EU_IVI",
        .asn_type = asn1_type_EU_IVI
    },
    {
        .cms_type = CMS_FAC_MSG_EU_RTCM,
        .name = "EU_RTCM",
        .asn_type = asn1_type_EU_RTCM
    },
    {
        .cms_type = CMS_FAC_MSG_EU_SRM,
        .name = "EU_SRM",
        .asn_type = asn1_type_EU_SRM
    },
    {
        .cms_type = CMS_FAC_MSG_EU_SSM,
        .name = "EU_SSM",
        .asn_type = asn1_type_EU_SSM
    },
    {
        .cms_type = CMS_FAC_MSG_EU_CPM,
        .name = "EU_CPM",
        .asn_type = asn1_type_EU_CPM
    },
    {
        .cms_type = CMS_FAC_MSG_US_BSM,
        .name = "US_BSM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_TIM,
        .name = "US_TIM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_RSA,
        .name = "US_RSA",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_MAP,
        .name = "US_MAP",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_SPAT,
        .name = "US_SPAT",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_SRM,
        .name = "US_SRM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_SSM,
        .name = "US_SSM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_PSM,
        .name = "US_PSM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_PVD,
        .name = "US_PVD",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_PDM,
        .name = "US_PDM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_RTCM,
        .name = "US_RTCM",
        .asn_type = asn1_type_US_MessageFrame
    },
    {
        .cms_type = CMS_FAC_MSG_US_WSA,
        .name = "US_WSA",
        .asn_type = asn1_type_US_MessageFrame
    },
    /* Terminating type */
    {
        .name = "",
        .asn_type = NULL
    }
};

/* Context type for the notification callback */
typedef struct notif_ctx {
    uint32_t param;
    uint32_t cnt;
} notif_ctx_t;

static const char* get_verify_result_string(cms_sec_verify_result_t verify_result_code);
static const ASN1CType* get_type(cms_fac_msg_type_t in_type);
static bool process_uper(cms_fac_msg_type_t type, cms_buffer_view_t msg);
static bool l_use_c_stuct(cms_fac_msg_type_t type, void* c_struct);
static bool l_print_xml_format(const ASN1CType* asn_type, void* c_struct);


static bool process_uper(cms_fac_msg_type_t type, cms_buffer_view_t msg)
{
    bool error = false;
    void* c_struct = NULL;
    const ASN1CType* asn_type = get_type(type);
    if(NULL == asn_type) {
        error = true;
        printf("Unable to detect asn type\n");
    }

    /* decode uper buffer to c struct */
    if(!error) {
        fprintf(stderr, "Decoding %d bytes\n", msg.length);
        ASN1Error err;
        int ret = asn1_uper_decode((void**)&c_struct, asn_type, msg.data, msg.length, &err);
        if((ret < 0) || (c_struct == NULL)) {
            fprintf(stderr, "Decoding error: %s\n", err.msg);
            error = true;
        }
    }

    /* use c struct according to your needs */
    /* in this example we will print number of intersections in EU & US MAP messages */
    error = error || l_use_c_stuct(type, c_struct);

    /* encode c struct to human readable xml struct and print to stdout */
    error = error || l_print_xml_format(asn_type, c_struct);

    if(c_struct != NULL) {
        asn1_free_value(asn_type, c_struct);
    }

    return error;
}

static bool l_use_c_stuct(cms_fac_msg_type_t type, void* c_struct)
{
    bool error = false;

    switch(type) {
    case CMS_FAC_MSG_EU_MAP: {
        EU_MAP* c_map = (EU_MAP*)c_struct;
        if(c_map->map.intersections_option) {
            printf("number of intersections: %zu\n", c_map->map.intersections.count);
        }
        break;
    }
    case CMS_FAC_MSG_US_MAP: {
        US_MessageFrame* c_message_frame = (US_MessageFrame*)c_struct;
        if(c_message_frame->value.type != asn1_type_US_MapData) {
            printf("Error, Message is not us map");
            error = true;
        } else {
            US_MapData* map_data = (US_MapData*)c_message_frame->value.u.data;
            if(map_data->intersections_option) {
                printf("number of intersections: %zu\n", map_data->intersections.count);
            }
        }
        break;
    }
    default:
        /* modify message handling according to your needs */
        break;
    }

    return error;
}

static bool l_print_xml_format(const ASN1CType* asn_type, void* c_struct)
{
    bool error = false;
    uint8_t* xer_ptr = NULL;

    int ret = asn1_xer_encode((uint8_t**)&xer_ptr, asn_type, c_struct);
    if((ret < 0) || (xer_ptr == NULL)) {
        fprintf(stderr, "Encoding error\n");
        error = true;
    } else {
        fprintf(stderr, "Encoded XML data:\n");
        printf("%s", xer_ptr);
        fprintf(stderr, "---- End of encoded XML data\n\n");
    }

    if(xer_ptr != NULL) {
        asn1_free(xer_ptr);
    }

    return error;
}

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
               get_verify_result_string(notif->security.verify_result),
               (unsigned long)notif->security.verify_result);
        printf("Data:\n");

        for(uint32_t i = 0UL; i < msg.length; ++i) {
            printf("%02X", msg.data[i]);
        }
        printf("\n");

        process_uper(type, msg);

        printf("=====================================================\n");

        ++notif_ctx->cnt;
    }
}

static const char* get_verify_result_string(cms_sec_verify_result_t verify_result_code)
{
    const char* result = NULL;

    switch(verify_result_code) {
    case CMS_SEC_VERIFY_UNSECURED:
        result = "Unsecured - No signature";
        break;
    case CMS_SEC_VERIFY_VERIFIED:
        result = "Verified";
        break;
    default:
        result = "Verification Failed";
    }

    return result;
}

static const ASN1CType* get_type(cms_fac_msg_type_t in_type)
{
    const ASN1CType* asn_type = NULL;

    for(int i = 0; asn_types[i].asn_type != NULL; i++) {
        if(in_type == asn_types[i].cms_type) {
            printf("type detected: %s\n", asn_types[i].name);
            asn_type = asn_types[i].asn_type;
            break;
        }
    }

    return asn_type;
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

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
        static const uint32_t EXIT_ON_RECV_COUNT = 20000UL;
        while(wildcard_ctx.cnt < EXIT_ON_RECV_COUNT) {
            printf("Waiting for facility messages\n");
            sleep(10);
        }
    }

    error = error || cms_fac_unsubscribe(&session, wildcard_subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

