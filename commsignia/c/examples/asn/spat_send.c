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
#include <pthread.h>
#include <string.h>

#include <cms_v2x/api.h>
#include <cms_v2x/gn.h>
#include <asn1/v2x_eu_asn.h>
#include <asn1defs.h>

/** @file
@brief Send an EU SPaT message.
This example demonstrates how to decode, modify, encode ASN1 UPER (Unaligned Packed Encoding Rules) message.
@ingroup ex
*/

static void create_and_send_a_spat(const cms_session_t* session);
static bool modify_spat(EU_SPAT* spat);
static void print_xml(const EU_SPAT* spat);
static bool send_as_geonet_message(const cms_session_t* session, cms_buffer_view_t msg);
static void fill_gnp_header(cms_gn_send_data_t* header);


/*
Creates and sends a SPaT message, demonstrating:
- decoding and encoding messages
- validation by ASN.1 constraint check
*/
static void create_and_send_a_spat(const cms_session_t* session)
{
    bool error = false;
    ASN1Error decode_err = {0};
    EU_SPAT* spat = NULL;

    /* We assume that the user created a SPAT what is close to her needs e.g. ORIGINAL_SPAT.
       We recommend to create it with asn1x or any other tool. */
    static const uint8_t ORIGINAL_SPAT[] = {
        0x02, 0x04, 0x00, 0x01, 0xE2, 0x40, 0x00, 0x08,
        0x2A, 0x01, 0x00, 0x02, 0x00, 0x03, 0xE8, 0x03,
        0x00, 0x12, 0x45, 0x00, 0x17, 0x22, 0x38, 0x00,
        0xBE, 0x10, 0xC0, 0x07, 0x08, 0x00, 0x44, 0x86,
        0x00, 0x2F, 0x84, 0x50, 0x01, 0xAE, 0x23, 0x80,
        0x0E, 0x10, 0x00, 0xC5, 0x14, 0x00, 0x5C, 0x88,
        0xE0, 0x02, 0xF8, 0x00, 0x42, 0x43, 0x00, 0x17,
        0xC2, 0x28, 0x00, 0xD7, 0x11, 0xC0, 0x07, 0x08,
    };

    /* Decode UPER encoded SPaT to C struct. This effectively allocates
       a EU_SPAT instance and fills it by the buffer contents. */
    if(0 >= asn1_uper_decode((void**)&spat,
                             asn1_type_EU_SPAT,
                             ORIGINAL_SPAT,
                             sizeof(ORIGINAL_SPAT),
                             &decode_err)) {
        printf("Size: %d", (int)sizeof(ORIGINAL_SPAT));
        printf("asn1_uper_decode error: %s at bit: %d\n", decode_err.msg, decode_err.bit_pos);
        error = true;
    } else {
        printf("SPaT successfully decoded:\n");
        print_xml(spat);
    }

    /* Modify the SPaT message, in this case add a 3. MovementEvent to 3. MovementState */
    error = error || modify_spat(spat);

    if(!error) {

        /* Validation: check message-specific constraints */
        ASN1Error err = {0};       /* Buffer for constraint check errors */
        if(FALSE == asn1_check_constraints(asn1_type_EU_SPAT, spat, &err)) {
            printf("Error in constraints: %s\n", err.msg);
            error = true;
        } else {
            printf("Constraints check passed\n");
            printf("SPaT to send:\n");
            print_xml(spat);
        }
    }

    if(!error) {

        /* Encode to an allocated buffer */
        uint8_t* encode_buff = NULL;
        int encoded_length = asn1_uper_encode2(&encode_buff, asn1_type_EU_SPAT, spat, &decode_err);
        if(encoded_length <= 0) {
            printf("Error in encode: %s\n", decode_err.msg);
            error = true;
        } else {

            cms_buffer_view_t msg = {
                .data = encode_buff,
                .length = encoded_length
            };

            /* Send as a GeoNetworking message */
            error = send_as_geonet_message(session, msg);
        }

        /* Free encode buffer */
        if(encode_buff != NULL) {
            asn1_free(encode_buff);
        }
    }

    /* Free the SPaT instance */
    if(spat != NULL) {
        asn1_free_value(asn1_type_EU_SPAT, spat);
    }

    if(error) {
        printf("Unable to send SPaT message\n");
    } else {
        printf("SPaT message sent\n");
    }
}


/* Modifies a SPaT message, demonstrating the use of ASN.1 types */
static bool modify_spat(EU_SPAT* spat)
{
    bool error = false;

    EU_IntersectionState* intersectionState = spat->spat.intersections.tab;
    EU_MovementList* movementStates = NULL;

    /* Modify it according your needs e.g add */
    /* e.g add to 3. MovementState a 3. MovementEvent */

    if(NULL == intersectionState) {
        printf("no intersections in spat\n");
        error = true;
    } else {

        movementStates = &intersectionState->states;
        if((movementStates->tab == NULL) || (movementStates->count < 3)) {
            printf("no 3. movementStates\n");
            error = true;
        }
    }

    if(!error) {

        EU_MovementEventList* movementEvents = &movementStates->tab[2].state_time_speed;
        if((movementEvents->tab == NULL) || (movementEvents->count != 2)) {
            printf("Expected to have 2 movement event\n");
            error = true;
        } else {

            /* Add new event to the sequence */
            EU_MovementEvent newEvent = {0};
            newEvent.eventState = EU_MovementPhaseState_stop_And_Remain;
            newEvent.timing_option = 1;
            newEvent.timing.minEndTime = 900;
            if(asn1_seq_add(asn1_type_EU_MovementEvent,
                            &movementEvents->count,
                            (void**)(&movementEvents->tab),
                            &newEvent)) {
                printf("Unable to add seq\n");
                error = true;
            }
        }
    }

    return error;
}


/* Print message as XML with encoding to XER */
static void print_xml(const EU_SPAT* spat)
{
    uint8_t* xer_buff = NULL;
    int xer_len = asn1_xer_encode(&xer_buff, asn1_type_EU_SPAT, spat);
    if(xer_len <= 0) {
        printf("Error in xer encode\n");
    } else {
        printf("%.*s\n", xer_len, xer_buff);
    }

    if(xer_buff != NULL) {
        asn1_free(xer_buff);
    }
}


/* Send a GeoNetworking message */
static bool send_as_geonet_message(const cms_session_t* session, cms_buffer_view_t msg)
{

    cms_gn_send_data_t gnp_hdr = {0};
    fill_gnp_header(&gnp_hdr);

    bool error = cms_gn_send(session, &gnp_hdr, msg, NULL);
    if(error) {
        printf("Unable to send GeoNet message\n");
    } else {
        printf("GeoNet message sent\n");
    }

    return error;
}


/* Fill GeoNetworking parameters */
static void fill_gnp_header(cms_gn_send_data_t* header)
{
    /* Radio parameters */
    header->radio.datarate = CMS_DATARATE_NA;
    header->radio.expiry_time = 1000U;
    header->radio.interface_id = 1U;
    header->radio.sps_index = CMS_SPS_CHANNEL_INDEX_NA;
    header->radio.tx_power = 20;
    header->radio.user_prio = CMS_MAC_USER_PRIO_NA;

    /* GeoNet parameters - BTP for SPAT */
    header->gn_params.btp_params.btp_port = 2004U;
    header->gn_params.btp_params.dest_port_info = 0U;

    /* GeoNet parameters - GeoNet header  */
    header->gn_params.gn_hdr.lifetime_ms = 60000U;
    header->gn_params.gn_hdr.max_hop_limit = 1;
    header->gn_params.gn_hdr.method = CMS_GN_SEND_METHOD_SHB;

    /* GeoNet parameters - GeoNet header  - Traffic class */
    header->gn_params.gn_hdr.traffic_class.chf_offl = CMS_GN_TRAFFIC_CLASS_CHF_OFFL_DISABLE;
    header->gn_params.gn_hdr.traffic_class.scf_en = CMS_GN_TRAFFIC_CLASS_SCF_DISABLE;
    header->gn_params.gn_hdr.traffic_class.tcid = 0U;

    /* Security info */
    header->security.sign_info.psid = 0U;
    header->security.sign_info.sign_method = CMS_SIGN_METH_NONE;
    header->security.sign_info.ssp.length = 0U;
    /* Keep header->security.ssp.ssp_field zeroed */
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Send a SPAT */
    if(!error) {
        create_and_send_a_spat(&session);
    }

    /* Close connection */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
