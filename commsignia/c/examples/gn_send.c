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
#include <pthread.h>

#include <cms_v2x/api.h>
#include <cms_v2x/gn.h>

/** @file
@brief Send a GeoNet message.
@ingroup ex
*/


/* Fill GeoNetwork parameters */
static void fill_send_header(cms_gn_send_data_t* header)
{
    /* Radio parameters */
    header->radio.datarate = CMS_DATARATE_NA;
    header->radio.expiry_time = 1000U;
    header->radio.interface_id = 1U;
    header->radio.sps_index = CMS_SPS_CHANNEL_INDEX_NA;
    header->radio.tx_power = 0;
    header->radio.user_prio = 1U;

    /* GeoNet parameters - BTP */
    header->gn_params.btp_params.btp_port = 1235U;
    header->gn_params.btp_params.dest_port_info = 0U;

    /* GeoNet parameters - GeoNet header - Destination area */
    header->gn_params.gn_hdr.dest_area.angle = 0;
    header->gn_params.gn_hdr.dest_area.dist_a = 1000U;
    header->gn_params.gn_hdr.dest_area.dist_b = 0U;
    header->gn_params.gn_hdr.dest_area.latitude = 111111111;
    header->gn_params.gn_hdr.dest_area.longitude = 555555555;
    header->gn_params.gn_hdr.dest_area.type = CMS_GN_DEST_AREA_TYPE_CIRCLE;

    /* GeoNet parameters - GeoNet header  */
    header->gn_params.gn_hdr.lifetime_ms = 1000U;
    header->gn_params.gn_hdr.max_hop_limit = 5;
    header->gn_params.gn_hdr.method = CMS_GN_SEND_METHOD_GBC;

    /* GeoNet parameters - GeoNet header  - Traffic class */
    header->gn_params.gn_hdr.traffic_class.chf_offl = CMS_GN_TRAFFIC_CLASS_CHF_OFFL_DISABLE;
    header->gn_params.gn_hdr.traffic_class.scf_en = CMS_GN_TRAFFIC_CLASS_SCF_DISABLE;
    header->gn_params.gn_hdr.traffic_class.tcid = 0U;

    /* Security info */
    header->security.sign_info.psid = 1U;
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

    /* Create send header */
    cms_gn_send_data_t send_hdr = {0};
    fill_send_header(&send_hdr);

    uint8_t msg_payload[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    /* Create a buffer view as a handle to the actual payload buffer */
    cms_buffer_view_t msg = {
        .data = msg_payload,
        .length = sizeof(msg_payload)
    };

    /* Send the prepared GeoNet message */
    error = error || cms_gn_send(&session, &send_hdr, msg, NULL);
    if(error) {
        printf("Unable to send GeoNet message\n");
    } else {
        printf("GeoNet message sent\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
