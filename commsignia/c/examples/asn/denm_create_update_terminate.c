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
#include <cms_v2x/nav.h>
#include <cms_v2x/denm.h>

#include <asn1/v2x_eu_asn.h>
#include <asn1defs.h>

/** @file
@brief Example of sending and updating a DENM message
@ingroup ex
*/

void fillDenmContext(cms_denm_create_t* denm_ctx, cms_nav_fix_t* nav_fix);

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Get the latest position */
    cms_nav_fix_t nav_fix = {0};
    error = error || cms_nav_get_last_fix(&session, NULL, &nav_fix);
    if(error) {
        printf("Unable to get NAV fix\n");
    } else {
        printf("Latitude angle: %ld\n", (long)nav_fix.latitude);
        printf("Longitudinal angle: %ld\n", (long)nav_fix.longitude);
    }

    /* Create DENM context */
    cms_denm_create_t denm_ctx = {0};
    fillDenmContext(&denm_ctx, &nav_fix);

    /* Create payload */
    EU_DecentralizedEnvironmentalNotificationMessage payload = {0};


    /* Valid for 5 seconds */
    payload.management.validityDuration = 5;

    /* UPER encode payload */
    ASN1Error asn1_error = {0};
    uint8_t* uper_create = NULL;
    int uper_create_len = asn1_uper_encode2(&uper_create, asn1_type_EU_DecentralizedEnvironmentalNotificationMessage,
                                            &payload,
                                            &asn1_error);

    if(uper_create_len < 0) {
        printf("Failed to UPER encode DENM\n");
        error = true;
    }

    /* Create buffer view */
    cms_buffer_view_t create_buf = {
        .data = uper_create,
        .length = uper_create_len
    };

    /* Create new DENM */
    cms_denm_action_id_t action_id = {0};
    if(cms_denm_create_or_update(&session, &denm_ctx, create_buf, &action_id)) {
        printf("Unable to create DENM\n");
        error = true;
    } else {
        printf("DENM created successfully\n");
        printf("Action Id: %d.%d\n", (int)action_id.station_id, (int)action_id.sequence_number);
    }

    /* Free UPER encoded DENM */
    if(uper_create) {
        asn1_free(uper_create);
    }

    /* Sleep for 1 second */
    sleep(1);

    /* Modify the context for an update call */
    denm_ctx.auto_flags.auto_calc_action_id = false;
    denm_ctx.management.gn_destination.latitude -= 12;
    denm_ctx.management.gn_destination.longitude += 12;

    /* Set action id */
    payload.management.actionID.originatingStationID = action_id.station_id;
    payload.management.actionID.sequenceNumber = action_id.sequence_number;

    /* UPER encode payload */
    uint8_t* uper_update = NULL;
    int uper_update_len = asn1_uper_encode2(&uper_update, asn1_type_EU_DecentralizedEnvironmentalNotificationMessage,
                                            &payload,
                                            &asn1_error);

    if(uper_update_len < 0) {
        printf("Failed to UPER encode DENM\n");
        error = true;
    }

    /* Create buffer view */
    cms_buffer_view_t update_buf = {
        .data = uper_update,
        .length = uper_update_len
    };


    /* Update DENM */
    if(cms_denm_create_or_update(&session, &denm_ctx, update_buf, &action_id)) {
        printf("Unable to update DENM\n");
        error = true;
    } else {
        printf("DENM updated successfully\n");
        printf("Action Id: %d.%d\n", (int)action_id.station_id, (int)action_id.sequence_number);
    }

    /* Free UPER encoded DENM */
    if(uper_update) {
        asn1_free(uper_update);
    }

    /* Sleep for 1 second */
    sleep(1);

    /* Create termination context */
    cms_denm_termination_t term_ctx = {0};
    term_ctx.action_id.station_id = action_id.station_id;
    term_ctx.action_id.sequence_number = action_id.sequence_number;

    /* Terminate DENM */
    if(cms_denm_terminate(&session, &term_ctx, NULL)) {
        printf("Unable to terminate DENM\n");
        error = true;
    } else {
        printf("DENM terminated successfully\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

void fillDenmContext(cms_denm_create_t* denm_ctx, cms_nav_fix_t* nav_fix)
{
    /* Set auto flags */
    denm_ctx->auto_flags.auto_calc_action_id = true;
    denm_ctx->auto_flags.auto_set_detection_time = true;
    denm_ctx->auto_flags.auto_set_event_position = true;
    denm_ctx->auto_flags.auto_set_event_trace = true;

    /* Set management data */
    denm_ctx->management.gn_hop_limit = 2U;
    denm_ctx->management.repetition_duration = 1000U;
    denm_ctx->management.repetition_interval = 100U;

    /* Set geonet traffic class */
    denm_ctx->management.gn_traffic_class.scf_en = CMS_GN_TRAFFIC_CLASS_SCF_DISABLE;
    denm_ctx->management.gn_traffic_class.chf_offl = CMS_GN_TRAFFIC_CLASS_CHF_OFFL_DISABLE;
    denm_ctx->management.gn_traffic_class.tcid = 1U;

    /* Set geonet destination */
    denm_ctx->management.gn_destination.type = CMS_GN_DEST_AREA_TYPE_CIRCLE;
    denm_ctx->management.gn_destination.latitude = nav_fix->latitude;
    denm_ctx->management.gn_destination.longitude = nav_fix->longitude;
    denm_ctx->management.gn_destination.dist_a = 1000U;
}
