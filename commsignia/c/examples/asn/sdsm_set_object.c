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
#include <cms_v2x/sdsm.h>
#include <asn1/v2x_us_asn.h>
#include <asn1defs.h>

/** @file
@brief Update or add a single perceived object's data to the temporary object storage.
*/

/*
Fill DetectedObjectData structure.
*/
static bool fill_detected_object_data(US_DetectedObjectData* det_obj_data)
{
    bool error = (NULL == det_obj_data);

    if(!error) {
        det_obj_data->detObjCommon.objType = US_ObjectType_vru;
        det_obj_data->detObjCommon.objTypeCfd = 1;
        det_obj_data->detObjCommon.objectID = 1;
        det_obj_data->detObjCommon.measurementTime = 1;
        det_obj_data->detObjCommon.timeConfidence = US_TimeConfidence_time_100_000;
        det_obj_data->detObjCommon.pos.offsetX = 1;
        det_obj_data->detObjCommon.pos.offsetY = 1;
        det_obj_data->detObjCommon.pos.offsetZ_option = FALSE;
        det_obj_data->detObjCommon.posConfidence.pos = US_PositionConfidence_a500m;
        det_obj_data->detObjCommon.posConfidence.elevation = US_ElevationConfidence_elev_500_00;
        det_obj_data->detObjCommon.speed = 1;
        det_obj_data->detObjCommon.speedConfidence = US_SpeedConfidence_prec100ms;
        det_obj_data->detObjCommon.speedZ_option = FALSE;
        det_obj_data->detObjCommon.speedConfidenceZ_option = FALSE;
        det_obj_data->detObjCommon.heading = 1;
        det_obj_data->detObjCommon.headingConf = US_HeadingConfidence_prec10deg;
        det_obj_data->detObjCommon.accel4way_option = FALSE;
        det_obj_data->detObjCommon.accCfdX_option = FALSE;
        det_obj_data->detObjCommon.accCfdY_option = FALSE;
        det_obj_data->detObjCommon.accCfdZ_option = FALSE;
        det_obj_data->detObjCommon.accCfdYaw_option = FALSE;

        /* Fill VRU parameters */
        det_obj_data->detObjOptData_option = TRUE;
        det_obj_data->detObjOptData.choice = US_DetectedObjectOptionalData_detVRU;
        det_obj_data->detObjOptData.u.detVRU.basicType_option = TRUE;
        det_obj_data->detObjOptData.u.detVRU.basicType = US_PersonalDeviceUserType_aPEDESTRIAN;
        det_obj_data->detObjOptData.u.detVRU.propulsion_option = TRUE;
        det_obj_data->detObjOptData.u.detVRU.propulsion.choice = US_PropelledInformation_human;
        det_obj_data->detObjOptData.u.detVRU.propulsion.u.human = US_HumanPropelledType_wheelchair;
        det_obj_data->detObjOptData.u.detVRU.attachment_option = TRUE;
        det_obj_data->detObjOptData.u.detVRU.attachment = US_Attachment_wheelchair;
        det_obj_data->detObjOptData.u.detVRU.radius_option = TRUE;
        det_obj_data->detObjOptData.u.detVRU.radius = 1;
    }

    return error;
}

/*
Creates a DetectedObjectData and encode it to a buffer
*/
static bool create_detected_object_data(uint8_t** data_out, size_t* length_out)
{
    /* Argument check */
    bool error = (NULL == data_out);
    error = error || (NULL == length_out);

    /* ASN.1 struct */
    US_DetectedObjectData det_obj_data = {0};

    /* Set DetectedObjectData */
    error = error || fill_detected_object_data(&det_obj_data);

    /* Check constraints */
    if(!error) {
        ASN1Error asn1_error = {0};
        if(FALSE == asn1_check_constraints(asn1_type_US_DetectedObjectData, &det_obj_data, &asn1_error)) {
            printf("Constraints check failed: %s\n", asn1_error.msg);
            error = true;
        }
    }

    /* Encode DetectedObjectData */
    ASN1Error encode_err = {0};
    if(!error) {
        int encoded_length = asn1_uper_encode2(data_out,
                                               asn1_type_US_DetectedObjectData,
                                               &det_obj_data,
                                               &encode_err);
        if(encoded_length <= 0) {
            printf("Error in encode: %s.\n", encode_err.msg);
            error = true;
        } else {
            *length_out = encoded_length;
        }
    }

    /* Free the struct */
    asn1_free_value_struct(asn1_type_US_DetectedObjectData, &det_obj_data);

    return error;
}

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set the new reference position */
    cms_sdsm_ref_position_t ref_pos = {0};
    ref_pos.timestamp = 1;
    ref_pos.latitude = 1;
    ref_pos.longitude = 1;
    ref_pos.altitude = 1;
    ref_pos.altitude_confidence = 1;
    ref_pos.pce_semi_major = 1;
    ref_pos.pce_semi_minor = 1;
    ref_pos.pce_orientation = 1;

    /* Update the reference time and reference position that will be used in the SDSM packets,
       and to which the relative data is calculated in the object data. */
    error = error || cms_sdsm_update_ref_position(&session, &ref_pos, NULL);

    if(error) {
        printf("Unable to update reference position.\n");
    } else {
        printf("Reference position successfully updated.\n");
    }

    /* Create DetectedObjectData */
    uint8_t* uper_encoded_object_data = NULL;
    size_t uper_encoded_object_size = 0;
    error = error || create_detected_object_data(&uper_encoded_object_data, &uper_encoded_object_size);

    /* Set input */
    cms_sdsm_set_object_t object_params = {0};
    object_params.id = 1U;
    object_params.priority = 5;
    cms_buffer_view_t buff = {
        .data = uper_encoded_object_data,
        .length = uper_encoded_object_size
    };

    /* Set object */
    error = error || cms_sdsm_set_object(&session, &object_params, buff, NULL);

    if(error) {
        printf("Unable to set object with ID: %hu.\n", (unsigned short)object_params.id);
    } else {
        printf("Object successfully set with ID: %hu.\n", (unsigned short)object_params.id);
    }

    if(uper_encoded_object_data != NULL) {
        asn1_free(uper_encoded_object_data);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
