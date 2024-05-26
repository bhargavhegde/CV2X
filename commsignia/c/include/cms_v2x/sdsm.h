/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021
*/

#ifndef CMS_V2X_SDSM_H_
#define CMS_V2X_SDSM_H_

#include <stdbool.h>
#include <stddef.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

/** @file
@brief Sensor Data Sharing Message (SDSM) API
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup sdsm Sensor Data Sharing Message (SDSM)
@ingroup api
*/

/**
ID parameter used in the SDSM module.
@ingroup sdsm
@experimentalapi

Provides a unique identifier that is assigned to each detected object.
The assigned ObjectID is maintained for a detected object as long as the object is detected.
*/
typedef uint16_t cms_sdsm_object_id_t;

/**
Priority of Detected object for SDSM sending.
@ingroup sdsm
@experimentalapi

Signed integer. Higher number means higher priority.
A suggestion is to use the distance from current ego position.
*/
typedef int32_t cms_sdsm_object_priority_t;

/**
Input parameters for cms_sdsm_set_object function.
@ingroup sdsm
@experimentalapi
@see cms_sdsm_set_object
*/
typedef struct cms_sdsm_set_object_t {
    cms_sdsm_object_id_t id;
    cms_sdsm_object_priority_t priority;
} CMS_PACKED cms_sdsm_set_object_t;

/**
Input parameters for cms_sdsm_delete_object function.
@ingroup sdsm
@experimentalapi
@see cms_sdsm_delete_object
*/
typedef struct cms_sdsm_delete_object_t {
    cms_sdsm_object_id_t id;
} CMS_PACKED cms_sdsm_delete_object_t;

/**
SDSM Reference Position - The absolute position of the host vehicle/RSU and the corresponding timestamp.
@ingroup sdsm
@experimentalapi
@see cms_sdsm_update_ref_position
*/
typedef struct cms_sdsm_ref_position_t {
    /** Timestamp at which the reference position was measured. UTC milliseconds since Unix epoch. */
    cms_utc_timestamp_ms_t timestamp;
    cms_latitude_t latitude;                        /**< Latitude angle */
    cms_longitude_t longitude;                      /**< Longitude angle */
    cms_altitude_t altitude;                        /**< Altitude */
    cms_altitude_t altitude_confidence;             /**< Confidence of altitude */
    cms_length_t pce_semi_major;                    /**< Confidence ellipse for latitude angle */
    cms_length_t pce_semi_minor;                    /**< Confidence ellipse for longitude angle */
    cms_heading_t pce_orientation;                  /**< Confidence ellipse orientation */
} CMS_PACKED cms_sdsm_ref_position_t;

/**
Add or update a measurement corresponding to a detected object.
@ingroup sdsm
@experimentalapi

@param session      Client session
@param set_params   ID & priority for the object to set
@param object_data  ASN.1 UPER encoded DetectedObjectData
@param unused_out   Unused output
@return true in case of error

@see cms_sdsm_set_object

The detected object going to be sent in the next SensorDataSharingMessage (SDSM).
ObjectID: object->id parameter and Object ID in the UPER encoded DetectedObjectData shall match.
Priority: Priority is an internal value used for including objects in the temporary object storage.
It is not used in the outgoing packet, and it is not described in the standard.
@see cms_sdsm_object_priority_t
The specified UPER DetectedObjectData shall contain the position offset relative to
the last reference position set by cms_sdsm_update_ref_position.
@see cms_sdsm_update_ref_position.

*/
bool cms_sdsm_set_object(const cms_session_t* session,
                         const cms_sdsm_set_object_t* set_params,
                         cms_buffer_view_t object_data,
                         void* unused_out);

/**
Remove a detected object from the temporary object storage.
@ingroup sdsm
@experimentalapi

@param session      Client session
@param object_id    Identifier of the object to be removed
@param unused_out   Unused output
@return true in case of error
*/
bool cms_sdsm_delete_object(const cms_session_t* session,
                            const cms_sdsm_delete_object_t* object_id,
                            void* unused_out);

/**
Update reference position in SDSM.
@ingroup sdsm
@experimentalapi

@param session      Client session
@param ref_pos      Reference position
@param unused_out   Unused output
@return true in case of error

Important: Reference position needs to be updated with at least 1Hz
           otherwise sending will not work due to time offsets sliding out of range.
*/
bool cms_sdsm_update_ref_position(const cms_session_t* session,
                                  const cms_sdsm_ref_position_t* ref_pos,
                                  void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SDSM_H_ */
