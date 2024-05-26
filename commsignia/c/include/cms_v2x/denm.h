/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#ifndef CMS_V2X_DENM_H_
#define CMS_V2X_DENM_H_

/** @file
@brief DENM data types
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/gn_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup denm DENM
@ingroup api
*/

/**
DENM Tx enable/disable.
@ingroup denm
*/
typedef struct cms_denm_tx_state_t {
    bool enable;
} CMS_PACKED cms_denm_tx_state_t;

/**
Function to enable/disable DENM Tx
@ingroup denm

@param  session         Client session
@param  state_in        DENM Tx enable/disable
@param  unused_out      Unused parameter
@return true in case of error
*/

bool cms_denm_tx_enable(const cms_session_t* session,
                        const cms_denm_tx_state_t* state_in,
                        void* unused_out);

/**
Function to get is DENM Tx enabled/disabled
@ingroup denm

@param  session         Client session
@param  unused_in       Unused parameter
@param  state_out       DENM Tx enabled/disabled
@return true in case of error
*/

bool cms_denm_tx_get_enabled(const cms_session_t* session,
                             const void* unused_in,
                             cms_denm_tx_state_t* state_out);

/**
DENM Action id
@ingroup denm
*/
typedef struct cms_denm_action_id_t {
    uint32_t station_id;      /**< Originating station ID */
    uint16_t sequence_number; /**< Sequence number */
} CMS_PACKED cms_denm_action_id_t;


/**
DENM Management data
@ingroup denm
*/
typedef struct cms_denm_management_data_t {
    cms_duration_ms_t repetition_duration;   /**< Repetition duration in milliseconds */
    cms_duration_ms_t repetition_interval;   /**< Repetition interval in milliseconds */
    cms_gn_dest_area_t gn_destination;       /**< GeoNetworking Destination area */
    cms_gn_traffic_class_t gn_traffic_class; /**< GeoNetworking traffic class */
    cms_gn_hop_limit_t gn_hop_limit;         /**< GeoNetworking hop limit */
} CMS_PACKED cms_denm_management_data_t;

/**
DENM auto-calculation flags
@ingroup denm
*/
typedef struct cms_denm_auto_flags_t {
    bool auto_calc_action_id;       /**< Auto-calculate ActionId */
    bool auto_set_detection_time;   /**< Auto-set detection time to the current time */
    bool auto_set_event_position;   /**< Auto-set event position to the current position */
    bool auto_set_event_trace;      /**< Auto-set event trace */
    bool auto_set_destination_area; /**< Auto-set geonet destination area */
} CMS_PACKED cms_denm_auto_flags_t;

/**
DENM termination parameters
@ingroup denm
*/
typedef struct cms_denm_termination_t {
    cms_denm_action_id_t action_id;                /**< Action id */
    cms_utc_timestamp_ms_t event_termination_time; /**< Event termination time */
    cms_duration_ms_t
    validity_duration; /**< Validity duration of the termination in milliseconds */
    cms_duration_ms_t repetition_duration; /**< Repetition duration in milliseconds */
    cms_duration_ms_t repetition_interval; /**< Repetition interval in milliseconds */
} CMS_PACKED cms_denm_termination_t;

/**
DENM creation parameters
@ingroup denm
*/
typedef struct cms_denm_create_t {
    cms_denm_management_data_t management; /**< Management data */
    cms_denm_auto_flags_t auto_flags;      /**< Auto-calculate field flags */
} CMS_PACKED cms_denm_create_t;

/**
Trigger DENM sending, or update an existing action.
@ingroup denm
If the auto_calc_action_id parameter is not set, a new Action Id is created, and parameters
will be set for this new action. This Id can then be used to update the action or
terminate it.
@param  session         Client session
@param  denm_ctx        Create/Update parameter
@param  payload         Payload
@param[out] action_id   Action id
@return true in case of error
*/
bool cms_denm_create_or_update(const cms_session_t* session,
                               const cms_denm_create_t* denm_ctx,
                               cms_buffer_view_t payload,
                               cms_denm_action_id_t* action_id);


/**
Trigger a DENM termination (negation or cancellation).
@ingroup denm
This function will cause sending of a termination DENM for
currently valid sent or received actions.
@param  session         Client session
@param  termination_ctx Termination parameter
@param[out] unused      Unused parameter
@return true in case of error
*/
bool cms_denm_terminate(const cms_session_t* session,
                        const cms_denm_termination_t* termination_ctx,
                        void* unused);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_DENM_H_ */
