/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_V2X_DTA_H_
#define CMS_V2X_DTA_H_

#include <cms_v2x/fwd.h>

/** @file
@brief DTA - Traffic Jam internal API
Internal API between CFF Traffic Jam helper and the DTA Traffic Jam application.
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup dta DENM Triggering Applications
@ingroup api
*/

/**
DENM triggering applications enable/disable.
@ingroup dta
*/
typedef struct cms_dta_state_t {
    bool enable;
} CMS_PACKED cms_dta_state_t;

/**
Function to enable/disable DENM triggering applications
@ingroup dta

@param  session         Client session
@param  state_in        DENM triggering applications enable/disable
@param  unused_out      Unused parameter
@return true in case of error
*/

bool cms_dta_enable(const cms_session_t* session,
                    const cms_dta_state_t* state_in,
                    void* unused_out);

/**
Function to get is DENM triggering applications enabled/disabled
@ingroup dta

@param  session         Client session
@param  unused_in       Unused parameter
@param  state_out       DENM triggering applications enabled/disabled
@return true in case of error
*/

bool cms_dta_get_enabled(const cms_session_t* session,
                         const void* unused_in,
                         cms_dta_state_t* state_out);

/**
Dangerous end of queue reception info
@ingroup dta
@experimentalapi
*/
typedef struct cms_dta_dangerous_end_of_queue_reception_info_t {
    uint16_t cam_count;           /**< Count of different CAM messages received */
    uint16_t dangerous_end_count; /**< Count of different 'C2CCC Traffic jam - Dangerous End of
                                     Queue’ DENM messages received */
    uint16_t jam_ahead_count;     /**< Count of different 'C2CCC Traffic Jam - Traffic Jam Ahead'
                                       DENM messages received */
    uint16_t safeguard_count;     /**< Count of different 'C2CCC Special Vehicle Warning -
                                       Static Safeguarding Emergency Vehicle' DENM messages received */
} CMS_PACKED cms_dta_dangerous_end_of_queue_reception_info_t;

/**
Traffic jam ahead reception info
@ingroup dta
@experimentalapi
*/
typedef struct cms_dta_traffic_jam_ahead_reception_info_t {
    uint16_t cam_count;       /**< Count of different CAM messages received */
    uint16_t jam_ahead_count; /**< Count of different 'C2CCC Traffic Jam - Traffic Jam Ahead'
                                   DENM messages received */
} CMS_PACKED cms_dta_traffic_jam_ahead_reception_info_t;

/**
Traffic jam reception info
@ingroup dta
@experimentalapi
*/
typedef struct cms_dta_traffic_jam_reception_info_t {
    cms_dta_dangerous_end_of_queue_reception_info_t dangerous_end_of_queue_info;
    cms_dta_traffic_jam_ahead_reception_info_t traffic_jam_ahead_info;
} CMS_PACKED cms_dta_traffic_jam_reception_info_t;

/**
Set traffic jam info in DTA Traffic Jam Application
@ingroup dta
@experimentalapi
@param session                          Client session
@param traffic_jam_reception_info       Traffic jam reception info to set
@param[out] unused_out                  Not used
@return true in case of error
*/
bool cms_dta_set_traffic_jam_reception_info(
    const cms_session_t* session,
    const cms_dta_traffic_jam_reception_info_t* traffic_jam_reception_info,
    void* unused_out);

/**
Triggering information of the use-cases.
@ingroup dta
@experimentalapi
*/
typedef struct cms_dta_use_case_status_t {
    /* Dangerous situation use-cases */
    bool electronic_emergency_brake_light;
    bool automatic_brake_intervention;
    bool reversible_occupant_restraint_system_intervention;
    /* Adverse Weather use-cases */
    bool fog;
    bool precipitation;
    bool traction_loss;
    /* Stopped vehicle use-cases */
    bool postcrash;
    bool broken_down_vehicle;
    bool stopped_vehicle;
    /* Traffic Jam use-cases */
    bool dangerous_end_of_queue;
    bool traffic_jam_ahead;
} CMS_PACKED cms_dta_use_case_status_t;

/**
Get triggering status of C2C DTA use-cases
@ingroup dta
@experimentalapi
@param session                      Client session
@param unused_in                    Not used
@param[out] status                  Triggering status
@return true in case of error
*/
bool cms_dta_get_use_case_status(const cms_session_t* session,
                                 const void* unused_in,
                                 cms_dta_use_case_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_DTA_H_ */
