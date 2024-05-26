/** @file
 *  @copyright
 *  (C) Commsignia Ltd. - All Rights Reserved.
 *  Unauthorised copying of this file, via any medium is strictly prohibited.
 *  Proprietary and confidential.
 *  @date 2021
 */

#ifndef CMS_V2X_CN_BSM_H_
#define CMS_V2X_CN_BSM_H_

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** BSM Tx state.
 *  @ingroup cn_bsm
 */
typedef struct cms_cn_bsm_tx_state_t {
    bool enable;
} CMS_PACKED cms_cn_bsm_tx_state_t;

/** China BSM event flags
 *  @ingroup cn_bsm
 */
typedef struct cms_cn_bsm_event_flags_t {
    bool hazard_lights;
    bool stop_line;
    bool antilock_braking_system;
    bool traction_control;
    bool stability_control;
    bool hazard_material;
    bool hard_brake;
    bool lights_changed;
    bool wipers_changed;
    bool flat_tire;
    bool disabled_vehicle;
    bool air_bag_deployed;
} CMS_PACKED cms_cn_bsm_event_flags_t;

/**
 * Control block for overriding event flags in transmitted BSM packets
 * @ingroup cn_bsm
 */
typedef struct cms_cn_bsm_event_flag_control_t {
    bool enable;                                                   /** Enables forcing the event flags */
    cms_cn_bsm_event_flags_t flags;                                /** Event flag contents (only used when
                                                                   forcing is enabled) */

} CMS_PACKED cms_cn_bsm_event_flag_control_t;

/**
 * Siren use from the CN BSM emergency extension
 * @ingroup cn_bsm
 */
typedef enum cms_cn_bsm_siren_use_t {
    CMS_CN_BSM_SIREN_UNKNOWN = 0,                                  /** Siren use is unknown */
    CMS_CN_BSM_SIREN_NOT_IN_USE,                                   /** Siren not in use */
    CMS_CN_BSM_SIREN_IN_USE,                                       /** Siren in use */
} cms_cn_bsm_siren_use_t;

/**
 * Lightbar from the CN BSM emergency extension
 * @ingroup cn_bsm
 */
typedef enum cms_cn_bsm_lightbar_use_t {
    CMS_CN_BSM_LIGHTBAR_UNKNOWN = 0,                               /** Lightbar use is unknown */
    CMS_CN_BSM_LIGHTBAR_NOT_IN_USE,                                /** There is a lightbar, but it is currently not in use */
    CMS_CN_BSM_LIGHTBAR_IN_USE,                                    /** Lightbar in use */
    CMS_CN_BSM_LIGHTBAR_YELLOW_CAUTION_LIGHTS,                     /** Yellow caution lights */
    CMS_CN_BSM_LIGHTBAR_SCHOOL_BUS_LIGHTS,                         /** School bus lights */
    CMS_CN_BSM_LIGHTBAR_ARROW_SIGNS_ACTIVE,                        /** Arrow signs active */
    CMS_CN_BSM_LIGHTBAR_SLOW_MOVING_VEHICLE,                       /** Slow moving vehicle lights */
    CMS_CN_BSM_LIGHTBAR_FREQ_STOPS                                 /** Freq stops */
} cms_cn_bsm_lightbar_use_t;

/**
 * Response type from the CN BSM emergency extension
 * @ingroup cn_bsm
 */
typedef enum cms_cn_bsm_response_type_t {
    CMS_CN_BSM_RESPONSE_TYPE_UNKNOWN = 0,                           /** Response type is unknown */
    CMS_CN_BSM_RESPONSE_TYPE_EMERGENCY,                             /** Active service call at emergency level */
    CMS_CN_BSM_RESPONSE_TYPE_NON_EMERGENCY,                         /** Used when returning from an emergency call */
    CMS_CN_BSM_RESPONSE_TYPE_PURSUIT,                               /** Sender driving maybe erratic */
    CMS_CN_BSM_RESPONSE_TYPE_STATIONARY,                            /** Sender not moving, stopped along roadside */
    CMS_CN_BSM_RESPONSE_TYPE_SLOW_MOVING,                           /** Such as litter truck etc. */
    CMS_CN_BSM_RESPONSE_TYPE_STOP_AND_GO_MOVEMENT,                  /** Such as school bus or garbage truck */
} cms_cn_bsm_response_type_t;

/**
 * CN BSM emergency extension contents
 * @ingroup cn_bsm
 */
typedef struct cms_cn_bsm_emergency_ext_t {
    cms_cn_bsm_siren_use_t siren_use;
    cms_cn_bsm_lightbar_use_t lightbar_use;
    cms_cn_bsm_response_type_t response_type;
} CMS_PACKED cms_cn_bsm_emergency_ext_t;

/**
 * Control block for overriding the emergency extensions in transmitted BSM packets
 * @ingroup cn_bsm
 */
typedef struct cms_cn_bsm_emergency_ext_control_t {
    bool enable;                                                    /** Enables forcing the emergecy extension */
    cms_cn_bsm_emergency_ext_t emergency_ext;                       /** Emergency extension contents (only used when
                                                                        forcing is enabled) */
} CMS_PACKED cms_cn_bsm_emergency_ext_control_t;

/** Enable/disable China BSM Tx
 *  @ingroup cn_bsm
 *
 *  @param  session     Client session
 *  @param  state_in    Input parameters (BSM Tx enable/disable)
 *  @param  unused_out  Output (unused)
 *  @return error
 */
bool cms_cn_bsm_tx_enable(const cms_session_t* session,
                          const cms_cn_bsm_tx_state_t* state_in,
                          const void* unused_out);

/** Override event flags and force the given values in transmitted BSM packets
 *  @ingroup cn_bsm
 *
 *  @param  session     Client session
 *  @param  control     Control block containing predefined values
 *  @param  unused_out  Output (unused)
 *  @return error
 */
bool cms_cn_bsm_force_event_flags(const cms_session_t* session,
                                  const cms_cn_bsm_event_flag_control_t* control,
                                  const void* unused_out);

/** Override event flags and force the given values in transmitted BSM packets
 *  @ingroup cn_bsm
 *
 *  @param  session      Client session
 *  @param  control      Control block containing predefined values
 *  @param  unused_out   Output (unused)
 *  @return error
 */
bool cms_cn_bsm_force_emergency_ext(const cms_session_t* session,
                                    const cms_cn_bsm_emergency_ext_control_t* control,
                                    const void* unused_out);



#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_CN_BSM_H_ */
