/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_GF_H_
#define CMS_GF_H_

#include <cms_v2x/common_types.h>

/** @file
@brief Geofencing
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup gf Geofencing
@ingroup api
*/

/**
Geofence enable/disable.
@ingroup gf
*/
typedef struct cms_gf_state_t {
    bool enable;
} CMS_PACKED cms_gf_state_t;

/**
Enumeration of Geofence controllable modules
@ingroup gf
*/
typedef enum cms_gf_module_t {
    CMS_GF_MODULE_CAM = 0,
    CMS_GF_MODULE_DENM,
    CMS_GF_MODULE_DTA,
    CMS_GF_MODULE_GNP,
    CMS_GF_MODULE_MIT,
    CMS_GF_NUMBER_OF_MODULES
} cms_gf_module_t;

/**
Enumeration of module state for the Geofence supervised modules
@ingroup gf
*/
typedef enum cms_gf_module_state_t {
    CMS_GF_MODULE_STATE_GEOFENCED = 0, /**< Module is geofence controlled */
    CMS_GF_MODULE_STATE_PRESERVE       /**< Module is controlled by its config and API */
} cms_gf_module_state_t;

/**
Enumeration of applied module action
@ingroup gf
*/
typedef enum cms_gf_module_action_t {
    /** Module is disabled */
    CMS_GF_MODULE_ACTION_DISABLE = 0,
    /** Module is enabled */
    CMS_GF_MODULE_ACTION_ENABLE
} cms_gf_module_action_t;

/**
Struct for referencing a module
@ingroup gf
*/
typedef struct cms_gf_module_ref_t {
    cms_gf_module_t module; /**< Module */
} CMS_PACKED cms_gf_module_ref_t;

/**
Struct for module control state
@ingroup gf
*/
typedef struct cms_gf_module_control_t {
    cms_gf_module_t module;             /**< Module */
    cms_gf_module_state_t module_state; /**< Module state */
} CMS_PACKED cms_gf_module_control_t;

/**
Geofencing module action notification struct
@ingroup gf
*/
typedef struct cms_gf_notif_data_t {
    cms_gf_module_t module;               /**< Module */
    cms_gf_module_action_t module_action; /**< Module state */
} CMS_PACKED cms_gf_notif_data_t;

/**
Geofencing module action reference
@ingroup gf
*/
typedef struct cms_gf_module_action_ref_t {
    cms_gf_module_action_t module_action; /**< Module state */
} CMS_PACKED cms_gf_module_action_ref_t;

/**
Geofencing module action notification callback
@ingroup gf
*/
typedef void (*cms_gf_notif_f)(const cms_gf_notif_data_t* gf_notif, void* ctx);

/**
Function to enable/disable Geofence
@ingroup gf
@param  session         Client session
@param  state_in        Geofence enable/disable
@param  unused_out      Unused parameter
@return true in case of error
*/
bool cms_gf_enable(const cms_session_t* session, const cms_gf_state_t* state_in, void* unused_out);

/**
Function to get is Geofence enabled/disabled
@ingroup gf
@param  session         Client session
@param  unused_in       Unused parameter
@param  state_out       Geofence enabled/disabled
@return true in case of error
*/
bool cms_gf_get_enabled(const cms_session_t* session,
                        const void* unused_in,
                        cms_gf_state_t* state_out);

/**
Function to get module control mode for a given module.
@ingroup gf
@param session                  Client session
@param module_ref               Module reference
@param[out] module_control      Module control mode
@return true in case of error
*/
bool cms_gf_get_module_control(const cms_session_t* session,
                               const cms_gf_module_ref_t* module_ref,
                               cms_gf_module_control_t* module_control);

/**
Function to set module control mode for a given module.
@ingroup gf
@param session                  Client session
@param module_control_to_set    Module and control mode to set
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_gf_set_module_control(const cms_session_t* session,
                               const cms_gf_module_control_t* module_control_to_set,
                               void* unused_out);

/**
GF notification subscribe for module the applied module action.
@ingroup gf
@param session              Client session
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_gf_unsubscribe
*/
bool cms_gf_subscribe(const cms_session_t* session,
                      cms_gf_notif_f callback_f,
                      void* ctx,
                      cms_subs_id_t* subs_id);

/**
GF notification unsubscribe.
@ingroup gf
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_gf_subscribe
@return true in case of error
@see cms_gf_subscribe
*/
bool cms_gf_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

/**
Get the enabled/disabled state of a module. This can be used e.g. to get the initial state
right after subscribing to module action notifications.

@ingroup gf
@param  session                 Client session
@param  module_ref              Reference to a module
@param[out] module_action       Module state
@return true in case of error
*/
bool cms_gf_get_module_action(const cms_session_t* session,
                              const cms_gf_module_ref_t* module_ref,
                              cms_gf_module_action_ref_t* module_action);

/**
String name of a Geofence controllable module.
@ingroup gf
@param module   module type
@return string literal of the name
*/
static inline const char* cms_gf_module_to_string(cms_gf_module_t module)
{
    const char* result = NULL;

    switch(module) {
    case CMS_GF_MODULE_CAM:
        result = "CAM";
        break;
    case CMS_GF_MODULE_DENM:
        result = "DENM";
        break;
    case CMS_GF_MODULE_DTA:
        result = "DTA";
        break;
    case CMS_GF_MODULE_GNP:
        result = "GNP";
        break;
    case CMS_GF_MODULE_MIT:
        result = "MIT";
        break;
    default:
        result = "invalid/unknown module";
    }
    return result;
}

/**
String name of a Geofence controllable module state.
@ingroup gf
@param state   module state type
@return string literal of the state
*/
static inline const char* cms_gf_module_state_to_string(cms_gf_module_state_t state)
{
    const char* result = NULL;

    switch(state) {
    case CMS_GF_MODULE_STATE_GEOFENCED:
        result = "geofenced";
        break;
    case CMS_GF_MODULE_STATE_PRESERVE:
        result = "preserve";
        break;
    default:
        result = "invalid/unknown module control";
    }
    return result;
}

/**
String name of a Geofence controllable module action.
@ingroup gf
@param module_action    module action
@return string literal of the module action
*/
static inline const char* cms_gf_module_action_to_string(cms_gf_module_action_t module_action)
{
    const char* result = NULL;

    switch(module_action) {
    case CMS_GF_MODULE_ACTION_DISABLE:
        result = "disable";
        break;
    case CMS_GF_MODULE_ACTION_ENABLE:
        result = "enable";
        break;
    default:
        result = "invalid/unknown module action";
    }
    return result;
}


#ifdef __cplusplus
}
#endif

#endif /* CMS_GF_H_ */
