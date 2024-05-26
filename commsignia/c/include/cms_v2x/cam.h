/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_CAM_H_
#define CMS_V2X_CAM_H_

/** @file
@brief CAM data types
*/

#include <stdint.h>
#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup cam CAM
@ingroup api
*/

/**
CAM Tx enable/disable.
@ingroup cam
*/
typedef struct cms_cam_tx_state_t {
    bool enable;
} CMS_PACKED cms_cam_tx_state_t;

/**
Function to enable/disable CAM Tx
@ingroup cam

@param  session         Client session
@param  state_in        CAM Tx enable/disable
@param  unused_out      Unused parameter
@return error
*/

bool cms_cam_tx_enable(const cms_session_t* session,
                       const cms_cam_tx_state_t* state_in,
                       void* unused_out);

/**
Function to get is CAM Tx enabled/disabled
@ingroup cam

@param  session         Client session
@param  unused_in       Unused parameter
@param  state_out       CAM Tx enabled/disabled
@return error
*/

bool cms_cam_tx_get_enabled(const cms_session_t* session,
                            const void* unused_in,
                            cms_cam_tx_state_t* state_out);

/**
Number of CAM extension fields
@ingroup cam
*/
typedef struct cms_cam_ext_num_t {
    uint8_t num_ext;
} CMS_PACKED cms_cam_ext_num_t;

/**
Function to set the number of CAM extensions
If it is called it deletes all the previously set extensions
@ingroup cam

@param  session         Client session
@param  ext_num_in      Number of extensions
@param  unused_out      Unused parameter
@return error
*/

bool cms_cam_set_extensions_num(const cms_session_t* session,
                                const cms_cam_ext_num_t* ext_num_in,
                                const void* unused_out);

/**
CAM Extension parameters
@ingroup cam
*/
typedef struct cms_cam_ext_t {
    uint8_t id; /* The ID of the extension to be set/reset */
} CMS_PACKED cms_cam_ext_t;

/**
Function to set CAM extensions by id. If extension already set, it updates the extension
If the data size zero, extension will be removed from CAM
@ingroup cam

@param  session         Client session
@param  ext_in          CAM extension parameter
@param  buff_in         CAM extension data
@param  unused_out      Unused parameter
@return error
*/

bool cms_cam_set_extension(const cms_session_t* session,
                           const cms_cam_ext_t* ext_in,
                           cms_buffer_view_t buff_in,
                           const void* unused_out);

/**
Function to reset CAM extension
The extension with the given id will be removed, other extensions stay unchanged.
@ingroup cam

@param  session         Client session
@param  ext_in          CAM extension parameter
@param  unused_out      Unused parameter
@return error
*/

static inline bool cms_cam_reset_extension(const cms_session_t* session,
                                           const cms_cam_ext_t* ext_in,
                                           const void* unused_out)
{
    const cms_buffer_view_t buff_in = {0};
    return cms_cam_set_extension(session, ext_in, buff_in, unused_out);
}

/**
Function to set CAM Special Vehicle Container. If already set, it updates the container
If the data size zero, container will be removed from CAM
@ingroup cam

@param  session         Client session
@param  buff_in         Special Vehicle Container data
@param  unused_out      Unused parameter
@return error
*/

bool cms_cam_set_special_vehicle(const cms_session_t* session,
                                 const void* unused_in,
                                 cms_buffer_view_t buff_in,
                                 const void* unused_out);

/**
Function to reset CAM Special Vehicle Container by id
@ingroup cam

@param  session         Client session
@param  unused_in       Unused parameter
@param  unused_out      Unused parameter
@return error
*/

static inline bool cms_cam_reset_special_vehicle(const cms_session_t* session,
                                                 const void* unused_in,
                                                 const void* unused_out)
{
    const cms_buffer_view_t buff_in = {0};
    return cms_cam_set_special_vehicle(session, unused_in, buff_in, unused_out);
}

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_CAM_H_ */
