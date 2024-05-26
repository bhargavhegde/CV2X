/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_BSM_H_
#define CMS_V2X_BSM_H_

/** @file
@brief BSM data types
*/

#include <stdint.h>
#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
BSM Tx state.
@ingroup bsm
*/
typedef struct cms_bsm_tx_state_t {
    bool enable;
} CMS_PACKED cms_bsm_tx_state_t;

/**
Function to enable/disable BSM Tx
@ingroup bsm

@param  session         Client session
@param  state_in        BSM Tx enable/disable
@param  unused_out      unused parameter
@return error
*/

bool cms_bsm_tx_enable(const cms_session_t* session,
                       const cms_bsm_tx_state_t* state_in,
                       const void* unused_out);

/**
Function to set Special Vehicle Extension in BSM
@ingroup bsm

@param  session         Client session
@param  unused_in       unused parameter
@param  buff_in         Special Vehicle Extension data. If the data size zero, Special Vehicle Extension will be removed from BSM.
@param  unused_out      unused parameter
@return error
*/

bool cms_bsm_set_spec_veh_extension(const cms_session_t* session,
                                    const void* unused_in,
                                    cms_buffer_view_t buff_in,
                                    const void* unused_out);

/**
Function to reset BSM Special Vehicle Extension in BSM
@ingroup bsm

@param  session         Client session
@param  unused_in       unused parameter
@param  unused_out      unused parameter
@return error
*/

static inline bool cms_bsm_reset_spec_veh_extension(const cms_session_t* session,
                                                    const void* unused_in,
                                                    const void* unused_out)
{
    const cms_buffer_view_t buff_in = {0};
    return cms_bsm_set_spec_veh_extension(session, unused_in, buff_in, unused_out);
}

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_BSM_H_ */
