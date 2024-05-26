/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#ifndef CMS_V2X_MIT_H_
#define CMS_V2X_MIT_H_

/** @file
@brief MIT data types
*/

#include <stdint.h>
#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup mit Mitigation
@ingroup api
*/

/**
Mitigation enable/disable.
@ingroup mit
*/
typedef struct cms_mit_state_t {
    bool enable;
} CMS_PACKED cms_mit_state_t;

/**
Function to enable/disable Mitigation
@ingroup mit

@param  session         Client session
@param  state_in        Mitigation enable/disable
@param  unused_out      Unused parameter
@return true in case of error
*/

bool cms_mit_enable(const cms_session_t* session,
                    const cms_mit_state_t* state_in,
                    void* unused_out);

/**
Function to get is Mitigation enabled/disabled
@ingroup mit

@param  session         Client session
@param  unused_in       Unused parameter
@param  state_out       Mitigation enabled/disabled
@return true in case of error
*/

bool cms_mit_get_enabled(const cms_session_t* session,
                         const void* unused_in,
                         cms_mit_state_t* state_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_MIT_H_ */
