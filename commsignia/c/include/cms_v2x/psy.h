/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_PSY_H_
#define CMS_V2X_PSY_H_

/** @file
@brief Station Info
*/

#include <stdbool.h>

#include <cms_v2x/fwd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup psy Pseudonimity
@ingroup api
*/

/**
Type of pseudonym change trigger
@ingroup psy
@experimentalapi
*/
typedef enum {
    CMS_PSY_TRIGGER_FORCED = 0,                    /**< Force pseudonym change (e.g. for testing purposes) */
    CMS_PSY_TRIGGER_FORCED_BY_HAHSEDID8_COLLISION  /**< Force pseudonym change triggered by hashedid8 collision */
} cms_psy_trigger_type_t;


/**
Input parameters for pseudonym change triggering
@ingroup psy
@experimentalapi
*/
typedef struct cms_psy_trigger_params_t {
    cms_psy_trigger_type_t type;
} CMS_PACKED cms_psy_trigger_params_t;

/**
Trigger pseudonym certificate change
@ingroup psy
Trigger a pseudonymity change immediately,
without taking into account any of the pseudonymity change criteria.
The function returns immediately, and does not wait for the actual change to happen.

@experimentalapi
@param session        Client session
@param params         Triggering parameters
@param unused_out     Unused parameter
@return true in case of error
*/
bool cms_psy_trigger(const cms_session_t* session,
                     const cms_psy_trigger_params_t* params,
                     void* unused_out);

/**
Input parameters for allowing/disallowing pseudonym certificate change
@ingroup psy
*/
typedef struct cms_psy_allow_change_data_t {
    bool allowed;
} CMS_PACKED cms_psy_allow_change_data_t;

/**
Allow/disallow pseudonym certificate change.
@ingroup psy

@experimentalapi
@param session          Client session
@param params           Input parameters
@param unused_out       Unused parameter
@return true in case of error
*/
bool cms_psy_allow_change(const cms_session_t* session,
                          const cms_psy_allow_change_data_t* params,
                          void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_PSY_H_ */
