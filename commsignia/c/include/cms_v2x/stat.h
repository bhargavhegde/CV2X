/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2022
*/

#ifndef CMS_V2X_STAT_H_
#define CMS_V2X_STAT_H_

/** @file
@brief Statistics
*/

#include <stdint.h>
#include <stddef.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/stat_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup stat Statistics
@ingroup api
*/

/**
Get statistics counter(s) from a specific module.
@ingroup stat
Retrieves either one specific counter or all counters from a module,
or all counters from all modules.
@param session      Client session
@param to_get       Counter(s) to retrieve from a module
@param[out] result  Counter(s) retrieved from a module
@return true in case of error
*/
bool cms_stat_get(const cms_session_t* session,
                  const cms_stat_counter_refs_t* to_get,
                  cms_stat_counters_t* result);

/**
Set statistics counter(s) in a specific module to a given value.
@ingroup stat
Sets either one specific counter or all counters in a module,
or all counters in all modules.
@param session      Client session
@param to_set       Counter(s) to set in a module
@param unused_out   Unused, expects NULL
@return true in case of error
*/
bool cms_stat_set(const cms_session_t* session,
                  const cms_stat_item_t* to_set,
                  void* unused_out);

/**
Resets statistics counter(s) in a specific module to zero.
@ingroup stat
Resets either one specific counter or all counters in a module,
or all counters in all modules.
@param session      Client session
@param to_reset     Counter(s) to reset in a module
@param unused_out   Unused, expects NULL
@return true in case of error
*/
static inline bool cms_stat_reset(const cms_session_t* session,
                                  const cms_stat_counter_ref_t* to_reset,
                                  void* unused_out)
{
    bool error = (NULL == to_reset);

    if(!error) {
        cms_stat_item_t zero = {
            .module = to_reset->module,
            .counter = to_reset->counter,
            .value = 0U
        };
        error = cms_stat_set(session, &zero, unused_out);
    }

    return error;
}

/**
Get device status
@ingroup stat
Gets device and miscellaneous data.
@param session      Client session
@param unused_in    Unused, expects NULL
@param[out] result  Device and miscellaneous data
@return true in case of error
*/
bool cms_stat_get_device_status(const cms_session_t* session,
                                const void* unused_in,
                                cms_stat_device_status_t* result);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_STAT_H_ */
