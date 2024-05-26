/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_PERF_H_
#define CMS_V2X_PERF_H_

/** @file
@brief Performance measurement
*/

#include <cms_v2x/perf_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup perf Performance measurement
@ingroup api
*/

/**
Get process information list
@ingroup perf
@experimentalapi
@param session                  Client session
@param unused_in                Unused, expects NULL
@param[out] process_info_list   Process information list
@return true in case of error
 */
bool cms_perf_get_process_info_list(const cms_session_t* session,
                                    const void* unused_in,
                                    cms_perf_process_info_list_t* process_info_list);

/**
Get Performance measurement result for a specific process
@ingroup perf
Note: The result contains the measurement for the process with aggregation of the child threads.
@experimentalapi
@param session                  Client session
@param process_ref              Process reference
@param[out] result              Measurement result
@return true in case of error
 */
bool cms_perf_get_process_measurement_result(const cms_session_t* session,
                                             const cms_perf_process_ref_t* process_ref,
                                             cms_perf_result_t* result);

/**
Get Performance measurement results for a range of threads of a given process
@ingroup perf
@experimentalapi
@param session                  Client session
@param thread_range_ref         Thread range reference
@param[out] results             List of measurement results
@return true in case of error
 */
bool cms_perf_get_thread_measurement_results(const cms_session_t* session,
                                             const cms_perf_thread_range_ref_t* thread_range_ref,
                                             cms_perf_result_list_t* results);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_PERF_H_ */
