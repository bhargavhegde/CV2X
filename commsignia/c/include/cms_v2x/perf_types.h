/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_PERF_TYPES_H_
#define CMS_V2X_PERF_TYPES_H_

/** @file
@brief Performance measurement types
*/

#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
Platform independent ID of the process
@ingroup perf
*/
typedef uint32_t cms_perf_process_id_t;

/**
Platform independent ID of the thread
@ingroup perf
*/
typedef uint32_t cms_perf_thread_id_t;

/**
Thread ID for the main thread of the process
@ingroup perf
The ID of the main thread (only applicable if performance measurement per thread is supported)
 */
static const cms_perf_thread_id_t CMS_PERF_MAIN_THREAD_ID = 0U;

/**
Thread ID not available
@ingroup perf
 */
static const cms_perf_thread_id_t CMS_PERF_THREAD_ID_NA = UINT32_MAX;

/**
General performance counter
@ingroup perf
 */
typedef uint64_t cms_perf_counter_t;

/**
General performance counter not available value
@ingroup perf
 */
static const cms_perf_counter_t CMS_PERF_COUNTER_NA = UINT64_MAX;

/**
Maximal length of the process info list
@ingroup perf
*/
#define CMS_PERF_PROCESS_INFO_LIST_MAX_SIZE          8U

/**
Process information
@ingroup perf
*/
typedef struct cms_perf_process_info_t {
    cms_perf_process_id_t process_id;           /**< Process ID */
    uint16_t thread_count;                      /**< Number of threads in the process */
    bool thread_measurement_supported;          /**< True, if the per thread measurement is supported */
} CMS_PACKED cms_perf_process_info_t;

/**
Process information list
@ingroup perf
*/
typedef struct cms_perf_process_info_list_t {
    uint32_t length;                                                            /**< Effective length of 'process_info' */
    cms_perf_process_info_t process_info[CMS_PERF_PROCESS_INFO_LIST_MAX_SIZE];  /**< Process information */
} CMS_PACKED cms_perf_process_info_list_t;

/**
Process reference in performance measurements
@ingroup perf
*/
typedef struct cms_perf_process_ref_t {
    cms_perf_process_id_t process_id;           /**< Process ID */
} CMS_PACKED cms_perf_process_ref_t;

/**
Thread range reference in performance measurements
@ingroup perf
*/
typedef struct cms_perf_thread_range_ref_t {
    cms_perf_process_id_t process_id;           /**< Process ID */
    uint16_t start_thread_index;                /**< Start index of the thread list */
    uint16_t end_thread_index;                  /**< End index of the thread list */
} CMS_PACKED cms_perf_thread_range_ref_t;

/**
Maximal length of the performance result list
@ingroup perf
*/
#define CMS_PERF_RESULT_LIST_MAX_SIZE             20U

/**
General performance measurement result
@ingroup perf
*/
typedef struct cms_perf_result_t {
    cms_perf_process_id_t process_id;           /**< Process ID */
    cms_perf_thread_id_t thread_id;             /**< Thread ID */
    cms_perf_counter_t timestamp;               /**< Time stamp in [ns] */
    cms_perf_counter_t running_time;            /**< Time in [ns] the process/thread spent running from start */
    cms_perf_counter_t user_time;               /**< Time in [ns] the process/thread spent running in
                                                     user space from start (if supported) */
    cms_perf_counter_t kernel_time;             /**< Time in [ns] the process/thread spent running in
                                                     kernel space from start (if supported) */
    cms_perf_counter_t context_switches;        /**< Number of context switches from start (if supported) */
} CMS_PACKED cms_perf_result_t;

/**
Performance measurement result list
@ingroup perf
*/
typedef struct cms_perf_result_list_t {
    uint32_t length;                                                /**< Effective length of the 'result' */
    cms_perf_result_t result[CMS_PERF_RESULT_LIST_MAX_SIZE];        /**< List of results */
} CMS_PACKED cms_perf_result_list_t;

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_PERF_TYPES_H_ */
