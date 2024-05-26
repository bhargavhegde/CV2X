/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022
*/

#ifndef CMS_DIAG_PRINT_H_
#define CMS_DIAG_PRINT_H_

/** @file
@brief Statistics print helpers
*/

#include <cms_v2x/fwd.h>
#include <cms_v2x/diag_error.h>
#include <cms_v2x/diag_status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
Print error in JSON format
@param error_id                 Error ID
@param error_stat               Error statistics
@param[inout] buffer            Buffer containing the counters deparsed to JSON format.
                                If it contains any string it tries to parse it as a JSON string and append the
                                counters to the JSON.
@return true in case of error
@ingroup diag
*/
bool cms_diag_print_error(const cms_diag_error_id_t* error_id,
                          const cms_diag_error_stat_t* error_stat,
                          cms_mutable_buffer_t* buffer);

/**
Print error in JSON format with 2 spaces indentation
@param error_id                 Error ID
@param error_stat               Error statistics
@param[inout] buffer            Buffer containing the counters deparsed to JSON format.
                                If it contains any string it tries to parse it as a JSON string and append the
                                counters to the JSON.
@return true in case of error
@ingroup diag
*/
bool cms_diag_pretty_print_error(const cms_diag_error_id_t* error_id,
                                 const cms_diag_error_stat_t* error_stat,
                                 cms_mutable_buffer_t* buffer);

/**
Print status in JSON format
@param status_id                Status ID
@param status_value             Status value
@param[inout] buffer            Buffer containing the counters deparsed to JSON format.
                                If it contains any string it tries to parse it as a JSON string and append the
                                counters to the JSON.
@return true in case of error
@ingroup diag
*/
bool cms_diag_print_status(const cms_diag_status_id_t* status_id,
                           cms_diag_status_value_t status_value,
                           cms_mutable_buffer_t* buffer);

/**
Print status in JSON format with 2 spaces indentation
@param status_id                Status ID
@param status_value             Status value
@param[inout] buffer            Buffer containing the counters deparsed to JSON format.
                                If it contains any string it tries to parse it as a JSON string and append the
                                counters to the JSON.
@return true in case of error
@ingroup diag
*/
bool cms_diag_pretty_print_status(const cms_diag_status_id_t* status_id,
                                  cms_diag_status_value_t status_value,
                                  cms_mutable_buffer_t* buffer);

#ifdef __cplusplus
}
#endif

#endif /* CMS_DIAG_PRINT_H_*/
