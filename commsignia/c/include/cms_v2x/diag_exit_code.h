/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#ifndef CMS_DIAG_EXIT_CODE_H_
#define CMS_DIAG_EXIT_CODE_H_

#include <cms_v2x/diag_error.h>

/** @file
@brief Exit Code mapping to Diagnostic Error Codes
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup diag Diagnostics
@ingroup api
*/

/**
@defgroup diag_exit_code DiagnosticsExitCodes
@ingroup diag
*/

/**
Diagnostics Exit Code
@ingroup diag_exit_code
*/
typedef uint8_t cms_exit_code_t;

/**
Generic error exit code
@ingroup diag_exit_code
*/
#define CMS_EXIT_CODE_GENERIC_ERROR 0xFF

/**
Converts exit code to diagnostics error
@ingroup diag_exit_code
@param exit_code            Exit Code
@return Diagnostics error group and error

If the exit code cannot be mapped to diagnostics error,
then the returned struct contains CMS_DIAG_ERROR_GROUP_NA and CMS_DIAG_ERROR_CODE_NA.
*/
cms_diag_error_id_t cms_conv_exit_code_to_diag_error_id(cms_exit_code_t exit_code);

/**
Converts diagnostics error to exit code value
@ingroup diag_exit_code
@param error_id             Diagnostics error
@return exit code, CMS_EXIT_CODE_GENERIC_ERROR

If the diagnostics error parameter cannot be mapped to an exit code,
then CMS_EXIT_CODE_GENERIC_ERROR is returned.
*/
cms_exit_code_t cms_conv_diag_error_id_to_exit_code(const cms_diag_error_id_t* error_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_DIAG_EXIT_CODE_H_ */
