/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#ifndef CMS_V2X_SSP_H_
#define CMS_V2X_SSP_H_

#include <cms_v2x/fac_types.h>
#include <cms_v2x/security_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fetch SSP from the given message.
 *
 * @param msg_type      Type of the message
 * @param msg           Message
 *
 * @return cms_ssp_t
 */
cms_ssp_t cms_fetch_ssp(cms_fac_msg_type_t msg_type, const void* msg);

/**
 * Result of the SSP check
 */
typedef enum cms_ssp_check_result_t {
    /** The check succeeded, the SSP of the message
     *  is a valid subset of the SSP of the certificate */
    CMS_SSP_CHECK_RES_MATCH = 0,
    CMS_SSP_CHECK_RES_INTERNAL_ERROR,           /**< Internal error */
    CMS_SSP_CHECK_RES_VERSION_MISMATCH,         /**< The SSP versions are different */
    CMS_SSP_CHECK_RES_SSP_TYPE_MISMATCH,        /**< The SSP types are different */
    CMS_SSP_CHECK_RES_SSP_LENGTH_MISMATCH,      /**< The length of the SSP fields are different */
    /** Not allowed feature is used in the message.
     *  (The feature SSP field of the message are not a subset of the SSP of the certificate) */
    CMS_SSP_CHECK_RES_DISALLOWED_FEATURE_USAGE
} cms_ssp_check_result_t;

/**
Convert SSP check result into a readable string.

@param  res             SSP check result
@return readable string
@see cms_ssp_check_result_t
*/
static inline const char* cms_ssp_check_res_to_str(cms_ssp_check_result_t res)
{
    const char* result = NULL;

    switch(res) {
    case CMS_SSP_CHECK_RES_MATCH:
        result = "Match";
        break;
    case CMS_SSP_CHECK_RES_INTERNAL_ERROR:
        result = "Internal error";
        break;
    case CMS_SSP_CHECK_RES_VERSION_MISMATCH:
        result = "Version mismatch";
        break;
    case CMS_SSP_CHECK_RES_SSP_TYPE_MISMATCH:
        result = "Type mismatch";
        break;
    case CMS_SSP_CHECK_RES_SSP_LENGTH_MISMATCH:
        result = "Length mismatch";
        break;
    case CMS_SSP_CHECK_RES_DISALLOWED_FEATURE_USAGE:
        result = "Disallowed feature usage";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
 * Check if the SSP of the message is a subset of the SSP of the certificate
 *
 * @param ssp_of_msg            SSP of the message
 * @param ssp_of_cert           SSP of the certificate
 *
 * @return result code
 * @see cms_ssp_check_result_t
 */
cms_ssp_check_result_t cms_check_ssp(const cms_ssp_t* ssp_of_msg, const cms_ssp_t* ssp_of_cert);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SSP_H_ */
