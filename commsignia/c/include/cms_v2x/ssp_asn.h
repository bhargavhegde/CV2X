/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#ifndef CMS_V2X_SSP_ASN_H_
#define CMS_V2X_SSP_ASN_H_

#include <cms_v2x/fac_types.h>
#include <cms_v2x/security_types.h>
#include <asn1/fwd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fetch SSP from a CAM message.
 *
 * @param msg   CAM Message
 *
 * @return cms_ssp_t
 */
cms_ssp_t cms_fetch_ssp_cam(const struct EU_CoopAwareness* msg);

/**
 * Fetch SSP from a DENM message.
 *
 * @param msg   DENM Message
 *
 * @return cms_ssp_t
 */
cms_ssp_t cms_fetch_ssp_denm(const struct EU_DecentralizedEnvironmentalNotificationMessage* msg);


/**
 * Fetch SSP from an IVI message.
 *
 * @param msg   IVI Message
 *
 * @return cms_ssp_t
 */
cms_ssp_t cms_fetch_ssp_ivi(const struct EU_IviStructure* msg);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SSP_ASN_H_ */
