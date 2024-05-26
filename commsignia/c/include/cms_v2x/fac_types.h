/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_V2X_FAC_TYPES_H_
#define CMS_V2X_FAC_TYPES_H_

/** @file
@brief Facility data types
*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup fac_types Facility data types
@ingroup fac_subscribe
*/

/**
Facility message types.
@ingroup fac_types
*/
typedef enum cms_fac_msg_type_t {

    CMS_FAC_MSG_EU_CAM = 1,     /**< EU Cooperative Awareness Message */
    CMS_FAC_MSG_EU_DENM = 2,    /**< EU Decentralized Environmental Notification Message */
    CMS_FAC_MSG_EU_MAP = 3,     /**< EU MAP */
    CMS_FAC_MSG_EU_SPAT = 4,    /**< EU Signal Phase & Timing */
    CMS_FAC_MSG_EU_IVI = 5,     /**< EU In-Vehicle Information */
    CMS_FAC_MSG_EU_RTCM = 6,    /**< EU Real Time Correction Message */
    CMS_FAC_MSG_EU_SRM = 7,     /**< EU Signal Request Message */
    CMS_FAC_MSG_EU_SSM = 8,     /**< EU Signal Status Message */
    CMS_FAC_MSG_EU_CPM = 9,     /**< EU Collective Perception Message */
    CMS_FAC_MSG_EU_VAM = 10,    /**< EU Vulnerable Road Users Awareness Message */

    CMS_FAC_MSG_US_BSM = 100,   /**< US Basic Safety Message */
    CMS_FAC_MSG_US_TIM = 101,   /**< US Traveler Information Message */
    CMS_FAC_MSG_US_RSA = 102,   /**< US Road Side Alert */
    CMS_FAC_MSG_US_MAP = 103,   /**< US MAP */
    CMS_FAC_MSG_US_SPAT = 104,  /**< US Signal Phase & Timing */
    CMS_FAC_MSG_US_SRM = 105,   /**< US Signal Request Message */
    CMS_FAC_MSG_US_SSM = 106,   /**< US Signal Status Message */
    CMS_FAC_MSG_US_PSM = 107,   /**< US Personal Safety Message */
    CMS_FAC_MSG_US_PVD = 108,   /**< US Probe Vehicle Data */
    CMS_FAC_MSG_US_PDM = 109,   /**< US Probe Data Management */
    CMS_FAC_MSG_US_RTCM = 110,  /**< US Real Time Correction Message */
    CMS_FAC_MSG_US_WSA = 111,   /**< US WAVE Service Advertisement */
    CMS_FAC_MSG_US_SDSM = 112,  /**< US Sensor Data Sharing Message */

    CMS_FAC_MSG_CN_BSM = 200,   /**< China Basic Safety Message */
    CMS_FAC_MSG_CN_MAP = 201,   /**< China MAP */
    CMS_FAC_MSG_CN_SPAT = 202,  /**< China Signal Phase & Timing */
    CMS_FAC_MSG_CN_RSI = 203,   /**< China Road Side Information */
    CMS_FAC_MSG_CN_RSM = 204,   /**< China Road Side Message */
} cms_fac_msg_type_t;


/**
Helpers to validate facility message type values.
@ingroup fac_types
@see cms_fac_msg_type_t
*/
enum {
    CMS_FAC_MSG_MIN = CMS_FAC_MSG_EU_CAM,   /**< Minimum of valid values in cms_fac_msg_type_t */
    CMS_FAC_MSG_MAX = CMS_FAC_MSG_CN_RSM    /**< Maximum of valid values in cms_fac_msg_type_t */
};


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_FAC_TYPES_H_ */
