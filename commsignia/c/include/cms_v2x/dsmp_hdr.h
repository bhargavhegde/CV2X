/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#ifndef CMS_V2X_DSMP_HDR_H_
#define CMS_V2X_DSMP_HDR_H_

/** @file
@brief Dedicated Short Message Protocol (DSMP) header types
*/

#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
Length of the 2D location field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_2d_location_t
*/
#define CMS_DSMP_EXT_2D_LOC_LENGTH 8U

/**
2D location field in the DSMP header info extension
@ingroup dsmp
@see CMS_DSMP_EXT_2D_LOC_LENGTH
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_2d_location_t[CMS_DSMP_EXT_2D_LOC_LENGTH];

/**
Length of the 3D location field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_3d_location_t
*/
#define CMS_DSMP_EXT_3D_LOC_LENGTH 11U

/**
3D location field in the DSMP header info extension
@ingroup dsmp
@see CMS_DSMP_EXT_3D_LOC_LENGTH
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_3d_location_t[CMS_DSMP_EXT_3D_LOC_LENGTH];

/**
Maximum length of the advertiser identifier
@ingroup dsmp
@see cms_dsmp_ext_advertiser_identifier_t
*/
#define CMS_DSMP_EXT_ADV_ID_MAX_LENGTH 32U

/**
Advertiser identifier field in the DSMP header info extension
@ingroup dsmp
@see CMS_DSMP_EXT_ADV_ID_MAX_LENGTH
@see cms_dsmp_ext_t
*/
typedef struct {
    uint8_t length;                                 /**< Length of the advertiser identifier */
    uint8_t data[CMS_DSMP_EXT_ADV_ID_MAX_LENGTH];   /**< Buffer for the advertiser identifier */
} CMS_PACKED cms_dsmp_ext_advertiser_identifier_t;

/**
Maximum length of the provider service context
@ingroup dsmp
@see cms_dsmp_ext_provide_service_ctx_t
*/
#define CMS_DSMP_EXT_PROVIDER_SRV_CTX_MAX_LENGTH 31U

/**
Provider service context field in the DSMP header info extension
@ingroup dsmp
@see CMS_DSMP_EXT_PROVIDER_SRV_CTX_MAX_LENGTH
@see cms_dsmp_ext_t
*/
typedef struct {
    uint8_t length;                                             /**< Length of the provider service context */
    uint8_t data[CMS_DSMP_EXT_PROVIDER_SRV_CTX_MAX_LENGTH];     /**< Buffer for the provider service context */
} CMS_PACKED cms_dsmp_ext_provider_service_ctx_t;

/**
Broadcast frequency field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_broadcast_freq_t;

/**
Maximum length of the area string
@ingroup dsmp
@see cms_dsmp_ext_area_str_t
*/
#define CMS_DSMP_EXT_AREA_STRING_LENGTH 3U

/**
Area string field in the DSMP header info extension
@ingroup dsmp
@see CMS_DSMP_EXT_AREA_STRING_LENGTH
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_area_str_t[CMS_DSMP_EXT_AREA_STRING_LENGTH];

/**
DSA RCPI threshold field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_dsa_rcpi_treshold_t;

/**
DSA count threshold field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_dsa_cnt_treshold_t;

/**
DSA count threshold interval field in the DSMP header info extension
@ingroup dsmp
@see cms_dsmp_ext_t
*/
typedef uint8_t cms_dsmp_ext_dsa_cnt_treshold_interval_t;

/**
DSMP header info extension with optional fields
@ingroup dsmp
@see cms_dsmp_hdr_info_t
*/
struct cms_dsmp_ext_t {
    bool location_2d_option;                                            /**< 2D location to be sent */
    cms_dsmp_ext_2d_location_t location_2d;                             /**< 2D location */
    bool location_3d_option;                                            /**< 3D location to be sent */
    cms_dsmp_ext_3d_location_t location_3d;                             /**< 3D location */
    bool advertiser_identifier_option;                                  /**< Advertiser identifier to be sent*/
    cms_dsmp_ext_advertiser_identifier_t advertiser_identifier;         /**< Advertiser identifier */
    bool provider_srv_ctx_option;                                       /**< Provider service context to be sent */
    cms_dsmp_ext_provider_service_ctx_t provider_service_ctx;           /**< Provider service context */
    bool ipv6_addr_option;                                              /**< IPv6 address to be sent */
    cms_ipv6_addr_t ipv6_addr;                                          /**< IPv6 address */
    bool service_port_option;                                           /**< Service port to be sent */
    cms_ip_port_t service_port;                                         /**< Service port */
    bool provider_mac_addr_option;                                      /**< Provider MAC address to be sent */
    cms_mac_addr_t provider_mac_addr;                                   /**< Provider MAC address*/
    bool broadcast_freq_option;                                         /**< Broadcast frequency to be sent */
    cms_dsmp_ext_broadcast_freq_t broadcast_freq;                       /**< Broadcast frequency */
    bool area_str_option;                                               /**< Area string to be sent */
    cms_dsmp_ext_area_str_t area_str;                                   /**< Area string */
    bool dsa_rcpi_treshold_option;                                      /**< DSA RCPI threshold to be sent */
    cms_dsmp_ext_dsa_rcpi_treshold_t dsa_rcpi_treshold;                 /**< DSA RCPI threshold */
    bool dsa_cnt_treshold_option;                                       /**< DSA count threshold to be sent */
    cms_dsmp_ext_dsa_cnt_treshold_t dsa_cnt_treshold;                   /**< DSA count threshold */
    bool dsa_cnt_treshold_interval_option;                              /**< DSA count threshold interval to be sent */
    cms_dsmp_ext_dsa_cnt_treshold_interval_t dsa_cnt_treshold_interval; /**< DSA count threshold interval */
} CMS_PACKED;

/**
DSMP header info parameters of a packet.
@ingroup dsmp
*/
struct cms_dsmp_hdr_info_t {
    cms_psid_t aid;             /**< Application ID */
    cms_dsmp_ext_t extension;   /**< Extension for the optional fields */
} CMS_PACKED;


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_DSMP_HDR_H_ */
