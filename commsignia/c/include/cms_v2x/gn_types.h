/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_GN_TYPES_H_
#define CMS_V2X_GN_TYPES_H_

/** @file
@brief GeoNetworking Protocol (GNP) types
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup gn_types GeoNetworking Protocol (GNP) types
@ingroup gn
@note In the GeoNetworking scope, when BTP (Basic Transport Protocol)
is mentioned, it is considered as BTP type B.
@note BTP type A is not supported.
*/

/**
Invalid BTP port.
@ingroup gn_types
@see cms_btp_port_t
*/
#define CMS_BTP_PORT_INVALID            UINT16_MAX

/**
BTP port.
@ingroup gn_types
@see CMS_BTP_PORT_INVALID
*/
typedef uint16_t cms_btp_port_t;

/**
Invalid BTP destination port info.
@ingroup gn_types
@see cms_btp_dest_port_info_t
*/
#define CMS_BTP_DEST_PORT_INFO_INVALID  UINT16_MAX

/**
Default BTP destination port info.
@ingroup gn_types
@see cms_btp_dest_port_info_t
*/
#define CMS_BTP_DEST_PORT_INFO_DEFAULT  0U

/**
BTP destination port info.
@ingroup gn_types
@see CMS_BTP_DEST_PORT_INFO_INVALID
@see CMS_BTP_DEST_PORT_INFO_DEFAULT
*/
typedef uint16_t cms_btp_dest_port_info_t;

/**
GeoNetworking full position data.
@ingroup common
*/
struct cms_gn_full_pos_t {
    cms_latitude_t latitude;            /**< Latitude [0.1 udeg] */
    cms_longitude_t longitude;          /**< Longitude [0.1 udeg] */
    cms_speed_t speed;                  /**< Speed value [mm/s] */
    cms_heading_t heading;              /**< Heading value [0.001 deg] */
} CMS_PACKED;

/**
Not available/invalid GeoNetworking traffic class.
@ingroup gn_types
@see cms_gn_traffic_class_t
*/
#define CMS_GN_TRAFFIC_CLASS_NA         UINT8_MAX

/**
GeoNetworking traffic class: class id
@ingroup gn_types
@see cms_gn_traffic_class_t
*/
typedef uint8_t cms_gn_traffic_class_tcid_t;

/**
GeoNetworking traffic class: Store Carry Forward enabled or not
@ingroup gn_types
@see cms_gn_traffic_class_t
*/
typedef enum cms_gn_traffic_class_scf_en_t {
    CMS_GN_TRAFFIC_CLASS_SCF_DISABLE = 0,   /**< SCF - Store Carry Forward disable */
    CMS_GN_TRAFFIC_CLASS_SCF_ENABLE = 1     /**< SCF - Store Carry Forward enable */
} cms_gn_traffic_class_scf_en_t;

/**
GeoNetworking traffic class: Channel Offload enabled or not
@ingroup gn_types
@see cms_gn_traffic_class_t
*/
typedef enum cms_gn_traffic_class_chf_offl_t {
    CMS_GN_TRAFFIC_CLASS_CHF_OFFL_DISABLE = 0,  /**< CHF OFFL - Channel Offload disabled */
    CMS_GN_TRAFFIC_CLASS_CHF_OFFL_ENABLE = 1    /**< CHF OFFL - Channel Offload enabled */
} cms_gn_traffic_class_chf_offl_t;

/**
GeoNetworking traffic class
@ingroup gn_types
@see cms_gn_traffic_class_t
*/
struct cms_gn_traffic_class_t {
    cms_gn_traffic_class_scf_en_t scf_en;       /**< Store Carry Forward enabled or not */
    cms_gn_traffic_class_chf_offl_t chf_offl;   /**< Channel Offload enabled or not */
    cms_gn_traffic_class_tcid_t tcid;           /**< Traffic class id */
} CMS_PACKED;

/**
Not available/invalid GeoNetworking lifetime.
@ingroup gn_types
@see cms_gn_lifetime_ms_t
*/
#define CMS_GN_LIFETIME_MS_NA           UINT32_MAX

/**
GeoNetworking lifetime.
Unit: [ms]
@ingroup gn_types
@see CMS_GN_LIFETIME_MS_NA
*/
typedef uint32_t cms_gn_lifetime_ms_t;

/**
GeoNetworking send method.
@ingroup gn_types
*/
typedef enum cms_gn_send_method_t {
    CMS_GN_SEND_METHOD_NA = 0,          /**< Not available/invalid */
    CMS_GN_SEND_METHOD_BEACON,          /**< Beacon */
    CMS_GN_SEND_METHOD_SHB,             /**< Single Hop Broadcast */
    CMS_GN_SEND_METHOD_TSB,             /**< Topologically Scoped Broadcast */
    CMS_GN_SEND_METHOD_GAC,             /**< GeoAnycast */
    CMS_GN_SEND_METHOD_GBC,             /**< GeoBroadcast */
    CMS_GN_SEND_METHOD_GUC,             /**< GeoUnicast */
    CMS_GN_SEND_METHOD_LS_REQ,          /**< Location Service Request */
    CMS_GN_SEND_METHOD_LS_REPLY         /**< Location Service Reply */
} cms_gn_send_method_t;

/**
Not available/invalid GeoNetworking hop limit.
@ingroup gn_types
@see cms_gn_hop_limit_t
*/
#define CMS_GN_HOP_LIMIT_NA             UINT8_MAX

/**
GeoNetworking hop limit.
@ingroup gn_types
@see CMS_GN_HOP_LIMIT_NA
*/
typedef uint8_t cms_gn_hop_limit_t;

/**
GeoNetworking destination area type.
@ingroup gn_types
*/
typedef enum cms_gn_dest_area_type_t {
    CMS_GN_DEST_AREA_TYPE_NA = 0,       /**< Destination area shape: not available/invalid */
    CMS_GN_DEST_AREA_TYPE_CIRCLE,       /**< Destination area shape: circle */
    CMS_GN_DEST_AREA_TYPE_ELLIPSE,      /**< Destination area shape: ellipse */
    CMS_GN_DEST_AREA_TYPE_RECTANGLE     /**< Destination area shape: rectangle */
} cms_gn_dest_area_type_t;

/**
GeoNetworking destination area.
@ingroup gn_types
*/
struct cms_gn_dest_area_t {
    cms_gn_dest_area_type_t type;       /**< Destination area type */
    cms_latitude_t latitude;            /**< Center position's latitude [0.1 udeg] */
    cms_longitude_t longitude;          /**< Center position's longitude [0.1 udeg] */
    cms_length_t dist_a;                /**< Distance A [mm] */
    cms_length_t dist_b;                /**< Distance B - only for specific area types [mm] */
    cms_heading_t angle;                /**< Angle (from North) - only for specific area types [0.001 deg] */
} CMS_PACKED;

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_GN_TYPES_H_ */
