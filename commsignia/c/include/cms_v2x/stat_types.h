/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_STAT_TYPES_H_
#define CMS_V2X_STAT_TYPES_H_

/** @file
@brief Statistics types
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup sta_types Statistics types
@ingroup stat
*/

/**
Identifier of a specific module that provides measurable data for statistics
@ingroup stat_types
*/
typedef enum cms_stat_module_id_t {
    CMS_STAT_MODULE_ID_RADIO_IF1 = 0,
    CMS_STAT_MODULE_ID_RADIO_IF2,
    CMS_STAT_MODULE_ID_RADIO_IF3,
    CMS_STAT_MODULE_ID_RADIO_IF4,
    CMS_STAT_MODULE_ID_RADIO_IF5,
    CMS_STAT_MODULE_ID_RADIO_BLACKLIST,
    CMS_STAT_MODULE_ID_WSMP,
    CMS_STAT_MODULE_ID_DSMP,
    CMS_STAT_MODULE_ID_GEONET,
    CMS_STAT_MODULE_ID_SEC_SIGNER_CERT,
    CMS_STAT_MODULE_ID_SEC_1609_2_EU,
    CMS_STAT_MODULE_ID_SEC_1609_2_US,
    CMS_STAT_MODULE_ID_SEC_1609_2_CN,
    CMS_STAT_MODULE_ID_FAC_RX_CN_BSM,
    CMS_STAT_MODULE_ID_FAC_RX_US_BSM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_CAM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_CPM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_DENM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_IVI,
    CMS_STAT_MODULE_ID_FAC_RX_CN_MAP,
    CMS_STAT_MODULE_ID_FAC_RX_EU_MAP,
    CMS_STAT_MODULE_ID_FAC_RX_US_MAP,
    CMS_STAT_MODULE_ID_FAC_RX_US_PDM,
    CMS_STAT_MODULE_ID_FAC_RX_US_PSM,
    CMS_STAT_MODULE_ID_FAC_RX_US_PVD,
    CMS_STAT_MODULE_ID_FAC_RX_US_RSA,
    CMS_STAT_MODULE_ID_FAC_RX_CN_RSI,
    CMS_STAT_MODULE_ID_FAC_RX_EU_RTCM,
    CMS_STAT_MODULE_ID_FAC_RX_US_RTCM,
    CMS_STAT_MODULE_ID_FAC_RX_CN_SPAT,
    CMS_STAT_MODULE_ID_FAC_RX_EU_SPAT,
    CMS_STAT_MODULE_ID_FAC_RX_US_SPAT,
    CMS_STAT_MODULE_ID_FAC_RX_EU_SRM,
    CMS_STAT_MODULE_ID_FAC_RX_US_SRM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_SSM,
    CMS_STAT_MODULE_ID_FAC_RX_US_SSM,
    CMS_STAT_MODULE_ID_FAC_RX_US_TIM,
    CMS_STAT_MODULE_ID_FAC_RX_US_WSA,
    CMS_STAT_MODULE_ID_FAC_RX_CN_RSM,
    CMS_STAT_MODULE_ID_FAC_RX_US_SDSM,
    CMS_STAT_MODULE_ID_FAC_RX_EU_VAM,
    /**
     * If you add a new CMS_STAT_MODULE_ID_FAC_RX_ module,
     * do not forget to extend the CMS_STAT_FAC_RX_NUM_OF_COUNTERS value
     * with the appropriate number:
     *      - If the new module has CMS_STAT_NUM_FAC_RX_COUNTERS number of counters, then
     *        increment CMS_STAT_FAC_RX_NUM_OF_MODULES by 1
     *      - If the num of counters differs from CMS_STAT_NUM_FAC_RX_COUNTERS, then
     *        add the number of counters of the new module directly to
     *        CMS_STAT_FAC_RX_NUM_OF_COUNTERS value
     * */
    CMS_STAT_MODULE_ID_FAC_TX_CN_BSM,
    CMS_STAT_MODULE_ID_FAC_TX_US_BSM,
    CMS_STAT_MODULE_ID_FAC_TX_EU_CAM,
    CMS_STAT_MODULE_ID_FAC_TX_EU_CPM,
    CMS_STAT_MODULE_ID_FAC_TX_EU_DENM,
    CMS_STAT_MODULE_ID_FAC_TX_US_PVD,
    CMS_STAT_MODULE_ID_FAC_TX_US_SRM,
    CMS_STAT_MODULE_ID_FAC_TX_CN_RSM,
    CMS_STAT_MODULE_ID_FAC_TX_US_SDSM,
    CMS_STAT_MODULE_ID_FAC_TX_EU_VAM,
    /**
     * If you add a new CMS_STAT_MODULE_ID_FAC_TX_ module,
     * do not forget to extend the CMS_STAT_FAC_TX_NUM_OF_COUNTERS value
     * with the appropriate number:
     *      - If the new module has only CMS_STAT_NUM_FAC_TX_COUNTERS counter, then
     *        increment CMS_STAT_FAC_TX_NUM_OF_MODULES by 1
     *      - If the new module has custom counters then add the number of
     *        counters of the new module directly to
     *        CMS_STAT_FAC_TX_NUM_OF_COUNTERS value (eg. US_SDSM module)
     */
    CMS_STAT_MODULE_ID_EAS,
    CMS_STAT_MODULE_ID_GEOFENCING,
    CMS_STAT_NUM_MODULES,
    CMS_STAT_MODULE_ID_ALL_MODULES = INT32_MAX
} cms_stat_module_id_t;

/**
Helpers for getting module ranges.
@ingroup stat_types
*/
enum {
    CMS_STAT_MODULE_ID_RADIO_IF_MIN   = CMS_STAT_MODULE_ID_RADIO_IF1,
    CMS_STAT_MODULE_ID_RADIO_IF_MAX   = CMS_STAT_MODULE_ID_RADIO_IF5,
    CMS_STAT_NUM_RADIO_IFACES = (CMS_STAT_MODULE_ID_RADIO_IF_MAX - CMS_STAT_MODULE_ID_RADIO_IF_MIN + 1),
    CMS_STAT_MODULE_ID_SEC_1609_2_MIN = CMS_STAT_MODULE_ID_SEC_1609_2_EU,
    CMS_STAT_MODULE_ID_SEC_1609_2_MAX = CMS_STAT_MODULE_ID_SEC_1609_2_CN,
    CMS_STAT_NUM_SEC_1609_2_MODULES = ((CMS_STAT_MODULE_ID_SEC_1609_2_MAX - CMS_STAT_MODULE_ID_SEC_1609_2_MIN) + 1)
};

/**
Identifier of a counter within a module
@ingroup stat_types
*/
typedef uint32_t cms_stat_counter_id_t;

/**
Integral constant to be used for retrieving all counters from module(s)
@ingroup stat_types
*/
static const cms_stat_counter_id_t CMS_STAT_COUNTER_ID_ALL_COUNTERS = UINT32_MAX;

/**
Identifier of a counter within radio interface module
@ingroup stat_types
*/
typedef enum cms_stat_radio_counter_id_t {
    CMS_STAT_RADIO_COUNTER_ID_TX_PACKET = 0,
    CMS_STAT_RADIO_COUNTER_ID_RX_PACKET,
    CMS_STAT_RADIO_COUNTER_ID_RX_UNKNOWN_PACKET,
    CMS_STAT_RADIO_COUNTER_ID_RX_INVALID_MAC_PHY_HEADER_PACKET,
    CMS_STAT_RADIO_COUNTER_ID_RX_RSSI_LAST_PACKET,
    CMS_STAT_NUM_RADIO_COUNTERS
} cms_stat_radio_counter_id_t;

/**
Identifier of a counter within radio blacklist module
@ingroup stat_types
*/
typedef enum cms_stat_radio_blacklist_counter_id_t {
    CMS_STAT_RADIO_COUNTER_ID_RX_BLOCKED_MAC_NUM = 0,
    CMS_STAT_RADIO_COUNTER_ID_RX_DROPPED_BY_BLACKLIST,
    CMS_STAT_NUM_RADIO_BLACKLIST_COUNTERS
} cms_stat_radio_blacklist_counter_id_t;

/**
Identifier of a counter within WSMP module
@ingroup stat_types
*/
typedef enum cms_stat_wsmp_counter_id_t {
    CMS_STAT_WSMP_COUNTER_ID_TX_PACKET = 0,
    CMS_STAT_WSMP_COUNTER_ID_RX_PACKET,
    CMS_STAT_WSMP_COUNTER_ID_RX_DROPPED_INVALID_HEADER_PACKET,
    CMS_STAT_NUM_WSMP_COUNTERS
} cms_stat_wsmp_counter_id_t;

/**
Identifier of a counter within DSMP module
@ingroup stat_types
*/
typedef enum cms_stat_dsmp_counter_id_t {
    CMS_STAT_DSMP_COUNTER_ID_TX_PACKET = 0,
    CMS_STAT_DSMP_COUNTER_ID_RX_PACKET,
    CMS_STAT_DSMP_COUNTER_ID_RX_DROPPED_PACKET,
    CMS_STAT_NUM_DSMP_COUNTERS,
} cms_stat_dsmp_counter_id_t;

/**
Identifier of a counter within GeoNet module
@ingroup stat_types
*/
typedef enum cms_stat_geonet_counter_id_t {
    CMS_STAT_GEONET_COUNTER_ID_TX_SHB_PACKET = 0,
    CMS_STAT_GEONET_COUNTER_ID_TX_TSB_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_TX_GAC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_TX_GBC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_TX_GUC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_TX_LS_REQ_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_TX_LS_REP_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_SHB_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_TSB_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_GAC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_GBC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_GUC_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_LS_REQ_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_LS_REP_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_DROPPED_INVALID_BTP_HEADER_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_RX_DROPPED_INVALID_GN_HEADER_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_FORWARDED_PACKET,
    CMS_STAT_GEONET_COUNTER_ID_LOC_TBL_ENTRY_IF1,
    CMS_STAT_GEONET_COUNTER_ID_LOC_TBL_ENTRY_IF2,
    CMS_STAT_GEONET_COUNTER_ID_LOC_TBL_ENTRY_IF3,
    CMS_STAT_GEONET_COUNTER_ID_LOC_TBL_ENTRY_IF4,
    CMS_STAT_NUM_GEONET_COUNTERS
} cms_stat_geonet_counter_id_t;

/**
Identifier of a counter within signer certificate module
@ingroup stat_types
*/
typedef enum cms_stat_sec_signer_cert_counter_id_t {
    CMS_STAT_SEC_SIGNER_CERT_COUNTER_ID_EU = 0,
    CMS_STAT_SEC_SIGNER_CERT_COUNTER_ID_US,
    CMS_STAT_SEC_SIGNER_CERT_COUNTER_ID_CN,
    CMS_STAT_NUM_SEC_SIGNER_CERT_COUNTERS
} cms_stat_sec_signer_cert_counter_id_t;

/**
Identifier of a counter within 1609.2 security module
@ingroup stat_types
*/
typedef enum cms_stat_sec_1609_2_counter_id_t {
    CMS_STAT_SEC_1609_2_COUNTER_ID_TX_SIGNED_PACKET = 0,
    CMS_STAT_SEC_1609_2_COUNTER_ID_TX_SIGNING_ERROR_PACKET,
    CMS_STAT_SEC_1609_2_COUNTER_ID_RX_VERIFIED_PACKET,
    CMS_STAT_SEC_1609_2_COUNTER_ID_RX_MALFORMED_PACKET,
    CMS_STAT_SEC_1609_2_COUNTER_ID_RX_VERIFICATION_FAILED_PACKET,
    CMS_STAT_SEC_1609_2_COUNTER_ID_RX_UNSECURED_PACKET,
    CMS_STAT_NUM_SEC_1609_2_COUNTERS
} cms_stat_sec_1609_2_counter_id_t;

/**
Identifier of a counter within a facility rx module
@ingroup stat_types
*/
typedef enum cms_stat_fac_rx_counter_id_t {
    CMS_STAT_FAC_RX_COUNTER_ID_HAS_SUBSCRIBERS = 0,
    CMS_STAT_FAC_RX_COUNTER_ID_RX_PACKET,
    CMS_STAT_FAC_RX_COUNTER_ID_RX_DROPPED_PACKET,
    CMS_STAT_NUM_FAC_RX_COUNTERS
} cms_stat_fac_rx_counter_id_t;

/**
Identifier of a counter within a facility tx module
@ingroup stat_types
*/
typedef enum cms_stat_fac_tx_counter_id_t {
    CMS_STAT_FAC_TX_COUNTER_ID_TX_PACKET,
    CMS_STAT_NUM_FAC_TX_COUNTERS
} cms_stat_fac_tx_counter_id_t;

/**
Identifier of an SDSM Tx counter within facility Tx module
@ingroup stat_types
*/
typedef enum cms_stat_sdsm_tx_counter_id_t {
    CMS_STAT_SDSM_TX_COUNTER_ID_TX_PACKET = 0,
    CMS_STAT_SDSM_TX_COUNTER_ID_OBJECTS_IN_STORAGE,
    CMS_STAT_SDSM_TX_COUNTER_ID_OBJECTS_LAST_SENT,
    CMS_STAT_NUM_SDSM_TX_COUNTERS
} cms_stat_sdsm_tx_counter_id_t;

/**
Identifier of a counter within external API server module
@ingroup stat_types
*/
typedef enum cms_stat_eas_counter_id_t {
    CMS_STAT_EAS_COUNTER_ID_API_CONNECTIONS = 0,
    CMS_STAT_NUM_EAS_COUNTERS
} cms_stat_eas_counter_id_t;

/**
Identifier of a counter within geofencing module.
@ingroup stat_types
*/
typedef enum cms_stat_geofencing_counter_id_t {
    CMS_STAT_GEOFENCING_COUNTER_ID_LOADED_REGIONS = 0,
    CMS_STAT_GEOFENCING_COUNTER_ID_CURR_IN_REGIONS,
    CMS_STAT_NUM_GEOFENCING_COUNTERS
} cms_stat_geofencing_counter_id_t;

/**
Statistics counter reference.
@ingroup stat_types
Identifies a specific counter in a specific module,
or all counters in a specific module.
*/
typedef struct cms_stat_counter_ref_t {
    cms_stat_module_id_t module;        /**< Module ID */
    cms_stat_counter_id_t counter;      /**< Counter ID */
} CMS_PACKED cms_stat_counter_ref_t;

/**
Integral constant to limit the number of module counters retrieved at a time.
@ingroup stat_types
*/
#define CMS_STAT_COUNTER_REFS_SIZE  5UL

/**
Statistics counter references.
@ingroup stat_types
Identifies either a specific counter in a specific module,
all counters in a specific module or all counters in
CMS_STAT_COUNTER_REFS_SIZE number of modules.
*/
typedef struct cms_stat_counter_refs_t {
    uint32_t length;                                            /**< Effective length of `refs` */
    cms_stat_counter_ref_t refs[CMS_STAT_COUNTER_REFS_SIZE];    /**< References */
} CMS_PACKED cms_stat_counter_refs_t;

/**
Value type for all statistics counters
@ingroup stat_types
*/
typedef int64_t cms_stat_value_t;

/**
Basic statistics data item.
@ingroup stat_types
Contains the value of a specific counter in a specific module
*/
typedef struct cms_stat_item_t {
    cms_stat_module_id_t module;        /**< Module ID */
    cms_stat_counter_id_t counter;      /**< Counter ID */
    cms_stat_value_t value;             /**< Counter value */
} CMS_PACKED cms_stat_item_t;

/**
Maximum length of counters array within cms_stat_counters_t
Note: counters should fit in CBF buffer's currently defined 1500 bytes data part
@ingroup stat_types
*/
#define CMS_STAT_COUNTERS_SIZE   90UL

/**
Statistics counters to be used in services
@ingroup stat_types
*/
typedef struct cms_stat_counters_t {
    uint32_t length;                                    /**< Effective length of `counters` */
    cms_stat_item_t counters[CMS_STAT_COUNTERS_SIZE];   /**< Counters */
} CMS_PACKED cms_stat_counters_t;


/**
Total number of FAC_RX modules that CMS_STAT_NUM_FAC_RX_COUNTERS number of counters
@ingroup stat_types
*/
#define CMS_STAT_FAC_RX_NUM_OF_MODULES       (27UL)

/**
Total number of facility rx statistics counters
@ingroup stat_types
*/
#define CMS_STAT_FAC_RX_NUM_OF_COUNTERS             (CMS_STAT_FAC_RX_NUM_OF_MODULES \
                                                    * CMS_STAT_NUM_FAC_RX_COUNTERS)

/**
Total number of FAC_TX modules that has CMS_STAT_NUM_FAC_TX_COUNTERS number of counters
@ingroup stat_types
*/
#define CMS_STAT_FAC_TX_NUM_OF_MODULES       (27UL)
/**
Total number of facility tx statistics counters
@ingroup stat_types
*/
#define CMS_STAT_FAC_TX_NUM_OF_COUNTERS             ((CMS_STAT_FAC_TX_NUM_OF_MODULES \
                                                    * CMS_STAT_NUM_FAC_TX_COUNTERS)\
                                                    + CMS_STAT_NUM_SDSM_TX_COUNTERS)

/**
Total number of all statistics counters
@ingroup stat_types
*/
#define CMS_STAT_NUM_COUNTERS   ((CMS_STAT_NUM_RADIO_IFACES * CMS_STAT_NUM_RADIO_COUNTERS) \
                               + CMS_STAT_NUM_RADIO_BLACKLIST_COUNTERS \
                               + CMS_STAT_NUM_WSMP_COUNTERS \
                               + CMS_STAT_NUM_DSMP_COUNTERS \
                               + CMS_STAT_NUM_GEONET_COUNTERS \
                               + CMS_STAT_NUM_SEC_SIGNER_CERT_COUNTERS \
                               + (CMS_STAT_NUM_SEC_1609_2_COUNTERS * CMS_STAT_NUM_SEC_1609_2_MODULES) \
                               + CMS_STAT_FAC_RX_NUM_OF_COUNTERS \
                               + CMS_STAT_FAC_TX_NUM_OF_COUNTERS \
                               + CMS_STAT_NUM_EAS_COUNTERS \
                               + CMS_STAT_NUM_GEOFENCING_COUNTERS)

/**
Length of version info string buffer.
@ingroup stat_types
*/
#define CMS_VERSION_INFO_SIZE   33U

/**
Version info string
@ingroup stat_types
*/
typedef char cms_version_info_t[CMS_VERSION_INFO_SIZE];

/**
Maximum length of unique id character array within cms_hw_unique_id_t
@ingroup stat_types
*/
#define CMS_HW_UNIQUE_ID_SIZE   33U

/**
HW unique ID string
@ingroup stat_types
*/
typedef char cms_hw_unique_id_t[CMS_HW_UNIQUE_ID_SIZE];

/**
Device data
@ingroup stat_types
*/
typedef struct cms_stat_device_data_t {
    cms_version_info_t version_info;    /**< Version */
    cms_hw_unique_id_t unique_id;       /**< Unique ID */
} CMS_PACKED cms_stat_device_data_t;

/**
Miscellaneous data
@ingroup stat_types
*/
typedef struct cms_stat_misc_data_t {
    uint64_t timestamp_ms;
    uint64_t system_uptime_ms;
    uint64_t stack_uptime_ms;
} CMS_PACKED cms_stat_misc_data_t;

/**
Device status and version information
@ingroup stat_types
*/
typedef struct cms_stat_device_status_t {
    cms_stat_device_data_t device_data;
    cms_stat_misc_data_t misc_data;
} CMS_PACKED cms_stat_device_status_t;

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_STAT_TYPES_H_ */
