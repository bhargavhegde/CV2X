/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_CBP_H_
#define CMS_CBP_H_

#include <stdbool.h>
#include <stddef.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

/** @file
@brief Context-Based Prioritization (CBP)
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup cbp Context-Based Prioritization (CBP)
@ingroup api
*/

/**
Not available/invalid packets per second (PPS) value.
@ingroup cbp
@see cms_pps_t
*/
#define CMS_PPS_NA                      UINT16_MAX

/**
Set target packets per second (PPS) input parameters.
@ingroup cbp
*/
typedef struct cms_cbp_set_target_pps_t {
    cms_pps_t lower_pps_threshold;
    cms_pps_t upper_pps_threshold;
} CMS_PACKED cms_cbp_set_target_pps_t;

/**
Set the target packets per second (PPS) of the
Context-Based Prioritization (CBP) module.
@ingroup cbp
@experimentalapi
@param session          Client session
@param target_pps       Target PPS value
@param[out] unused_out  Unused output parameter
@return true in case of error
*/
bool cms_cbp_set_target_pps(const cms_session_t* session,
                            const cms_cbp_set_target_pps_t* target_pps,
                            void* unused_out);

/**
Maximum number of source addresses in the list.
@ingroup cbp
*/
#define CMS_CBP_ADDR_LIST_MAX_SIZE  128U

/**
Context-Based Prioritization (CBP) module priority levels.
@ingroup cbp
*/
typedef enum cms_cbp_prio_t {
    CMS_CBP_PRIO_0 = 0,        /**< Highest priority level. */
    CMS_CBP_PRIO_1 = 1,
    CMS_CBP_PRIO_2 = 2,
    CMS_CBP_PRIO_3 = 3,
    CMS_CBP_PRIO_4 = 4,
    CMS_CBP_PRIO_5 = 5,
    CMS_CBP_PRIO_6 = 6,
    CMS_CBP_PRIO_7 = 7,
    CMS_CBP_PRIO_8 = 8,
    CMS_CBP_PRIO_9 = 9,
    CMS_CBP_PRIO_10 = 10,
    CMS_CBP_PRIO_11 = 11,
    CMS_CBP_PRIO_12 = 12,
    CMS_CBP_PRIO_13 = 13,
    CMS_CBP_PRIO_14 = 14,
    CMS_CBP_PRIO_15 = 15,
    CMS_CBP_PRIO_16 = 16,
    CMS_CBP_PRIO_17 = 17,
    CMS_CBP_PRIO_18 = 18,
    CMS_CBP_PRIO_19 = 19,
    CMS_CBP_PRIO_20 = 20,
    CMS_CBP_PRIO_21 = 21,
    CMS_CBP_PRIO_22 = 22,
    CMS_CBP_PRIO_23 = 23,
    CMS_CBP_PRIO_24 = 24,
    CMS_CBP_PRIO_25 = 25,
    CMS_CBP_PRIO_26 = 26,
    CMS_CBP_PRIO_27 = 27,
    CMS_CBP_PRIO_28 = 28,
    CMS_CBP_PRIO_29 = 29,
    CMS_CBP_PRIO_30 = 30,
    CMS_CBP_PRIO_31 = 31,       /**< Lowest priority level. */
    CMS_CBP_PRIO_NA = 32,       /**< Priority not available (for querying address priority) */
} cms_cbp_prio_t;

/**
Number of priority levels in the Context-Based Prioritization module
@ingroup cbp
*/
#define CMS_CBP_PRIO_NUM_OF_LEVELS          33U

/**
Data structure to describe the priority of a source address.
@ingroup cbp
*/
typedef struct cms_cbp_addr_prio_item_t {
    cms_mac_addr_t src_addr;        /**< Source address */
    cms_cbp_prio_t prio;            /**< Priority level */
} CMS_PACKED cms_cbp_addr_prio_item_t;

/**
List of Context Based Prioritization (CBP) module source address priority pairs.
@ingroup cbp
*/
typedef struct cms_cbp_addr_prio_list_t {
    uint32_t length;        /**< Number of items in the list */
    cms_cbp_addr_prio_item_t items[CMS_CBP_ADDR_LIST_MAX_SIZE];    /**< Source address priority list */
} CMS_PACKED cms_cbp_addr_prio_list_t;

/**
Set the priority of the given source addresses in the
Context Based Prioritization (CBP) module.
@ingroup cbp
@experimentalapi
@param session              Client session
@param items                Source address priority pair list, includes the values to set
@param[out] unused_out      Unused output parameter
@return true in case of error
*/
bool cms_cbp_set_addr_prio(const cms_session_t* session,
                           const cms_cbp_addr_prio_list_t* items,
                           void* unused_out);

/**
Structure for the priority level status.
@ingroup cbp
*/
typedef struct cms_cbp_prio_level_status_t {
    uint32_t cnt;           /**< Number of source addresses on the priority level */
    cms_pps_t pps;          /**< Packets per seconds for the priority level. */
} CMS_PACKED cms_cbp_prio_level_status_t;

/**
Context-Based Prioritization (CBP) module status.
@ingroup cbp
*/
typedef struct cms_cbp_status_t {
    cms_pps_t lower_pps_threshold;  /**< Currently set lower PPS threshold */
    cms_pps_t upper_pps_threshold;  /**< Currently set upper PPS threshold */
    cms_pps_t measured_pps;         /**< Currently measured PPS */
    uint8_t filtering_stage;        /**< Current filtering stage */
    cms_cbp_prio_level_status_t levels[CMS_CBP_PRIO_NUM_OF_LEVELS];     /**< Priority levels status */
    uint32_t addr_cnt;              /**< Number of handled addresses */
} CMS_PACKED cms_cbp_status_t;

/**
Get the status of the Context Based Prioritization (CBP) module.
@ingroup cbp
@experimentalapi
@param session              Client session
@param[in] unused_in        Unused input parameter
@param[out] status          CBP status
@return true in case of error
*/
bool cms_cbp_get_status(const cms_session_t* session,
                        void* unused_in,
                        cms_cbp_status_t* status);

/**
Source address list structure.
@ingroup cbp
*/
typedef struct cms_cbp_address_list_t {
    uint16_t length;    /**< Length of the source address list */
    cms_mac_addr_t addresses[CMS_CBP_ADDR_LIST_MAX_SIZE];   /**< Source addresses */
} CMS_PACKED cms_cbp_address_list_t;

/**
Get the priority of the source addresses.
@ingroup cbp
@experimentalapi
@param session           Client session
@param addresses         Source addresses
@param[out] priorities   Source address priority output list
@return true in case of error
*/
bool cms_cbp_get_addr_prio(const cms_session_t* session,
                           const cms_cbp_address_list_t* addresses,
                           cms_cbp_addr_prio_list_t* priorities);

/**
Flush the CBP (Context Based Prioritization) source address database.
@ingroup cbp
@experimentalapi
This does not affect the set target PPS (packets per second) value,
only the source addresses are removed.
@param session            Client session
@param[in] unused_in      Unused input parameter
@param[out] unused_out    Unused output parameter
@return true in case of error
*/
bool cms_cbp_flush_addr(const cms_session_t* session,
                        void* unused_in,
                        void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_CBP_H_ */

