/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_DIAG_STATUS_H_
#define CMS_DIAG_STATUS_H_

#include <cms_v2x/common_types.h>

/** @file
@brief Diagnostics Status API
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup diag_status DiagnosticsStatus
@ingroup diag
*/

/**
Diagnostics status group
@ingroup diag_status
*/
typedef uint16_t cms_diag_status_group_t;

/**
Not available (invalid) value for diagnostics error group
@ingroup diag_status
*/
#define CMS_DIAG_STATUS_GROUP_NA             0xFFFFU

/**
Diagnostics error code
@ingroup diag_status
*/
typedef uint16_t cms_diag_status_code_t;

/**
Not available (invalid) value for diagnostics error code
@ingroup diag_status
*/
#define CMS_DIAG_STATUS_CODE_NA              0xFFFFU

/**
Diagnostics status ID struct
@ingroup diag_status
*/
typedef struct cms_diag_status_id_t {
    cms_diag_status_group_t group;                      /**< Status group */
    cms_diag_status_code_t code;                        /**< Status code */
} CMS_PACKED cms_diag_status_id_t;

/**
Status value type
@ingroup diag_status
*/
typedef uint32_t cms_diag_status_value_t;

/**
Diagnostics status groups
@ingroup diag_status
*/
typedef enum cms_diag_status_groups_t {
    CMS_DIAG_STATUS_GROUP_V2X = 0,                      /**< V2X status group */
    CMS_DIAG_STATUS_GROUP_RESERVED_01,
    CMS_DIAG_STATUS_GROUP_RESERVED_02,
    CMS_DIAG_STATUS_GROUP_RESERVED_03,
    CMS_DIAG_STATUS_GROUP_PROJECT_01,                   /**< Project specific status group */
    CMS_DIAG_STATUS_GROUP_PROJECT_02,
    CMS_DIAG_STATUS_GROUP_PROJECT_03,
    CMS_DIAG_STATUS_GROUP_PROJECT_04,
    CMS_DIAG_STATUS_GROUP_LENGTH
} cms_diag_status_groups_t;

/**
Possible V2X status codes
@ingroup diag_status
@experimentalapi
*/
typedef enum cms_diag_v2x_status_codes_t {
    /**
    Radio interface 1 status.
    The value of this code can be one of `cms_diag_rio_if_status_t`
    */
    CMS_DIAG_V2X_STATUS_CODE_RIO_IF1 = 0,
    CMS_DIAG_V2X_STATUS_CODE_RIO_IF2,       /**< Radio interface 2 status */
    CMS_DIAG_V2X_STATUS_CODE_RIO_IF3,       /**< Radio interface 3 status */
    CMS_DIAG_V2X_STATUS_CODE_RIO_IF4,       /**< Radio interface 4 status */
    CMS_DIAG_V2X_STATUS_CODE_RIO_IF5,       /**< Radio interface 5 status */
} cms_diag_v2x_status_codes_t;

/**
Possible radio interface status values
@ingroup diag_status
@experimentalapi
*/
typedef enum cms_diag_rio_if_status_t {
    CMS_DIAG_RIO_IF_STATUS_NA = 0,      /**< Non-existent or disabled */
    CMS_DIAG_RIO_IF_STATUS_INACTIVE,    /**< Inactive */
    CMS_DIAG_RIO_IF_STATUS_ACTIVE,      /**< Active */
    CMS_DIAG_RIO_IF_STATUS_ERROR,       /**< Error */
} cms_diag_rio_if_status_t;

/**
Maximum number of status list items that can be get in one batch
@ingroup diag_status
*/
#define CMS_DIAG_STATUS_LIST_MAX_SIZE                   64U

/**
Status list item struct
@ingroup diag_status
*/
typedef struct cms_diag_status_list_item_t {
    cms_diag_status_id_t id;                                            /**< Status ID */
    cms_diag_status_value_t value;                                      /**< Status value */
} CMS_PACKED cms_diag_status_list_item_t;

/**
Status ID list struct
@ingroup diag_status
*/
typedef struct cms_diag_status_id_list_t {
    uint32_t length;                                                    /**< Length of the list */
    cms_diag_status_id_t items[CMS_DIAG_STATUS_LIST_MAX_SIZE];          /**< List items */
} CMS_PACKED cms_diag_status_id_list_t;

/**
Status list struct
@ingroup diag_status
*/
typedef struct cms_diag_status_list_t {
    uint32_t length;                                                    /**< Length of the list */
    cms_diag_status_list_item_t items[CMS_DIAG_STATUS_LIST_MAX_SIZE];   /**< List items */
} CMS_PACKED cms_diag_status_list_t;

/**
Function to get one or more status values in batch
@ingroup diag_status
@param session                  Client session
@param status_id_list           List of status codes to get
@param[out] status_list         List of status values
@return true in case of error
*/
bool cms_diag_status_get(const cms_session_t* session,
                         const cms_diag_status_id_list_t* status_id_list,
                         cms_diag_status_list_t* status_list);

/**
Structure for setting a given status
@ingroup diag_status
*/
typedef cms_diag_status_list_item_t cms_diag_status_set_t;

/**
Function to set state of the specific status code
@ingroup diag_status
@param session              Client session
@param status_set           Status ID and value to set
@param[out] unused_out      Unused parameter
@return true in case of error
*/
bool cms_diag_status_set(const cms_session_t* session,
                         const cms_diag_status_set_t* status_set,
                         void* unused_out);

/**
Status notification struct
@ingroup diag_status
*/
typedef cms_diag_status_list_item_t cms_diag_status_notif_data_t;

/**
Status notification callback
@ingroup diag_status
*/
typedef void (*cms_diag_status_notif_f)(const cms_diag_status_notif_data_t* status_notif,
                                        void* ctx);

/**
Status notification subscribe.
@ingroup diag_status
@param session              Client session
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_diag_status_unsubscribe
*/
bool cms_diag_status_subscribe(const cms_session_t* session,
                               cms_diag_status_notif_f callback_f,
                               void* ctx,
                               cms_subs_id_t* subs_id);

/**
Status notification unsubscribe.
@ingroup diag_status
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_diag_status_subscribe
@return true in case of error
@see cms_diag_status_subscribe
*/
bool cms_diag_status_unsubscribe(const cms_session_t* session,
                                 cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_DIAG_STATUS_H_ */
