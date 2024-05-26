/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_DIAG_ERROR_H_
#define CMS_DIAG_ERROR_H_

#include <cms_v2x/common_types.h>

/** @file
@brief Diagnostics Error API
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup diag Diagnostics
@ingroup api
*/

/**
@defgroup diag_error DiagnosticsError
@ingroup diag
*/

/**
Diagnostics error group
@ingroup diag_error
*/
typedef uint16_t cms_diag_error_group_t;

/**
Not available (invalid) value for diagnostics error group
@ingroup diag_error
*/
#define CMS_DIAG_ERROR_GROUP_NA             0xFFFFU

/**
Diagnostics error code
@ingroup diag_error
*/
typedef uint16_t cms_diag_error_code_t;

/**
Not available (invalid) value for diagnostics error code
@ingroup diag_error
*/
#define CMS_DIAG_ERROR_CODE_NA              0xFFFFU

/**
Diagnostics error ID struct
@ingroup diag_error
*/
typedef struct cms_diag_error_id_t {
    cms_diag_error_group_t group;               /**< Error group */
    cms_diag_error_code_t code;                 /**< Error code */
} CMS_PACKED cms_diag_error_id_t;

/**
Diagnostics error groups
@ingroup diag_error
*/
typedef enum cms_diag_error_groups_t {
    CMS_DIAG_ERROR_GROUP_V2X = 0,               /**< V2X error group */
    CMS_DIAG_ERROR_GROUP_RESERVED_01,
    CMS_DIAG_ERROR_GROUP_RESERVED_02,
    CMS_DIAG_ERROR_GROUP_RESERVED_03,
    CMS_DIAG_ERROR_GROUP_RESERVED_04,
    CMS_DIAG_ERROR_GROUP_RESERVED_05,
    CMS_DIAG_ERROR_GROUP_RESERVED_06,
    CMS_DIAG_ERROR_GROUP_RESERVED_07,
    CMS_DIAG_ERROR_GROUP_PROJECT_01,            /**< Project specific error group */
    CMS_DIAG_ERROR_GROUP_PROJECT_02,
    CMS_DIAG_ERROR_GROUP_PROJECT_03,
    CMS_DIAG_ERROR_GROUP_PROJECT_04,
    CMS_DIAG_ERROR_GROUP_PROJECT_05,
    CMS_DIAG_ERROR_GROUP_PROJECT_06,
    CMS_DIAG_ERROR_GROUP_PROJECT_07,
    CMS_DIAG_ERROR_GROUP_PROJECT_08,
    CMS_DIAG_ERROR_GROUP_LENGTH
} cms_diag_error_groups_t;

/**
Possible V2X error codes
@ingroup diag_error
*/
typedef enum cms_diag_v2x_error_codes_t {
    CMS_DIAG_V2X_ERROR_CODE_INTERNAL_ERROR = 0,             /**< Internal SW error during initialization of the V2X stack */
    CMS_DIAG_V2X_ERROR_CODE_LICENSE_ERROR,                  /**< V2X SW is not licensed */
    CMS_DIAG_V2X_ERROR_CODE_CONFIG_ERROR,                   /**< Invalid or missing its.json, project.json */
    CMS_DIAG_V2X_ERROR_CODE_HW_INIT_ERROR,                  /**< HW error during init */
    CMS_DIAG_V2X_ERROR_CODE_PRS_INIT_ERROR,                 /**< Failed to init persistency */
    CMS_DIAG_V2X_ERROR_CODE_GF_INIT_ERROR,                  /**< Error during Geo-fencing init (error loading map file) */
    CMS_DIAG_V2X_ERROR_CODE_RIO_INIT_ERROR,                 /**< HW error during radio interface init */
    CMS_DIAG_V2X_ERROR_CODE_HSM_INIT_ERROR,                 /**< HW error during HSM init */
    CMS_DIAG_V2X_ERROR_CODE_CERT_INIT_ERROR,                /**< Error loading certificates */
    CMS_DIAG_V2X_ERROR_CODE_MSG_SEND_FAILED,                /**< Failed to send a message */
    CMS_DIAG_V2X_ERROR_CODE_MSG_SIGN_FAILED,                /**< Failed to sign a message before sending */
    CMS_DIAG_V2X_ERROR_CODE_RIO_ERROR,                      /**< Failed to send or receive on radio interface */
    CMS_DIAG_V2X_ERROR_CODE_INVALID_TIME,                   /**< Invalid time set */
    CMS_DIAG_V2X_ERROR_CODE_RADIO_ACCESS_ERROR,             /**< Radio access error */
    CMS_DIAG_V2X_ERROR_CODE_MISSING_NAV_INFORMATION,        /**< Missing navigation information */
    CMS_DIAG_V2X_ERROR_CODE_CAM_TX_POSITION_ERROR,          /**< Invalid position for CAM transmission */
    CMS_DIAG_V2X_ERROR_CODE_MISSING_CTL,                    /**< Certificate trust list is missing - not used */
    CMS_DIAG_V2X_ERROR_CODE_MISSING_SIGNER_CERT,            /**< Signer certificate is missing */
    CMS_DIAG_V2X_ERROR_CODE_ECDSA_ACCESS_ERROR,             /**< ECDSA accelerator access error */
    CMS_DIAG_V2X_ERROR_CODE_CFG_FILE_ACCESS_ERROR,          /**< Configuration file access error */
    CMS_DIAG_V2X_ERROR_CODE_CAM_TX_ERROR,                   /**< CAM transmission error*/
    CMS_DIAG_V2X_ERROR_CODE_DENM_TX_ERROR,                  /**< DENM transmission error*/
    CMS_DIAG_V2X_ERROR_CODE_DENM_TX_POSITION_ERROR          /**< Invalid position for DENM transmission */
} cms_diag_v2x_error_codes_t;

/**
Error statistics struct
@ingroup diag_error
*/
typedef struct cms_diag_error_stat_t {
    uint32_t recent_error_count;                            /**< Number of recent errors */
    uint32_t total_error_count;                             /**< Overall number of errors since startup */
    uint32_t first_occurrence_timestamp;                    /**< First error occurrence since startup in [ms] */
    uint32_t latest_occurrence_timestamp;                   /**< Latest error occurrence since startup in [ms] */
    bool state;                                             /**< True, if error is set */
} CMS_PACKED cms_diag_error_stat_t;

/**
Maximum number of error list items that can be get in one batch
@ingroup diag_error
*/
#define CMS_DIAG_ERROR_LIST_MAX_SIZE                    64U

/**
Error list item struct
@ingroup diag_error
*/
typedef struct cms_diag_error_list_item_t {
    cms_diag_error_id_t id;                                         /**< Error ID code */
    cms_diag_error_stat_t stat;                                     /**< Error statistics */
} CMS_PACKED cms_diag_error_list_item_t;

/**
Error id list struct
@ingroup diag_error
*/
typedef struct cms_diag_error_id_list_t {
    uint32_t length;                                                /**< Length of the list */
    cms_diag_error_id_t items[CMS_DIAG_ERROR_LIST_MAX_SIZE];        /**< List items */
} CMS_PACKED cms_diag_error_id_list_t;

/**
Error list struct
@ingroup diag_error
*/
typedef struct cms_diag_error_list_t {
    uint32_t length;                                                /**< Length of the list */
    cms_diag_error_list_item_t items[CMS_DIAG_ERROR_LIST_MAX_SIZE];      /**< List items */
} CMS_PACKED cms_diag_error_list_t;

/**
Function to get one or more errors in batch
@ingroup diag_error
@param session                  Client session
@param error_id_list            List of error IDs
@param[out] error_list          List of errors
@return true in case of error
*/
bool cms_diag_error_get(const cms_session_t* session,
                        const cms_diag_error_id_list_t* error_id_list,
                        cms_diag_error_list_t* error_list);

/**
Structure for setting a given error
@ingroup diag_error
*/
typedef struct cms_diag_error_set_t {
    cms_diag_error_id_t error_id;
} CMS_PACKED cms_diag_error_set_t;

/**
Function to set a given error
@ingroup diag_error
@param session                  Client session
@param error_set                Error to set
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_diag_error_set(const cms_session_t* session,
                        const cms_diag_error_set_t* error_set,
                        void* unused_out);

/**
Structure for setting a given error
@ingroup diag_error
*/
typedef cms_diag_error_set_t cms_diag_error_clear_t;

/**
Function to clear a given error
@ingroup diag_error
@param session                  Client session
@param error_clear              Error to clear
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_diag_error_clear(const cms_session_t* session,
                          const cms_diag_error_clear_t* error_clear,
                          void* unused_out);

/**
Function to get the last error occurred
@ingroup diag_error
@param session                  Client session
@param unused_in                Unused argument
@param[out] error_id            Error ID of the last error
@return true in case of error
*/
bool cms_diag_error_get_last(const cms_session_t* session,
                             void* unused_in,
                             cms_diag_error_id_t* error_id);

/**
Error notification struct
@ingroup diag_error
*/
typedef cms_diag_error_list_item_t cms_diag_error_notif_data_t;

/**
Error notification callback
@ingroup diag_error
*/
typedef void (*cms_diag_error_notify_f)(const cms_diag_error_notif_data_t* error_notif,
                                        void* ctx);

/**
Error notification subscribe. Error notification is sent only when an error is set or cleared.
@ingroup diag_error
@param session                  Client session
@param callback_f               Notification callback function
@param ctx                      User context
@param[out] subs_id             Subscription ID output
@return true in case of error
@see cms_diag_error_code_unsubscribe
*/
bool cms_diag_error_subscribe(const cms_session_t* session,
                              cms_diag_error_notify_f callback_f,
                              void* ctx,
                              cms_subs_id_t* subs_id);

/**
Error notification unsubscribe.
@ingroup diag_error
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_diag_error_code_subscribe
@return true in case of error
@see cms_diag_error_code_subscribe
*/
bool cms_diag_error_unsubscribe(const cms_session_t* session,
                                cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_DIAG_ERROR_H_ */
