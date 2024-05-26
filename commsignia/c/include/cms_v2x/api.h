/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2022
*/

#ifndef CMS_V2X_API_H_
#define CMS_V2X_API_H_

/** @file
@brief API Control
*/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <cms_v2x/session.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup api API
*/

/**
@defgroup api_ctrl API Control
@ingroup api
*/

/**
Create a session for using the V2X API.
@ingroup api_ctrl
@return session to use
*/
cms_session_t cms_get_session(void);

/**
Create a session for using the V2X API with a user defined ID.
@ingroup api_ctrl
@param client_id custom client id
@return session to use
*/
cms_session_t cms_get_session_for_id(uint64_t client_id);


/**
Callback type for API disconnect event.
@ingroup api_ctrl
*/
typedef void(*cms_disconnect_callback_t)(void*);

/**
Connect to V2X remote API.
@ingroup api_ctrl
Connect to a remote V2X stack using the session.
@note This has to be called before any other session uses.
@param session session to use
@param hostname null-terminated string of the host address or name to use
@param try_count limit of tries when connection fails
@param disconnect_cb callback to use on disconnect events
@param disconnect_cb_ctx parameters for the disconnect callback
@return true in case of error
*/
bool cms_api_connect(const cms_session_t* session,
                     const char* hostname,
                     uint32_t try_count,
                     cms_disconnect_callback_t disconnect_cb,
                     void* disconnect_cb_ctx);

/**
Connect to V2X remote API in a simplified way.
@ingroup api_ctrl
Connect to a remote V2X stack using the session.
This has to be called before any other session uses.
@param session session to use
@param hostname null-terminated string of the host address or name to use
@return true in case of error
*/
static inline bool cms_api_connect_easy(const cms_session_t* session,
                                        const char* hostname)
{
    static const uint32_t DEFAULT_RETRIES = 2U;
    return cms_api_connect(session, hostname, DEFAULT_RETRIES, NULL, NULL);
}


/**
Disconnect V2X remote API for a session.
@ingroup api_ctrl
@param session session to use
@return true in case of error
*/
bool cms_api_disconnect(const cms_session_t* session);

/**
Initialize V2X remote API.
@ingroup api_ctrl
@note This function is automatically called by cms_api_connect() on demand.
Manual call is necessary only at advanced use cases.
@return true in case of error
*/
bool cms_api_init(void);

/**
Cleanup V2X remote API.
@ingroup api_ctrl
*/
void cms_api_clean(void);


/**
Log level for remote SDK logging.
@ingroup api_ctrl
*/
typedef enum {
    CMS_API_LOG_OFF,    /**< No logging */
    CMS_API_LOG_ERROR,  /**< Error log enabled */
    CMS_API_LOG_INFO,   /**< Info and error log enabled */
} cms_api_log_level_t;


/**
Setup log level for the remote SDK.
@ingroup api_ctrl
@note Default log level is CMS_API_LOG_ERROR.
@param level level to set
*/
void cms_api_set_logging(cms_api_log_level_t level);


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_API_H_ */
