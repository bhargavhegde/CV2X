/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2022
*/

#ifndef CMS_WDT_H_
#define CMS_WDT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <cms_v2x/common_types.h>

/**
@defgroup wdt Watchdog
@ingroup api
*/

/**
Possible values for WDT status
@ingroup wdt
*/
typedef enum cms_wdt_status_t {
    CMS_WDT_STATUS_OK = 0,          /**< all threads are running */
    CMS_WDT_STATUS_THREAD_STUCK,    /**< one or more threads are stuck */
} cms_wdt_status_t;

/**
WDT notification data
@ingroup wdt
*/
typedef struct cms_wdt_notif_data_t {
    cms_wdt_status_t status;
} CMS_PACKED cms_wdt_notif_data_t;

/**
WDT notify callback function.
@ingroup wdt
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_wdt_subscribe function call.
@param data         Notification
@param ctx          User context, registered during subscribe
@see cms_wdt_subscribe
*/
typedef void (*cms_wdt_notify_f)(const cms_wdt_notif_data_t* data,
                                 void* ctx);

/**
WDT fix subscribe.
@ingroup wdt
Upon successful subscription, the ITS software stack sends a notification to
the subscriber in every second about the state of the stack
client with some metadata.
@param session              Client session
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_wdt_notify_f
@see cms_wdt_unsubscribe
*/
bool cms_wdt_subscribe(const cms_session_t* session,
                       cms_wdt_notify_f callback_f,
                       void* ctx,
                       cms_subs_id_t* subs_id);

/**
WDT fix unsubscribe.
@ingroup wdt
Upon successful unsubscription, the ITS stack does not send a watchdog notification
to the client any more.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_wdt_subscribe
@return true in case of error
@see cms_wdt_subscribe
*/
bool cms_wdt_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);


#ifdef __cplusplus
}
#endif

#endif  // CMS_WDT_H_
