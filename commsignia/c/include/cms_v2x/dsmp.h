/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2022
*/

#ifndef CMS_V2X_DSMP_H_
#define CMS_V2X_DSMP_H_

/** @file
@brief Dedicated Short Message Protocol (DSMP)
*/

#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/dsmp_hdr.h>
#include <cms_v2x/security_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup dsmp Dedicated Short Message Protocol (DSMP)
@ingroup api
*/

/**
Notification data for received DSMP messages
@ingroup dsmp
@see cms_dsmp_rx_notify_f
*/
struct cms_dsmp_rx_notif_data_t {
    cms_radio_rx_params_t radio;        /**< Radio parameters of the received packet */
    cms_dsmp_hdr_info_t dsmp_hdr;       /**< DSMP header info parameters of the received packet */
    cms_sec_dot2_rx_info_t security;    /**< Security info parameters of a received packet */
} CMS_PACKED;

/**
DSMP send data
@ingroup dsmp
*/
struct cms_dsmp_send_data_t {
    cms_radio_tx_params_t radio;        /**< Radio parameters of the transmitted packet */
    cms_dsmp_hdr_info_t dsmp_hdr;       /**< DSMP header info parameters of a transmitted packet */
    cms_sec_dot2_tx_info_t security;    /**< Security tx parameters of the packet */
} CMS_PACKED;

typedef cms_dsmp_send_data_t cms_dsmp_tx_notif_data_t;

/**
Received DSMP notify callback function type
@ingroup dsmp
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_dsmp_rx_subscibe function call.
@param aid          AID of the received message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_dsmp_rx_subscribe
*/
typedef void (*cms_dsmp_rx_notify_f)(cms_psid_t aid,
                                     const cms_dsmp_rx_notif_data_t* notif,
                                     cms_buffer_view_t msg,
                                     void* ctx);

/**
Transmitted DSMP notify callback function type
@ingroup dsmps
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_dsmp_tx_subscribe function call.
@param aid          AID of the received message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_dsmp_tx_subscribe
*/
typedef void (*cms_dsmp_tx_notify_f)(cms_psid_t aid,
                                     const cms_dsmp_tx_notif_data_t* notif,
                                     cms_buffer_view_t msg,
                                     void* ctx);

/**
Wildcard value for the DSMP subscribe function's AID parameter.
@ingroup dsmp
When this value is used as a key for the DSMP subscribe function,
all received DSMP packets will trigger a notification.
@see cms_dsmp_rx_subscribe
*/
#define CMS_DSMP_SUBSCRIBE_WILDCARD     0UL

/**
Subscribe for received DSMP messages.
@ingroup dsmp
Upon successful subscription, when a DSMP message is received
with a matching AID, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param aid                  AID to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_dsmp_rx_notify_f
@see CMS_DSMP_SUBSCRIBE_WILDCARD
@see cms_dsmp_rx_unsubscribe
*/
bool cms_dsmp_rx_subscribe(const cms_session_t* session,
                           cms_psid_t aid,
                           cms_dsmp_rx_notify_f callback_f,
                           void* ctx,
                           cms_subs_id_t* subs_id);

/**
Unsubscribe from received DSMP messages.
@ingroup dsmp
Upon successful unsubscription, when a DSMP message is received
with a matching AID, the ITS software stack does not send a notification
to the subscribed client any more. Messages with different subscribed AID
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_dsmp_rx_subscribe
@return true in case of error
@see cms_dsmp_rx_subscribe
*/
bool cms_dsmp_rx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

/**
Subscribe for transmitted DSMP messages.
@ingroup dsmp
Upon successful subscription, when a DSMP message is transmitted
with a matching AID, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param aid                  AID to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_dsmp_tx_notify_f
@see CMS_DSMP_SUBSCRIBE_WILDCARD
@see cms_dsmp_tx_unsubscribe
*/
bool cms_dsmp_tx_subscribe(const cms_session_t* session,
                           cms_psid_t aid,
                           cms_dsmp_tx_notify_f callback_f,
                           void* ctx,
                           cms_subs_id_t* subs_id);

/**
Unsubscribe from transmitted DSMP messages.
@ingroup dsmp
Upon successful unsubscription, when a DSMP message is transmitted
with a matching AID, the ITS software stack does not send a notification
to the subscribed client any more. Messages with different subscribed AID
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_dsmp_tx_subscribe
@return true in case of error
@see cms_dsmp_tx_subscribe
*/
bool cms_dsmp_tx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);


/**
DSMP send.
@ingroup dsmp
Send a DSMP packet with the supplied parameters and information.
@param session              Client session
@param params               Parameters of the DSMP packet
@param msg                  Information to be sent in the DSMP packet
@return true in case of error
*/
bool cms_dsmp_send(const cms_session_t* session,
                   const cms_dsmp_send_data_t* params,
                   cms_buffer_view_t msg,
                   void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_DSMP_H_ */
