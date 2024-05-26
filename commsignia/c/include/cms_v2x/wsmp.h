/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_WSMP_H_
#define CMS_V2X_WSMP_H_

/** @file
@brief WAVE Short Message Protocol (WSMP)
*/

#include <stdbool.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/security_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup wsmp WAVE Short Message Protocol (WSMP)
@ingroup api
*/

/**
WSMP header info parameters of a received packet.
@ingroup wsmp
*/
struct cms_wsmp_rx_hdr_info_t {
    cms_psid_t psid;                    /**< Non p-encoded PSID */
    cms_power_dbm_t tx_power;           /**< TxPower [dBm] if sent in a WSMP header extension */
    cms_datarate_kbps_t datarate;       /**< Datarate [kbps] if sent in a WSMP header extension */
    cms_channel_id_t channel_id;        /**< Channel ID if sent in a WSMP header extension */
} CMS_PACKED;

/**
WSMP header info parameters of a transmitted packet.
@ingroup wsmp
*/
struct cms_wsmp_tx_hdr_info_t {
    cms_psid_t psid;                    /**< Non p-encoded PSID */
    bool tx_power;                      /**< TxPower WSMP header extension to be sent */
    bool datarate;                      /**< DataRate WSMP header extension to be sent */
    bool channel_id;                    /**< Channel ID WSMP header extension to be sent */
} CMS_PACKED;

/**
Notification data for received WSMP messages.
@ingroup wsmp
*/
struct cms_wsmp_rx_notif_data_t {
    cms_radio_rx_params_t radio;        /**< Radio parameters of the received packet */
    cms_wsmp_rx_hdr_info_t wsmp_hdr;    /**< WSMP header info parameters of a received packet */
    cms_sec_dot2_rx_info_t security;    /**< Security info parameters of a received packet */
} CMS_PACKED;

/**
WSMP send data.
@ingroup wsmp
*/
struct cms_wsmp_send_data_t {
    cms_radio_tx_params_t radio;        /**< Radio parameters of the transmitted packet */
    cms_wsmp_tx_hdr_info_t wsmp_hdr;    /**< WSMP header info parameters of a transmitted packet */
    cms_sec_dot2_tx_info_t security;    /**< Security tx parameters of the packet */
} CMS_PACKED;

/**
Notification data for transmitted WSMP messages.
@ingroup wsmp
*/
typedef cms_wsmp_send_data_t cms_wsmp_tx_notif_data_t;

/**
WSMP send.
@ingroup wsmp
Send a WSMP packet with the supplied parameters and information.
@param session              Client session
@param params               Parameters of the WSMP packet
@param msg                  Information to be sent in the WSMP packet
@return true in case of error
*/
bool cms_wsmp_send(const cms_session_t* session,
                   const cms_wsmp_send_data_t* params,
                   cms_buffer_view_t msg,
                   void* unused_out);

/**
Received WSMP notify callback function type.
@ingroup wsmp
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_wsmp_rx_subscribe function call.
@param psid         PSID of the received message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_wsmp_rx_subscribe
*/
typedef void (*cms_wsmp_rx_notify_f)(cms_psid_t psid,
                                     const cms_wsmp_rx_notif_data_t* notif,
                                     cms_buffer_view_t msg,
                                     void* ctx);

/**
Sent WSMP notify callback function type.
@ingroup wsmp
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_wsmp_tx_subscribe function call.
@param psid         PSID of the received message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_wsmp_tx_subscribe
*/
typedef void (*cms_wsmp_tx_notify_f)(cms_psid_t psid,
                                     const cms_wsmp_tx_notif_data_t* notif,
                                     cms_buffer_view_t msg,
                                     void* ctx);

/**
Wildcard value for the WSMP subscribe function's psid parameter.
@ingroup wsmp
When this value is used as a key for the WSMP subscribe function,
all received WSMP packets will trigger a notification.
@see cms_wsmp_rx_subscribe
@see cms_wsmp_tx_subscribe
*/
#define CMS_WSMP_SUBSCRIBE_WILDCARD     0UL

/**
Subscribe for received WSMP messages.
@ingroup wsmp
Upon successful subscription, when a WSMP message is received
with a matching PSID, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param psid                 PSID to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_wsmp_rx_notify_f
@see CMS_WSMP_SUBSCRIBE_WILDCARD
@see cms_wsmp_rx_unsubscribe
*/
bool cms_wsmp_rx_subscribe(const cms_session_t* session,
                           cms_psid_t psid,
                           cms_wsmp_rx_notify_f callback_f,
                           void* ctx,
                           cms_subs_id_t* subs_id);

/**
Unsubscribe from received WSMP messages.
@ingroup wsmp
Upon successful unsubscription, when a WSMP message is received
with a matching PSID, the ITS software stack does not send a notification
to the subscribed client any more. Messages with different subscribed PSID
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_wsmp_rx_subscribe
@return true in case of error
@see cms_wsmp_rx_subscribe
*/
bool cms_wsmp_rx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

/**
Subscribe for sent WSMP messages.
@ingroup wsmp
Upon successful subscription, when a WSMP message is sent
with a matching PSID, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param psid                 PSID to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_wsmp_tx_notify_f
@see CMS_WSMP_SUBSCRIBE_WILDCARD
@see cms_wsmp_tx_unsubscribe
*/
bool cms_wsmp_tx_subscribe(const cms_session_t* session,
                           cms_psid_t psid,
                           cms_wsmp_tx_notify_f callback_f,
                           void* ctx,
                           cms_subs_id_t* subs_id);

/**
Unsubscribe from sent WSMP messages.
@ingroup wsmp
Upon successful unsubscription, when a WSMP message is sent
with a matching PSID, the ITS software stack does not send a notification
to the subscribed client any more. Messages with different subscribed PSID
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_wsmp_tx_subscribe
@return true in case of error
@see cms_wsmp_tx_subscribe
*/
bool cms_wsmp_tx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_WSMP_H_ */
