/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_GN_H_
#define CMS_V2X_GN_H_

/** @file
@brief GeoNetworking Protocol (GNP)
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/security_types.h>
#include <cms_v2x/gn_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup gn GeoNetworking Protocol (GNP)
@ingroup api
@note In the GeoNetworking scope, when BTP (Basic Transport Protocol)
is mentioned, it is considered as BTP type B.
@note BTP type A is not supported.
*/

/**
GeoNetworking module state
@ingroup gn
@see cms_gn_enable
@see cms_gn_get_enabled
*/
typedef struct cms_gn_state_t {
    bool enable;                    /**< Module enable */
} CMS_PACKED cms_gn_state_t;

/**
Enable GeoNetworking module
@ingroup gn
@param session              Client session
@param state_in             Module state
@param[out] unused_out      Not used parameter
@return true in case of error
@see cms_gn_state_t
*/
bool cms_gn_enable(const cms_session_t* session,
                   const cms_gn_state_t* state_in,
                   void* unused_out);

/**
Get enabled state of the GeoNetworking module
@ingroup gn
@param session              Client session
@param unused_in            Not used parameter
@param[out] state_out       Module state output
@return true in case of error
@see cms_gn_state_t
*/
bool cms_gn_get_enabled(const cms_session_t* session,
                        void* unused_in,
                        cms_gn_state_t* state_out);

/**
BTP parameters of a GeoNetworking packet.
@ingroup gn
*/
struct cms_btp_params_t {
    cms_btp_port_t btp_port;                 /**< BTP port of the received/sent message */
    cms_btp_dest_port_info_t dest_port_info; /**< BTP destination port info */
} CMS_PACKED;

/**
GeoNetworking header info.
@ingroup gn
*/
struct cms_gn_hdr_info_t {
    cms_gn_traffic_class_t traffic_class; /**< Traffic class */
    cms_gn_lifetime_ms_t lifetime_ms;     /**< Lifetime [ms] */
    cms_gn_send_method_t method;          /**< GeoNetworking method */
    cms_gn_hop_limit_t max_hop_limit;     /**< Maximum hop limit - only for specific methods */
    cms_gn_dest_area_t dest_area;         /**< Destination area - only for specific methods */
} CMS_PACKED;

/**
GeoNetworking specific parameters of a received packet.
@ingroup gn
*/
struct cms_gn_rx_params_t {
    cms_btp_params_t btp_params; /**< BTP parameters */
    cms_gn_full_pos_t full_pos;  /**< Full position data */
    cms_gn_hdr_info_t gn_hdr;    /**< GeoNetworking packet header info */
} CMS_PACKED;

/**
GeoNetworking specific send parameters of a packet.
@ingroup gn
*/
struct cms_gn_tx_params_t {
    cms_btp_params_t btp_params; /**< BTP parameters */
    cms_gn_hdr_info_t gn_hdr;    /**< GeoNetworking packet header info */
} CMS_PACKED;

/**
GeoNetworking rx notification data.
@ingroup gn
*/
struct cms_gn_rx_notif_data_t {
    cms_radio_rx_params_t radio;     /**< Radio parameters of the received packet */
    cms_gn_rx_params_t gn_params;    /**< GeoNetworking specific parameters of a received packet */
    cms_sec_dot2_rx_info_t security; /**< Security info parameters of a received packet */
} CMS_PACKED;

/**
GeoNetworking send data.
@ingroup gn
*/
struct cms_gn_send_data_t {
    cms_radio_tx_params_t radio;     /**< Radio tx parameters of the packet */
    cms_gn_tx_params_t gn_params;    /**< GeoNetworking specific parameters */
    cms_sec_dot2_tx_info_t security; /**< Security tx parameters of the packet */
} CMS_PACKED;

/**
GeoNetworking tx notification data.
@ingroup gn
*/
typedef cms_gn_send_data_t cms_gn_tx_notif_data_t;

/**
GeoNetworking send.
@ingroup gn
Send a GeoNetworking packet with the supplied parameters and information
@param session              Client session
@param params               Send parameters of the GeoNetworking packet
@param msg                  Message to be sent
@return true in case of error
*/
bool cms_gn_send(const cms_session_t* session,
                 const cms_gn_send_data_t* params,
                 cms_buffer_view_t msg,
                 void* unused_out);

/**
GeoNetworking notify callback function type.
@ingroup gn
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_gn_rx_subscribe function call.
@param btp_port     BTP port of the received message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_gn_rx_subscribe
*/
typedef void (*cms_gn_rx_notify_f)(cms_btp_port_t btp_port,
                                   const cms_gn_rx_notif_data_t* notif,
                                   cms_buffer_view_t msg,
                                   void* ctx);

/**
GeoNetworking tx notify callback function type.
@ingroup gn
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_gn_tx_subscribe function call.
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_gn_tx_subscribe
*/
typedef void (*cms_gn_tx_notify_f)(cms_btp_port_t btp_port,
                                   const cms_gn_tx_notif_data_t* notif,
                                   cms_buffer_view_t msg,
                                   void* ctx);

/**
Wildcard value for the GeoNetworking subscribe function's BTP port parameter.
@ingroup gn
When this value is used as a key for the GeoNetworking subscribe function,
all received GeoNetworking packets will trigger a notification.
@see cms_gn_rx_subscribe
@see cms_gn_tx_subscribe
*/
#define CMS_GN_SUBSCRIBE_WILDCARD 0UL

/**
GeoNetworking receive subscribe.
@ingroup gn
Upon successful subscription, when a GeoNetworking message is received
with a matching BTP port, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param btp_port             BTP port to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_gn_rx_notify_f
@see CMS_GN_SUBSCRIBE_WILDCARD
@see cms_gn_rx_unsubscribe
*/
bool cms_gn_rx_subscribe(const cms_session_t* session,
                         cms_btp_port_t btp_port,
                         cms_gn_rx_notify_f callback_f,
                         void* ctx,
                         cms_subs_id_t* subs_id);

/**
GeoNetworking receive unsubscribe.
@ingroup gn
Upon successful unsubscription, when a GeoNetworking message is received
with a matching BTP port, the ITS software stack does send a notification
to the subscribed client any more. Messages with different subscribed BTP port
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_gn_rx_subscribe
@return true in case of error
@see cms_gn_rx_subscribe
*/
bool cms_gn_rx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

/**
GeoNetworking transmit subscribe.
@ingroup gn
Upon successful subscription, when a GeoNetworking message is sent
with a matching BTP port, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param btp_port             BTP port to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_gn_tx_notify_f
@see CMS_GN_SUBSCRIBE_WILDCARD
@see cms_gn_tx_unsubscribe
*/
bool cms_gn_tx_subscribe(const cms_session_t* session,
                         cms_btp_port_t btp_port,
                         cms_gn_tx_notify_f callback_f,
                         void* ctx,
                         cms_subs_id_t* subs_id);

/**
GeoNetworking transmit unsubscribe.
@ingroup gn
Upon successful unsubscription, when a GeoNetworking message is sent. The ITS
software stack does send a notification to the subscribed client any more.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_gn_rx_subscribe
@return true in case of error
@see cms_gn_tx_subscribe
*/
bool cms_gn_tx_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_GN_H_ */
