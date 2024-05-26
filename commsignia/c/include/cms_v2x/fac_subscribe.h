/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_FAC_SUBSCRIBE_H_
#define CMS_V2X_FAC_SUBSCRIBE_H_

/** @file
@brief Facility subscription/notification
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/security_types.h>
#include <cms_v2x/fac_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup fac_subscribe Facility subscription/notification
@ingroup api
*/

/**
Security info parameters of a received packet for facility notification
@ingroup fac_subscribe
*/
struct cms_fac_rx_sec_info_t {
    cms_sec_verify_result_t verify_result;  /**< Security verification result of the received packet */
    /**
     * PSID/AID of the received packet,
     * Only valid if the received packet is signed.
     */
    cms_psid_t psid;
    /**
     * Service specific permissions for the given PSID.
     */
    cms_ssp_t ssp;
    /**
     * Hashedid8 of the signer certificate of the received packet,
     * Only valid if the received packet is signed.
     */
    cms_hashid8_t signer_id;
} CMS_PACKED;

/**
Convert IEEE 1609.2 Specific notification into facility security notification
@ingroup fac_subscribe
@experimentalapi
@param dot2         Input argument - 1609.2 specific security notification
@param fac          Output argument - facility receive notification, security info
@see cms_sec_dot2_rx_info_t
*/
static inline void cms_conv_sec_dot2_rx_to_fac_info(const cms_sec_dot2_rx_info_t* dot2,
                                                    cms_fac_rx_sec_info_t* fac)
{
    if((NULL != dot2) && (NULL != fac)) {
        fac->verify_result = dot2->verify_result;
        fac->psid = dot2->psid;
        fac->ssp = dot2->ssp;
        fac->signer_id = dot2->signer_id;
    }
}

/**
Facility notification data.
@ingroup fac_subscribe
*/
struct cms_fac_notif_data_t {
    cms_fac_msg_type_t type;                /**< Facility message type */
    cms_radio_rx_params_t radio;            /**< Radio parameters of the received packet */
    cms_fac_rx_sec_info_t security;         /**< Security info parameters of a received packet */
} CMS_PACKED;

/**
Facility notify callback function type.
@ingroup fac_subscribe
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_fac_subscribe function call.
@param type         Type of the received facility message
@param notif        Notification metadata
@param msg          Raw payload buffer
@param ctx          User context, registered during subscribe
@see cms_fac_subscribe
*/
typedef void (*cms_fac_notify_f)(cms_fac_msg_type_t type,
                                 const cms_fac_notif_data_t* notif,
                                 cms_buffer_view_t msg,
                                 void* ctx);

/**
Subscribed message type for facility subscription.
@ingroup fac_subscribe
It holds either
- a value of cms_fac_msg_type_t, or
- the value CMS_FAC_SUBSCRIBE_ALL

@see cms_fac_subscribe
*/
typedef uint32_t cms_fac_subscribed_msg_t;

/**
Wildcard value for the facility subscribe function's type parameter.
@ingroup fac_subscribe
When this value is used as a key for the facility subscribe function,
all received facility messages will trigger a notification.
@see cms_fac_subscribe
*/
static const cms_fac_subscribed_msg_t CMS_FAC_SUBSCRIBE_ALL = 0UL;

/**
Facility subscribe.
@ingroup fac_subscribe
Upon successful subscription, when a facility message is received
with a matching type, the ITS software stack sends a notification
to the subscribed client with some metadata and the raw message.
@param session              Client session
@param type                 Facility type to subscribe for
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_fac_notify_f
@see cms_fac_subscribed_msg_t
@see CMS_FAC_SUBSCRIBE_ALL
@see cms_fac_unsubscribe
*/
bool cms_fac_subscribe(const cms_session_t* session,
                       cms_fac_subscribed_msg_t type,
                       cms_fac_notify_f callback_f,
                       void* ctx,
                       cms_subs_id_t* subs_id);

/**
Facility unsubscribe.
@ingroup fac_subscribe
Upon successful unsubscription, when a facility message is received
with a matching type, the ITS software stack does not send a notification
to the subscribed client any more. Messages with different subscribed type
will still be sent as a notification.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_fac_subscribe
@return true in case of error
@see cms_fac_subscribe
*/
bool cms_fac_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_FAC_SUBSCRIBE_H_ */
