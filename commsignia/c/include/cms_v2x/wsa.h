/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_WSA_H_
#define CMS_V2X_WSA_H_

/** @file
@brief WAVE Service Advertisement (WSA) module API
*/

#include <stdint.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup wsa WAVE Service Advertisement (WSA)
@ingroup api
*/

/**
Priority of a registered service.
@ingroup wsa
Lower value means higher priority.
@see CMS_WSA_SERVICE_HIGHEST_PRIO
@see CMS_WSA_SERVICE_HIGHER_PRIO
@see CMS_WSA_SERVICE_DEFAULT_PRIO
@see CMS_WSA_SERVICE_LOWER_PRIO
@see CMS_WSA_SERVICE_LOWEST_PRIO
*/
typedef uint8_t cms_wsa_service_prio_t;

/**
Highest priority of a WSA service.
@ingroup wsa
@see cms_wsa_service_prio_t
@see CMS_WSA_SERVICE_HIGHER_PRIO
@see CMS_WSA_SERVICE_DEFAULT_PRIO
@see CMS_WSA_SERVICE_LOWER_PRIO
@see CMS_WSA_SERVICE_LOWEST_PRIO
*/
#define CMS_WSA_SERVICE_HIGHEST_PRIO    0U

/**
Higher priority of a WSA service.
@ingroup wsa
@see cms_wsa_service_prio_t
@see CMS_WSA_SERVICE_HIGHEST_PRIO
@see CMS_WSA_SERVICE_DEFAULT_PRIO
@see CMS_WSA_SERVICE_LOWER_PRIO
@see CMS_WSA_SERVICE_LOWEST_PRIO
*/
#define CMS_WSA_SERVICE_HIGHER_PRIO     64U

/**
Default priority of a WSA service.
@ingroup wsa
@see cms_wsa_service_prio_t
@see CMS_WSA_SERVICE_HIGHEST_PRIO
@see CMS_WSA_SERVICE_HIGHER_PRIO
@see CMS_WSA_SERVICE_LOWER_PRIO
@see CMS_WSA_SERVICE_LOWEST_PRIO
*/
#define CMS_WSA_SERVICE_DEFAULT_PRIO    128U

/**
Lower priority of a WSA service.
@ingroup wsa
@see cms_wsa_service_prio_t
@see CMS_WSA_SERVICE_HIGHEST_PRIO
@see CMS_WSA_SERVICE_HIGHER_PRIO
@see CMS_WSA_SERVICE_DEFAULT_PRIO
@see CMS_WSA_SERVICE_LOWEST_PRIO
*/
#define CMS_WSA_SERVICE_LOWER_PRIO      192U

/**
Lowest priority of a WSA service.
@ingroup wsa
@see cms_wsa_service_prio_t
@see CMS_WSA_SERVICE_HIGHEST_PRIO
@see CMS_WSA_SERVICE_HIGHER_PRIO
@see CMS_WSA_SERVICE_DEFAULT_PRIO
@see CMS_WSA_SERVICE_LOWER_PRIO
*/
#define CMS_WSA_SERVICE_LOWEST_PRIO     UINT8_MAX

/**
ID of a registered WSA client.
@ingroup wsa
@see CMS_WSA_CLIENT_ID_MAX
*/
typedef uint8_t cms_wsa_client_id_t;

/**
Maximum value of a registered WSA client ID.
@ingroup wsa
@see cms_wsa_client_id_t
*/
#define CMS_WSA_CLIENT_ID_MAX           15U

/**
Not available/invalid WSA client ID.
@ingroup wsa
@see cms_wsa_client_id_t
*/
#define CMS_WSA_CLIENT_ID_NA            UINT8_MAX

/**
WSA client service registration data.
@ingroup wsa
@see cms_wsa_register_client_service
*/
typedef struct cms_wsa_reg_client_srv_param_t {
    cms_psid_t psid;                /**< Non p-encoded PSID identifying the service */
    cms_wsa_service_prio_t prio;    /**< Priority of the service */
} CMS_PACKED cms_wsa_reg_client_srv_param_t;

/**
WSA client service registration result.
@ingroup wsa
@see cms_wsa_register_client_service
*/
typedef struct cms_wsa_reg_client_srv_out_t {
    cms_wsa_client_id_t id;         /**< Client ID, which can be used for deregistration */
} CMS_PACKED cms_wsa_reg_client_srv_out_t;

/**
Function to register a client's need and priority for a WSA service.
@ingroup wsa
Upon successful registration, a client ID is returned, which can be used
for deregistration.
Successful registration can result in a channel change of the V2X hardware,
and making already registered services available/unavailable, depending on
the registered services' priority and the currently active (received) WSAs.
In case of reprioritization of a WSA service, cms_wsa_deregister shall be called first,
then cms_wsa_register again.
@param  session     Client session
@param  params      Register client service parameters
@param[out] out     Register client service output parameters
@return true in case of error
*/
bool cms_wsa_register_client_service(const cms_session_t* session,
                                     const cms_wsa_reg_client_srv_param_t* params,
                                     cms_wsa_reg_client_srv_out_t* out);

/**
WSA client service deregistration data.
@ingroup wsa
@see cms_wsa_deregister_client_service
@see cms_wsa_reg_client_srv_out_t
*/
typedef struct cms_wsa_dereg_client_srv_param_t {
    cms_wsa_client_id_t id;     /**< Client ID retrieved from registration */
} CMS_PACKED cms_wsa_dereg_client_srv_param_t;

/**
Function to deregister a client's need for a WSA service.
@ingroup wsa
Successful deregistration can result in a channel change of the V2X hardware,
and making remaining registered services available/unavailable, depending on
the registered services' priority and the currently active (received) WSAs.
@param  session         Client session
@param  params          Deregister client service parameters
@param[out] unused_out  Currently unused output parameter
@return true in case of error
*/
bool cms_wsa_deregister_client_service(const cms_session_t* session,
                                       const cms_wsa_dereg_client_srv_param_t* params,
                                       void* unused_out);

/**
Length of Provider Service Context byte array.
@ingroup wsa
*/
#define CMS_PROVIDER_SERVICE_CONTEXT_LEN   31U

/**
Provider Service Context of a WSA service information.
@ingroup wsa
*/
typedef struct cms_wsa_srv_psc_t {
    uint32_t length;                                /**< Provider Service Context length */
    uint8_t data[CMS_PROVIDER_SERVICE_CONTEXT_LEN]; /**< Provider Service Context data */
} CMS_PACKED cms_wsa_srv_psc_t;

/**
WSA service information.
@ingroup wsa
*/
typedef struct cms_wsa_srv_info_t {
    cms_wsa_srv_psc_t psc;                  /**< Provider Service Context */
    cms_ipv6_addr_t ipv6_addr;              /**< IPv6 address */
    cms_ip_port_t ipv6_port;                /**< IPv6 port */
    cms_mac_addr_t provider_mac_addr;       /**< MAC address of the provider */
} CMS_PACKED cms_wsa_srv_info_t;

/**
WSA service status notification data.
@ingroup wsa
*/
typedef struct cms_wsa_service_status_notif_data_t {
    cms_psid_t psid;                        /**< Non p-encoded PSID identifying the service */
    bool active;                            /**< True if the service identified by PSID is currently active */
    cms_wsa_srv_info_t wsa_srv_info;        /**< WSA service info */
    cms_interface_id_t interface_id;        /**< Radio interface ID */
} CMS_PACKED cms_wsa_service_status_notif_data_t;

/**
WSA service status notify callback function type.
@ingroup wsa

When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_wsa_service_status_subscribe function call.

@param psid         Non p-encoded PSID identifying the service
@param notif        Notification data
@param msg          Raw WSA payload buffer
@param ctx          User context, registered during subscribe
@see cms_wsa_service_status_subscribe
*/
typedef void (*cms_wsa_service_status_notify_f)(cms_psid_t psid,
                                                const cms_wsa_service_status_notif_data_t* notif,
                                                cms_buffer_view_t msg,
                                                void* ctx);

/**
WSA service status notification subscribe.
@ingroup wsa

After successful subscription, when there is a change in the availability of a service
identified by its PSID, i.e. if there is a frequency change, a notification is sent
for each service that became available or unavailable.

This notification is sent for every PSID for which the status has changed regardless
of current service clients, i.e. even if there are no registered service clients
a received WSA will trigger this notification for every contained PSID.
@param session                      Client session
@param psid                         Non p-encoded PSID identifying the service
@param wsa_service_status_notify    WSA service status notification callback
@param ctx                          User context
@param[out] subs_id                 Subscription ID output
@return true in case of error
@see cms_wsa_service_status_notify_f
@see cms_wsa_service_status_unsubscribe
*/
bool cms_wsa_service_status_subscribe(const cms_session_t* session,
                                      cms_psid_t psid,
                                      cms_wsa_service_status_notify_f wsa_service_status_notify,
                                      void* ctx,
                                      cms_subs_id_t* subs_id);

/**
WSA service status notification unsubscribe.
@ingroup wsa

After successful unsubscription, when there is a change in the availability of a service
identified by its PSID, i.e. if there is a frequency change, a notification is not sent to
the subscribed client any more.

@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_wsa_service_status_subscribe
@return true in case of error
@see cms_wsa_service_status_subscribe
*/
bool cms_wsa_service_status_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_WSA_H_ */

