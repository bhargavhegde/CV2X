/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_COMMON_TYPES_H_
#define CMS_V2X_COMMON_TYPES_H_

/** @file
@brief Common data types
*/

#include <stdint.h>
#include <stdlib.h>

#include <cms_v2x/fwd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup common Common data types
@ingroup api
*/

/**
Radio interface ID.
@ingroup common
*/
typedef uint32_t cms_interface_id_t;

/**
Radio interface N/A value.
@ingroup common
@see cms_interface_id_t
*/
#define CMS_INTERFACE_ID_NA               UINT32_MAX

/**
Length of a MAC address.
@ingroup common
@see cms_mac_addr_t
*/
#define CMS_MAC_ADDRESS_LENGTH          6UL

/**
MAC address.
@ingroup common
@see CMS_MAC_ADDRESS_LENGTH
*/
typedef uint8_t cms_mac_addr_t[CMS_MAC_ADDRESS_LENGTH];

#define CMS_MAC_ADDR_FMT "%02X:%02X:%02X:%02X:%02X:%02X"
#define CMS_MAC_ADDR_FMT_ARGS(mac) mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]

/**
Optional MAC address.
@ingroup common
@see cms_mac_addr_t
*/
typedef struct {
    bool present;               /**< Whether the MAC address is present or not */
    cms_mac_addr_t mac_address; /**< MAC address */
} cms_mac_addr_optional_t;

/**
Length of an IPv4 address.
@ingroup common
@see cms_ipv4_addr_t
*/
#define CMS_IPV4_ADDRESS_LENGTH         4UL

/**
IPv4 address.
@ingroup common
@see CMS_IPV4_ADDRESS_LENGTH
*/
typedef uint8_t cms_ipv4_addr_t[CMS_IPV4_ADDRESS_LENGTH];

/**
Length of an IPv6 address.
@ingroup common
@see cms_ipv6_addr_t
*/
#define CMS_IPV6_ADDRESS_LENGTH         16UL

/**
IPv6 address.
@ingroup common
@see CMS_IPV6_ADDRESS_LENGTH
*/
typedef uint8_t cms_ipv6_addr_t[CMS_IPV6_ADDRESS_LENGTH];

/**
Optional IPv6 address.
@ingroup common
@see cms_ipv6_addr_t
*/
typedef struct {
    bool present;                   /**< Whether the IPv6 address is present or not */
    cms_ipv6_addr_t ipv6_address;   /**< IPv6 address */
} cms_ipv6_addr_optional_t;

/**
IPv4/IPv6 port.
@ingroup common
@see cms_ipv4_addr_t
@see cms_ipv6_addr_t
*/
typedef uint16_t cms_ip_port_t;

/**
Datarate n/a value.
@ingroup common
@see cms_datarate_kbps_t
*/
#define CMS_DATARATE_NA                 UINT16_MAX

/**
Datarate in [kbps].
@ingroup common
@see CMS_DATARATE_NA
*/
typedef uint16_t cms_datarate_kbps_t;

/**
MAC user priority n/a value.
@ingroup common
@see cms_mac_user_prio_t
*/
#define CMS_MAC_USER_PRIO_NA            UINT8_MAX

/**
MAC user priority.
@ingroup common
@see CMS_MAC_USER_PRIO_NA
*/
typedef uint8_t cms_mac_user_prio_t;

/**
Power level n/a value.
@ingroup common
@see cms_power_dbm_t
*/
#define CMS_POWER_LEVEL_NA              INT16_MAX

/**
Power level in [dBm].
@ingroup common
@see CMS_POWER_LEVEL_NA
*/
typedef int16_t cms_power_dbm_t;

/**
Non p-encoded PSID n/a value.
@ingroup common
@see cms_psid_t
*/
#define CMS_PSID_NA                     UINT32_MAX

/**
Non p-encoded PSID value.
@ingroup common
*/
typedef uint32_t cms_psid_t;

/**
Channel ID n/a value.
@ingroup common
@see cms_channel_id_t
*/
#define CMS_CHANNEL_ID_NA               UINT8_MAX

/**
Radio channel ID.
@ingroup common
@see CMS_CHANNEL_ID_NA
*/
typedef uint8_t cms_channel_id_t;

/**
SPS Channel Index n/a value.
@ingroup common
@see cms_sps_channel_index_t
*/
#define CMS_SPS_CHANNEL_INDEX_NA        UINT32_MAX

/**
Radio channel ID.
@ingroup common
@see CMS_SPS_CHANNEL_INDEX_NA
*/
typedef uint32_t cms_sps_channel_index_t;

/**
Timestamp n/a value.
@ingroup common
@see cms_utc_timestamp_ms_t
*/
#define CMS_UTC_TIMESTAMP_NA            UINT64_MAX

/**
UTC timestamp in [ms]
@ingroup common
@see CMS_UTC_TIMESTAMP_NA
*/
typedef uint64_t cms_utc_timestamp_ms_t;

/**
Duration n/a value.
@ingroup common
@see cms_duration_ms_t
*/
#define CMS_DURATION_NA            0

/**
Duration in [ms]
@ingroup common
@see CMS_DURATION_NA
*/
typedef uint32_t cms_duration_ms_t;

/**
Expiration time n/a value.
@ingroup common
@see cms_expiry_time_ms_t
*/
#define CMS_EXPIRY_TIME_NA              0

/**
Expiration time in [ms]
@ingroup common
@see CMS_EXPIRY_TIME_NA
*/
typedef uint16_t cms_expiry_time_ms_t;

/**
Radio parameters of a received packet.
@ingroup common
*/
struct cms_radio_rx_params_t {
    cms_utc_timestamp_ms_t timestamp;       /**< Packet reception timestamp [ms] */
    cms_interface_id_t interface_id;        /**< Interface ID on which the packet was received */
    cms_mac_addr_t source_address;          /**< Source address of the received packet */
    cms_mac_addr_t dest_address;            /**< Destination address of the received packet */
    cms_datarate_kbps_t datarate;           /**< Datarate [kbps] of the received packet */
    cms_mac_user_prio_t user_prio;          /**< MAC user priority of the received packet */
    cms_power_dbm_t rssi;                   /**< RSSI value [dBm] of the received packet */
} CMS_PACKED;

/**
Radio parameters of a transmitted packet.
@ingroup common
*/
struct cms_radio_tx_params_t {
    cms_interface_id_t interface_id;        /**< Interface ID on which the packet is sent */
    cms_mac_addr_t dest_address;            /**< Destination address of the sent packet */
    cms_datarate_kbps_t datarate;           /**< Datarate [kbps] of the sent packet */
    cms_mac_user_prio_t user_prio;          /**< MAC user priority of the sent packet */
    cms_power_dbm_t tx_power;               /**< Transmit power [dBm] of the sent packet */
    cms_expiry_time_ms_t expiry_time;       /**< Packet expiration time [ms] */
    cms_sps_channel_index_t sps_index;      /**< SPS channel index */
} CMS_PACKED;

/**
Buffer view.
@ingroup common
Contains a read-only buffer with length.
@note This type is handled with value semantics.
*/
struct cms_buffer_view_t {
    const uint8_t* data;    /**< Pointer to the buffer data */
    uint32_t length;        /**< Data length */
} CMS_PACKED;

/**
Mutable buffer.
@ingroup common
Contains a mutable buffer with length and capacity.
@note This type can be handled with value semantics.
*/
struct cms_mutable_buffer_t {
    uint8_t* data;          /**< Pointer to the buffer data */
    uint32_t length;        /**< Actual data length */
    uint32_t capacity;      /**< Max length of the buffer */
} CMS_PACKED;

/**
Invalid subscription ID.
In case a subscription fails, the subscription ID output argument is set to
this invalid value
@ingroup common
@see cms_subs_id_t
*/
#define CMS_SUBS_ID_INVALID             0UL

/**
Subscription ID.
The ID is by any subscription function in an output argument
and can be used for the matching unsubscription function call.
@ingroup common
@see CMS_SUBS_ID_INVALID
*/
typedef uint32_t cms_subs_id_t;

/**
Standard/region type.
@ingroup common
*/
typedef enum cms_std_type_t {
    CMS_STD_TYPE_ETSI = 0,              /**< EU type */
    CMS_STD_TYPE_WAVE,                  /**< US type */
    CMS_STD_TYPE_CHINA,                 /**< China type */
} cms_std_type_t;

/**
Convert a standard type into a readable string.
@ingroup common

@param  std                 Standard/region type
@return readable string
@see cms_std_type_t
*/
static inline const char* cms_std_type_to_string(cms_std_type_t std)
{
    const char* result = NULL;

    switch(std) {
    case CMS_STD_TYPE_ETSI:
        result = "ETSI";
        break;
    case CMS_STD_TYPE_WAVE:
        result = "WAVE";
        break;
    case CMS_STD_TYPE_CHINA:
        result = "CHINA";
        break;
    default:
        result = "Invalid";
        break;
    }
    return result;
}

/**
Not available/invalid latitude value.
@ingroup common
@see cms_latitude_t
*/
#define CMS_LATITUDE_NA                 900000001L

/**
WGS84 Latitude in [0.1 udeg]
@ingroup common
@see CMS_LATITUDE_NA
*/
typedef int32_t cms_latitude_t;

/**
Not available/invalid longitude value.
@ingroup common
@see cms_longitude_t
*/
#define CMS_LONGITUDE_NA                1800000001L

/**
WGS84 Longitude in [0.1 udeg]
@ingroup common
@see CMS_LONGITUDE_NA
*/
typedef int32_t cms_longitude_t;

/**
Not available/invalid longitude value.
@ingroup common
@see cms_altitude_t
*/
#define CMS_ALTITUDE_NA                 INT32_MIN

/**
Altitude in [mm]
@ingroup common
@see CMS_ALTITUDE_NA
*/
typedef int32_t cms_altitude_t;

/**
Not available/invalid speed value.
@ingroup common
@see cms_speed_t
*/
#define CMS_SPEED_NA                    UINT32_MAX

/**
Speed value in [mm/s]
@ingroup common
@see CMS_SPEED_NA
*/
typedef uint32_t cms_speed_t;

/**
Not available/invalid acceleration value.
@ingroup common
@see cms_acceleration_t
*/
#define CMS_ACCELERATION_NA             INT32_MAX

/**
Speed value in [mm/s^2]
@ingroup common
@see CMS_ACCELERATION_NA
*/
typedef int32_t cms_acceleration_t;

/**
Not available/invalid heading value.
@ingroup common
@see cms_heading_t
*/
#define CMS_HEADING_NA                  UINT32_MAX

/**
Heading value in [0.001 degree].
Absolute value with North reference.
0 means North, 90000 is East, 180000 is South, 270000 is West.
@ingroup common
@see CMS_HEADING_NA
@see cms_heading_confidence_t
*/
typedef uint32_t cms_heading_t;

/**
Not available/invalid heading confidence value.
@ingroup common
@see cms_heading_confidence_t
*/
#define CMS_HEADING_CONFIDENCE_NA       UINT32_MAX

/**
Heading confidence value in [0.001 degree].
Always represents a relative value.
@ingroup common
@see CMS_HEADING_CONFIDENCE_NA
@see cms_heading_t
*/
typedef uint32_t cms_heading_confidence_t;

/**
Not available/invalid yaw rate value.
@ingroup common
@see cms_yaw_rate_t
*/
#define CMS_YAW_RATE_NA                 INT32_MAX

/**
Yaw rate value in [0.001 degree/s].
Always represents a relative value.
Positive values are CCW (left), negative values are CW (right).
@ingroup common
@see CMS_YAW_RATE_NA
@see cms_yaw_rate_t
*/
typedef int32_t cms_yaw_rate_t;

/**
Not available/invalid length/distance value.
@ingroup common
@see cms_length_t
*/
#define CMS_LENGTH_NA                   UINT64_MAX

/**
Generic length/distance type in [mm].
@ingroup common
@see CMS_LENGTH_NA
*/
typedef uint64_t cms_length_t;

/**
Not available/invalid probability value.
@ingroup common
@see cms_probability_t
*/
#define CMS_PROBABILITY_NA              UINT8_MAX

/**
Probability type in [%].
@ingroup common
@see CMS_PROBABILITY_NA
*/
typedef uint8_t cms_probability_t;

/**
Packets per second [pps].
@ingroup common
*/
typedef uint16_t cms_pps_t;

#define CMS_PPS_UNLIMITED               UINT16_MAX

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_COMMON_TYPES_H_ */
