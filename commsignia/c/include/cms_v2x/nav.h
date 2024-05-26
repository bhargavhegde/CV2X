/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_NAV_H_
#define CMS_NAV_H_

#include <cms_v2x/common_types.h>

/** @file
@brief Navigation
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup nav Navigation
@ingroup api
*/

/**
Possible values for NAV drive direction.
@ingroup nav
@note Unavailable, forward or backward.
      If backward, the heading value of nav fix is the opposite to
      the physical heading of the vehicle.
@see cms_nav_fix_t
*/
typedef enum cms_nav_drive_direction_t {
    CMS_NAV_DRIVE_DIR_UNAVAILABLE = 0,
    CMS_NAV_DRIVE_DIR_FORWARD,
    CMS_NAV_DRIVE_DIR_BACKWARD,
} cms_nav_drive_direction_t;

/**
Possible values for NAV source.
@ingroup nav
*/
typedef enum cms_nav_source_t {
    /**
    Nav fix is initialized according to configuration at startup.
    Nav fix can be set via cms_nav_set_manual. Timestamp is updated to system time if not explicitly set.
    */
    CMS_NAV_SOURCE_MANUAL = 0,
    /** Nav fix is read from GNSS hardware. */
    CMS_NAV_SOURCE_REAL,
    /** Nav fix is read from third party GPSD server. */
    CMS_NAV_SOURCE_GPSD,
} cms_nav_source_t;

/**
NAV mode structure
@ingroup nav
*/
typedef struct cms_nav_mode_t {
    cms_nav_source_t source;
} CMS_PACKED cms_nav_mode_t;

/**
NAV fix structure
@ingroup nav
*/
typedef struct cms_nav_fix_t {

    /** Flag to indicate whether the NAV fix is valid.
    @note If this flag is true, all other fields will be valid, recent and
    consistent. If the flag is false, either all fields will be N/A, or they
    will contain the most recent consistent data, but it can be very old. */
    bool is_valid;

    cms_utc_timestamp_ms_t timestamp;               /**< UTC milliseconds since Unix epoch */
    uint64_t leap_seconds;                          /**< Leap seconds, i.e the difference between UTC and TAI in [s] */
    cms_latitude_t latitude;                        /**< Latitude angle */
    cms_longitude_t longitude;                      /**< Longitude angle */
    cms_altitude_t altitude;                        /**< Altitude */
    cms_altitude_t altitude_confidence;             /**< Confidence of altitude */
    cms_length_t pce_semi_major;                    /**< Position Confidence Ellipse: half of major axis length */
    cms_length_t pce_semi_minor;                    /**< Position Confidence Ellipse: half of minor axis length */
    cms_heading_t pce_orientation;                  /**< Position Confidence Ellipse: direction of the major axis */
    cms_heading_t heading;                          /**< Heading of the motion */
    cms_heading_confidence_t heading_confidence;    /**< Confidence of heading */
    cms_speed_t speed;                              /**< Speed of the motion */
    cms_speed_t speed_confidence;                   /**< Confidence of speed */
    cms_nav_drive_direction_t drive_direction;      /**< Drive direction */
    uint8_t number_of_used_satellites;              /**< Satellites used to determine position */
} CMS_PACKED cms_nav_fix_t;


/**
Input parameter type of NAV manual set
@ingroup nav
@see cms_nav_set_manual
@see cms_nav_fix_t
*/
typedef struct cms_nav_set_manual_t {
    cms_nav_fix_t nav_fix;      /**< Navigation fix parameters */
    bool auto_update;           /**< Enables the emulation of 10Hz NAV update @experimentalfield */
} CMS_PACKED cms_nav_set_manual_t;

/**
NAV time structure
@ingroup nav
@see cms_nav_set_time
*/
typedef struct cms_nav_time_t {
    cms_utc_timestamp_ms_t timestamp;      /**< Timestamp in UTC milliseconds since Unix epoch */
} CMS_PACKED cms_nav_time_t;

/**
Input parameter type of leap seconds set function
the number of leap seconds since from 1970-01-01 00:00:00
@ingroup nav
@see cms_nav_set_leap_seconds
*/
typedef struct cms_nav_set_leap_seconds_t {
    uint32_t leap_seconds;                  /**< current number of leap seconds */
} CMS_PACKED cms_nav_set_leap_seconds_t;


/**
NAV get last fix
@ingroup nav
The navigation fix may not be valid even if this function returns with a
success. Check the is_valid flag in the structure.
If an invalid timestamp was set with cms_nav_set_manual and adjustSystemTime config parameter
is enabled, then this function overwrites the invalid timestamp with the current system time.
@param session               Client session
@param[in] unused_in         Unused parameter
@param[out] nav_fix          Navigation fix output
@return true in case of error
@see cms_nav_fix_t
*/
bool cms_nav_get_last_fix(const cms_session_t* session,
                          void* unused_in,
                          cms_nav_fix_t* nav_fix);


/**
Set the navigation fix for manual positioning mode
@ingroup nav

@note auto-update functionality is not yet supported.

About is_valid field:
nav_set.nav_fix.is_valid field is automatically calculated by the stack,
in case of latitude or longitude have NA value: is_valid will be overwritten to false,
any other case: is_valid from the nav_set struct will be used

In case of invalid or zero timestamp:
nav_set.nav_fix.timestamp is invalid timestamp in case of 0 or CMS_UTC_TIMESTAMP_NA.
In case of invalid timestamp the timestamp will be overwritten by current system time when the fix is used.

Expiration of the navigational fix:
After cms_nav_set_manual function with valid timestamp usage
the stack is expecting continuous refresh of navigation data.
Make sure you are calling cms_nav_set_manual with at least 1 HZ (recommended: 10HZ) frequency
if you want to use this function with valid timestamp.

Setting system time:
In case of valid timestamp and adjustSystemTime (config parameter) is enabled:
system time and leapseconds value is set to the values from nav_set
If you want to set only the system time please use cms_nav_set_time.

@param session              Client session
@param[in] nav_set          Navigation fix input
@param[out] unused_out      Unused parameter
@return true in case of error
@see cms_nav_set_manual_t
*/
bool cms_nav_set_manual(const cms_session_t* session,
                        const cms_nav_set_manual_t* nav_set,
                        void* unused_out);

/**
Switches between different navigation sources
@ingroup nav
@param session        Client session
@param nav_source     Navigation source
@param unused_out     Unused parameter
@return true in case of error
*/
bool cms_nav_set_source(const cms_session_t* session,
                        const cms_nav_source_t* nav_source,
                        void* unused_out);

/**
Gets the current navigation source
@ingroup nav
@param session              Client session
@param unused_in            Unused parameter
@param[out] nav_source      Navigation source
@return true in case of error
*/
bool cms_nav_get_source(const cms_session_t* session,
                        void* unused_in,
                        cms_nav_mode_t* nav_source);

/**
Sets the device time.
@note This feature disables time synchronisation with GNSS or GPSD server.
@ingroup nav
@param session        Client session
@param nav_time       Time
@param unused_out     Unused parameter
@return true in case of error
*/
bool cms_nav_set_time(const cms_session_t* session,
                      const cms_nav_time_t* nav_time,
                      void* unused_out);

/**
Sets the current number of leap seconds
@ingroup nav
@param session        Client session
@param leap_seconds   input parameter contains the current number of leap seconds
@param unused_out     Unused parameter
@return true in case of error

Sets the number of leap seconds since 1970-01-01 00:00:00.
This is the offset between UTC (Coordinated Universal Time) and TAI (International Atomic Time).
*/
bool cms_nav_set_leap_seconds(const cms_session_t* session,
                              const cms_nav_set_leap_seconds_t* leap_seconds,
                              void* unused_out);


/**
NAV notification data.
@ingroup nav
*/
typedef cms_nav_fix_t cms_nav_notif_data_t;


/**
NAV notify callback function.
@ingroup nav
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_nav_subscribe function call.
@param data         Notification
@param ctx          User context, registered during subscribe
@see cms_nav_subscribe
*/
typedef void (*cms_nav_notify_f)(const cms_nav_notif_data_t* data,
                                 void* ctx);

/**
NAV fix subscribe.
@ingroup nav
Upon successful subscription, when a GPS fix received by the navigation
module, the ITS software stack sends a notification to the subscribed
client with some metadata.
@param session              Client session
@param callback_f           Notification callback function
@param ctx                  User context
@param[out] subs_id         Subscription ID output
@return true in case of error
@see cms_nav_notify_f
@see cms_nav_unsubscribe
*/
bool cms_nav_subscribe(const cms_session_t* session,
                       cms_nav_notify_f callback_f,
                       void* ctx,
                       cms_subs_id_t* subs_id);

/**
NAV fix unsubscribe.
@ingroup nav
Upon successful unsubscription, when a GPS fix is received, the ITS
software stack does not send a notification to the subscribed client any more.
@param session              Client session
@param subs_id              Subscription ID, retrieved by cms_nav_subscribe
@return true in case of error
@see cms_nav_subscribe
*/
bool cms_nav_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_NAV_H */
