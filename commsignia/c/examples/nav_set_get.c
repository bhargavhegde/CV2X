/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <cms_v2x/api.h>
#include <cms_v2x/nav.h>

/** @file
@brief Set and get nav fix.
@ingroup ex
*/

const char* l_nav_drive_direction_to_str(cms_nav_drive_direction_t drive_direction)
{
    const char* str = "unavailable";

    switch(drive_direction) {
    case CMS_NAV_DRIVE_DIR_FORWARD:
        str = "forward";
        break;

    case CMS_NAV_DRIVE_DIR_BACKWARD:
        str = "backward";
        break;

    default:
        break;
    }

    return str;
}

void l_nav_print_fix(const cms_nav_fix_t* nav_fix)
{
    printf("NAV fix %s valid\n", nav_fix->is_valid ? "is" : "is not");
    printf("Timestamp (since 1970.01.01): ");
    if(CMS_UTC_TIMESTAMP_NA != nav_fix->timestamp) {
        printf("%llu [ms]\n", (unsigned long long)nav_fix->timestamp);
    } else {
        printf("Not Available\n");
    }
    printf("Leap seconds (TAI - UTC): %llu [s]\n", (unsigned long long)nav_fix->leap_seconds);
    printf("Latitude: ");
    if(CMS_LATITUDE_NA != nav_fix->latitude) {
        printf("%ld [0.1 udeg]\n", (long)nav_fix->latitude);
    } else {
        printf("Not Available\n");
    }
    printf("Longitude: ");
    if(CMS_LONGITUDE_NA != nav_fix->longitude) {
        printf("%ld [0.1 udeg]\n", (long)nav_fix->longitude);
    } else {
        printf("Not Available\n");
    }
    printf("Altitude: ");
    if(CMS_ALTITUDE_NA != nav_fix->altitude) {
        printf("%ld [mm]\n", (long)nav_fix->altitude);
    } else {
        printf("Not Available\n");
    }
    printf("Heading: ");
    if(CMS_HEADING_NA != nav_fix->heading) {
        printf("%lu [0.001 deg]\n", (unsigned long)nav_fix->heading);
    } else {
        printf("Not Available\n");
    }
    printf("Speed: ");
    if(CMS_SPEED_NA != nav_fix->speed) {
        printf("%lu [mm/s]\n", (unsigned long)nav_fix->speed);
    } else {
        printf("Not Available\n");
    }
    printf("Confidence ellipse major axis half length: ");
    if(CMS_LENGTH_NA != nav_fix->pce_semi_major) {
        printf("%llu [mm]\n", (unsigned long long)nav_fix->pce_semi_major);
    } else {
        printf("Not Available\n");
    }
    printf("Confidence ellipse minor axis half length: ");
    if(CMS_LENGTH_NA != nav_fix->pce_semi_minor) {
        printf("%llu [mm]\n", (unsigned long long)nav_fix->pce_semi_minor);
    } else {
        printf("Not Available\n");
    }
    printf("Confidence ellipse major axis orientation: ");
    if(CMS_HEADING_NA != nav_fix->pce_orientation) {
        printf("%lu [0.001 deg]\n", (unsigned long)nav_fix->pce_orientation);
    } else {
        printf("Not Available\n");
    }
    printf("Altitude Confidence ");
    if(CMS_ALTITUDE_NA != nav_fix->altitude_confidence) {
        printf("%ld [mm]\n", (long)nav_fix->altitude_confidence);
    } else {
        printf("Not Available\n");
    }
    printf("Heading Confidence: ");
    if(CMS_HEADING_CONFIDENCE_NA != nav_fix->heading_confidence) {
        printf("%lu [0.001 deg]\n", (unsigned long)nav_fix->heading_confidence);
    } else {
        printf("Not Available\n");
    }
    printf("Speed Confidence: ");
    if(CMS_SPEED_NA != nav_fix->speed_confidence) {
        printf("%lu [mm/s]\n", (unsigned long)nav_fix->speed_confidence);
    } else {
        printf("Not Available\n");
    }
    printf("Direction: %s\n", l_nav_drive_direction_to_str(nav_fix->drive_direction));
}

void l_print_cms_source_t(cms_nav_mode_t nav_source)
{
    printf("navigation is set by ");
    switch(nav_source.source) {
    case CMS_NAV_SOURCE_MANUAL:
        printf("manual\n");
        break;
    case CMS_NAV_SOURCE_REAL:
        printf("GNSS\n");
        break;
    case CMS_NAV_SOURCE_GPSD:
        printf("GPSD\n");
        break;
    }

}

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Get the latest position */
    cms_nav_fix_t nav_fix = {0};
    error = error || cms_nav_get_last_fix(&session, NULL, &nav_fix);
    if(error) {
        printf("Unable to get NAV fix\n");
    } else {
        printf("Nav fix before manual setting:\n");
        l_nav_print_fix(&nav_fix);
    }

    /* set the current number of leapseconds
     * in this example it is set to the same as was received
     */
    cms_nav_set_leap_seconds_t set_leap_sec = {
        .leap_seconds = (uint32_t)nav_fix.leap_seconds
    };
    error = error || cms_nav_set_leap_seconds(&session, &set_leap_sec, NULL);

    /*
    Create a nav data to set,
    e.g. set the position of the Commsignia office with 1.5m accuracy:
    - latitude: 47.4754593°
    - longitude: 19.0582323°
    */
    cms_nav_set_manual_t set_nav = {0};
    set_nav.auto_update = false; /* this functionality not yet supported */
    /* Set the time to current time, enable adjustSystemTime in config
    if you want to update system time of the Commsignia V2X software stack */
    set_nav.nav_fix.is_valid = true;
    set_nav.nav_fix.timestamp = (time(NULL) * 1000);
    set_nav.nav_fix.latitude = 474754590L;
    set_nav.nav_fix.longitude = 190582320L;
    set_nav.nav_fix.altitude = 3000;
    set_nav.nav_fix.pce_semi_major = 1500;
    set_nav.nav_fix.pce_semi_minor = 1500;
    set_nav.nav_fix.pce_orientation = 250;
    set_nav.nav_fix.heading = 300;
    set_nav.nav_fix.speed = 18000;
    set_nav.nav_fix.drive_direction = CMS_NAV_DRIVE_DIR_FORWARD;

    cms_nav_mode_t nav_s = {.source = CMS_NAV_SOURCE_MANUAL };
    error = error || cms_nav_get_source(&session, NULL, &nav_s);
    if(error) {
        printf("Unable to get navigation source\n");
    } else {
        l_print_cms_source_t(nav_s);
    }

    /* Switch to Manual navigation mode */
    cms_nav_source_t nav_source = CMS_NAV_SOURCE_MANUAL;
    error = error || cms_nav_set_source(&session, &nav_source, NULL);
    if(error) {
        printf("Unable to switch to manual mode\n");
    } else {
        printf("Switched to manual mode\n");
    }

    nav_s.source = CMS_NAV_SOURCE_REAL;
    error = error || cms_nav_get_source(&session, NULL, &nav_s);
    if(error) {
        printf("Unable to get navigation source\n");
    } else {
        l_print_cms_source_t(nav_s);
    }

    if(!error) {
        /* Set the position and time */
        for(int index = 0; index < 10; index++) {
            /* in every 0.1s set time 0.1s ahead and latitude increase with 10 micro degree*/
            set_nav.nav_fix.timestamp += 100;
            set_nav.nav_fix.latitude += 100;
            error = error || cms_nav_set_manual(&session, &set_nav, NULL);
            if(error) {
                printf("Unable to set NAV fix\n");
            }
            /* sleep 0.1 s */
            const unsigned int sleep_us = 100000;
            usleep(sleep_us);
        }
    }

    /* Get the latest position after set position, expected to be the same previously set */
    error = error || cms_nav_get_last_fix(&session, NULL, &nav_fix);
    if(error) {
        printf("Unable to get NAV fix\n");
    } else {
        printf("Nav fix after manual setting:\n");
        l_nav_print_fix(&nav_fix);
    }

    /* timestamp will be overwritten by current system time in case of NA timestamp*/
    set_nav.nav_fix.timestamp = CMS_UTC_TIMESTAMP_NA;
    error = error || cms_nav_set_manual(&session, &set_nav, NULL);
    if(error) {
        printf("Unable to set NAV fix\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
