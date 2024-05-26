/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <cms_v2x/api.h>
#include <cms_v2x/nav.h>

static cms_utc_timestamp_ms_t get_current_time_ms(void)
{
    cms_utc_timestamp_ms_t result = CMS_UTC_TIMESTAMP_NA;

    struct timespec tp = {0};

    if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
        result = tp.tv_sec * 1000ULL;
        result += tp.tv_nsec / 1000000ULL;
    }

    return result;
}

/** @file
@brief Set nav fix periodically.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /*
    Create a nav data to set,
    e.g. set the position of the Commsignia office with 1.5m accuracy:
    - latitude: 47.4754593°
    - longitude: 19.0582323°
    */
    cms_nav_set_manual_t set_nav = {0};
    set_nav.auto_update = false; /* this functionality is currently not supported */
    set_nav.nav_fix.is_valid = true;
    set_nav.nav_fix.latitude = 474754590L;
    set_nav.nav_fix.longitude = 190582320L;
    set_nav.nav_fix.altitude = 3000;
    set_nav.nav_fix.pce_semi_major = 1500;
    set_nav.nav_fix.pce_semi_minor = 1500;
    set_nav.nav_fix.pce_orientation = 250;
    set_nav.nav_fix.heading = 300;
    set_nav.nav_fix.speed = 18000;
    set_nav.nav_fix.drive_direction = CMS_NAV_DRIVE_DIR_FORWARD;

    /* Switch to Manual navigation source */
    cms_nav_source_t nav_source = CMS_NAV_SOURCE_MANUAL;
    error = error || cms_nav_set_source(&session, &nav_source, NULL);
    if(error) {
        printf("Unable to switch to Manual source\n");
    } else {
        printf("Switched to Manual source\n");
    }

    /* Set the position and time */
    while(true) {
        /* Update timestamp to current time */
        set_nav.nav_fix.timestamp = get_current_time_ms();
        /* increase latitude with 10 microdegrees */
        set_nav.nav_fix.latitude += 100;
        error = error || cms_nav_set_manual(&session, &set_nav, NULL);
        if(error) {
            printf("Unable to set NAV fix\n");
        }
        /* sleep 0.1 s */
        const unsigned int sleep_us = 100000;
        usleep(sleep_us);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
