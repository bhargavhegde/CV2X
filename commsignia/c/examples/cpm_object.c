/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2022
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cpm.h>
#include <cms_v2x/nav.h>

/** @file
@brief Manipulate CPM perceived objects.
@ingroup ex
*/


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Get position that will serve as a base for the detected object parameters */
    cms_nav_fix_t nav_fix = {0};
    error = error || cms_nav_get_last_fix(&session, NULL, &nav_fix);

    /* Fill detection parameters */
    cms_cpm_perceived_object_t object = {0U};
    object.id = 1U;
    object.position.latitude = nav_fix.latitude + 1234L;
    object.position.longitude = nav_fix.longitude - 9876L;
    object.ground_speed.direction = nav_fix.heading + 3800U;
    object.ground_speed.value = nav_fix.speed + 410U;
    object.yaw_angle.accuracy = CMS_HEADING_NA;
    object.length.value = CMS_LENGTH_NA;
    object.width.value = CMS_LENGTH_NA;
    object.height.value = CMS_LENGTH_NA;
    object.detection_confidence = 67U;
    object.detection_class_num = 1U;
    object.detection_classes[0].object_class = CMS_CPM_DETECTION_CLASS_PERSON;
    object.detection_classes[0].object_subclass = CMS_CPM_PERSON_SUBCLASS_PERSON_ON_SKATES;
    object.detection_classes[0].confidence = 82U;
    object.matched_position.longitudinal_lane_position = CMS_LENGTH_NA;
    object.timestamp = nav_fix.timestamp;
    object.timeout = 5000U;

    /* Inject detection the CPM module */
    error = error || cms_cpm_add_perceived_object(&session, &object, NULL);

    if(error) {
        printf("Unable to add CPM object.\n");
    } else {
        printf("CPM object successfully added.\n");
    }

    /* Sleep a little and remove it before it expires */
    usleep(700000U);

    cms_cpm_element_ref_t to_remove = {
        .id = object.id
    };
    error = error || cms_cpm_remove_perceived_object(&session, &to_remove, NULL);

    if(error) {
        printf("Unable to remove CPM object.\n");
    } else {
        printf("CPM object successfully removed.\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
