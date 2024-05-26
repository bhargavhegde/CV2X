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
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cpm.h>
#include <cms_v2x/nav.h>

/** @file
@brief Manipulate CPM sensor information and free space.
@ingroup ex
*/

/**
ASN.1 encoded SensorInformation
@note generated using the C++ code below:

@code
SensorInformation sensorInfo {};

sensorInfo.sensorID = 17;
sensorInfo.type = SensorType_nightvision;
sensorInfo.detectionArea.choice = DetectionArea_stationarySensorRadial;

auto& radial = sensorInfo.detectionArea.u.stationarySensorRadial;
radial.range = 1230U;
radial.stationaryHorizontalOpeningAngleStart = 780U;
radial.stationaryHorizontalOpeningAngleEnd = 1360U;
radial.verticalOpeningAngleStart_option = TRUE;
radial.verticalOpeningAngleStart = 3150;
radial.verticalOpeningAngleEnd_option = TRUE;
radial.verticalOpeningAngleEnd = 3530;
radial.sensorHeight_option = TRUE;
radial.sensorHeight = 62U;

std::array<uint8_t, 128U> buf;
uint32_t size = asn_encode_uper_f(asn1_type_SensorInformation, &sensorInfo, buf.data(), buf.size());

std::string hexStr;
for (uint32_t ix = 0U; ix < size; ++ix) {
    char hexByte[5];
    std::sprintf(hexByte, "0x%02x", (int)buf[ix]);

    hexStr.append(hexByte, 4);
    if (ix != size - 1) {
        hexStr.append(", ");
    }
}
std::printf("%s\n", hexStr.c_str());
@endcode

*/
static const uint8_t SENSOR_INFO_DATA[] = {
    0x04, 0x54, 0x5a, 0x26, 0x71, 0x86, 0x2a, 0x86, 0x27, 0x6e, 0x52, 0x78, 0xc0
};


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Create sensor information */
    cms_cpm_sensor_info_t sensor_info = {0U};
    memcpy(sensor_info.data, SENSOR_INFO_DATA, sizeof(SENSOR_INFO_DATA));
    sensor_info.data_length = sizeof(SENSOR_INFO_DATA);

    /* Inject sensor into the CPM module */
    error = error || cms_cpm_add_sensor_info(&session, &sensor_info, NULL);

    /* Get position that will serve as a base for the free space area parameters */
    cms_nav_fix_t nav_fix = {0};
    error = error || cms_nav_get_last_fix(&session, NULL, &nav_fix);

    /* Create a free space area */
    cms_cpm_free_space_t free_space = {0U};
    free_space.id = 12U;
    free_space.area.type = CMS_CPM_FREE_SPACE_AREA_TYPE_ELLIPSE;
    free_space.area.ellipse.center.latitude = nav_fix.latitude + 2345;
    free_space.area.ellipse.center.longitude = nav_fix.longitude + 8765;
    free_space.area.ellipse.semi_major_length = 310U;
    free_space.area.ellipse.semi_minor_length = 160U;
    free_space.area.ellipse.semi_major_direction = 870U;
    free_space.confidence = 92U;
    free_space.sensor_ids[0] = 17U;
    free_space.sensor_id_num = 1U;
    free_space.shadowing_applies = true;
    free_space.timeout = 7000U;

    /* Inject free space area into the CPM module */
    error = error || cms_cpm_add_free_space(&session, &free_space, NULL);

    if(error) {
        printf("Unable to add free space area.\n");
    } else {
        printf("Free space area successfully added.\n");
    }

    /* Sleep a little and remove it before it expires */
    usleep(4000000U);

    cms_cpm_element_ref_t to_remove = {
        .id = free_space.id
    };
    error = error || cms_cpm_remove_free_space(&session, &to_remove, NULL);

    if(error) {
        printf("Unable to remove free space area.\n");
    } else {
        printf("Free space area successfully removed.\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
