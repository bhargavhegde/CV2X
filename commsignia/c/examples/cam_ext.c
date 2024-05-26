/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cam.h>

/** @file
@brief Set/reset CAM Extension
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* UPER encoded CAM extension data */
    uint8_t payload_ext[] = {0x60};

    cms_cam_ext_num_t ext_num = {
        .num_ext = 1U
    };

    cms_cam_ext_t ext_in = {
        .id = 0U
    };

    cms_buffer_view_t cam_ext_encoded = {
        .data = payload_ext,
        .length = sizeof(payload_ext)
    };

    error = error || cms_cam_set_extensions_num(&session, &ext_num, NULL);

    if(error) {
        printf("Unable to set CAM Extension number\n");
    } else {
        printf("CAM Extension number set successful\n");
    }

    /* Set CAM Extension */
    error = error || cms_cam_set_extension(&session, &ext_in, cam_ext_encoded, NULL);

    if(error) {
        printf("Unable to set CAM Extension\n");
    } else {
        printf("CAM Extension set successful\n");
    }

    /* Wait some time */
    sleep(1);

    /* Reset CAM Extension  */
    error = error || cms_cam_reset_extension(&session, &ext_in, NULL);

    if(error) {
        printf("Unable to reset CAM Extension\n");
    } else {
        printf("CAM Extension reset successful\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

