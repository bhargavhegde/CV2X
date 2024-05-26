/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cam.h>

/** @file
@brief Set/reset CAM Special Vehicle Container
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    uint8_t payload_publicTransportCont[] = {0x08, 0x00, 0xA0, 0x02, 0x24, 0x46, 0x68, 0x8A, 0xA0};

    /* EXAMPLE emergencyContainer data: uint8_t payload_emergencyCont[] = {0x52}; */

    cms_buffer_view_t veh_set_uper_encoded = {
        .data = payload_publicTransportCont,
        .length = sizeof(payload_publicTransportCont)
    };
    /* Set CAM Special Vehicle Container */
    error = error || cms_cam_set_special_vehicle(&session, NULL, veh_set_uper_encoded, NULL);

    if(error) {
        printf("Unable to set CAM Special Vehicle Container\n");
    } else {
        printf("CAM Special Vehicle Container set successful\n");
    }

    /* Wait some time */
    sleep(1);

    /* Reset CAM Special Vehicle Container  */
    error = error || cms_cam_reset_special_vehicle(&session, NULL, NULL);

    if(error) {
        printf("Unable to reset CAM Special Vehicle Container\n");
    } else {
        printf("CAM Special Vehicle Container reset successful\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

