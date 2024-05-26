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
#include <cms_v2x/radio.h>

/** @file
@brief Set radio channel on a given interface.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set radio channel on interface */
    const cms_rio_channel_setup_t set_channel = {
        .interface_id = 1,
        .channel_id = 182
    };

    if(!error) {
        switch(cms_rio_set_channel(&session, &set_channel, NULL)) {
        case CMS_RIO_RESULT_OK:
            printf("Channel is %u on interface %u.\n", set_channel.channel_id, set_channel.interface_id);
            break;
        case CMS_RIO_RESULT_NO_IFACE_ERROR:
            printf("Unable to get radio device on interface %u.\n", set_channel.interface_id);
            error = true;
            break;
        case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
            printf("Interface %u is disabled.\n", set_channel.interface_id);
            error = true;
            break;
        default:
            printf("Unable to set channel on interface %u.\n", set_channel.interface_id);
            error = true;
            break;
        }
    }

    /* Get radio channel on interface */
    const cms_rio_interface_ref_t interface = {
        .interface_id = 1,
    };

    cms_rio_channel_setup_t channel_setup = {
        .channel_id = 0
    };

    if(!error) {
        switch(cms_rio_get_channel(&session, &interface, &channel_setup)) {
        case CMS_RIO_RESULT_OK:
            printf("Channel is %u on interface %u.\n", channel_setup.channel_id, interface.interface_id);
            break;
        case CMS_RIO_RESULT_NO_IFACE_ERROR:
            printf("Unable to get radio device on interface %u.\n", interface.interface_id);
            error = true;
            break;
        case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
            printf("Interface %u is disabled.\n", interface.interface_id);
            error = true;
            break;
        default:
            printf("Unable to get channel on interface %u.\n", interface.interface_id);
            error = true;
            break;
        }
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

