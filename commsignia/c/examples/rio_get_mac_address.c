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
@brief Get MAC address of the radio interfaces
@ingroup ex
*/

static void print_mac_address(const cms_rio_mac_setup_t* mac_setup)
{
    printf("Interface %lu, %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
           (unsigned long)mac_setup->interface_id,
           mac_setup->mac_address[0],
           mac_setup->mac_address[1],
           mac_setup->mac_address[2],
           mac_setup->mac_address[3],
           mac_setup->mac_address[4],
           mac_setup->mac_address[5]);
}

static void get_mac_and_print(cms_session_t* session, cms_interface_id_t interface_id)
{
    const cms_rio_interface_ref_t interface = {
        .interface_id = interface_id,
    };
    cms_rio_mac_setup_t mac_setup = {0};

    cms_rio_result_t error_code = cms_rio_get_mac_address(session, &interface, &mac_setup);

    switch(error_code) {
    case CMS_RIO_RESULT_OK:
        print_mac_address(&mac_setup);
        break;
    case CMS_RIO_RESULT_NO_IFACE_ERROR:
        printf("Unable to get radio device on interface %u.\n", interface.interface_id);
        break;
    case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
        printf("Interface %u is disabled.\n", interface.interface_id);
        break;
    default:
        fprintf(stderr, "Cannot get mac address for interface %lu, error code: %d\n",
                (unsigned long)interface_id, (int)error_code);
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

    if(!error) {
        /* print mac address for 1st interface */
        get_mac_and_print(&session, 1);

        /* print mac address for 2nd interface */
        get_mac_and_print(&session, 2);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

