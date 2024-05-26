/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/radio.h>

/** @file
@brief Get Channel Busy Ratio (CBR) on a given interface.
@ingroup ex
*/


static bool l_get_cbr_for_iface(cms_session_t session, cms_interface_id_t iface_id)
{
    bool error = false;

    /* Get radio channel on interface */
    const cms_rio_interface_ref_t interface = {
        .interface_id = iface_id,
    };

    cms_rio_cbr_t rio_cbr = 0;

    switch(cms_rio_get_cbr(&session, &interface, &rio_cbr)) {
    case CMS_RIO_RESULT_OK:
        if(CMS_RIO_CBR_NA != rio_cbr) {
            printf("CBR is %u on interface %u.\n", rio_cbr, interface.interface_id);
        } else {
            printf("CBR is not available on interface %u.\n", interface.interface_id);
        }
        break;
    case CMS_RIO_RESULT_NO_IFACE_ERROR:
        printf("Interface %u does not exist.\n", interface.interface_id);
        error = true;
        break;
    case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
        printf("Interface %u is disabled.\n", interface.interface_id);
        error = true;
        break;
    default:
        printf("Unable to get CBR on interface %u.\n", interface.interface_id);
        error = true;
        break;
    }

    return error;
}

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    error = error || l_get_cbr_for_iface(session, 1);
    error = error || l_get_cbr_for_iface(session, 2);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

