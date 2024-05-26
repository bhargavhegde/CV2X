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
#include <cms_v2x/wsa.h>

/** @file
@brief Register and deregister the client's need and priority for a WSA service.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Register the client's priority for a WSA service */
    const cms_wsa_reg_client_srv_param_t reg_client_srv_param = {
        .psid = 0x82UL,
        .prio = 1U
    };
    cms_wsa_reg_client_srv_out_t out;

    error = error || cms_wsa_register_client_service(&session, &reg_client_srv_param, &out);
    if(error) {
        printf("Unable to register the client's priority for a WSA service.\n");
    } else {
        printf("Successfully registered the client's priority for a WSA service.\n");
    }

    /* Deregister the client's priority for a WSA service using received client ID */
    const cms_wsa_dereg_client_srv_param_t dereg_client_srv_param = {
        .id = out.id
    };
    error = error || cms_wsa_deregister_client_service(&session, &dereg_client_srv_param, NULL);
    if(error) {
        printf("Unable to deregister the client's priority for a WSA service.\n");
    } else {
        printf("Successfully deregistered the client's priority for a WSA service.\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

