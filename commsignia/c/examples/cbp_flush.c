/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stdio.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cbp.h>

/** @file
@brief Flush / Reset Context Based Prioritisation state
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create connected session to host */
    cms_session_t session = cms_get_session();
    bool error = cms_api_connect(&session, host, 2U, NULL, NULL);

    /* Get flush CBP status  */
    error = error || cms_cbp_flush_addr(&session, NULL, NULL);

    /* Disconnect from host */
    cms_api_disconnect(&session);

    return error ? 1 : 0;
}
