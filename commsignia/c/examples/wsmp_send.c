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
#include <string.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/wsmp.h>

/** @file
@brief Send a WSMP message.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    static const uint8_t BROADCAST_ADDR[CMS_MAC_ADDRESS_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    /* Create send header */
    cms_wsmp_send_data_t send_hdr = {0};

    send_hdr.radio.datarate = 0;
    memcpy(send_hdr.radio.dest_address, BROADCAST_ADDR, CMS_MAC_ADDRESS_LENGTH);
    send_hdr.radio.expiry_time = 0;
    send_hdr.radio.interface_id = 1;
    send_hdr.radio.sps_index = 0;
    send_hdr.radio.tx_power = 0;
    send_hdr.radio.user_prio = 0;

    /*
    The PSID value must match in the WSMP and security header.
    This value is not p-encoded.
    */
    static const cms_psid_t PSID = 0x82;

    send_hdr.wsmp_hdr.channel_id = true;
    send_hdr.wsmp_hdr.datarate = true;
    send_hdr.wsmp_hdr.psid = PSID;
    send_hdr.wsmp_hdr.tx_power = true;

    send_hdr.security.sign_info.psid = PSID;
    send_hdr.security.sign_info.sign_method = CMS_SIGN_METH_SIGN_CERT;

    uint8_t msg_payload[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    /* Create a buffer view as a handle to the actual payload buffer */
    cms_buffer_view_t msg = {
        .data = msg_payload,
        .length = sizeof(msg_payload)
    };

    /* Send the prepared WSMP message */
    error = error || cms_wsmp_send(&session, &send_hdr, msg, NULL);
    if(error) {
        printf("Unable to send WSMP message\n");
    } else {
        printf("WSMP message sent\n");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

