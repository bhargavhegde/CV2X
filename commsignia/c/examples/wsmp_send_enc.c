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
#include <string.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/wsmp.h>

/** @file
@brief Send a WSMP message that contains a 1609Dot2Data with EncryptedData.
@ingroup ex
*/

static const cms_psid_t PSID = 0x87UL; /* Non p-encoded PSID */

static const uint8_t msg_payload[] = {
    0x03, 0x82, 0x01, 0x01, 0x83, 0x16, 0x31, 0xAF,
    0xB5, 0xFC, 0x25, 0x5D, 0x0F, 0x80, 0x83, 0x1F,
    0x3C, 0xF0, 0x5A, 0x49, 0xC8, 0x62, 0x14, 0x23,
    0x5A, 0x0D, 0xDE, 0x81, 0xD8, 0xBB, 0xD3, 0x3C,
    0xCB, 0xDB, 0xAB, 0xD4, 0xF1, 0x49, 0x70, 0xA8,
    0x87, 0x9A, 0xFE, 0x7F, 0x0D, 0x40, 0xD4, 0x16,
    0xE1, 0x51, 0x05, 0x20, 0x24, 0xDB, 0xAB, 0x54,
    0x55, 0xB0, 0x11, 0x3E, 0x16, 0xC2, 0x08, 0x15,
    0x7E, 0xFA, 0xD8, 0x11, 0xEB, 0x86, 0x04, 0xED,
    0x6E, 0x8D, 0x05, 0x49, 0xF9, 0x5D, 0xDF, 0x80,
    0x99, 0x0F, 0xEA, 0x64, 0x49, 0xD0, 0xF2, 0x47,
    0x4A, 0x6C, 0x95, 0x77, 0x17, 0x3D, 0xE8, 0xD3,
    0x20, 0x08, 0x84, 0x17, 0x2B, 0xA8, 0x71, 0x96,
    0xDA, 0xC6, 0x93, 0x91, 0x84, 0x4D, 0xB8, 0x09,
    0x40, 0xC7, 0xA7, 0x42
};

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

    send_hdr.wsmp_hdr.channel_id = true;
    send_hdr.wsmp_hdr.datarate = true;
    send_hdr.wsmp_hdr.psid = PSID;
    send_hdr.wsmp_hdr.tx_power = true;

    send_hdr.security.sign_info.psid = PSID;
    send_hdr.security.sign_info.sign_method = CMS_SIGN_METH_SIGN_CERT;
    send_hdr.security.payload_type = CMS_SEC_DOT2_TX_PAYLOAD_TYPE_EXT_DOT2_DATA;

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

