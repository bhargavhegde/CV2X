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
#include <cms_v2x/dsmp.h>

/** @file
@brief Sends a DSMP message.
@ingroup ex
*/

static void l_fill_radio_header(cms_radio_tx_params_t* radio);

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Send DSMP message*/
    if(!error) {
        /* Send a DSMP message */
        cms_dsmp_send_data_t send_data = {0};

        /*
        The AID value must match in the DSMP and security header.
        AID is called PSID in the security header.
        This value is not p-encoded.
        */
        static const cms_psid_t EXAMPLE_AID = 0x6F;

        send_data.dsmp_hdr.aid = EXAMPLE_AID;

        send_data.security.sign_info.psid = EXAMPLE_AID;
        send_data.security.sign_info.sign_method = CMS_SIGN_METH_SIGN;

        l_fill_radio_header(&send_data.radio);

        /* Construct dummy payload */
        uint8_t payload[10] = {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        };

        cms_buffer_view_t message = {
            .data = payload,
            .length = sizeof(payload),
        };

        error = cms_dsmp_send(&session, &send_data, message, NULL);
        if(error) {
            printf("Failed to send DSMP message\n");
        } else {
            printf("DSMP message sent successfully\n");
        }
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

void l_fill_radio_header(cms_radio_tx_params_t* radio)
{
    static const uint8_t BROADCAST_ADDR[CMS_MAC_ADDRESS_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    /* Radio parameters */
    memcpy(radio->dest_address, BROADCAST_ADDR, CMS_MAC_ADDRESS_LENGTH);
    radio->datarate = CMS_DATARATE_NA;
    radio->expiry_time = 1000U;
    radio->interface_id = 1U;
    radio->sps_index = 0U;
    radio->tx_power = CMS_POWER_LEVEL_NA;
    radio->user_prio = 1U;
}
