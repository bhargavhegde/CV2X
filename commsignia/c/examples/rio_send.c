/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <cms_v2x/api.h>
#include <cms_v2x/radio.h>

/** @file
@brief Send a radio message.
@ingroup ex
*/


static void fill_send_data(cms_rio_send_data_t* send_data, cms_rio_packet_t packet_type)
{
    /* Radio parameters */
    send_data->radio_params.datarate = 6000U;
    send_data->radio_params.expiry_time = 1000U;
    send_data->radio_params.interface_id = 1U;
    send_data->radio_params.sps_index = 0U;
    send_data->radio_params.tx_power = 20;
    send_data->radio_params.user_prio = 1U;

    /* use broadcast address */
    memset(send_data->radio_params.dest_address, 0xFF, sizeof(send_data->radio_params.dest_address));

    /* Packet type */
    send_data->packet_type = packet_type;
}

static void log_rio_send_result(cms_rio_result_t result, const cms_rio_send_data_t* send_data)
{
    switch(result) {
    case CMS_RIO_RESULT_OK:
        printf("Radio message sent\n");
        break;
    case CMS_RIO_RESULT_NO_IFACE_ERROR:
        printf("Unable to get radio device on interface %u.\n", send_data->radio_params.interface_id);
        break;
    case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
        printf("Interface %u is disabled.\n", send_data->radio_params.interface_id);
        break;
    default:
        fprintf(stderr, "Cannot send out radio message, error code: %d\n", (int)result);
        break;
    }
}

static uint32_t l_read_file(const char* path, uint8_t* buf, size_t size)
{
    uint32_t result = 0UL;
    FILE* f = fopen(path, "rb");
    if(NULL != f) {
        size_t res = fread(buf, 1, size, f);
        if((res > 0) && (res <= size)) {
            result = res;
        } else {
            fprintf(stderr, "Invalid read %lu\n", (unsigned long)res);
        }
        /* check the whole file was read */
        if(!feof(f)) {
            fprintf(stderr,
                    "Could not read the whole input file into a buffer with %lu bytes\n",
                    (unsigned long)size);
            result = 0UL;
        }
        if(0 != fclose(f)) {
            fprintf(stderr, "Cannot close %s - %s\n", path, strerror(errno));
            result = 0UL;
        }
    } else {
        fprintf(stderr, "Cannot open %s - %s\n", path, strerror(errno));
    }
    return result;
}

enum { MAX_PAYLOAD_SIZE = 1500 };

int main(int argc, char* argv[])
{
    cms_rio_packet_t packet_type = (argc > 1) ?
                                   (cms_rio_packet_t)atoi(argv[1]) :
                                   CMS_RIO_GNP_PACKET;
    const char* host = (argc > 2) ? argv[2] : "127.0.0.1";
    const char* input_file = (argc > 3) ? argv[3] : NULL;
    uint8_t payload_buf[MAX_PAYLOAD_SIZE] = {0};
    uint32_t payload_length = 0UL;

    bool error = false;

    if(NULL != input_file) {
        payload_length = l_read_file(input_file, payload_buf, sizeof(payload_buf));
        if(0 == payload_length) {
            fprintf(stderr, "Cannot read %s\n", input_file);
            error = true;
        }
    } else {
        static const uint8_t DEFAULT_MSG_PAYLOAD[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        memcpy(payload_buf, DEFAULT_MSG_PAYLOAD, sizeof(DEFAULT_MSG_PAYLOAD));
        payload_length = sizeof(DEFAULT_MSG_PAYLOAD);
    }


    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    error = error || cms_api_connect_easy(&session, host);

    if(!error) {

        /* Create send header */
        cms_rio_send_data_t send_data = {0};
        fill_send_data(&send_data, packet_type);

        /* Create a buffer view as a handle to the actual payload buffer */
        cms_buffer_view_t payload = {
            .data = payload_buf,
            .length = payload_length
        };

        /* Send radio message with the given data and payload */
        cms_rio_result_t result = cms_rio_send(&session, &send_data, payload, NULL);
        log_rio_send_result(result, &send_data);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
