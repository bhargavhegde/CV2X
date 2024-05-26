/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2021-2023
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <cms_v2x/api.h>
#include <cms_v2x/radio.h>

/** @file
@brief Inject a radio packet  with the given parameters.
Injected packets are handled the same way by
facility and application layer as packets received on real radio.
This api can be useful for testing.
@ingroup ex
*/


static const uint8_t IVI_PAYLOAD[] = {
    0x11, 0x00, 0x4C, 0x01, 0x20, 0x50, 0x02, 0x80,
    0x00, 0x75, 0x01, 0x00, 0xBC, 0x00, 0x11, 0x22,
    0x33, 0x44, 0x55, 0x66, 0x73, 0xD8, 0x12, 0x48,
    0x1C, 0x95, 0xA1, 0x99, 0x0D, 0x35, 0xAD, 0x3B,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xD6, 0x00, 0x00, 0x02, 0x06, 0x00, 0x40,
    0x3F, 0x86, 0xA1, 0x5C, 0x0A, 0x68, 0x00, 0xA4,
    0x7C, 0x73, 0xD8, 0x12, 0x48, 0x02, 0x01, 0x48,
    0xEA, 0x2A, 0x65, 0xE1, 0xFD, 0xFC, 0xEF, 0xFF,
    0xFF, 0xFF, 0x84, 0x46, 0xC6, 0x63, 0xC0, 0x10,
    0x04, 0x02, 0xFF, 0xFF, 0xBF, 0xFF, 0xF3, 0xF6,
    0x47, 0xB1, 0xE5, 0xF4, 0x08, 0x06, 0x00, 0x80,
    0x01, 0x02, 0x01, 0x18, 0x94, 0x17, 0x82, 0xA9,
    0x78, 0x72, 0xCA, 0xC8, 0xAA, 0x98, 0x71, 0x2A,
    0x69, 0x24, 0x0A, 0x98, 0x70, 0x2A, 0xC9, 0xE9,
    0x8A, 0x78, 0x70, 0x28, 0xF1, 0x03, 0x06, 0xA5,
    0xA2, 0xA2, 0xA8, 0x10, 0x22, 0x24, 0xA9, 0xAA,
    0x20, 0xA7, 0x21, 0xA2, 0xC4, 0xA0, 0x1C, 0x82,
    0x84, 0xA6, 0xA8, 0x82, 0x9C, 0x88, 0x40, 0x90,
    0x82, 0x98, 0xA8, 0x8A, 0x9C,
};


static void fill_data(cms_rio_rx_notif_data_t* data, cms_rio_packet_t packet_type)
{
    /* Radio parameters */
    data->radio_params.datarate = 6000U;
    data->radio_params.timestamp = 1629891534000U;
    data->radio_params.interface_id = 1U;
    data->radio_params.rssi = -40;
    data->radio_params.user_prio = 1U;

    /* Packet type */
    data->packet_type = packet_type;
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
        memcpy(payload_buf, IVI_PAYLOAD, sizeof(IVI_PAYLOAD));
        payload_length = sizeof(IVI_PAYLOAD);
    }

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    error = error || cms_api_connect_easy(&session, host);

    if(!error) {

        cms_rio_rx_notif_data_t inject_data = {0};
        fill_data(&inject_data, packet_type);

        /* Create a buffer view as a handle to the actual payload buffer */
        cms_buffer_view_t payload = {
            .data = payload_buf,
            .length = payload_length
        };

        /* Send radio message with the given data and payload */
        error = cms_rio_inject(&session, &inject_data, payload, NULL);
        if(error) {
            printf("Unable to inject data\n");
        } else {
            printf("Inject was successful\n");
        }
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
