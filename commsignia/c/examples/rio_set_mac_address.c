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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include <cms_v2x/api.h>
#include <cms_v2x/radio.h>


/** @file
@brief Set MAC address of the radio interfaces
@ingroup ex
*/

static void print_mac_address(const cms_rio_mac_setup_t* mac_setup);
static bool is_id_format_valid(const char* id_str);
static bool is_mac_format_valid(const char* mac_str);
static uint32_t string_to_id(const char* str);
static void string_to_mac(const char* str, cms_mac_addr_t mac);

static void set_mac_address(cms_session_t* session,
                            cms_interface_id_t interface_id,
                            cms_mac_addr_t mac_address)
{
    cms_rio_mac_setup_t mac_setup = {
        .interface_id = interface_id,
        .mac_address = {0},
    };

    memcpy(mac_setup.mac_address, mac_address, sizeof(cms_mac_addr_t));

    cms_rio_result_t error_code = cms_rio_set_mac_address(session, &mac_setup, NULL);

    switch(error_code) {
    case CMS_RIO_RESULT_OK:
        print_mac_address(&mac_setup);
        break;
    case CMS_RIO_RESULT_NO_IFACE_ERROR:
        printf("Unable to set radio device on interface %u.\n", interface_id);
        break;
    case CMS_RIO_RESULT_DISABLED_IFACE_ERROR:
        break;
    default:
        fprintf(stderr, "Cannot set mac address for interface %lu, error code: %d\n",
                (unsigned long)interface_id, (int)error_code);
        break;
    }
}

int main(int argc, char* argv[])
{
    if((argc < 3) || (4 < argc)) {
        printf("\nUsage:\trio_set_mac_address [HOST] <ID> <MAC>\n\n"
               "HOST is optional, default value is 127.0.0.1\n"
               "ID   is the interface identifier\n"
               "MAC  is the new MAC address (XX:XX:XX:XX:XX:XX)\n\n");
        return 1;
    }

    const char* host = (4 == argc) ? argv[1] : "127.0.0.1";
    const char* id_str = (4 == argc) ? argv[2] : argv[1];
    const char* mac_str = (4 == argc) ? argv[3] : argv[2];

    if(!is_id_format_valid(id_str)) {
        printf("Invalid ID format, use decimal digits only\n");
        return 1;
    }

    if(!is_mac_format_valid(mac_str)) {
        printf("Invalid MAC address format, use XX:XX:XX:XX:XX:XX\n");
        return 1;
    }

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    if(!error) {

        uint32_t interface_id = string_to_id(id_str);

        cms_mac_addr_t mac_address = {0};
        string_to_mac(mac_str, mac_address);

        set_mac_address(&session, interface_id, mac_address);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

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

static inline bool is_digit(char c)
{
    return 0 != isdigit(c);
}

static inline bool is_hexa_digit(char c)
{
    return 0 != isxdigit(c);
}

static inline bool is_colon(char c)
{
    return (':' == c);
}

static bool is_id_format_valid(const char* id_str)
{
    bool is_valid = true;
    for(size_t i = 0; is_valid && (i < strlen(id_str)); ++i) {
        is_valid = is_digit(id_str[i]);
    }

    return is_valid;
}

static bool is_mac_format_valid(const char* mac_str)
{
    const size_t expected_len = strlen("XX:XX:XX:XX:XX:XX");
    bool is_valid = (expected_len == strlen(mac_str));

    for(size_t i = 0; is_valid && (i < expected_len); i += 3) {
        is_valid = is_valid && is_hexa_digit(mac_str[i]);
        is_valid = is_valid && is_hexa_digit(mac_str[i + 1]);

        // The last octet is not terminated with a colon, skip it.
        if((i + 2) < expected_len) {
            is_valid = is_valid && is_colon(mac_str[i + 2]);
        }
    }

    return is_valid;
}

static uint32_t string_to_id(const char* str)
{
    return strtoul(str, NULL, 10);
}

static void string_to_mac(const char* str, cms_mac_addr_t mac)
{
    static const size_t octet_count_in_mac = 6U;
    for(size_t i = 0; i < octet_count_in_mac; ++i) {
        static char* end = NULL;
        const char* begin = (NULL == end) ? str : end;
        mac[i] = strtoul(begin, &end, 16);
        ++end;
    }
}