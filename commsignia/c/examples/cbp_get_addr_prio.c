/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cbp.h>

/** @file
@brief Get the priority of up to 128 stations inside the Context Based Prioritisation module
@ingroup ex
*/

static bool l_parse_args(int argc, char* argv[], cms_cbp_address_list_t* addrs, char** host);
static bool l_print_result(const cms_cbp_addr_prio_list_t* prios);
static void l_print_addr_prio(cms_cbp_addr_prio_item_t item);
static void l_usage(char* name);
static bool l_str_to_mac(const char* str, cms_mac_addr_t mac);
static bool l_hex_to_u8(const char* hex, uint8_t* u8);

int main(int argc, char* argv[])
{
    /* Parse command line arguments */
    char* host;
    cms_cbp_address_list_t addrs = {0};

    bool error = l_parse_args(argc, argv, &addrs, &host);

    /* Create sessions */
    cms_session_t session = cms_get_session();
    error = error || cms_api_connect(&session, host, 2U, NULL, NULL);

    /* Get address priorities */
    cms_cbp_addr_prio_list_t prios = {0};
    error = error || cms_cbp_get_addr_prio(&session, &addrs, &prios);
    error = error || l_print_result(&prios);

    cms_api_disconnect(&session);

    return error ? 1 : 0;
}

static bool l_parse_args(int argc, char* argv[], cms_cbp_address_list_t* addrs, char** host)
{
    bool error = (NULL == argv) || (NULL == addrs) || (NULL == host);

    if((!error) && ((argc < 2) || ((unsigned int)argc > (CMS_CBP_ADDR_LIST_MAX_SIZE + 2)))) {
        l_usage(argv[0]);
        error = true;
    }

    if(!error) {
        *host = argv[1];
        addrs->length = 0;
        for(int i_argv = 2; i_argv < argc; i_argv++) {
            int i_addr = i_argv - 2;
            if(!l_str_to_mac(argv[i_argv], addrs->addresses[i_addr])) {
                addrs->length++;
            } else {
                printf("Invalid MAC address: %s\n", argv[i_argv]);
                error = true;
                break;
            }
        }
    }

    return error;
}

static void l_print_addr_prio(cms_cbp_addr_prio_item_t item)
{
    printf("\"" CMS_MAC_ADDR_FMT "\": ", CMS_MAC_ADDR_FMT_ARGS(item.src_addr));

    if(CMS_CBP_PRIO_NA == item.prio) {
        printf("null");
    } else {
        printf("%d", item.prio);
    }
}

static bool l_print_result(const cms_cbp_addr_prio_list_t* prios)
{
    bool error = (NULL == prios);

    if(!error) {
        printf("{");
        if(prios->length > 0) {
            l_print_addr_prio(prios->items[0]);

            for(size_t i = 1; i < prios->length; i++) {
                printf(", ");
                l_print_addr_prio(prios->items[i]);
            }
        }
        printf("}\n");
    }

    return error;
}

static void l_usage(char* name)
{
    assert(name != NULL);
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "\t%s [MAC addresses]\n\n", name);
    fprintf(stderr, "MAC addresses shall be in 0123456789AB or 01:23:45:67:89:AB format. They are\n");
    fprintf(stderr, "not case sensitive. There shall be at most %d address in a single request.\n",
            CMS_CBP_ADDR_LIST_MAX_SIZE);
}

bool l_str_to_mac(const char* str, cms_mac_addr_t mac)
{
    bool error = (NULL == str) || (NULL == mac);

    if(!error && (12 == strlen(str))) {
        for(size_t i = 0; i < 6; i++) {
            if(l_hex_to_u8(&str[2 * i], &mac[i])) {
                error = true;
                break;
            }
        }
    } else if(!error && (17 == strlen(str))) {
        for(size_t i = 0; i < 6; i++) {
            if(l_hex_to_u8(&str[3 * i], &mac[i])) {
                error = true;
                break;
            }
        }
        for(size_t i = 0; i < 5; i++) {
            if(str[2 + 3 * i] != ':') {
                error = true;
                break;
            }
        }
    } else {
        error = true;
    }

    return error;
}

static bool l_hex_to_u8(const char* hex, uint8_t* u8)
{
    bool error = (NULL == hex) || (NULL == u8);

    if(!error) {
        *u8 = 0;
        for(size_t i = 0; i < 2; i++) {
            *u8 <<= 4;
            if('0' <= hex[i] && hex[i] <= '9') {
                *u8 += (hex[i] - '0');
            } else if('a' <= hex[i] && hex[i] <= 'f') {
                *u8 += (hex[i] - 'a' + 10);
            } else if('A' <= hex[i] && hex[i] <= 'F') {
                *u8 += (hex[i] - 'A' + 10);
            } else {
                error = true;
                break;
            }
        }
    }

    return error;
}
