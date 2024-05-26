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
#include <cms_v2x/security.h>

/** @file
@brief Get raw certificate.
@ingroup ex
*/

static cms_hashid8_t SIGNER2 = 0xFA506EDAFD42CB44ULL;

/* write binary buffer to stdout in hex */
bool l_write_hex_to_stdout(const uint8_t* buf, uint32_t length)
{
    bool error = (NULL == buf);

    if(!error) {
        for(uint32_t i = 0UL; i < length; ++i) {
            printf("%.2X", buf[i]);
        }
        printf("\n");
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

    /* Set the hashid8 of the certificate to get */
    cms_sec_get_raw_cert_specifier_t specifier = {
        .std = CMS_STD_TYPE_WAVE,
        .hashid8 = SIGNER2
    };

    /* Create a mutable buffer for the certificate to be queried */
    uint8_t data[1500] = {0};
    cms_mutable_buffer_t raw_cert = {
        .data = data,
        .length = 0UL,
        .capacity = sizeof(data)
    };

    /* Create output parameter for the query */
    cms_sec_get_raw_cert_out_t raw_cert_out = {0};

    cms_sec_get_raw_cert_result_t result = cms_sec_get_raw_cert(&session, &specifier, &raw_cert_out, &raw_cert);
    switch(result) {
    case CMS_SEC_GET_RAW_CERT_RES_OK:
        fprintf(stderr, "Raw certificate queried, state: %d\n", raw_cert_out.cert_state);
        if(l_write_hex_to_stdout(raw_cert.data, raw_cert.length)) {
            fprintf(stderr, "Cannot print raw cert");
            error = true;
        }
        break;
    case CMS_SEC_GET_RAW_CERT_RES_NOT_EXIST:
        fprintf(stderr, "Certificate not found\n");
        error = true;
        break;
    case CMS_SEC_GET_RAW_CERT_RES_MISSING_RAW_CERT:
        fprintf(stderr, "Certificate found, but raw certificate is not stored, state: %d\n", raw_cert_out.cert_state);
        error = true;
        break;
    case CMS_SEC_GET_RAW_CERT_RES_GENERAL_ERROR:
    default:
        fprintf(stderr, "Unable to get raw certificate\n");
        error = true;
        break;
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

