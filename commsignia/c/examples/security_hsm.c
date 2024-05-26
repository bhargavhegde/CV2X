/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <cms_v2x/api.h>
#include <cms_v2x/security.h>
#include <cms_v2x/security_types.h>

/** @file
@brief Use the Hardware Security Module (HSM) via the Commsignia software stack.
@ingroup ex
*/

static inline void print_scalar(const cms_sec_scalar_t scalar)
{
    if(NULL != scalar) {
        for(uint32_t i = 0UL; i < sizeof(cms_sec_scalar_t); ++i) {
            printf("%02X", scalar[i]);
        }
    }
}

static inline void print_public_key(const cms_sec_pubkey_t* public_key)
{
    if(NULL != public_key) {
        printf("  Public Key:\n");
        printf("  ECC curve type: %d\n", public_key->curve);
        printf("  Point compression type: %d\n", public_key->type);
        printf("  X coordinate:\n    ");
        print_scalar(public_key->x);
        printf("\n  Y coordinate:\n    ");
        print_scalar(public_key->y);
        printf("\n\n");
    }
}

static inline void print_key_info(const cms_sec_hsm_key_info_t* key_info)
{
    if(NULL != key_info) {
        printf("  Key info:\n");
        printf("    ECC curve type of key: %d\n", key_info->curve);
        printf("    Private key type: %d\n", key_info->private_key_type);
        printf("    Public key type: %d\n", key_info->public_key_type);
    }
}

static inline void print_signature(cms_sec_signature_t* signature)
{
    if(NULL != signature) {
        printf("  Point compression type: %d\n", signature->r_type);
        printf("  X xoordinate of R point:\n    ");
        print_scalar(signature->r_x);
        printf("\n  Scalar part of signature:\n    ");
        print_scalar(signature->s);
        printf("\n\n");
    }
}


static bool do_generate_key(cms_session_t* session, uint32_t key_index)
{
    bool error = false;

    /* Generate private key */
    cms_sec_hsm_generate_key_in_t generate_key_in = {
        .slot_info.slot_index = key_index,
        .key_info.curve = CMS_SEC_CURVE_NIST_P256,
        .key_info.private_key_type = CMS_SEC_PRIVATE_KEY_TYPE_PSEUDO,
        .key_info.public_key_type = CMS_SEC_PUBLIC_KEY_TYPE_SIGN
    };
    cms_sec_hsm_generate_key_result_t generate_key_result = {0};
    cms_sec_hsm_result_t res = cms_sec_hsm_generate_key(session, &generate_key_in, &generate_key_result);

    if(CMS_SEC_HSM_RES_OK != res) {
        printf("Unable to generate new private key\n");
        error = true;
    } else {

        printf("Generate private key result:\n");
        printf("  HSM slot index: %lu\n", (unsigned long)generate_key_result.slot_index);
        print_public_key(&generate_key_result.public_key);
    }

    return error;
}


static bool do_get_pubkey(cms_session_t* session, uint32_t key_index)
{
    bool error = false;

    /* Get public key of the generated private key */
    cms_sec_hsm_get_pubkey_in_t get_pubkey_in = {
        .slot_index = key_index,
    };
    cms_sec_hsm_get_pubkey_result_t get_pubkey_result = {0};
    cms_sec_hsm_result_t res = cms_sec_hsm_get_pubkey(session, &get_pubkey_in, &get_pubkey_result);

    if(CMS_SEC_HSM_RES_OK != res) {
        printf("Unable to get public key\n");
        error = true;
    } else {

        printf("Get public key result:\n");
        print_key_info(&get_pubkey_result.key_info);
        print_public_key(&get_pubkey_result.public_key);
    }

    return error;
}

static bool do_sign(cms_session_t* session, uint32_t key_index)
{
    bool error = false;

    /* Sign hash with generated private key */
    cms_sec_hsm_sign_in_t sign_in = {
        .slot_index = key_index,
        .hash.algo = CMS_SEC_HASH_SHA_256,
        /* First 16 bytes are padded with zeroes as the curve type is 256 bit long */
        .hash.value = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
            0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
            0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
            0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02
        }
    };

    cms_sec_hsm_sign_result_t sign_result = {0};
    cms_sec_hsm_result_t res = cms_sec_hsm_sign(session, &sign_in, &sign_result);

    if(CMS_SEC_HSM_RES_OK != res) {
        printf("Unable to sign hash with key #%ul\n", key_index);
        error = true;
    } else {

        printf("Sign hash with private key #%ul result:\n", key_index);
        print_signature(&sign_result.signature);
    }

    return error;
}


static bool do_import_key(cms_session_t* session, uint32_t key_index)
{
    bool error = false;

    /* Import private key */
    cms_sec_hsm_import_key_in_t import_key_in = {
        .slot_info.slot_index = key_index,
        .key_info.curve = CMS_SEC_CURVE_NIST_P256,
        .key_info.private_key_type = CMS_SEC_PRIVATE_KEY_TYPE_STANDALONE,
        .key_info.public_key_type = CMS_SEC_PUBLIC_KEY_TYPE_SIGN,
        /* First 16 bytes are padded with zeroes as the curve type is 256 bit long */
        .privkey = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
        }
    };

    cms_sec_hsm_import_key_result_t import_key_result = {0};
    cms_sec_hsm_result_t res = cms_sec_hsm_import_key(session, &import_key_in, &import_key_result);

    if(CMS_SEC_HSM_RES_OK != res) {
        printf("Unable to import private key\n");
        error = true;
    } else {

        printf("Import private key result:\n");
        printf("  HSM slot index: %lu\n", (unsigned long)import_key_result.slot_index);
        print_public_key(&import_key_result.public_key);
    }

    return error;
}


static bool do_private_multiply_add(cms_session_t* session)
{
    bool error = false;

    cms_sec_hsm_private_multiply_add_in_t priv_mul_add_in = {
        .source_slot  = 0,
        .destination_slot = {
            .slot_index = CMS_SEC_HSM_SLOT_INDEX_NEXT_FREE,
            .slot_range = CMS_SEC_HSM_SLOT_RANGE_VOLATILE,
        },
    };

    cms_sec_hsm_private_multiply_add_result_t priv_mul_add_result = {0};

    cms_sec_hsm_result_t hsm_result = cms_sec_hsm_private_multiply_add(session,
                                                                       &priv_mul_add_in,
                                                                       &priv_mul_add_result);

    if(CMS_SEC_HSM_RES_OK != hsm_result) {
        fprintf(stderr, "Unable to private multiply-add\n");
        error = true;
    } else {
        printf("Successfully private multiply-add. Slot index: %u\n",
               priv_mul_add_result.slot_index);
    }

    return error;
}


static bool do_ecdh(cms_session_t* session)
{
    bool error = false;

    cms_sec_hsm_ecdh_in_t ecdh_in = {
        .slot_index = 0,
        .public_key = {
            .curve = CMS_SEC_CURVE_NIST_P256,
            .type  = CMS_SEC_POINT_COMPRESSED_Y0,
            .x = {
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x84, 0x7D, 0x8E, 0x79, 0x50, 0xFD, 0xED, 0x5F,
                0x43, 0x98, 0xA9, 0xDE, 0x6C, 0x95, 0x7B, 0xA5,
                0xE7, 0xAB, 0x8D, 0xA6, 0xB6, 0x25, 0x4D, 0x98,
                0xF9, 0xC4, 0x3E, 0xA6, 0x84, 0x57, 0x99, 0xF8,
            },
        },
        .kdf_params = {
            .type = CMS_SEC_ECDH_KDF2,
            .p1 = {
                .algo = CMS_SEC_HASH_SHA_256,
                .value = {
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
                    0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
                    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
                    0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
                },
            }
        },
        .key_length = CMS_SEC_ECDH_KEY_LEN,
    };

    cms_sec_hsm_ecdh_result_t ecdh_result = {0};

    cms_sec_hsm_result_t hsm_result = cms_sec_hsm_ecdh(session,
                                                       &ecdh_in,
                                                       &ecdh_result);

    if(CMS_SEC_HSM_RES_OK != hsm_result) {
        fprintf(stderr, "Unable to ECDH key derive\n");
        error = true;
    } else {
        printf("ECDH key derived\n  length: %u\n  key:    ",
               ecdh_result.derived_key.length);

        for(uint32_t i = 0; i < ecdh_result.derived_key.length; ++i) {
            printf("%.2X", ecdh_result.derived_key.key[i]);
        }
        printf("\n");
    }

    return error;
}


static bool do_put_slot(cms_session_t* session)
{
    bool error = false;

    cms_sec_hsm_put_slot_in_t put_slot_in = {
        .slot_index = 42
    };

    cms_sec_hsm_result_t hsm_result = cms_sec_hsm_put_slot(session,
                                                           &put_slot_in,
                                                           NULL);

    if(CMS_SEC_HSM_RES_OK != hsm_result) {
        fprintf(stderr,
                "Unable to put volatile slot with id:%u\n",
                put_slot_in.slot_index);
        error = true;
    } else {
        fprintf(stderr,
                "Successfully put volatile slot\n");
    }

    return error;
}


static bool do_is_hw_hsm(cms_session_t* session)
{
    bool error = false;

    cms_sec_hsm_is_hw_result_t is_hw_result = {0};

    cms_sec_hsm_result_t hsm_result = cms_sec_hsm_is_hw(session,
                                                        NULL,
                                                        &is_hw_result);

    if(CMS_SEC_HSM_RES_OK != hsm_result) {
        fprintf(stderr,
                "Unable to read hsm type\n");
        error = true;
    } else {
        fprintf(stderr,
                "Successfully get hsm type: %s\n",
                (is_hw_result.hsm_is_hw == true) ? "HW" : "SW");
    }

    return error;
}

int main(int argc, char* argv[])
{
    if(argc < 3) {
        fprintf(stderr, "Usage: %s <Software stack IP address> "
                "<HSM slot index for key generation> "
                "[HSM slot index for key import]\n", argv[0]);
        return 1;
    }

    const char* host = argv[1];
    uint32_t generate_key_index = strtoul(argv[2], NULL, 0);
    uint32_t import_key_index = generate_key_index;
    if(argc > 3) {
        import_key_index = strtoul(argv[3], NULL, 0);
    }

    /* You can enable verbose logging with uncommenting the next lines */
    /* cms_api_init(); */
    /* cms_api_set_logging(CMS_API_LOG_INFO); */

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Generate private key */
    error = error || do_generate_key(&session, generate_key_index);
    /* Get public key of the generated private key */
    error = error || do_get_pubkey(&session, generate_key_index);
    /* Sign hash with generated private key */
    error = error || do_sign(&session, generate_key_index);

    /* Import private key */
    error = error || do_import_key(&session, import_key_index);
    /* Sign hash with imported private key */
    error = error || do_sign(&session, import_key_index);

    /* Private multiply-add */
    error = error || do_private_multiply_add(&session);
    /* ECDH key derivation */
    error = error || do_ecdh(&session);
    /* Put volatile slot */
    error = error || do_put_slot(&session);
    /* Check HSM type */
    error = error || do_is_hw_hsm(&session);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

