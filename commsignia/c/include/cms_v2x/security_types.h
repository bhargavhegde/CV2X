/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_SECURITY_TYPES_H_
#define CMS_V2X_SECURITY_TYPES_H_

/** @file
@brief Security data types
*/

#include <stdint.h>
#include <string.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup sec_types Security data types
@ingroup sec
@note Experimental features.
*/

/**
256 bit scalar value size in bytes used by cryptographic functions.
* e.g NISTP-256, SM2, Brainpool256
@ingroup sec_types
*/
#define CMS_SEC_256_SCALAR_LEN              32U

/**
384 bit scalar value size in bytes used by cryptographic functions.
eg. NISTP-384, Brainpool384T1, Brainpool384R1
@ingroup sec_types
*/
#define CMS_SEC_384_SCALAR_LEN              48U

/**
Scalar value size in bytes used by cryptographic functions.
@ingroup sec_types
*/
#define CMS_SEC_MAX_SCALAR_LEN              CMS_SEC_384_SCALAR_LEN

/**
Scalar value size in bytes used by cryptographic functions.
@ingroup sec_types
Use network byte order, MSB first.
Padded by 16 leading zero bytes for 32 byte long (256 bit) scalars.
*/
typedef uint8_t cms_sec_scalar_t[CMS_SEC_MAX_SCALAR_LEN];

/**
AES-KEY value size in bytes used by cryptographic functions.
@ingroup sec_types
*/
#define CMS_SEC_AES_KEY_LEN                  16U

/**
AES-KEY
@ingroup sec_types
Use network byte order, MSB first
*/
typedef uint8_t cms_sec_aes_key_t[CMS_SEC_AES_KEY_LEN];

/**
Supported Elliptic Curves.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_CURVE_NIST_P256 = 0,            /**< NISTp256 ECC curve */
    CMS_SEC_CURVE_NIST_P384 = 1,            /**< NISTp384 ECC curve */
    CMS_SEC_CURVE_BRAINPOOL_P256_T1 = 2,    /**< Brainpoolp256T1 ECC curve */
    CMS_SEC_CURVE_BRAINPOOL_P256_R1 = 3,    /**< Brainpoolp256R1 ECC curve */
    CMS_SEC_CURVE_BRAINPOOL_P384_T1 = 4,    /**< Brainpoolp384T1 ECC curve */
    CMS_SEC_CURVE_BRAINPOOL_P384_R1 = 5,    /**< Brainpoolp384R1 ECC curve */
    CMS_SEC_CURVE_SM2 = 6,                  /**< Chinese SM2 256 bit ECC curve */
} cms_sec_curve_t;

/**
Convert an elliptic curve into a readable string.
@ingroup sec_types

@param  curve            Elliptic curve
@return readable string
@see cms_sec_curve_t
*/
static inline const char* cms_sec_curve_to_string(cms_sec_curve_t curve)
{
    const char* result = NULL;

    switch(curve) {
    case CMS_SEC_CURVE_NIST_P256:
        result = "NIST P256";
        break;
    case CMS_SEC_CURVE_NIST_P384:
        result = "NIST P384";
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P256_T1:
        result = "Brainpool P256 T1";
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P256_R1:
        result = "Brainpool P256 R1";
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P384_T1:
        result = "Brainpool P384 T1";
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P384_R1:
        result = "Brainpool P384 R1";
        break;
    case CMS_SEC_CURVE_SM2:
        result = "SM2";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Get key length for elliptic curve.
@ingroup sec_types

@param  curve                           Elliptic curve type
@return length of key in bytes
@see cms_sec_curve_t
*/
static inline uint32_t cms_sec_curve_get_key_length(cms_sec_curve_t curve)
{
    uint32_t key_length = 0UL;

    switch(curve) {
    case CMS_SEC_CURVE_NIST_P256:
        key_length = CMS_SEC_256_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_NIST_P384:
        key_length = CMS_SEC_384_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P256_T1:
        key_length = CMS_SEC_256_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P256_R1:
        key_length = CMS_SEC_256_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P384_T1:
        key_length = CMS_SEC_384_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_BRAINPOOL_P384_R1:
        key_length = CMS_SEC_384_SCALAR_LEN;
        break;
    case CMS_SEC_CURVE_SM2:
        key_length = CMS_SEC_256_SCALAR_LEN;
        break;
    default:
        key_length = 0UL;
        break;
    }

    return key_length;
}

/**
Add padding zeros for cms_sec_scalar_t
@ingroup sec_types
@experimentalapi

@param in          unpadded buffer
@param in_len      length of unpadded buffer
@param out          cms_sec_scalar with padding zeros
@return void
@see cms_sec_scalar_t
*/
static inline void cms_sec_pad(cms_sec_scalar_t out,
                               const uint8_t* in,
                               uint32_t in_len)
{
    if((0 != out) && (0 != in)) {
        uint32_t skip = sizeof(cms_sec_scalar_t) - in_len;
        memset(out, 0, skip);
        memcpy(out + skip, in, in_len);
    }
}

/**
return with useful payload, which does not contain padding
@ingroup sec_types
@experimentalapi

@param in          buffer which is padded with zeros
@param payload_length      length of unpadded payload
@return unpadded, useful payload
@see cms_sec_scalar_t
*/
static inline const uint8_t* cms_sec_unpad(const cms_sec_scalar_t in,
                                           uint32_t payload_length)
{
    const uint8_t* result = NULL;

    if((0 != in) && (sizeof(cms_sec_scalar_t) >= payload_length)) {
        uint32_t skip = sizeof(cms_sec_scalar_t) - payload_length;
        result = in + skip;
    }

    return result;
}

/**
SHA256 hash value size in bytes used by cryptographic functions.
@ingroup sec_types
*/
#define CMS_SEC_SHA256_LEN                  32U

/**
SHA384 hash value size in bytes used by cryptographic functions.
@ingroup sec_types
*/
#define CMS_SEC_SHA384_LEN                  48U

/**
SM3 hash value size in bytes used by cryptographic functions.
@ingroup sec_types
*/
#define CMS_SEC_SM3_LEN                     32U

/**
Supported hashing algorithms.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_HASH_SHA_256 = 0,       /**< SHA2-256 */
    CMS_SEC_HASH_SHA_384 = 1,       /**< SHA2-384 */
    CMS_SEC_HASH_SM3 = 2,           /**< Chinese SM3 */
} cms_sec_hash_algo_t;

/**
Convert a hashing algorithm into a readable string.
@ingroup sec_types

@param  hash_algo            Hashing algorithm
@return readable string
@see cms_sec_hash_algo_t
*/
static inline const char* cms_sec_hash_algo_to_string(cms_sec_hash_algo_t hash_algo)
{
    const char* result = NULL;

    switch(hash_algo) {
    case CMS_SEC_HASH_SHA_256:
        result = "SHA256";
        break;
    case CMS_SEC_HASH_SHA_384:
        result = "SHA384";
        break;
    case CMS_SEC_HASH_SM3:
        result = "SM3";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Cryptographic hash to be signed or verified by ECDSA.
@ingroup sec_types
*/
struct cms_sec_hash_t {
    cms_sec_hash_algo_t algo;       /**< Hash algorithm */
    cms_sec_scalar_t value;         /**< Hash value - MSB first, padded */
} CMS_PACKED;

/**
Get key length from hash algorithm.
@ingroup sec_types

@param  hash_algo               Hash algorithm
@return length of key in bytes
@see cms_sec_hash_algo_t
*/
static inline uint32_t cms_sec_keylen_from_hash_algo(const cms_sec_hash_algo_t hash_algo)
{
    uint32_t keylen = 0UL;

    switch(hash_algo) {
    case CMS_SEC_HASH_SHA_256:
    case CMS_SEC_HASH_SM3:
        keylen = CMS_SEC_256_SCALAR_LEN;
        break;
    case CMS_SEC_HASH_SHA_384:
        keylen = CMS_SEC_384_SCALAR_LEN;
        break;
    default:
        break;
    }

    return keylen;
}

/**
Convert public key elliptic curve to corresponding hash algorithm.
@ingroup sec
@experimentalapi

@param curve                Elliptic curve type
@param[out] hash_algo       Hash algorithm, SHA-256, SHA-384 or SM3
@return true in case of error
*/
static inline bool cms_sec_curve_to_hash_algo(cms_sec_curve_t curve,
                                              cms_sec_hash_algo_t* hash_algo)
{
    bool error = (NULL == hash_algo);

    if(!error) {
        switch(curve) {
        case CMS_SEC_CURVE_NIST_P256:
        case CMS_SEC_CURVE_BRAINPOOL_P256_T1:
        case CMS_SEC_CURVE_BRAINPOOL_P256_R1:
            *hash_algo = CMS_SEC_HASH_SHA_256;
            break;
        case CMS_SEC_CURVE_NIST_P384:
        case CMS_SEC_CURVE_BRAINPOOL_P384_T1:
        case CMS_SEC_CURVE_BRAINPOOL_P384_R1:
            *hash_algo = CMS_SEC_HASH_SHA_384;
            break;
        case CMS_SEC_CURVE_SM2:
            *hash_algo = CMS_SEC_HASH_SM3;
            break;
        default:
            error = true;
        }
    }

    return error;
}

/**
Compression type of and elliptic curve point.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_POINT_X_COORD_ONLY = 0,     /**< Only X coordinate of the point is specified */
    CMS_SEC_POINT_COMPRESSED_Y0 = 2,    /**< Y coordinate LSB is 0 */
    CMS_SEC_POINT_COMPRESSED_Y1 = 3,    /**< Y coordinate LSB is 1 */
    CMS_SEC_POINT_UNCOMPRESSED = 4,     /**< Both X and Y coordinates are specified */
} cms_sec_point_type_t;

/**
Convert an elliptic curve point compression type into a readable string.
@ingroup sec_types

@param  point_type            Elliptic curve point type
@return readable string
@see cms_sec_curve_t
*/
static inline const char* cms_sec_point_type_to_string(cms_sec_point_type_t point_type)
{
    const char* result = NULL;

    switch(point_type) {
    case CMS_SEC_POINT_X_COORD_ONLY:
        result = "X coordinate only";
        break;
    case CMS_SEC_POINT_COMPRESSED_Y0:
        result = "Compressed, Y coordinate LSB is 0";
        break;
    case CMS_SEC_POINT_COMPRESSED_Y1:
        result = "Compressed, Y coordinate LSB is 1";
        break;
    case CMS_SEC_POINT_UNCOMPRESSED:
        result = "Uncompressed";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Elliptic Curve signature.
@ingroup sec_types
ECDSA fast verification signature if `r_type` is compressed `y0` or `y1`.
`r_type` cannot be uncompressed.
*/
struct cms_sec_signature_t {
    cms_sec_point_type_t r_type;    /**< R point compression type, cannot be uncompressed */
    cms_sec_scalar_t r_x;           /**< X coordinate of R point */
    cms_sec_scalar_t s;             /**< Scalar part of signature */
} CMS_PACKED;

/**
Elliptic Curve point.
@ingroup sec_types
`y` field contains valid value only if `type` is uncompressed.
*/
struct cms_sec_pubkey_t {
    cms_sec_curve_t curve;          /**< Elliptic Curve type */
    cms_sec_point_type_t type;      /**< Point compression type */
    cms_sec_scalar_t x;             /**< X coordinate of the point */
    cms_sec_scalar_t y;             /**< Y coordinate - only if uncompressed */
} CMS_PACKED;

/**
Input for ECDSA signature verification.
@ingroup sec_types
*/
struct cms_sec_ecdsa_verify_input_t {
    cms_sec_hash_t hash;            /**< Hash to be verified */
    cms_sec_signature_t signature;  /**< Elliptic Curve Signature to be verified */
    cms_sec_pubkey_t pubkey;        /**< Public key to be verified with */
} CMS_PACKED;

/**
Result of an ECDSA operation.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_ECDSA_SUCCESS = 0,              /**< ECDSA operation was successful */
    CMS_SEC_ECDSA_TIMEOUT = 1,              /**< Internal timeout */
    CMS_SEC_ECDSA_FAILED = 2,               /**< Signature verification failed */
    CMS_SEC_ECDSA_DROPPED = 3,              /**< Request was dropped due to internal throttle limiting */
    CMS_SEC_ECDSA_INVALID_POINT = 4,        /**< Invalid input point, public key */
    CMS_SEC_ECDSA_DECOMPRESS_ERROR = 5,     /**< Error during point decompression */
    CMS_SEC_ECDSA_GENERIC_ERROR = 6,        /**< Other error */
} cms_sec_ecdsa_result_t;

/**
Convert a result of an ECDSA operation into a readable string.
@ingroup sec_types

@param  ecdsa_result            Result of an ECDSA operation
@return readable string
@see cms_sec_ecdsa_result_t
*/
static inline const char* cms_sec_ecdsa_result_to_string(cms_sec_ecdsa_result_t ecdsa_result)
{
    const char* result = NULL;

    switch(ecdsa_result) {
    case CMS_SEC_ECDSA_SUCCESS:
        result = "Success";
        break;
    case CMS_SEC_ECDSA_TIMEOUT:
        result = "Timeout";
        break;
    case CMS_SEC_ECDSA_FAILED:
        result = "Failed";
        break;
    case CMS_SEC_ECDSA_DROPPED:
        result = "Request dropped";
        break;
    case CMS_SEC_ECDSA_INVALID_POINT:
        result = "Invalid point";
        break;
    case CMS_SEC_ECDSA_DECOMPRESS_ERROR:
        result = "Decompress error";
        break;
    case CMS_SEC_ECDSA_GENERIC_ERROR:
        result = "Generic error";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Current verification state of a certificate.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_CERT_STATE_EMPTY = 0,           /**< Invalid, internal state */
    /* HASH_ID_ONLY - RESERVED */
    CMS_SEC_CERT_STATE_NOT_STARTED = 2,     /**< Certificate validation was not started yet */
    CMS_SEC_CERT_STATE_IN_PROGRESS = 3,     /**< Certificate validation is in progress */
    CMS_SEC_CERT_STATE_VERIFIED = 4,        /**< Certificate is successfully verified and valid */
    CMS_SEC_CERT_STATE_FAILED = 5,          /**< Certificate validation failed, and it is invalid */
} cms_sec_cert_state_t;

/**
Convert a verification state of a certificate into a readable string.
@ingroup sec_types

@param  cert_state              Verification state of a certificate
@return readable string
@see cms_sec_cert_state_t
*/
static inline const char* cms_sec_cert_state_to_string(cms_sec_cert_state_t cert_state)
{
    const char* result = NULL;

    switch(cert_state) {
    case CMS_SEC_CERT_STATE_EMPTY:
        result = "Empty - Invalid, internal state";
        break;
    case CMS_SEC_CERT_STATE_NOT_STARTED:
        result = "Not started";
        break;
    case CMS_SEC_CERT_STATE_IN_PROGRESS:
        result = "In progress";
        break;
    case CMS_SEC_CERT_STATE_VERIFIED:
        result = "Verified";
        break;
    case CMS_SEC_CERT_STATE_FAILED:
        result = "Verify failed";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Certificate type
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_CERT_TYPE_END_ENTITY_VERIFY = 0,/**< End Entity cert for message verification, not a signer cert */
    CMS_SEC_CERT_TYPE_END_ENTITY_SIGN = 1,  /**< End Entity cert for message signing, signer cert with known privatekey */
    CMS_SEC_CERT_TYPE_INTERMEDIATE = 2,     /**< Intermediate cert, CA cert that can issue certificates */
    CMS_SEC_CERT_TYPE_ROOT = 3,             /**< Root cert, Self signed CA cert that can issue certificates */
} cms_sec_cert_type_t;

/**
Convert a certificate type into a readable string.
@ingroup sec_types

@param  cert_type               Certificate type
@return readable string
@see cms_sec_cert_type_t
*/
static inline const char* cms_sec_cert_type_to_string(cms_sec_cert_type_t cert_type)
{
    const char* result = NULL;

    switch(cert_type) {
    case CMS_SEC_CERT_TYPE_END_ENTITY_VERIFY:
        result = "End entity verification";
        break;
    case CMS_SEC_CERT_TYPE_END_ENTITY_SIGN:
        result = "End entity signer";
        break;
    case CMS_SEC_CERT_TYPE_INTERMEDIATE:
        result = "Intermediate";
        break;
    case CMS_SEC_CERT_TYPE_ROOT:
        result = "Root";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Security verification result of a received packet.
@ingroup sec_types
*/
typedef enum {
    CMS_SEC_VERIFY_UNSECURED = 0,           /**< Unsecured message */
    CMS_SEC_VERIFY_VERIFIED = 1,            /**< Signed msg successfully verified */
    CMS_SEC_VERIFY_DISABLED = 2,            /**< Signed msg but security is disabled */
    CMS_SEC_VERIFY_TIMEOUT = 3,             /**< Signed msg but verification is aborted by timeout, possible overload */
    CMS_SEC_VERIFY_INVALID_SIGNATURE = 4,   /**< Signed msg but signature is invalid */
    CMS_SEC_VERIFY_INVALID_SIGNERID = 5,    /**< Signed msg but signer id is invalid (ie: self, self-signed cert) */
    CMS_SEC_VERIFY_INVALID_CERT = 6,        /**< Signed msg but signer certificate is not valid */
    CMS_SEC_VERIFY_INVALID_PSID = 7,        /**< Signed msg but PSID is invalid */
    CMS_SEC_VERIFY_INVALID_TIME = 8,        /**< Signed msg but generation time is invalid */
    CMS_SEC_VERIFY_INVALID_LOCATION = 9,    /**< Signed msg but generation location is invalid */
    CMS_SEC_VERIFY_INTERNAL_ERROR = 10,     /**< Verification process failed, with internal error */
    CMS_SEC_VERIFY_INVALID_HASHALGO = 11,   /**< Signed msg but the hash algo is invalid or does not match with the public key */
    CMS_SEC_VERIFY_MALFORMED_MESSAGE = 12,  /**< The message cannot be parsed */
    CMS_SEC_VERIFY_MALFORMED_INVALID_VERSION = 13,  /**< The secured message protocol version number is invalid */
    CMS_SEC_VERIFY_INVALID_PROFILE = 14,    /**< Signed message but invalid security profile for the PSID */
    CMS_SEC_VERIFY_REPLAY_DETECTED = 15     /**< Signed and replayed message */
} cms_sec_verify_result_t;

/**
Convert a message verification result into a readable string.
@ingroup sec_types

@param  verbose_result          Message verification result
@return readable string
@see cms_sec_verify_result_t
*/
static inline const char* cms_sec_verify_result_to_string(cms_sec_verify_result_t verbose_result)
{
    const char* result = NULL;

    switch(verbose_result) {
    case CMS_SEC_VERIFY_UNSECURED:
        result = "Unsecured";
        break;
    case CMS_SEC_VERIFY_VERIFIED:
        result = "Verified";
        break;
    case CMS_SEC_VERIFY_DISABLED:
        result = "Disabled";
        break;
    case CMS_SEC_VERIFY_TIMEOUT:
        result = "Timeout";
        break;
    case CMS_SEC_VERIFY_INVALID_SIGNATURE:
        result = "Invalid signature";
        break;
    case CMS_SEC_VERIFY_INVALID_SIGNERID:
        result = "Invalid signerID";
        break;
    case CMS_SEC_VERIFY_INVALID_CERT:
        result = "Invalid cert";
        break;
    case CMS_SEC_VERIFY_INVALID_PSID:
        result = "Invalid PSID";
        break;
    case CMS_SEC_VERIFY_INVALID_TIME:
        result = "Invalid time";
        break;
    case CMS_SEC_VERIFY_INVALID_LOCATION:
        result = "Invalid location";
        break;
    case CMS_SEC_VERIFY_INTERNAL_ERROR:
        result = "Internal error";
        break;
    case CMS_SEC_VERIFY_INVALID_HASHALGO:
        result = "Invalid Hashalgo";
        break;
    case CMS_SEC_VERIFY_MALFORMED_MESSAGE:
        result = "Malformed message";
        break;
    case CMS_SEC_VERIFY_MALFORMED_INVALID_VERSION:
        result = "Invalid version in security header";
        break;
    case CMS_SEC_VERIFY_INVALID_PROFILE:
        result = "Invalid profile";
        break;
    case CMS_SEC_VERIFY_REPLAY_DETECTED:
        result = "Replayed message";
        break;
    default:
        result = "Invalid";
        break;
    }
    return result;
}

/**
Certificate State
@ingroup sec
*/
typedef enum {
    CMS_SEC_CERT_SOURCE_LOADED = 0,             /**< Certificate is loaded with the API */
    CMS_SEC_CERT_SOURCE_RECEIVED_ON_RADIO = 1,  /**< Certificate received on the radio */
} cms_sec_cert_source_t;

/**
Convert a certificate source into a readable string.
@ingroup sec_types

@param  cert_source             Certificate source
@return readable string
@see cms_sec_cert_source_t
*/
static inline const char* cms_sec_cert_source_to_string(cms_sec_cert_source_t cert_source)
{
    const char* result = NULL;

    switch(cert_source) {
    case CMS_SEC_CERT_SOURCE_LOADED:
        result = "Loaded";
        break;
    case CMS_SEC_CERT_SOURCE_RECEIVED_ON_RADIO:
        result = "Received";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Service Specific Permissions (SSP) maximum length.
@ingroup sec_types
*/
#define CMS_SSP_MAX_LENGTH              31UL

/**
Service Specific Permissions (SSP) field.
@ingroup sec_types
*/
typedef uint8_t cms_ssp_field_t[CMS_SSP_MAX_LENGTH];

/**
Service Specific Permissions field type (SSP).
@ingroup sec_types
*/
typedef enum {
    CMS_SSP_TYPE_OPAQUE = 0,            /**< Opaque SSP: further encoding */
    CMS_SSP_TYPE_BITMAP = 1,            /**< Bitmap SSP: bitmask */
} cms_ssp_type_t;

/**
Service Specific Permissions (SSP).
@ingroup sec_types
*/
struct cms_ssp_t {
    cms_ssp_type_t ssp_type;
    cms_ssp_field_t ssp_field;          /**< SSP field */
    uint32_t length;                    /**< Length of SSP field in bytes */
} CMS_PACKED;

/**
HashId8
@ingroup sec_types
*/
typedef uint64_t cms_hashid8_t;

/**
Type of the payload received in the notification.
Do not use if not needed explicitly by the use case.
This type relates to IEEE 1609.2 security standard.
@ingroup sec_types
@note Experimental feature, it can change in the future.
*/
typedef enum {
    /**
     * The payload field is the bytes extracted from the innermost unsecuredData element.
     * This is the default value.
     */
    CMS_SEC_DOT2_RX_PAYLOAD_TYPE_RAW = 0,
    /**
     * The payload is an OER encoded Ieee1609Dot2Data element.
     * Only sent in advanced use case, such as externally encrypted data.
     * @note Experimental feature
     */
    CMS_SEC_DOT2_RX_PAYLOAD_TYPE_EXT_DOT2_DATA = 1,
    /**
     * The payload is invalid.
     * @note Experimental feature
     */
    CMS_SEC_DOT2_RX_PAYLOAD_TYPE_INVALID = 2,
} cms_sec_dot2_rx_payload_type_t;

/**
Security info parameters of a received packet.
This type relates to IEEE 1609.2 security standard.
@ingroup sec_types
*/
struct cms_sec_dot2_rx_info_t {
    cms_sec_verify_result_t verify_result;  /**< Security verification result of the received packet */
    cms_psid_t psid;                        /**< PSID/AID of the received packet */
    cms_ssp_t ssp;                          /**< Service Specific Permissions (SSP) @experimentalfield */
    cms_hashid8_t signer_id;                /**< Signer's HashId8 */
    /**
     * Type of the payload received in the notification.
     * Do not use if not needed explicitly by the use case.
     * @experimentalfield
     */
    cms_sec_dot2_rx_payload_type_t payload_type;
} CMS_PACKED;

/**
Signing method of a transmitted packet.
@ingroup sec_types
*/
typedef enum {
    CMS_SIGN_METH_NONE = 0,             /**< Do not sign message */
    CMS_SIGN_METH_SIGN = 1,             /**< Sign message with appropriate security profile */
    CMS_SIGN_METH_SIGN_CERT = 2,        /**< Message is signed and contains the whole signer certificate */
    CMS_SIGN_METH_SIGN_DIGEST = 3,      /**< Message is signed and contains the digest of the signer certificate */
} cms_sign_method_t;

/**
Convert a signing method into a readable string.
@ingroup sec_types

@param  sign_method         Signing method of a transmitted packet
@return readable string
@see cms_sign_method_t
*/
static inline const char* cms_sign_method_to_string(cms_sign_method_t sign_method)
{
    const char* result = NULL;

    switch(sign_method) {
    case CMS_SIGN_METH_NONE:
        result = "Do not sign";
        break;
    case CMS_SIGN_METH_SIGN:
        result = "Sign";
        break;
    case CMS_SIGN_METH_SIGN_CERT:
        result = "Sign with cert";
        break;
    case CMS_SIGN_METH_SIGN_DIGEST:
        result = "Sign with digest";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Signing info parameters of a transmitted packet.
@ingroup sec_types
*/
struct cms_sec_dot2_tx_sign_info_t {
    cms_sign_method_t sign_method;                              /**< Signing method of the transmitted packet */
    cms_psid_t psid;                                            /**< PSID/AID of the transmitted packet */
    cms_ssp_t ssp;                                              /**< Service Specific Permissions (SSP).
                                                                     SSP check performed on transmitted packets
                                                                     only if this field is set
                                                                     and the check is enabled by configuration.
                                                                     @experimentalfield */
} CMS_PACKED;

/**
Type of the payload sent by the user via the API.
Do not set a value other than the default zero if not needed explicitly by the use case.
This type relates to IEEE 1609.2 security standard.
@ingroup sec_types
@note Experimental feature, it can change in the future.
*/
typedef enum {
    /**
     * The provided payload will be put into the innermost unsecuredData element, and fully encapsulated into security headers.
     * This is the default value.
     */
    CMS_SEC_DOT2_TX_PAYLOAD_TYPE_RAW = 0,
    /**
     * The provided payload shall be an OER encoded Ieee1609Dot2Data element and will be further encapsulated into a SignedData structure.
     * The protocolVersion shall be correct, and only EncryptedData inner structure is supported.
     * Direct sending (with no signature) of user provided Ieee1609Dot2Data payload is currently not supported.
     * @note Experimental feature, it can change in the future.
     */
    CMS_SEC_DOT2_TX_PAYLOAD_TYPE_EXT_DOT2_DATA = 1,
} cms_sec_dot2_tx_payload_type_t;

/**
Security info parameters of a transmitted packet.
This type relates to IEEE 1609.2 security standard.
@ingroup sec_types
*/
struct cms_sec_dot2_tx_info_t {
    cms_sec_dot2_tx_sign_info_t sign_info; /**< Contains the information necessary for the signing */
    /**
      Type of the payload sent by the user via the API.
      Do not set a value other than the default zero if not needed explicitly by the use case.
      @experimentalfield
     */
    cms_sec_dot2_tx_payload_type_t payload_type;
} CMS_PACKED;

/**
HSM slot index value: use next free.
@ingroup sec_types
When this value is set to a cms_sec_hsm_slot_index_t type,
the next free index will be used for the called function.
@see cms_sec_hsm_slot_index_t
*/
#define CMS_SEC_HSM_SLOT_INDEX_NEXT_FREE    UINT32_MAX

/**
HSM slot index.
@ingroup sec_types
@see CMS_SEC_HSM_SLOT_INDEX_NEXT_FREE
*/
typedef uint32_t cms_sec_hsm_slot_index_t;

/**
HSM slot range information.
@ingroup sec_types
*/
typedef enum cms_sec_hsm_slot_range_t {
    CMS_SEC_HSM_SLOT_RANGE_VOLATILE = 0,        /**< Volatile range  */
    CMS_SEC_HSM_SLOT_RANGE_EU_CN_PSY,           /**< Combined EU/CN Pseudonym range */
    CMS_SEC_HSM_SLOT_RANGE_ENROLL_SEED_USER     /**< Combined enrollment/seed/user range */
} cms_sec_hsm_slot_range_t;

/**
HSM slot information.
@ingroup sec_types
*/
struct cms_sec_hsm_slot_info_t {
    cms_sec_hsm_slot_range_t slot_range;        /**< HSM slot range */
    cms_sec_hsm_slot_index_t slot_index;        /**< HSM slot index */
} CMS_PACKED;

/**
Private key type.
@ingroup sec_types
*/
typedef enum cms_sec_private_key_type_t {
    CMS_SEC_PRIVATE_KEY_TYPE_STANDALONE = 0,    /**< Imported */
    CMS_SEC_PRIVATE_KEY_TYPE_PSEUDO,            /**< Normal end entity signing key (implicit: US or csr member: EU) */
    CMS_SEC_PRIVATE_KEY_TYPE_LONGTERM,          /**< Signs a cert */
    CMS_SEC_PRIVATE_KEY_TYPE_IMPLICIT_SEED,     /**< Implicit cert, generated for PKI input */
    CMS_SEC_PRIVATE_KEY_TYPE_MODULE_AUTH,       /**< Device registration in EU */
    CMS_SEC_PRIVATE_KEY_TYPE_ENROLLMENT,        /**< Device registration in US */
    CMS_SEC_PRIVATE_KEY_TYPE_UPDATE             /**< Private key for update */
} cms_sec_private_key_type_t;

/**
Public key type.
@ingroup sec_types
*/
typedef enum cms_sec_public_key_type_t {
    CMS_SEC_PUBLIC_KEY_TYPE_SIGN = 0,           /**< Used for ECDSA signing */
    CMS_SEC_PUBLIC_KEY_TYPE_ENCRYPT,            /**< Used for ECIES encryption */
    CMS_SEC_PUBLIC_KEY_TYPE_ANY                 /**< Can be used for ECDSA signing and ECIES encryption as well.
                                                     Supported only on certain platforms. */
} cms_sec_public_key_type_t;

/**
Convert a public key type to a human readable string.
@ingroup sec_types

@param  pubkey_type                  Type of public key
@return readable string
@see cms_sec_public_key_type_t
*/
static inline const char* cms_sec_public_key_type_to_string(cms_sec_public_key_type_t pubkey_type)
{
    const char* result = NULL;

    switch(pubkey_type) {
    case CMS_SEC_PUBLIC_KEY_TYPE_SIGN:
        result = "Sign";
        break;
    case CMS_SEC_PUBLIC_KEY_TYPE_ENCRYPT:
        result = "Encrypt";
        break;
    case CMS_SEC_PUBLIC_KEY_TYPE_ANY:
        result = "Any";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
HSM key information.
@ingroup sec_types
*/
struct cms_sec_hsm_key_info_t {
    cms_sec_curve_t curve;                          /**< Elliptic Curve type */
    cms_sec_private_key_type_t private_key_type;    /**< Private key type */
    cms_sec_public_key_type_t public_key_type;      /**< Public key type */
} CMS_PACKED;

/**
HSM request's result code.
@ingroup sec_types
*/
typedef enum cms_sec_hsm_result_t {
    CMS_SEC_HSM_RES_OK = 0,                 /**< Success */
    CMS_SEC_HSM_RES_GENERAL_ERROR,          /**< General error */
    CMS_SEC_HSM_RES_INVALID_ARGS,           /**< Invalid arguments given */
} cms_sec_hsm_result_t;

/**
Convert a HSM request's result code into a readable string.
@ingroup sec_types

@param  hsm_result                  HSM request's result code
@return readable string
@see cms_sec_hsm_result_t
*/
static inline const char* cms_sec_hsm_result_to_string(cms_sec_hsm_result_t hsm_result)
{
    const char* result = NULL;

    switch(hsm_result) {
    case CMS_SEC_HSM_RES_OK:
        result = "OK";
        break;
    case CMS_SEC_HSM_RES_GENERAL_ERROR:
        result = "General error";
        break;
    case CMS_SEC_HSM_RES_INVALID_ARGS:
        result = "Invalid arguments";
        break;
    default:
        result = "Invalid";
    }
    return result;
}


/**
SEC request's result code.
@ingroup sec_types
*/
typedef enum cms_sec_load_cert_result_t {
    CMS_SEC_RES_LOAD_CERT_OK = 0,           /**< Success */
    CMS_SEC_RES_LOAD_CERT_GENERAL_ERROR,    /**< General error */
    CMS_SEC_RES_LOAD_CERT_PARSE_ERROR,      /**< Parse error */
    CMS_SEC_RES_LOAD_CERT_ALREADY_EXISTS,   /**< Cert already exist error */
} cms_sec_load_cert_result_t;

/**
Convert a certificate load result code into a readable string.
@ingroup sec_types

@param  load_cert_result        Certificate load result code
@return readable string
@see cms_sec_load_cert_result_t
*/
static inline const char* cms_sec_load_cert_result_to_string(cms_sec_load_cert_result_t load_cert_result)
{
    const char* result = NULL;

    switch(load_cert_result) {
    case CMS_SEC_RES_LOAD_CERT_OK:
        result = "OK";
        break;
    case CMS_SEC_RES_LOAD_CERT_GENERAL_ERROR:
        result = "General error";
        break;
    case CMS_SEC_RES_LOAD_CERT_PARSE_ERROR:
        result = "Parse error";
        break;
    case CMS_SEC_RES_LOAD_CERT_ALREADY_EXISTS:
        result = "Cert already exists";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
ECDH derived key default kdf2 key length
@ingroup sec_types
*/
#define CMS_SEC_ECDH_AES_KEY_LEN     16     /**< Key length of the ECIES encryption */
#define CMS_SEC_ECDH_AUTH_KEY_LEN    32     /**< Length of the ECIES message authentication code */
#define CMS_SEC_ECDH_KEY_LEN         ((CMS_SEC_ECDH_AES_KEY_LEN + CMS_SEC_ECDH_AUTH_KEY_LEN))

/**
ECDH derived key maximum length
@ingroup sec_types
*/
#define CMS_SEC_MAX_KDF_KEY_LEN      64     /**< maximum derived key length */

/**
ECDH key derivation function types
@ingroup sec_types
*/
typedef enum cms_sec_hsm_kdf_type_t {
    CMS_SEC_ECDH_PLAIN,                     /**< No KDF applied to the input ecc keys */
    CMS_SEC_ECDH_KDF2,                      /**< KDF2 with ha256 applied to the input ecc keys */
    CMS_SEC_ECDH_KDF_SM3,                   /**< KDF with SM3 hash, for SM2 ECDH */
} cms_sec_hsm_kdf_type_t;

/**
ECDH derived key
@ingroup sec_types
**/
typedef struct cms_sec_kdf_derived_key_t {
    uint8_t key[CMS_SEC_MAX_KDF_KEY_LEN];   /**< derived key */
    uint32_t length;                        /**< length of the derived key */
} CMS_PACKED cms_sec_kdf_derived_key_t;

/**
Key derivation information
@ingroup sec_types
**/
typedef struct cms_sec_kdf_t {
    cms_sec_hsm_kdf_type_t type;            /**< type of the requested kdf operation */
    cms_sec_hash_t p1;                      /**< parameter to the kdf function,
                                                used only if type is CMS_SEC_ECDH_KDF2 */
} CMS_PACKED cms_sec_kdf_t;

/**
ECIES p1 value for KDF2
This needs to be a SHA256 hash
@ingroup sec_types
@experimentalapi
*/
typedef uint8_t cms_sec_ecies_p1_t[CMS_SEC_256_SCALAR_LEN];


/**
Certification verification failure codes
@ingroup sec_types
 */
typedef enum cms_sec_cert_fail_code_t {
    CMS_SEC_CERT_OK = 0,                    /**< Cert is not failed */
    CMS_SEC_CERT_REVOKED = 1,               /**< Cert has been revoked */
    CMS_SEC_CERT_ISSUER_FAILED = 2,         /**< Cert's issuer is failed */
    CMS_SEC_CERT_CONSISTENCY_FAILED = 3,    /**< Cert is not consistent with the issuer  */
    CMS_SEC_CERT_SIGNATURE_FAILED = 4,      /**< Cert's signature is failed */
    CMS_SEC_CERT_PMA_FAILED = 5,            /**< Public key extraction failed (only for implicit certs) */
    CMS_SEC_CERT_UNCOMPRESS_FAILED = 6,     /**< Public key decompress failed */
    CMS_SEC_CERT_NON_TRUSTED_ROOT = 7,      /**< Selfsigned (root) cert is not trusted */
} cms_sec_cert_fail_code_t;

/**
Convert a certificate verification failure code into a readable string.
@ingroup sec_types

@param  fail_code           Certification verification failure code
@return readable string
@see cms_sec_cert_fail_code_t
*/
static inline const char* cms_print_cert_fail_code(cms_sec_cert_fail_code_t fail_code)
{
    const char* result = NULL;

    switch(fail_code) {
    case CMS_SEC_CERT_OK:
        result = "Cert is not failed";
        break;
    case CMS_SEC_CERT_REVOKED:
        result = "Revocation problem in the cert chain";
        break;
    case CMS_SEC_CERT_ISSUER_FAILED:
        result = "Cert's issuer is failed";
        break;
    case CMS_SEC_CERT_CONSISTENCY_FAILED:
        result = "Cert is not consistent with the issuer";
        break;
    case CMS_SEC_CERT_SIGNATURE_FAILED:
        result = "Cert's signature is failed";
        break;
    case CMS_SEC_CERT_PMA_FAILED:
        result = "Public key extraction failed";
        break;
    case CMS_SEC_CERT_UNCOMPRESS_FAILED:
        result = "Public key decompress failed";
        break;
    case CMS_SEC_CERT_NON_TRUSTED_ROOT:
        result = "Selfsigned (root) cert is not trusted";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Struct for storing the input of getting the slots of a given range.
@ingroup sec_types
*/
struct cms_sec_hsm_get_slots_in_t {
    cms_sec_hsm_slot_range_t range;
} CMS_PACKED;

/**
Struct for storing the result of getting slots of a given range.
(first slot in range, number of slots in range)
@ingroup sec_types
*/
struct cms_sec_hsm_get_slots_result_t {
    cms_sec_hsm_slot_index_t first_slot;
    uint32_t number_of_slots;
} CMS_PACKED;

/**
Time validity of a certificate.
@ingroup sec_types
*/
typedef struct cms_sec_cert_time_validity_t {
    uint64_t start;             /**< valid from, [system time ms] (0: non-spec) */
    uint64_t end;               /**< cert is valid until, [system time ms] (0: non-spec) */
} CMS_PACKED cms_sec_cert_time_validity_t;


#define CMS_SEC_SYMM_KEY_LEN     16
/**
Symmetric key
@ingroup sec_types
*/
typedef uint8_t cms_sec_symm_key_t[CMS_SEC_SYMM_KEY_LEN];

/**
SM2 ECIES authentication tag length.
@ingroup sec_types
*/
#define CMS_SEC_SM2_ECIES_TAG_LEN    32      /**< length of SM2 ecies tag in bytes */
/**
SM2 ECIES authentication tag type.
@ingroup sec_types
*/
typedef uint8_t cms_sec_sm2_ecies_tag_t[CMS_SEC_SM2_ECIES_TAG_LEN];      /**< 256 bit array for SM2 ecies tag value */

/**
Authentication tag type for asymmetric encrypt/decrypt.
@ingroup sec_types
*/
typedef cms_sec_sm2_ecies_tag_t cms_sec_ecies_auth_tag_t;

/**
Struct for storing the result of ECIES
(an ephemeral public key, an encrypted symmetric key, and an authentication tag )
@ingroup sec_types
*/
typedef struct cms_sec_ecies_vct_t {         /**< ECIES output fields in one struct  */
    cms_sec_pubkey_t v;                      /**< Ephemeral public key  */
    cms_sec_symm_key_t c;                    /**< Encrypted 128 bit long symmetric key  */
    cms_sec_ecies_auth_tag_t t;              /**< 32 bytes long tag value for message integrity checking.
                                                  The full length of tag is used if V is an SM2 curve point.
                                                  If V is not an SM2 curve point, then only the first 16 bytes should
                                                  be used.  */
} cms_sec_ecies_vct_t;

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SECURITY_TYPES_H_ */
