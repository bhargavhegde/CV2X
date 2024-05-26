/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_SECURITY_H_
#define CMS_V2X_SECURITY_H_

/** @file
@brief Security data types
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/security_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup sec Security
@ingroup api
@note Experimental features.
*/

/**
Information on generating a private key with HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_generate_key_in_t {
    cms_sec_hsm_slot_info_t slot_info;          /**< HSM slot information */
    cms_sec_hsm_key_info_t key_info;            /**< HSM key information */
} CMS_PACKED cms_sec_hsm_generate_key_in_t;

/**
HSM generate private key result.
@ingroup sec
*/
typedef struct cms_sec_hsm_generate_key_result_t {
    cms_sec_hsm_slot_index_t slot_index;        /**< HSM slot index where the key was generated */
    cms_sec_pubkey_t public_key;                /**< Public key of the generated private key */
} CMS_PACKED cms_sec_hsm_generate_key_result_t;

/**
Information on importing a private key into HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_import_key_in_t {
    cms_sec_hsm_slot_info_t slot_info;          /**< HSM slot info */
    cms_sec_hsm_key_info_t key_info;            /**< HSM key information */
    cms_sec_scalar_t privkey;                   /**< Plaintext private key */
} CMS_PACKED cms_sec_hsm_import_key_in_t;

/**
HSM import private key result.
@ingroup sec
*/
typedef cms_sec_hsm_generate_key_result_t cms_sec_hsm_import_key_result_t;

/**
Information on getting a public key from HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_get_pubkey_in_t {
    cms_sec_hsm_slot_index_t slot_index;        /**< Slot index of the requested public key */
} CMS_PACKED cms_sec_hsm_get_pubkey_in_t;

/**
HSM get public key result.
@ingroup sec
*/
typedef struct cms_sec_hsm_get_pubkey_result_t {
    cms_sec_pubkey_t public_key;                /**< Requested public key */
    cms_sec_hsm_key_info_t key_info;            /**< HSM key information */
} CMS_PACKED cms_sec_hsm_get_pubkey_result_t;

/**
Information on signing a hash via HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_sign_in_t {
    cms_sec_hsm_slot_index_t slot_index;        /**< HSM slot index */
    cms_sec_hash_t hash;                        /**< Hash for signing */
} CMS_PACKED cms_sec_hsm_sign_in_t;

/**
HSM sign result.
@ingroup sec
*/
typedef struct cms_sec_hsm_sign_result_t {
    cms_sec_signature_t signature;              /**< Signature */
} CMS_PACKED cms_sec_hsm_sign_result_t;

/**
Information on private multiply add via HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_private_multiply_add_in_t {
    cms_sec_hsm_slot_index_t source_slot;
    cms_sec_hsm_slot_info_t destination_slot;
    cms_sec_scalar_t mul_value;
    cms_sec_scalar_t add_value;
} CMS_PACKED cms_sec_hsm_private_multiply_add_in_t;

/**
HSM private multiply add result.
@ingroup sec
*/
typedef struct cms_sec_hsm_private_multiply_add_result_t {
    cms_sec_hsm_slot_index_t slot_index;
} CMS_PACKED cms_sec_hsm_private_multiply_add_result_t;

/**
Information on derive ECDH key via HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_ecdh_in_t {
    cms_sec_hsm_slot_index_t slot_index;
    cms_sec_pubkey_t public_key;
    cms_sec_kdf_t kdf_params;
    uint32_t key_length;
} CMS_PACKED cms_sec_hsm_ecdh_in_t;

/**
HSM derive ECDH key result.
@ingroup sec
*/
typedef struct cms_sec_hsm_ecdh_result_t {
    cms_sec_kdf_derived_key_t derived_key;
} CMS_PACKED cms_sec_hsm_ecdh_result_t;

/**
HSM derived ECDH key length using KDF2.
@ingroup sec
*/
#define CMS_SEC_ECDH_KDF2_KEY_LEN       48UL
/**
HSM derived ECDH key length using SM3 KDF.
@ingroup sec
*/
#define CMS_SEC_ECDH_SM3_KDF_KEY_LEN    16UL

/**
Information on put slot to HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_put_slot_in_t {
    cms_sec_hsm_slot_index_t slot_index;
} CMS_PACKED cms_sec_hsm_put_slot_in_t;

/**
HSM is implemented in hw result.
@ingroup sec
*/
typedef struct cms_sec_hsm_is_hw_result_t {
    bool hsm_is_hw;
} CMS_PACKED cms_sec_hsm_is_hw_result_t;

/**
Information on derive ECDH key via HSM.
@ingroup sec
*/
typedef struct cms_sec_hsm_ecdh_ecies_decrypt_in_t {
    cms_sec_hsm_slot_index_t slot_index;    /**< HSM slot index */
    cms_sec_ecies_vct_t ecies_vct;          /**< ECIES encryption result */
    cms_sec_ecies_p1_t p1;                  /**< P1 parameter for KDF2 (not used for SM2) */
} CMS_PACKED cms_sec_hsm_ecdh_ecies_decrypt_in_t;

/**
HSM decrypted symmetric key result.
@ingroup sec
*/
typedef struct cms_sec_hsm_ecdh_ecies_decrypt_result_t {
    cms_sec_symm_key_t symmetric_key;       /**< Decrypted symmetric key */
} CMS_PACKED cms_sec_hsm_ecdh_ecies_decrypt_result_t;

/**
Information on load CA certificate.
@ingroup sec
*/
typedef struct cms_sec_load_ca_cert_params_t {
    cms_std_type_t std;
    bool trusted;
} CMS_PACKED cms_sec_load_ca_cert_params_t;

/**
Load CA certificate result.
@ingroup sec
*/
typedef struct cms_sec_load_ca_cert_out_t {
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_load_ca_cert_out_t;

/**
Information on unload CA certificate.
@ingroup sec
*/
typedef struct cms_sec_unload_ca_cert_params_t {
    cms_std_type_t std;
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_unload_ca_cert_params_t;

/**
Signer certificate type
@ingroup sec
*/
typedef enum cms_signer_certificate_type_t {
    CMS_SEC_SIGNER_EXPLICIT = 0,        /**< Explicit certificate (EU) or plaintext private key with SW HSM*/
    CMS_SEC_SIGNER_EXPLICIT_BFLY,       /**< Explicit butterfly expansion certificate (currently not used by any standard) */
    CMS_SEC_SIGNER_IMPLICIT_CONTRIB,    /**< Implicit certificate with contribution data (US RSU) */
    CMS_SEC_SIGNER_IMPLICIT_BFLY,       /**< Implicit certificate with contribution data and butterfly expansion (US OBU) */
} cms_sec_signer_certificate_type_t;

/**
Convert a signer certificate type into a readable string.
@ingroup sec_types

@param  signer_type             Signer certificate type
@return readable string
@see cms_sec_signer_certificate_type_t
*/
static inline const char* cms_sec_signer_certificate_type_to_string(cms_sec_signer_certificate_type_t signer_type)
{
    const char* result = NULL;

    switch(signer_type) {
    case CMS_SEC_SIGNER_EXPLICIT:
        result = "Explicit certificate";
        break;
    case CMS_SEC_SIGNER_EXPLICIT_BFLY:
        result = "Explicit butterfly expansion certificater";
        break;
    case CMS_SEC_SIGNER_IMPLICIT_CONTRIB:
        result = "Implicit certificate with contribution data";
        break;
    case CMS_SEC_SIGNER_IMPLICIT_BFLY:
        result = "Implicit certificate with contribution data and butterfly expansion";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Butterfly expansion information
@ingroup sec
*/
typedef struct cms_sec_bfly_t {
    cms_sec_aes_key_t aes_key;
    uint32_t i_value;
    uint32_t j_value;
} CMS_PACKED cms_sec_bfly_t;

/**
Information on load signer certificate.
@ingroup sec
*/
typedef struct cms_sec_load_signer_cert_params_t {
    cms_std_type_t std;                             /**< std type of the certificate*/
    cms_sec_signer_certificate_type_t signer_type;  /**< Signer type of certificate*/
    cms_sec_hsm_slot_index_t privkey_index;         /**< used only in explicit cases*/
    cms_sec_hsm_slot_index_t seed_index;            /**< used in not explicit cases */
    cms_sec_scalar_t  contribution_data;            /**< used in implicit certificate cases */
    cms_sec_bfly_t  bfly;                           /**< struct for butterfly expansion information */
} CMS_PACKED cms_sec_load_signer_cert_params_t;

/**
Load signer certificate result.
@ingroup sec
*/
typedef struct cms_sec_load_signer_cert_out_t {
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_load_signer_cert_out_t;

/**
Information on select signer certificate.
@ingroup sec
*/
typedef struct cms_sec_select_signer_cert_params_t {
    cms_std_type_t std;
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_select_signer_cert_params_t;

/**
Information on unload signer certificate.
@ingroup sec
*/
typedef cms_sec_select_signer_cert_params_t cms_sec_unload_signer_cert_params_t;

/**
Information on flush certs.
@ingroup sec
*/
typedef struct cms_sec_flush_certs_params_t {
    cms_std_type_t std;
} CMS_PACKED cms_sec_flush_certs_params_t;

/**
Information on security enable/disable. Security can be enabled separarely
for different standard types. (EU / US / CHINA)
@ingroup sec
*/
typedef struct cms_sec_enable_params_t {
    cms_std_type_t std;
    bool enable;
} CMS_PACKED cms_sec_enable_params_t;

/**
Reload method
@ingroup sec
*/
typedef enum {
    CMS_SEC_CRT_RELOAD_ALL = 0, /**< Flush and reload all certificates and certificate revocation lists (Including CA certificates and all received certificates) */
    CMS_SEC_CRT_RELOAD_SLC,     /**< Remove short lived certs, then load slc directories from the file system */
    CMS_SEC_CRT_RELOAD_ADD,     /**< Does not flush any certs, only add new slc directories */
    CMS_SEC_CRT_RELOAD_CRL_ONLY /**< Reload Certificate Revocation Lists only */
} cms_sec_crt_reload_method_t;

/**
Information on certificate reload.
@ingroup sec
*/
typedef struct cms_sec_crt_reload_t {
    cms_sec_crt_reload_method_t type;
} CMS_PACKED cms_sec_crt_reload_t;

/** Input parameters for get signer cert
 *  @ingroup sec
 */
typedef struct cms_sec_get_signer_cert_specifier_t {
    cms_std_type_t std;
    cms_psid_t psid;
} CMS_PACKED cms_sec_get_signer_cert_specifier_t;

/** Output parameters for get signer cert
 *  @ingroup sec
 */
typedef struct cms_sec_get_signer_cert_result_t {
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_get_signer_cert_result_t;

/**
Function to generate private key into a specific HSM slot.
@ingroup sec
@experimentalapi

@param  session                 Client session
@param  generate_key_in         Information for generating a private key
@param[out] generate_key_result Result of the generation
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_generate_key(const cms_session_t* session,
                                              const cms_sec_hsm_generate_key_in_t* generate_key_in,
                                              cms_sec_hsm_generate_key_result_t* generate_key_result);

/**
Function to import private key into a specific HSM slot.
@ingroup sec
@experimentalapi

@param  session                 Client session
@param  import_key_in           Information for importing a private key
@param[out] import_key_result   Result of the importing: HSM slot index
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_import_key(const cms_session_t* session,
                                            const cms_sec_hsm_import_key_in_t* import_key_in,
                                            cms_sec_hsm_import_key_result_t* import_key_result);

/**
Function to get the public key and type for a HSM slot.
@ingroup sec
@experimentalapi

@param  session                 Client session
@param  get_pubkey_in           Information for getting a public key: slot index
@param[out] get_pubkey_result   Result of the get public key request
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_get_pubkey(const cms_session_t* session,
                                            const cms_sec_hsm_get_pubkey_in_t* get_pubkey_in,
                                            cms_sec_hsm_get_pubkey_result_t* get_pubkey_result);

/**
Function to sign a hash with a private key in a specific HSM slot.
@ingroup sec
@experimentalapi

@param  session         Client session
@param  sign_in         Information for signing a hash
@param[out] sign_result Result of the signing
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_sign(const cms_session_t* session,
                                      const cms_sec_hsm_sign_in_t* sign_in,
                                      cms_sec_hsm_sign_result_t* sign_result);

/**
Function to generate a HSM key into a HSM slot with the multiply-add operation
@ingroup sec
@experimentalapi

@param  session                         Client session
@param  private_multiply_add_in         Information for private multiply add operation
@param[out] private_multiply_add_result Result of operation
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_private_multiply_add(const cms_session_t* session,
                                                      const cms_sec_hsm_private_multiply_add_in_t* private_multiply_add_in,
                                                      cms_sec_hsm_private_multiply_add_result_t* private_multiply_add_result);

/**
Function to derive ECDH key
@ingroup sec
@experimentalapi

@param  session         Client session
@param  ecdh_in         Information for key derivation
@param[out] ecdh_result Result of operation
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_ecdh(const cms_session_t* session,
                                      const cms_sec_hsm_ecdh_in_t* ecdh_in,
                                      cms_sec_hsm_ecdh_result_t* ecdh_result);

/**
Function to derive an ECDH key and ECIES decrypt a symmetric key with it
@ingroup sec
@experimentalapi

@param  session                     Client session
@param  ecies_decrypt_in            Information for key derivation and decryption
@param[out] ecies_decrypt_result    Result of operation
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_ecdh_ecies_decrypt(const cms_session_t* session,
                                                    const cms_sec_hsm_ecdh_ecies_decrypt_in_t* ecies_decrypt_in,
                                                    cms_sec_hsm_ecdh_ecies_decrypt_result_t* ecies_decrypt_result);

/**
Function to mark a specific HSM slot as unused
@ingroup sec
@experimentalapi

@param  session             Client session
@param  put_slot_in         Information on put volatile slot to HSM
@param  unused_out          Unused parameter
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_put_slot(const cms_session_t* session,
                                          const cms_sec_hsm_put_slot_in_t* put_slot_in,
                                          const void* unused_out);

/**
Function to check if HSM is implemented in HW
@ingroup sec
@experimentalapi

@param  session          Client session
@param  unused_in        Unused parameter
@param[out] is_hw_result Result of operation
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_is_hw(const cms_session_t* session,
                                       const void* unused_in,
                                       cms_sec_hsm_is_hw_result_t* is_hw_result);

/**
Function to get the the first slot and the length
 of a specified private key range
@ingroup sec
@experimentalapi

@param  session             Client session
@param  get_slots_in        Input values (specified privkey range)
@param[out] get_slots_out   Output values (first slot and length of the specified range)
@return HSM result code.
*/
cms_sec_hsm_result_t cms_sec_hsm_get_slot_range(const cms_session_t* session,
                                                const cms_sec_hsm_get_slots_in_t* get_slots_in,
                                                cms_sec_hsm_get_slots_result_t* get_slots_out);

/**
Function to load certificate to the security module
@ingroup sec
@experimentalapi

@param  session         Client session
@param  load_in         Load params
@param  certificate     certificate buffer
@param[out] load_out contains the hashid8 of the certificate
@return cms_sec_load_cert_result_t
*/

cms_sec_load_cert_result_t cms_sec_load_ca_cert(const cms_session_t* session,
                                                const cms_sec_load_ca_cert_params_t* load_in,
                                                cms_buffer_view_t certificate,
                                                cms_sec_load_ca_cert_out_t* load_out);

/**
Function to unload certificate from the security module
@ingroup sec
@experimentalapi

@param  session          Client session
@param  unload_in        Unload params
@param[out] unused_out   Unused parameter
@return true in case of error
*/

bool cms_sec_unload_ca_cert(const cms_session_t* session,
                            const cms_sec_unload_ca_cert_params_t* unload_in,
                            const void* unused_out);

/**
Function to load signer certificate to the security module
@ingroup sec
@experimentalapi

@param  session         Client session
@param  load_in         Load params
@param  certificate     certificate buffer
@param[out] load_out contains the hashid8 of the certificate
@return cms_sec_load_cert_result_t
*/

cms_sec_load_cert_result_t cms_sec_load_signer_cert(const cms_session_t* session,
                                                    const cms_sec_load_signer_cert_params_t* load_in,
                                                    cms_buffer_view_t certificate,
                                                    cms_sec_load_signer_cert_out_t* load_out);

/**
Function to unload certificate from the security module
@ingroup sec
@experimentalapi

@param  session         Client session
@param  unload_in       Unload params
@param  unused_out      unused parameter
@return true in case of error
*/

bool cms_sec_unload_signer_cert(const cms_session_t* session,
                                const cms_sec_unload_signer_cert_params_t* unload_in,
                                const void* unused_out);

/**
Function to select certificate in the security module
@ingroup sec
@experimentalapi

@param  session         Client session
@param  select_in       Select params
@param  unused_out      unused parameter
@return true in case of error
*/

bool cms_sec_select_signer_cert(const cms_session_t* session,
                                const cms_sec_select_signer_cert_params_t* select_in,
                                const void* unused_out);

/**
Function to flush all certificates in the security module
@ingroup sec
@experimentalapi

@param  session         Client session
@param  flush_certs_in  Flush params
@param  unused_out      unused parameter
@return true in case of error
*/

bool cms_sec_flush_certs(const cms_session_t* session,
                         const cms_sec_flush_certs_params_t* flush_certs_in,
                         const void* unused_out);

/**
Function to enable/disable security
@ingroup sec
@experimentalapi

@param  session         Client session
@param  enable_in       Information for security enable/disable
@param  unused_out      unused parameter
@return true in case of error
*/

bool cms_sec_enable(const cms_session_t* session,
                    const cms_sec_enable_params_t* enable_in,
                    const void* unused_out);

/**
Function to start certificate reloading
@ingroup sec
@experimentalapi

@param  session         Client session
@param  in              Reload params
@param  unused_out      unused parameter
@return true in case of error
*/
bool cms_sec_crt_reload(const cms_session_t* session,
                        const cms_sec_crt_reload_t* in,
                        const void* unused_out);

/**
Get the hashid8 of the active signer cert for a given standard type and psid
@ingroup sec
@experimentalapi

@param  session     Client session
@param  specifier   Input of the query (std and psid)
@param  result      Result of the query
@return true in case of error
*/
bool cms_sec_get_signer_cert(const cms_session_t* session,
                             const cms_sec_get_signer_cert_specifier_t* specifier,
                             cms_sec_get_signer_cert_result_t* result);

/**
Information about the raw certificate to query.
@ingroup sec
*/
typedef struct cms_sec_get_raw_cert_specifier_t {
    cms_std_type_t std;
    cms_hashid8_t hashid8;
} CMS_PACKED cms_sec_get_raw_cert_specifier_t;

/**
Get raw certificate result code.
@ingroup sec_types
*/
typedef enum cms_sec_get_raw_cert_result_t {
    CMS_SEC_GET_RAW_CERT_RES_OK = 0,            /**< Success */
    CMS_SEC_GET_RAW_CERT_RES_GENERAL_ERROR,     /**< General error */
    CMS_SEC_GET_RAW_CERT_RES_NOT_EXIST,         /**< Certificate does not exist */
    CMS_SEC_GET_RAW_CERT_RES_MISSING_RAW_CERT,  /**< Certificate is found but raw cert is not stored */
} cms_sec_get_raw_cert_result_t;

/**
Get raw certificate output.
@ingroup sec
*/
typedef struct cms_sec_get_raw_cert_out_t {
    cms_sec_cert_state_t cert_state;            /**< Verification state of the requested certificate */
} CMS_PACKED cms_sec_get_raw_cert_out_t;

/**
Get the raw certificate based on its hashid8.
@ingroup sec

@param  session         Client session
@param  specifier       Parameters of the raw certificate query (std and hashid8)
@param  raw_cert_out    Raw certificate output parameters
@param  raw_cert        Raw certificate retrieved from certificate database
@return result of get raw cert operation
*/
cms_sec_get_raw_cert_result_t cms_sec_get_raw_cert(const cms_session_t* session,
                                                   const cms_sec_get_raw_cert_specifier_t* specifier,
                                                   cms_sec_get_raw_cert_out_t* raw_cert_out,
                                                   cms_mutable_buffer_t* raw_cert);

/**
Parameters of the certificate to get the information to.
@ingroup sec
*/
typedef struct cms_sec_get_cert_info_params_t {
    cms_std_type_t std;                         /**< Standard type of the certificate */
    cms_hashid8_t hashid8;                      /**< Hashid8 of the certificate */
} CMS_PACKED cms_sec_get_cert_info_params_t;

/**
Get certificate info result code.
@ingroup sec_types
*/
typedef enum cms_sec_get_cert_info_result_t {
    CMS_SEC_GET_CERT_INFO_RES_OK = 0,            /**< Success */
    CMS_SEC_GET_CERT_INFO_RES_GENERAL_ERROR,     /**< General error */
    CMS_SEC_GET_CERT_INFO_RES_NOT_EXIST,         /**< Certificate does not exist */
} cms_sec_get_cert_info_result_t;

/**
Converts get cert info type into a readable string.
@ingroup sec_types

@param  result_code             Get cert info result
@return readable string
@see cms_sec_signer_certificate_type_t
*/
static inline const char* cms_sec_get_cert_info_result_to_string(cms_sec_get_cert_info_result_t result_code)
{
    const char* result = NULL;

    switch(result_code) {
    case CMS_SEC_GET_CERT_INFO_RES_OK:
        result = "OK";
        break;
    case CMS_SEC_GET_CERT_INFO_RES_GENERAL_ERROR:
        result = "General error";
        break;
    case CMS_SEC_GET_CERT_INFO_RES_NOT_EXIST:
        result = "Certificate does not exist";
        break;
    default:
        result = "Invalid";
    }
    return result;
}

/**
Get certificate info output.
@ingroup sec
*/
typedef struct cms_sec_get_cert_info_out_t {
    cms_sec_cert_state_t cert_state;            /**< Verification state of the requested certificate */
    cms_hashid8_t issuer_hashid8;               /**< hashid8 of the issuer certificate */
    cms_sec_cert_type_t type;                   /**< certificate type */
    cms_sec_cert_source_t source;               /**< certificate source */
    cms_sec_cert_time_validity_t time_validity; /**< time validity of the certificate */
} CMS_PACKED cms_sec_get_cert_info_out_t;
/**
Function to get certificate information
@ingroup sec
@experimentalapi

@param  session         Client session
@param  cert_info_in    Parameters of the desired certificate (std and hashid8)
@param  cert_info_out   Contains the cert properties
@return true in case of error
*/
cms_sec_get_cert_info_result_t cms_sec_get_cert_info(const cms_session_t* session,
                                                     const cms_sec_get_cert_info_params_t* cert_info_in,
                                                     cms_sec_get_cert_info_out_t* cert_info_out);


/**
Certificate verification parameter type. (common)
@ingroup sec
*/
typedef cms_sec_select_signer_cert_params_t cms_sec_verify_cert_params_t;

/**
Cert verification result code.
@ingroup sec
*/
typedef enum {
    /** Indicates the call succeeded, see 'verify_result' output argument for further state infos */
    CMS_SEC_VERIFY_CERT_OK = 0,

    /** Timeout in certificate verification progress */
    CMS_SEC_VERIFY_CERT_TIMEOUT,

    /** Certificate verification module is used by an other caller. You may retry. */
    CMS_SEC_VERIFY_CERT_IN_USE,

    /** The specified certificate not found */
    CMS_SEC_VERIFY_CERT_NOT_FOUND,

    /** Certificate from the chain is missing */
    CMS_SEC_VERIFY_CERT_ISSUER_MISSING,

    /** Generic internal error */
    CMS_SEC_VERIFY_CERT_ERROR,

    /** CRL is missing */
    CMS_SEC_VERIFY_CERT_CRL_IS_MISSING

} cms_sec_verify_cert_rc_t;


/**
Convert a certificate verification return type into a readable string.
@ingroup sec_types

@param  rc                      Certificate verification result code
@return readable string
@see cms_sec_verify_cert_rc_t
*/
static inline const char* cms_sec_verify_cert_rc_to_string(cms_sec_verify_cert_rc_t rc)
{
    const char* result = NULL;

    switch(rc) {
    case CMS_SEC_VERIFY_CERT_OK:
        result = "Call succeeded";
        break;
    case CMS_SEC_VERIFY_CERT_TIMEOUT:
        result = "Cert verification call timeout";
        break;
    case CMS_SEC_VERIFY_CERT_IN_USE:
        result = "Cert verification module is used by an other caller. You may retry.";
        break;
    case CMS_SEC_VERIFY_CERT_NOT_FOUND:
        result = "Specified cert is not found";
        break;
    case CMS_SEC_VERIFY_CERT_ISSUER_MISSING:
        result = "Cert from the chain is missing";
        break;
    case CMS_SEC_VERIFY_CERT_ERROR:
        result = "Generic internal error in cert verification";
        break;
    case CMS_SEC_VERIFY_CERT_CRL_IS_MISSING:
        result = "CRL is missing";
        break;
    default:
        result = "Invalid";
    }
    return result;
}


/**
Get raw certificate output.
@ingroup sec
*/
typedef struct cms_sec_verify_cert_result_t {
    cms_sec_cert_state_t cert_state;      /**< Verification state of the requested certificate */
} CMS_PACKED cms_sec_verify_cert_result_t;

/**
Function to verify a loaded certificate
@ingroup sec
@experimentalapi

@param       session         Client session
@param       verify_in       Certificate identifiers
@param[out]  verify_result   Verification result valid only if function return CMS_SEC_VERIFY_CERT_OK
@return      result status code

The function starts the verification of a specified certificate and waits the verification result.
The certificate need to be loaded previously or received on radio.
@see cms_sec_load_ca_cert

*/
cms_sec_verify_cert_rc_t cms_sec_verify_cert(const cms_session_t* session,
                                             const cms_sec_verify_cert_params_t* verify_in,
                                             cms_sec_verify_cert_result_t* verify_result);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SECURITY_H_ */
