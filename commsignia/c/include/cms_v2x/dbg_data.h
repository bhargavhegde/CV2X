/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022
*/

#ifndef CMS_DBG_DATA_H_
#define CMS_DBG_DATA_H_

#include <cms_v2x/common_types.h>

/** @file
@brief Debug Data API
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup dbg DebugData
@ingroup api
*/

/**
Not available (invalid) value for debug data provider ID
@ingroup dbg
@experimentalapi
*/
#define CMS_DBG_PROVIDER_ID_NA                      0xFFFFFFFFU

/**
Generate provider ID like 0x80 'D' 'B' 'G'
@ingroup dbg
@experimentalapi
*/
#define CMS_DBG_PROVIDER_ID_GEN(pref, id0, id1, id2)     ((pref) | ((id0) << 8) | ((id1) << 16) | ((id2) << 24))

/**
Debug data provider ID
@ingroup dbg
@experimentalapi
*/
typedef uint32_t cms_dbg_provider_id_t;

/**
Length of the provider name
@ingroup dbg
@experimentalapi
*/
#define CMS_DBG_PROVIDER_NAME_LENGTH                12U

/**
Debug data provider name
@ingroup dbg
@experimentalapi
*/
typedef char cms_dbg_provider_name_t[CMS_DBG_PROVIDER_NAME_LENGTH];

/**
Not available (invalid) value for debug data type ID
@ingroup dbg
@experimentalapi
*/
#define CMS_DBG_DATA_TYPE_ID_NA                     0xFFFFFFFFU

/**
Debug data type ID
@ingroup dbg
@experimentalapi
*/
typedef uint32_t cms_dbg_data_type_id_t;

/**
Debug data provider struct
@ingroup dbg
@experimentalapi
*/
typedef struct cms_dbg_provider_t {
    cms_dbg_provider_id_t id;                   /**< Provider ID */
    cms_dbg_provider_name_t name;               /**< Provider name */
} CMS_PACKED cms_dbg_provider_t;

/**
Function to register a debug data provider
@ingroup dbg
@experimentalapi
@param session                  Client session
@param provider                 Debug data provider
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_dbg_data_register_provider(const cms_session_t* session,
                                    const cms_dbg_provider_t* provider,
                                    void* unused_out);


/**
Debug data fragment size
@ingroup dbg
@experimentalapi
*/
#define CMS_DBG_DATA_FRAGMENT_SIZE          1024U

/**
Debug data sequence ID
@ingroup dbg
@experimentalapi
*/
typedef uint32_t cms_dbg_data_sequence_id_t;

/**
Fragmented debug data header
@ingroup dbg
@experimentalapi
*/
typedef struct cms_dbg_data_fragment_header_t {
    cms_dbg_provider_id_t provider_id;              /**< Provider ID */
    cms_dbg_data_type_id_t data_type_id;            /**< Debug data type ID */
    cms_dbg_data_sequence_id_t data_sequence_id;    /**< Debug data sequence ID */
    uint32_t data_length;                           /**< Debug data length */
    uint16_t fragment_index;                        /**< Fragment index */
} CMS_PACKED cms_dbg_data_fragment_header_t;

/**
Function to send debug data fragment
@ingroup dbg
@experimentalapi
@param session                  Client session
@param header                   Debug data fragment header
@param data                     Debug data fragment
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_dbg_data_send_fragment(const cms_session_t* session,
                                const cms_dbg_data_fragment_header_t* header,
                                cms_buffer_view_t data,
                                void* unused_out);

/**
Debug data header
@ingroup dbg
@experimentalapi
*/
typedef struct cms_dbg_data_header_t {
    cms_dbg_provider_id_t provider_id;              /**< Provider ID */
    cms_dbg_data_type_id_t data_type_id;            /**< Debug data type ID */
    cms_dbg_data_sequence_id_t data_sequence_id;    /**< Debug data sequence ID */
} CMS_PACKED cms_dbg_data_header_t;

/**
Function to fragment and send debug data
@ingroup dbg
@experimentalapi
@param session                  Client session
@param header                   Debug data header
@param data                     Debug data
@param[out] unused_out          Unused argument
@return true in case of error
*/
bool cms_dbg_data_send(const cms_session_t* session,
                       const cms_dbg_data_header_t* header,
                       cms_buffer_view_t data,
                       void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_DBG_DATA_H_ */
