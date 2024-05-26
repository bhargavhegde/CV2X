/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_LICENCE_H_
#define CMS_V2X_LICENCE_H_

/** @file
@brief Licence API
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup licence Licensing
@ingroup api
@note Experimental features.
*/

/**
Licence data.
@ingroup licence
Provides the list of licensed modules as a bit flag set.
*/
struct cms_licence_data_t {
    uint64_t modules;       /**< Enabled modules of the host V2X stack */
    uint64_t cff_flags;     /**< Enabled components of the _CFF_ in the host V2X stack */
    uint64_t app_flags;     /**< Enabled safety applications of the host V2X stack */
} CMS_PACKED;


/**
Get the licensed components.
@ingroup licence
@experimentalapi
@param session      Client session
@param unused_in    Unused, expects NULL
@param[out] result  Licence data output
@return true in case of error
*/
bool cms_get_licence_data(const cms_session_t* session,
                          const void* unused_in,
                          cms_licence_data_t* result);


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_LICENCE_H_ */
