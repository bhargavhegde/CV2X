/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#ifndef CMS_V2X_SESSION_H_
#define CMS_V2X_SESSION_H_

/** @file
@brief Client session
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
Client session.
@ingroup api_ctrl
A client session provides a handle to a host-client connection.
It is used as an argument in all API functions except cms_get_session()
and cms_get_session_for_id().
@note Always create / initialize a session object with cms_get_session()
or cms_get_session_for_id(). User modification of the session object is not allowed.
*/
struct cms_session_t {
    uint32_t version;
    uint64_t client_id;
};


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_SESSION_H_ */
