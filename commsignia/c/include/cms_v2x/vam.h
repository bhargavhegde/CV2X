/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_V2X_VAM_H_
#define CMS_V2X_VAM_H_

#include <stdbool.h>
#include <stddef.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

/** @file
@brief Vulnerable Road Users Awareness Message (VAM)
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup vam Vulnerable Road Users Awareness Message (VAM)
@ingroup api
*/

/**
Single path prediction point coordinate.
@ingroup vam
*/
struct cms_vam_path_prediction_point_t {
    /** Latitude coordinate of the object's position */
    cms_latitude_t latitude;

    /** Longitude coordinate of the object's position */
    cms_longitude_t longitude;
} CMS_PACKED;

/**
Maximum quantity of the path prediction points.
@ingroup vam
*/
#define CMS_VAM_PATH_PREDICTION_POINTS_SIZE_MAX 40

/**
Container of the path prediction.
@ingroup vam
*/
struct cms_vam_path_prediction_t {
    /**size of the object's block of path prediction points */
    uint32_t length;
    /** Object's block of path prediction points*/
    cms_vam_path_prediction_point_t points[CMS_VAM_PATH_PREDICTION_POINTS_SIZE_MAX];
} CMS_PACKED;

/**
Add path prediction to the VAM module.
@ingroup vam
@experimentalapi
@param session      Client session
@param path_prediction       Path prediction object
@param unused_out   Unused output
@return true in case of error
@see cms_vam_path_prediction_t
*/
bool cms_vam_add_path_prediction(const cms_session_t* session,
                                 const cms_vam_path_prediction_t* path_prediction,
                                 void* unused_out);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_VAM_H_ */
