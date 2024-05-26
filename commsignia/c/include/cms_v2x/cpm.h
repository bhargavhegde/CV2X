/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_CPM_H_
#define CMS_V2X_CPM_H_

#include <stdbool.h>
#include <stddef.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

/** @file
@brief Collective Perception Message (CPM)
*/

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup cpm Collective Perception Message (CPM)
@ingroup api
*/

/**
Not available value for ID parameters used in the CPM module.
@ingroup cpm
*/
#define CMS_CPM_ID_NA                   UINT8_MAX

/**
ID parameter used in the CPM module.
@ingroup cpm
*/
typedef uint8_t cms_cpm_id_t;

/**
Not available value for the perceived objects' age.
@ingroup cpm
*/
#define CMS_CPM_OBJECT_AGE_NA           UINT16_MAX

/**
Type describing how much time has elapsed in milliseconds since the first detection of an object till
the current measurement.
@ingroup cpm
*/
typedef uint16_t cms_cpm_object_age_t;

/**
Perceived object position.
@ingroup cpm
*/
struct cms_cpm_position_t {
    /** Latitude coordinate of the object's position */
    cms_latitude_t latitude;

    /** Longitude coordinate of the object's position */
    cms_longitude_t longitude;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_length_t accuracy;
} CMS_PACKED;

/**
Perceived object altitude.
@ingroup cpm
*/
struct cms_cpm_altitude_t {
    /** Altitude coordinate of the object's position (may be set to unavailable) */
    cms_altitude_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_altitude_t accuracy;
} CMS_PACKED;

/**
Perceived object groundspeed.
@ingroup cpm
*/
struct cms_cpm_ground_speed_t {
    /** Direction of the ground components of the speed vector */
    cms_heading_t direction;

    /** Absolute value of the ground components of the speed vector */
    cms_speed_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_speed_t accuracy;
} CMS_PACKED;

/**
Perceived object vertical speed.
@ingroup cpm
*/
struct cms_cpm_vertical_speed_t {
    /** Vertical component of the object's speed (may be set to unavailable) */
    cms_speed_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_speed_t accuracy;
} CMS_PACKED;

/**
Perceived object ground acceleration.
@ingroup cpm
*/
struct cms_cpm_ground_acceleration_t {
    /** Direction of the ground components of the acceleration vector (may be set to unavailable) */
    cms_heading_t direction;

    /** Absolute value of the ground components of the acceleration vector (may be set to unavailable) */
    cms_acceleration_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_acceleration_t accuracy;
} CMS_PACKED;

/**
Perceived object vertical acceleration.
@ingroup cpm
*/
struct cms_cpm_vertical_acceleration_t {
    /** Vertical component of the acceleration vector (may be set to unavailable) */
    cms_acceleration_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_acceleration_t accuracy;
} CMS_PACKED;

/**
Perceived object yaw angle.
@ingroup cpm
*/
struct cms_cpm_yaw_angle_t {
    /** Yaw angle of the object, not necessarily identical to the ground_speed_direction value
     *  (may be set to unavailable) */
    cms_heading_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_heading_t accuracy;
} CMS_PACKED;

/**
Perceived object size.
@ingroup cpm
*/
struct cms_cpm_object_size_t {
    /** Object size (may be set to unavailable) */
    cms_length_t value;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_length_t accuracy;
} CMS_PACKED;

/**
Possible classification types of perceived objects.
@ingroup cpm

The class that best describes the detected object. Each class provides optional subclasses and
an associated confidence for the subclass.
*/
typedef enum cms_cpm_detection_class_t {
    CMS_CPM_DETECTION_CLASS_UNK = 0,                    /**< The VRU type for the detected object is unknown */
    CMS_CPM_DETECTION_CLASS_VEHICLE = 1,                /**< Vehicle */
    CMS_CPM_DETECTION_CLASS_PERSON = 2,                 /**< Person */
    CMS_CPM_DETECTION_CLASS_ANIMAL = 3,                 /**< Animal */
    CMS_CPM_DETECTION_CLASS_OTHER = 4                   /**< Other kind of an object */
} cms_cpm_detection_class_t;

/**
Possible further classification types of unknown perceived objects.
@ingroup cpm
*/
typedef enum cms_cpm_unknown_subclass_t {
    CMS_CPM_UNKNOWN_SUBCLASS_UNK = 0                    /**< Unknown */
} cms_cpm_unknown_subclass_t;

/**
Possible further classification types of perceived vehicles.
@ingroup cpm
*/
typedef enum cms_cpm_vehicle_subclass_t {
    /** Unknown vehicle */
    CMS_CPM_VEHICLE_SUBCLASS_UNK = 0,

    /** Light motor vehicle with less than four wheels as defined in UNECE/TRANS/WP.29/78/Rev.4 class L1, L2 */
    CMS_CPM_VEHICLE_SUBCLASS_MOPED = 1,

    /** Light motor vehicle with less than four wheels as defined in
     *  UNECE/TRANS/WP.29/78/Rev.4 class L3, L4, L5, L6, L7 */
    CMS_CPM_VEHICLE_SUBCLASS_MOTORCYCLE = 2,

    /** Small passenger car as defined in UNECE/TRANS/WP.29/78/Rev.4 class M1 */
    CMS_CPM_VEHICLE_SUBCLASS_PASSENGER_CAR = 3,

    /** Large passenger vehicle as defined in UNECE/TRANS/WP.29/78/Rev.4 class M2, M3 */
    CMS_CPM_VEHICLE_SUBCLASS_BUS = 4,

    /** Light goods vehicle as defined in UNECE/TRANS/WP.29/78/Rev.4 class N1 */
    CMS_CPM_VEHICLE_SUBCLASS_LIGHT_TRUCK = 5,

    /** Heavy goods vehicle as defined in UNECE/TRANS/WP.29/78/Rev.4 class N2, N3 */
    CMS_CPM_VEHICLE_SUBCLASS_HEAVY_TRUCK = 6,

    /** Unpowered vehicle that is intended to be towed by a powered vehicle as defined in
     *  UNECE/TRANS/WP.29/78/Rev.4 class O */
    CMS_CPM_VEHICLE_SUBCLASS_TRAILER = 7,

    /** Vehicle which has a special purpose other than the above (e.g. moving road works vehicle) */
    CMS_CPM_VEHICLE_SUBCLASS_SPECIAL_VEHICLES = 8,

    /** Vehicle running on tracks along public streets */
    CMS_CPM_VEHICLE_SUBCLASS_TRAM = 9,

    /** Vehicle used in an emergency situation such as an ambulance, police car or fire engine */
    CMS_CPM_VEHICLE_SUBCLASS_EMERGENCY_VEHICLE = 10,

    /** Vehicle used for agricultural purposes */
    CMS_CPM_VEHICLE_SUBCLASS_AGRICULTURAL = 11
} cms_cpm_vehicle_subclass_t;

/**
Possible further classification types of perceived humans.
@ingroup cpm

Describes the subclass of a detected object for class persons. Persons are a subset of the
vulnerable road users as defined in Section 4.2 of ETSI TR 103 300-2 as well as in the
classification provided in Annex 1 of Regulation EU 168/2013.
*/
typedef enum cms_cpm_person_subclass_t {
    /** The VRU type for the detected object is unknown */
    CMS_CPM_PERSON_SUBCLASS_UNK = 0,

    /** Pedestrian travelling on foot */
    CMS_CPM_PERSON_SUBCLASS_PEDESTRIAN = 1,

    /** Person travelling in a wheelchair */
    CMS_CPM_PERSON_SUBCLASS_PERSON_IN_WHEELCHAIR = 2,

    /** One or multiple persons travelling on non-motorized unicycles, bicycles, tricycles or quadricycle */
    CMS_CPM_PERSON_SUBCLASS_CYCLIST = 3,

    /** Person travelling on foot pushing or pulling a stroller potentially carrying by one or multiple other
     *  persons */
    CMS_CPM_PERSON_SUBCLASS_PERSON_WITH_STROLLER = 4,

    /** Person travelling on skates, skateboards or a small electric or foot operated scooter */
    CMS_CPM_PERSON_SUBCLASS_PERSON_ON_SKATES = 5,

    /** A group of people with similar movement characteristics */
    CMS_CPM_PERSON_SUBCLASS_PERSON_GROUP = 6
} cms_cpm_person_subclass_t;

/**
Possible further classification types of perceived animals.
@ingroup cpm
*/
typedef enum cms_cpm_animal_subclass_t {
    CMS_CPM_ANIMAL_SUBCLASS_UNK = 0                     /**< Unknown animal */
} cms_cpm_animal_subclass_t;

/**
Possible further classification types of other perceived objects.
@ingroup cpm
*/
typedef enum cms_cpm_other_subclass_t {
    CMS_CPM_OTHER_SUBCLASS_UNK = 0,                     /**< Unknown other object */
    CMS_CPM_OTHER_SUBCLASS_ROAD_SIDE_UNIT = 1           /**< Road side unit */
} cms_cpm_other_subclass_t;

/**
Possible further classification types of perceived objects.
@ingroup cpm
*/
typedef uint8_t cms_cpm_object_subclass_t;

/**
Object classification description.
@ingroup cpm

Provides the classification of the described object. Multi-dimensional classification may be provided.
*/
struct cms_cpm_detection_t {
    cms_cpm_detection_class_t object_class;         /**< Object class */
    cms_cpm_object_subclass_t object_subclass;      /**< Object sub-class */
    cms_probability_t confidence;                   /**< Confidence value in [%] */
} CMS_PACKED;

/**
Lane ID.
@ingroup cpm
*/
typedef uint8_t cms_cpm_lane_id_t;

/**
MAP-matched position description
@ingroup cpm
Indicates the position of the object mapped on the intersection topology description transmitted.
*/
struct cms_cpm_matched_position_t {
    /** Conveys an assigned index that is unique within the intersection with InterSectionReferenceId of the
     *  OriginatingRSUContainer. It shall be presented as specified in Clause 7.88 of SAE J2735 */
    cms_cpm_lane_id_t lane_id;

    /** Indicates the longitudinal offset of the MAP-matched position of the object along the lane */
    cms_length_t longitudinal_lane_position;

    /** Accuracy corresponding to a confidence level of 95% (may be set to unavailable) */
    cms_length_t longitudinal_lane_position_accuracy;
} CMS_PACKED;

/**
Maximum number of sensors detecting an object.
@ingroup cpm
*/
#define CMS_CPM_DETECTING_SENSORS_MAX 8U

/**
Maximum number of detection classes an object may belong to.
@ingroup cpm
*/
#define CMS_CPM_DETECTION_CLASSES_MAX 8U

/**
Perceived object description.
@ingroup cpm
*/
struct cms_cpm_perceived_object_t {
    /** Object ID */
    cms_cpm_id_t id;

    /** Measurement timestamp */
    cms_utc_timestamp_ms_t timestamp;

    /** Time elapsed since the first detection of the object */
    cms_cpm_object_age_t object_age;

    /** Position value and accuracy */
    cms_cpm_position_t position;

    /** Altitude value and accuracy */
    cms_cpm_altitude_t altitude;

    /** Ground speed value and accuracy */
    cms_cpm_ground_speed_t ground_speed;

    /** Vertical speed value and accuracy */
    cms_cpm_vertical_speed_t vertical_speed;

    /** Ground acceleration value and accuracy */
    cms_cpm_ground_acceleration_t ground_acceleration;

    /** Vertical acceleration value and accuracy */
    cms_cpm_vertical_acceleration_t vertical_acceleration;

    /** Yaw angle value and accuracy */
    cms_cpm_yaw_angle_t yaw_angle;

    /** Object length value and accuracy */
    cms_cpm_object_size_t length;

    /** Object width value and accuracy */
    cms_cpm_object_size_t width;

    /** Object height value and accuracy */
    cms_cpm_object_size_t height;

    /** Detection confidence (may be set to unavailable) */
    cms_probability_t detection_confidence;

    /** Number of items in the detecting sensor ID list. Zero means unavailable */
    uint32_t detecting_sensor_num;

    /** List of sensor IDs detecting the object */
    cms_cpm_id_t detecting_sensors[CMS_CPM_DETECTING_SENSORS_MAX];

    /** Number of classes the object may belong to */
    uint32_t detection_class_num;

    /** Object classification description */
    cms_cpm_detection_t detection_classes[CMS_CPM_DETECTION_CLASSES_MAX];

    /** Indicates whether MAP-matched position is available */
    bool has_matched_position;

    /** MAP-matched position */
    cms_cpm_matched_position_t matched_position;

    /** Measurement validity timeout. Has to be positive */
    cms_expiry_time_ms_t timeout;
} CMS_PACKED;


/**
Initialize the parameters of a perceived object
@ingroup cpm
@experimentalapi

@param obj  Object to initialize
@see cms_cpm_perceived_object_t
*/
static inline void cms_init_cpm_perceived_object(cms_cpm_perceived_object_t* obj)
{
    if(obj != NULL) {

        obj->id = 0;
        obj->timestamp = CMS_UTC_TIMESTAMP_NA;
        obj->object_age = CMS_CPM_OBJECT_AGE_NA;

        obj->position.latitude = CMS_LATITUDE_NA;
        obj->position.longitude = CMS_LONGITUDE_NA;
        obj->position.accuracy = CMS_LENGTH_NA;
        obj->altitude.value = CMS_ALTITUDE_NA;
        obj->altitude.accuracy = CMS_ALTITUDE_NA;

        obj->ground_speed.direction = CMS_HEADING_NA;
        obj->ground_speed.value = CMS_SPEED_NA;
        obj->ground_speed.accuracy = CMS_SPEED_NA;
        obj->vertical_speed.value = CMS_SPEED_NA;
        obj->vertical_speed.accuracy = CMS_SPEED_NA;

        obj->ground_acceleration.direction = CMS_HEADING_NA;
        obj->ground_acceleration.value = CMS_ACCELERATION_NA;
        obj->ground_acceleration.accuracy = CMS_ACCELERATION_NA;
        obj->vertical_acceleration.value = CMS_ACCELERATION_NA;
        obj->vertical_acceleration.accuracy = CMS_ACCELERATION_NA;

        obj->yaw_angle.value = CMS_HEADING_NA;
        obj->yaw_angle.accuracy = CMS_HEADING_NA;

        obj->length.value = CMS_LENGTH_NA;
        obj->length.accuracy = CMS_LENGTH_NA;
        obj->width.value = CMS_LENGTH_NA;
        obj->width.accuracy = CMS_LENGTH_NA;
        obj->height.value = CMS_LENGTH_NA;
        obj->height.accuracy = CMS_LENGTH_NA;

        obj->detection_confidence = CMS_PROBABILITY_NA;

        obj->detecting_sensor_num = 0U;
        obj->detection_class_num = 0U;

        obj->has_matched_position = false;

        obj->timeout = 0U;
    }
}


/**
Possible shape types of a free space area.
@ingroup cpm
*/
typedef enum cms_cpm_free_space_area_type_t {
    CMS_CPM_FREE_SPACE_AREA_TYPE_POLYGON = 0,       /**< Polygon area */
    CMS_CPM_FREE_SPACE_AREA_TYPE_CIRCLE = 1,        /**< Cirular area */
    CMS_CPM_FREE_SPACE_AREA_TYPE_ELLIPSE = 2,       /**< Elliptic area */
    CMS_CPM_FREE_SPACE_AREA_TYPE_RECTANGLE = 3      /**< Rectangular area */
} cms_cpm_free_space_area_type_t;

/**
Free space polygon vertex description.
@ingroup cpm
*/
struct cms_cpm_free_space_point_t {
    cms_latitude_t latitude;                        /**< Latitude coordinate */
    cms_longitude_t longitude;                      /**< Longitude coordinate */
} CMS_PACKED;

/**
Maximum number of vertices within a free space polygon.
@ingroup cpm
*/
#define CMS_CPM_FREE_SPACE_POLYGON_VERTEX_NUM_MAX 16U

/**
Free space polygon vertex list.
@ingroup cpm
*/
struct cms_cpm_free_space_polygon_t {
    uint32_t vertex_num;                                                                /**< Number of vertices */
    cms_cpm_free_space_point_t vertices[CMS_CPM_FREE_SPACE_POLYGON_VERTEX_NUM_MAX];     /**< Vertex coordinates */
} CMS_PACKED;

/**
Geometrical description of a circle-shaped shaped free space area.
@ingroup cpm
*/
struct cms_cpm_free_space_circle_t {
    cms_cpm_free_space_point_t center;      /**< Center point */
    cms_length_t radius;                    /**< Radius */
} CMS_PACKED;

/**
Geometrical description of a ellipse-shaped free space area.
@ingroup cpm
*/
struct cms_cpm_free_space_ellipse_t {
    cms_cpm_free_space_point_t center;      /**< Center point */
    cms_length_t semi_major_length;         /**< Semi-major axis length */
    cms_length_t semi_minor_length;         /**< Semi-minor axis length */
    cms_heading_t semi_major_direction;     /**< Semi-major axis direction */
} CMS_PACKED;

/**
Geometrical description of a rectangular free space area.
@ingroup cpm
*/
struct cms_cpm_free_space_rectangle_t {
    cms_cpm_free_space_point_t center;      /**< Center point */
    cms_length_t semi_major_length;         /**< Semi-major axis length */
    cms_length_t semi_minor_length;         /**< Semi-minor axis length */
    cms_heading_t semi_major_direction;     /**< Semi-major axis direction */
} CMS_PACKED;

/**
Geometrical description of a free space area.
@ingroup cpm
*/
struct cms_cpm_free_space_area_t {
    cms_cpm_free_space_area_type_t type;    /**< Shape type */
    cms_cpm_free_space_polygon_t polygon;       /**< Polygon description */
    cms_cpm_free_space_circle_t circle;         /**< Circle description */
    cms_cpm_free_space_ellipse_t ellipse;       /**< Ellipse description */
    cms_cpm_free_space_rectangle_t rectangle;   /**< Rectangle description */
} CMS_PACKED;

/**
Maximum number of sensor IDs that take part in a free space area detection.
@ingroup cpm
*/
#define CMS_CPM_SENSOR_ID_NUM_MAX       8U

/**
Free space area description.
@ingroup cpm
*/
struct cms_cpm_free_space_t {
    /** Free space ID */
    cms_cpm_id_t id;

    /** Geometric description of ree space area */
    cms_cpm_free_space_area_t area;

    /** Describes an isotropic free space confidence that applies to the entire area as defined in the area field */
    cms_probability_t confidence;

    /** Number of items in the snesor ID list */
    uint32_t sensor_id_num;

    /** List of sensor IDs which performed the measurement to indicate the free space */
    cms_cpm_id_t sensor_ids[CMS_CPM_SENSOR_ID_NUM_MAX];

    /** Indicates if the simple shadowing mechanism  applies within the area described */
    bool shadowing_applies;

    /** Validity timeout. Has to be positive */
    cms_expiry_time_ms_t timeout;
} CMS_PACKED;

/**
Maximum length of an ASN.1 encoded sensor information object.
@ingroup cpm
*/
#define CMS_CPM_SENSOR_DATA_LENGTH_MAX  128U

/**
Sensor information description.
@ingroup cpm
*/
struct cms_cpm_sensor_info_t {
    uint8_t data[CMS_CPM_SENSOR_DATA_LENGTH_MAX];   /**< ASN.1 encoded SensorInformation data */
    uint32_t data_length;                           /**< Data length */
} CMS_PACKED;


/**
Reference for an existing object or free space
@ingroup cpm
*/
struct cms_cpm_element_ref_t {
    cms_cpm_id_t id;        /**< Object/free space id */
} CMS_PACKED;


/**
Add/update the measurement data corresponding to a perceived object to the perception database.
@ingroup cpm
@experimentalapi

@param session      Client session
@param object       Perceived object description
@param unused_out   Unused output
@return true in case of error
@see cms_cpm_perceived_object_t
*/
bool cms_cpm_add_perceived_object(const cms_session_t* session,
                                  const cms_cpm_perceived_object_t* object,
                                  void* unused_out);

/**
Remove an object from the perception database.
@ingroup cpm
@experimentalapi

@param session      Client session
@param to_remove    Reference to the object to remove
@param unused_out   Unused output
@return true in case of error
*/
bool cms_cpm_remove_perceived_object(const cms_session_t* session,
                                     const cms_cpm_element_ref_t* to_remove,
                                     void* unused_out);

/**
Add/update a free space area to the perception database.
@ingroup cpm
@experimentalapi

@param session      Client session
@param free_space   Free space area description
@param unused_out   Unused output
@return true in case of error
@see cms_cpm_free_space_t
*/
bool cms_cpm_add_free_space(const cms_session_t* session,
                            const cms_cpm_free_space_t* free_space,
                            void* unused_out);

/**
Remove free space area from the perception database.
@ingroup cpm
@experimentalapi

@param session      Client session
@param to_remove    Reference to the free space to remove
@param unused_out   Unused output
@return true in case of error
@see cms_cpm_id_t
*/
bool cms_cpm_remove_free_space(const cms_session_t* session,
                               const cms_cpm_element_ref_t* to_remove,
                               void* unused_out);

/**
Add information about a sensor to the perception database.
@ingroup cpm
@experimentalapi

@param session      Client session
@param sensor_info  Sensor description
@param unused_out   Unused output
@return true in case of error
@see cms_cpm_sensor_info_t
*/
bool cms_cpm_add_sensor_info(const cms_session_t* session,
                             const cms_cpm_sensor_info_t* sensor_info,
                             void* unused_out);


#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_CPM_H_ */
