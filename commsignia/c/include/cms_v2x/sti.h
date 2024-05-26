/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#ifndef CMS_V2X_STI_H_
#define CMS_V2X_STI_H_

/** @file
@brief Station Info
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup sti Station Info
@ingroup api
*/

/**
Maximum number of StationInfo parameters that can be set or get in one batch.
@ingroup sti
*/
#define CMS_STI_MAX_VALUES          48UL

/**
Not available value for all StationInfo parameters
@ingroup sti
*/
#define CMS_STI_VALUE_NA            INT64_MIN

/**
Minimum value for all StationInfo parameters which are not enumerated types
@ingroup sti
*/
#define CMS_STI_VALUE_OOR_MIN       (INT64_MIN + 1LL)

/**
Maximum value for all StationInfo parameters which are not enumerated types
@ingroup sti
*/
#define CMS_STI_VALUE_OOR_MAX       INT64_MAX


/**
Possible values for Transmission State StationInfo parameter
@ingroup sti
@note Data element is used to provide the current state of the vehicle transmission

@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.201 DE_TransmissionState
*/
typedef enum cms_sti_transmission_state_t {
    CMS_STI_TRANSMISSION_STATE_NEUTRAL = 0,                     /**< Neutral state */
    CMS_STI_TRANSMISSION_PARK,                                  /**< Parking state */
    CMS_STI_TRANSMISSION_FWD_GEARS,                             /**< Forward gears */
    CMS_STI_TRANSMISSION_REVERSE_GEARS                          /**< Reverse gears */
} cms_sti_transmission_state_t;

/**
Possible values for Auxiliary Brake StationInfo parameter
@ingroup sti
@note The status of the auxiliary brakes (sometimes referred to as the
      parking brake) of the vehicle. The auxiliary brakes are in a fully
      released (Off) state or in an engaged or in the process of
      being engaged (On) state

@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.14 DE_AuxiliaryBrakeStatus
*/
typedef enum cms_sti_aux_brakes_t {
    CMS_STI_AUX_BRAKES_OFF = 0,                                 /**< Vehicle's Aux Brakes are off */
    CMS_STI_AUX_BRAKES_ON                                       /**< Vehicle's Aux Brakes are on (engaged) */
} cms_sti_aux_brakes_t;

/**
Possible values for Vehicle Length Confidence StationInfo parameter
@ingroup sti
@note To indicate whether the presence of a trailer is detectable or
      whether its length is included in a reported vehicle length value.

@see Intelligent Transport Systemst (ITS); User and application requirements
     ETSI TS 102 894-2 V1.3.1; Part2; Annex 91 - DE_VehicleLengthConfidenceIndication
*/
typedef enum cms_sti_vehicle_length_conf_t {
    CMS_STI_VEHICLE_LENGTH_CONF_NO_TRAILER = 0,                 /**< No trailer is present */
    CMS_STI_VEHICLE_LENGTH_CONF_TRAILER_WITH_LENGTH,            /**< Trailer present with known length */
    CMS_STI_VEHICLE_LENGTH_CONF_TRAILER_WITH_NA_LENGTH,         /**< Trailer present with unknown length */
    CMS_STI_VEHICLE_LENGTH_CONF_TRAILER_PRESENCE_IS_UNK         /**< Trailer presence is unknown */
} cms_sti_vehicle_length_conf_t;

/**
Possible values for Dangerous Goods StationInfo parameter
@ingroup sti
@note Indicates the type of the dangerous goods being carried by a heavy vehicle.

@see European Agreement (Applicable as from 1 January 2011): "Concerning the International Carriage
     of Dangerous Goods by Road" part II, chapter 2.1.1.1
@see Available at http://www.unece.org/trans/danger/publi/adr/adr2011/11ContentsE.html
*/
typedef enum cms_sti_dangerous_good_t {
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES1 = 0,                    /**< Explosive substance type 1 */
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES2,                        /**< Explosive substance type 2 */
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES3,                        /**< Explosive substance type 3 */
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES4,                        /**< Explosive substance type 4 */
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES5,                        /**< Explosive substance type 5 */
    CMS_STI_DANGEROUS_GOODS_EXPLOSIVES6,                        /**< Explosive substance type 6 */
    CMS_STI_DANGEROUS_GOODS_FLAMMABLE_GASES,                    /**< Flammable gases */
    CMS_STI_DANGEROUS_GOODS_NON_FLAMMABLE_GASES,                /**< Non flammable gases */
    CMS_STI_DANGEROUS_GOODS_TOXIC_GASES,                        /**< Toxic gases */
    CMS_STI_DANGEROUS_GOODS_FLAMMABLE_LIQUIDS,                  /**< Flammable liquids */
    CMS_STI_DANGEROUS_GOODS_FLAMMABLE_SOLIDS,                   /**< Flammable solids */
    CMS_STI_DANGEROUS_GOODS_SUBSTANCES_SPONTAIN_COMBUSTION,     /**< Substances liable to spontaneous combustion */
    CMS_STI_DANGEROUS_GOODS_SUBSTANCES_FLAMBLE_CONTACT_WATER,   /**< Substances emitting flammable gases upon contact with water */
    CMS_STI_DANGEROUS_GOODS_OXIDIZING_SUBSTANCES,               /**< Oxidizing substances */
    CMS_STI_DANGEROUS_GOODS_ORGANIC_PEROXIDES,                  /**< Organic peroxides */
    CMS_STI_DANGEROUS_GOODS_TOXIC_SUBSTANCES,                   /**< Toxic substances */
    CMS_STI_DANGEROUS_GOODS_INFECTIOUS_SUBSTANCES,              /**< Infectious substances */
    CMS_STI_DANGEROUS_GOODS_RADIOACTIVE_MATERIAL,               /**< Radioactive material */
    CMS_STI_DANGEROUS_GOODS_CORROSIVE_SUBSTANCES,               /**< Corrosive substances */
    CMS_STI_DANGEROUS_GOODS_MISC_DANGEROUS_SUBSTANCES           /**< Miscellaneous dangerous substances */
} cms_sti_dangerous_good_t;

/**
Possible values for Station type parameter
@ingroup sti
@note Not to be confused with vehicle role. It may seem redundant,
      but while class doesn't change for the lifetime of the vehicle,
      role may change based on what it's doing currently.
@note It is used to provide a common classification system to categorize DSRC
      equipped devices for various cross-cutting uses. Several other
      classification systems in this data dictionary can be used to provide
      more domain specific detail when required.

@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.15 DE_BasicVehicleClass
*/
typedef enum cms_sti_station_type_t {
    CMS_STI_STATION_TYPE_UNK = 0,                              /**< Not equipped, not known or unavailable */
    CMS_STI_STATION_TYPE_SPECIAL,                              /**< Special use */
    CMS_STI_STATION_TYPE_PASSENGER_UNK,                        /**< Unknown type passenger vehicle */
    CMS_STI_STATION_TYPE_PASSENGER_OTHER,                      /**< Other type passenger vehicle */
    CMS_STI_STATION_TYPE_LIGHT_TRUCK_UNK,                      /**< Unknown type light truck */
    CMS_STI_STATION_TYPE_LIGHT_TRUCK_OTHER,                    /**< Other type light truck */
    CMS_STI_STATION_TYPE_TRUCK_UNK,                            /**< Unknown type truck */
    CMS_STI_STATION_TYPE_TRUCK_OTHER,                          /**< Other type truck */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_2,                     /**< Two axle, six tire single units */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_3,                     /**< Three axle single units */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_4,                     /**< Four or more axle single unit */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_4_TRAILER,             /**< Four or less axle single trailer */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_5_TRAILER,             /**< Five or less axle single trailer */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_6_TRAILER,             /**< Six or more axle single trailer */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_5_MULTI_TRAILER,       /**< Five or less axle multi-trailer */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_6_MULTI_TRAILER,       /**< Six axle multi-trailer */
    CMS_STI_STATION_TYPE_TRUCK_AXLE_CNT_7_MULTI_TRAILER,       /**< Seven or more axle multi-trailer */
    CMS_STI_STATION_TYPE_MOTORCYCLE_UNK,                       /**< Unknown type motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_OTHER,                     /**< Other type motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_CRUISER_STANDARD,          /**< Cruiser standard motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_SPORT_UNCLAD,              /**< Unclad sport motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_SPORT_TOURING,             /**< Sport touring motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_SUPER_SPORT,               /**< Super sport motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_TOURING,                   /**< Touring motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_TRIKE,                     /**< Trike motorcycle */
    CMS_STI_STATION_TYPE_MOTORCYCLE_WITH_PASSENGERS,           /**< Motorcycle with passengers */
    CMS_STI_STATION_TYPE_TRANSIT_UNK,                          /**< Unknown type transit */
    CMS_STI_STATION_TYPE_TRANSIT_OTHER,                        /**< Other type transit */
    CMS_STI_STATION_TYPE_TRANSIT_BRT,                          /**< Bus rapid transit */
    CMS_STI_STATION_TYPE_TRANSIT_EXPRESS_BUS,                  /**< Express bus */
    CMS_STI_STATION_TYPE_TRANSIT_LOCAL_BUS,                    /**< Local bus */
    CMS_STI_STATION_TYPE_TRANSIT_SCHOOL_BUS,                   /**< School bus */
    CMS_STI_STATION_TYPE_TRANSIT_FIXED_GUIDE_WAY,              /**< Fixed guideway transit, like tram */
    CMS_STI_STATION_TYPE_TRANSIT_PARATRANSIT,                  /**< Paratransit */
    CMS_STI_STATION_TYPE_TRANSIT_PARATRANSIT_AMBULANCE,        /**< Paratransit ambulance */
    CMS_STI_STATION_TYPE_EMERGENCY_UNK,                        /**< Unknown type emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_OTHER,                      /**< Other type emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_FIRE_LIGHT,                 /**< Light fire emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_FIRE_HEAVY,                 /**< Heavy fire emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_FIRE_PARAMEDIC,             /**< Fire paramedic emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_FIRE_AMBULANCE,             /**< Fire ambulance emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_POLICE_LIGHT,               /**< Light police emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_POLICE_HEAVY,               /**< Heavy police emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_OTHER_RESPONDER,            /**< Other type responder emergency vehicle */
    CMS_STI_STATION_TYPE_EMERGENCY_OTHER_AMBULANCE,            /**< Other type ambulance emergency vehicle */
    CMS_STI_STATION_TYPE_OTHER_UNK,                            /**< DSRC equipped unknown type other traveler */
    CMS_STI_STATION_TYPE_OTHER_OTHER,                          /**< DSRC equipped other type other traveler */
    CMS_STI_STATION_TYPE_OTHER_PEDESTRIAN,                     /**< DSRC equipped other type pedestrian */
    CMS_STI_STATION_TYPE_OTHER_VISUALLY_DISABLED,              /**< DSRC equipped visually disabled other traveler */
    CMS_STI_STATION_TYPE_OTHER_PHYSICALLY_DISABLED,            /**< DSRC equipped physically disabled other traveler */
    CMS_STI_STATION_TYPE_OTHER_BICYCLE,                        /**< DSRC equipped bicycle */
    CMS_STI_STATION_TYPE_OTHER_VULNERABLE_ROADWORKER,          /**< DSRC equipped vulnerable road worker */
    CMS_STI_STATION_TYPE_INFRASTRUCTURE_UNK,                   /**< DSRC equipped unknown type device */
    CMS_STI_STATION_TYPE_INFRASTRUCTURE_FIXED,                 /**< DSRC equipped fixed device, typically Road Side Units (RSU) */
    CMS_STI_STATION_TYPE_INFRASTRUCTURE_MOVABLE,               /**< DSRC equipped movable device */
    CMS_STI_STATION_TYPE_EQUIPPED_CARGO_TRAILER,               /**< DSRC equipped cargo trailer */
    CMS_STI_STATION_TYPE_LIGHT_VRU_VEHICLE,                    /**< Micrmomobility users (EU CDD release 2) */
    CMS_STI_STATION_TYPE_ANIMAL                                /**< DSRC equipped animals (EU CDD release 2) */
} cms_sti_station_type_t;

/**
Possible values for Basic Vehicle Role StationInfo parameter
@ingroup sti
@note Not to be confused with vehicle class. It may seem redundant,
      but while class doesn't change for the lifetime of the vehicle,
      role may change based on what it's doing currently,
      e.g. simple passenger vehicle can be a safety car while it's in service,
      but a basic vehicle when it's not.

@see Intelligent Transport Systemst (ITS); User and application requirements
     ETSI TS 102 894-2 V1.3.1; Part2; Annex 94 - DE_VehicleRole
@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.16 DE_BasicVehicleRole
*/
typedef enum cms_sti_vehicle_role_t {
    CMS_STI_VEHICLE_ROLE_BASIC_VEHICLE = 0,                     /**< Default vehicle role as indicated by the vehicle type */
    CMS_STI_VEHICLE_ROLE_PUBLIC_TRANSPORT,                      /**< Vehicle is used to operate public transport service */
    CMS_STI_VEHICLE_ROLE_SPECIAL_TRANSPORT,                     /**< Vehicle is used for special transport purpose, like oversized trucks */
    CMS_STI_VEHICLE_ROLE_DANGEROUS_GOODS,                       /**< Vehicle is used for dangerous goods transportation */
    CMS_STI_VEHICLE_ROLE_ROAD_WORK,                             /**< Vehicle is used to realize roadwork or road maintenance mission */
    CMS_STI_VEHICLE_ROLE_RESCUE,                                /**< Vehicle is used for rescue purpose in case of an accident, like as a towing service */
    CMS_STI_VEHICLE_ROLE_EMERGENCY,                             /**< Vehicle is used for emergency mission, like ambulance, fire brigade */
    CMS_STI_VEHICLE_ROLE_SAFETY_CAR,                            /**< Vehicle is used for public safety, like patrol */
    CMS_STI_VEHICLE_ROLE_TRUCK,                                 /**< Heavy trucks with additional BSM rights and obligations */
    CMS_STI_VEHICLE_ROLE_MOTORCYCLE,                            /**< Motorcycle role */
    CMS_STI_VEHICLE_ROLE_ROAD_SIDE_SOURCE,                      /**< For infrastructure generated calls such as fire house, rail infrastructure, roadwork site, etc. */
    CMS_STI_VEHICLE_ROLE_POLICE,                                /**< Police role */
    CMS_STI_VEHICLE_ROLE_FIRE,                                  /**< Fire role */
    CMS_STI_VEHICLE_ROLE_AMBULANCE,                             /**< (does not include private para-transit etc.) */
    CMS_STI_VEHICLE_ROLE_DOT,                                   /**< All roadwork vehicles */
    CMS_STI_VEHICLE_ROLE_TRANSIT,                               /**< All transit vehicles */
    CMS_STI_VEHICLE_ROLE_SLOW_MOVING,                           /**< Also include oversize etc. */
    CMS_STI_VEHICLE_ROLE_STOP_N_GO,                             /**< To include trash trucks, school buses and others that routinely disturb the free flow of traffic */
    CMS_STI_VEHICLE_ROLE_CYCLIST,                               /**< Cyclist role */
    CMS_STI_VEHICLE_ROLE_PEDESTRIAN,                            /**< Also includes those with mobility limitations */
    CMS_STI_VEHICLE_ROLE_NON_MOTORIZED,                         /**< other, such as horse drawn */
    CMS_STI_VEHICLE_ROLE_MILITARY,                              /**< Military role */
    CMS_STI_VEHICLE_ROLE_AGRICULTURE,                           /**< Vehicle is used for agriculture, like farm tractor */
    CMS_STI_VEHICLE_ROLE_COMMERCIAL,                            /**< Vehicle is used for transportation of commercial goods */
    CMS_STI_VEHICLE_ROLE_ROAD_OPERATOR,                         /**< Vehicle is used in road operator missions */
    CMS_STI_VEHICLE_ROLE_TAXI                                   /**< Vehicle is used to provide an authorized taxi service */
} cms_sti_vehicle_role_t;

/**
Possible values for Weather Precipitation Situation StationInfo parameter
@ingroup sti
@note Describes the weather situation in terms of precipitation.
*/
typedef enum cms_sti_weather_precip_situation_t {
    CMS_STI_WEATHER_PRECIP_SITUATON_OTHER = 0,                  /**< Other type precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_UNK,                        /**< Unknown type precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_NO_PRECIP,                  /**< No precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_UNK_SLIGHT,                 /**< Unknown type slight precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_UNK_MODERATE,               /**< Unknown type moderate precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_UNK_HEAVY,                  /**< Unknown type heavy precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_SNOW_SLIGHT,                /**< Slight snow precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_SNOW_MODERATE,              /**< Moderate snow precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_SNOW_HEAVY,                 /**< Heavy snow precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_RAIN_SLIGHT,                /**< Slight rain precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_RAIN_MODERATE,              /**< Moderate rain precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_RAIN_HEAVY,                 /**< Heavy rain precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_FROZEN_PRECIP_SLIGHT,       /**< Slight frozen precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_FROZEN_PRECIP_MODERATE,     /**< Moderate frozen precipitation */
    CMS_STI_WEATHER_PRECIP_SITUATON_FROZEN_PRECIP_HEAVY         /**< Heavy frozen precipitation */
} cms_sti_weather_precip_situation_t;

/**
Possible values for Wiper State StationInfo parameter
@ingroup sti
@note It is intended to inform other users whether or not it was
      raining/snowing at the vehicle's location at the time it was taken.
      The element also includes whether the wipers were turned on manually
      (driver activated) or automatically (rain sensor activated) to provide
      additional information as to driving conditions in the area of the vehicle.

@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.227 DE_WiperStatus
*/
typedef enum cms_sti_wiper_state_t {
    CMS_STI_WIPER_STATE_OFF = 0,                                /**< Off state */
    CMS_STI_WIPER_STATE_INTERMITTENT,                           /**< Intermittent state */
    CMS_STI_WIPER_STATE_LOW,                                    /**< Low state */
    CMS_STI_WIPER_STATE_HIGH,                                   /**< High state */
    CMS_STI_WIPER_STATE_WASHER_IN_USE,                          /**< Washer in use */
    CMS_STI_WIPER_STATE_AUTO                                    /**< Auto wiper state */
} cms_sti_wiper_state_t;

/**
Possible values for Lightbar StationInfo parameter
@ingroup sti
@note It is intended to inform if any sort of additional visible
      lighting-alerting system is currently in use by a vehicle.
      This includes light bars and the various symbols they can indicate
      as well as arrow boards, flashing lights (including back up alerts),
      and any other form of lighting not found on normal vehicles of this type
      or related to safety systems. Used to reflect any type or style of visual
      alerting when a vehicle is progressing and transmitting DSRC messages
      to other nearby vehicles about its path.
*/
typedef enum cms_sti_lightbar_t {
    CMS_STI_LIGHTBAR_NOT_IN_USE = 0,                            /**< There is a lightbar, but it is currently not in use */
    CMS_STI_LIGHTBAR_IN_USE,                                    /**< Lightbar in use */
    CMS_STI_LIGHTBAR_YELLOW_CAUTION_LIGHTS,                     /**< Yellow caution lights */
    CMS_STI_LIGHTBAR_SCHOOL_BUS_LIGHTS,                         /**< School bus lights */
    CMS_STI_LIGHTBAR_ARROW_SIGNS_ACTIVE,                        /**< Arrow signs active */
    CMS_STI_LIGHTBAR_SLOW_MOVING_VEHICLE,                       /**< Slow moving vehicle lights */
    CMS_STI_LIGHTBAR_FREQ_STOPS                                 /**< Freq stops */
} cms_sti_lightbar_t;

/**
Possible values for Siren StationInfo parameter
@ingroup sti
@note A data element which is set if any sort of audible alarm is being
      emitted from the vehicle. This includes various common sirens as well
      as backup beepers and other slow speed maneuvering alerts.

@see Dedicated Short Range Communications (DSRC) Message Set Dictionary
     SAE J2735_201603 Chapter 7.174 DE_SirenInUse
*/
typedef enum cms_sti_siren_t {
    CMS_STI_SIREN_NOT_IN_USE = 0,                               /**< Siren not in use */
    CMS_STI_SIREN_IN_USE                                        /**< Siren in use */
} cms_sti_siren_t;

/**
Possible values for Lane Position StationInfo parameter
@ingroup sti
@note Data frame is used to provide information regarding
      what lane a subject vehicle (or other object) is in.
      ETSI TS 102 894-2 V1.3.1 supports only 13 lanes.
*/
typedef enum cms_sti_lane_pos_t {
    CMS_STI_LANE_POSITION_INNER_HARD_SHOULDER = 0,              /**< Inner hard shoulder position */
    CMS_STI_LANE_POSITION_INNERMOST_DRIVING_LANE,               /**< Innermost driving lane */
    CMS_STI_LANE_POSITION_SECOND_LANE_FROM_INSIDE,              /**< Second lane from inside */
    CMS_STI_LANE_POSITION_THIRD_LANE_FROM_INSIDE,               /**< Third lane from inside */
    CMS_STI_LANE_POSITION_FOURTH_LANE_FROM_INSIDE,              /**< Fourth lane from inside */
    CMS_STI_LANE_POSITION_FIFTH_LANE_FROM_INSIDE,               /**< Fifth lane from inside */
    CMS_STI_LANE_POSITION_SIXTH_LANE_FROM_INSIDE,               /**< Sixth lane from inside */
    CMS_STI_LANE_POSITION_SEVENTH_LANE_FROM_INSIDE,             /**< Seventh lane from inside */
    CMS_STI_LANE_POSITION_EIGHTH_LANE_FROM_INSIDE,              /**< Eighth lane from inside */
    CMS_STI_LANE_POSITION_NINTH_LANE_FROM_INSIDE,               /**< Ninth lane from inside */
    CMS_STI_LANE_POSITION_TENTH_LANE_FROM_INSIDE,               /**< Tenth lane from inside */
    CMS_STI_LANE_POSITION_ELEVENTH_LANE_FROM_INSIDE,            /**< Eleventh lane from inside */
    CMS_STI_LANE_POSITION_TWELFTH_LANE_FROM_INSIDE,             /**< Twelfth lane from inside */
    CMS_STI_LANE_POSITION_THIRTEENTH_LANE_FROM_INSIDE,          /**< Thirteenth lane from inside */
    CMS_STI_LANE_POSITION_FOURTEENTH_LANE_FROM_INSIDE,          /**< Fourteenth lane from inside */
    CMS_STI_LANE_POSITION_FIFTEENTH_LANE_FROM_INSIDE,           /**< Fifteenth lane from inside */
    CMS_STI_LANE_POSITION_SIXTEENTH_LANE_FROM_INSIDE,           /**< Sixteenth lane from inside */
    CMS_STI_LANE_POSITION_SEVENTEENTH_LANE_FROM_INSIDE,         /**< Seventeenth lane from inside */
    CMS_STI_LANE_POSITION_EIGHTEENTH_LANE_FROM_INSIDE,          /**< Eighteenth lane from inside */
    CMS_STI_LANE_POSITION_NINETEENTH_LANE_FROM_INSIDE,          /**< Nineteenth lane from inside */
    CMS_STI_LANE_POSITION_TWENTIETH_LANE_FROM_INSIDE,           /**< Twentieth lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_FIRST_LANE_FROM_INSIDE,        /**< Twenty-first lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_SECOND_LANE_FROM_INSIDE,       /**< Twenty-second lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_THIRD_LANE_FROM_INSIDE,        /**< Twenty-third lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_FOURTH_LANE_FROM_INSIDE,       /**< Twenty-fourth lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_FIFTH_LANE_FROM_INSIDE,        /**< Twenty-fifth lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_SIXTH_LANE_FROM_INSIDE,        /**< Twenty-sixth lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_SEVENTH_LANE_FROM_INSIDE,      /**< Twenty-seventh lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_EIGHTH_LANE_FROM_INSIDE,       /**< Twenty-eighth lane from inside */
    CMS_STI_LANE_POSITION_TWENTY_NINTH_LANE_FROM_INSIDE,        /**< Twenty-ninth lane from inside */
    CMS_STI_LANE_POSITION_THIRTIETH_LANE_FROM_INSIDE,           /**< Thirtieth lane from inside */
    CMS_STI_LANE_POSITION_THIRTY_FIRST_LANE_FROM_INSIDE,        /**< Thirty-first lane from inside */
    CMS_STI_LANE_POSITION_THIRTY_SECOND_LANE_FROM_INSIDE,       /**< Thirty-second lane from inside */
    CMS_STI_LANE_POSITION_OUTER_HARD_SHOULDER,                  /**< Outer hard shoulder position */
    CMS_STI_LANE_POSITION_OFF_THE_ROAD                          /**< Off the road position */
} cms_sti_lane_pos_t;

/**
Possible values for fuel type
@ingroup sti
*/
typedef enum cms_sti_fuel_type_t {
    CMS_STI_FUEL_TYPE_GASOLINE = 0,                             /**< Gasoline fuel type */
    CMS_STI_FUEL_TYPE_ETHANOL,                                  /**< Ethanol fuel type */
    CMS_STI_FUEL_TYPE_DIESEL,                                   /**< Diesel fuel type */
    CMS_STI_FUEL_TYPE_ELECTRIC,                                 /**< Electric fuel type */
    CMS_STI_FUEL_TYPE_HYBRID,                                   /**< Hybrid fuel type */
    CMS_STI_FUEL_TYPE_HYDROGEN,                                 /**< Hydrogen fuel type */
    CMS_STI_FUEL_TYPE_NAT_GAS_LIQUID,                           /**< NAT gas liquefied fuel type */
    CMS_STI_FUEL_TYPE_NAT_GAS_COMP,                             /**< NAT gas comppressed fuel type */
    CMS_STI_FUEL_TYPE_PROPANE,                                  /**< Propane fuel type */
} cms_sti_fuel_type_t;

/**
Possible values for environment area type
@ingroup sti
*/
typedef enum cms_sti_area_type_t {
    CMS_STI_AREA_TYPE_RURAL = 0,                                /**< Rural area */
    CMS_STI_AREA_TYPE_URBAN,                                    /**< Urban area */
} cms_sti_area_type_t;

/**
Possible values for door state related StationInfo parameters
@ingroup sti
*/
typedef enum cms_sti_door_state_t {
    CMS_STI_DOOR_STATE_NA = 0,                                  /**< N/A state */
    CMS_STI_DOOR_STATE_CLOSED,                                  /**< Closed state */
    CMS_STI_DOOR_STATE_OPEN,                                    /**< Open state */
} cms_sti_door_state_t;

/**
Possible values for belt buckle state related StationInfo parameters
@ingroup sti
*/
typedef enum cms_sti_belt_buckle_status_t {
    CMS_STI_BELT_BUCKLE_DISCONNECTED = 0,                       /**< Belt buckle is disconnected */
    CMS_STI_BELT_BUCKLE_CONNECTED,                              /**< Belt buckle is connected */
} cms_sti_belt_buckle_status_t;

/**
Possible values for Transmission type
@ingroup sti
*/
typedef enum cms_sti_transmission_type_t {
    CMS_STI_TRANSMISSION_TYPE_AUTOMATIC = 0,                    /**< Transmission type is automatic */
    CMS_STI_TRANSMISSION_TYPE_MANUAL,                           /**< Transmission type is manual */
} cms_sti_transmission_type_t;

/**
Possible values for presence of physical separation between own and oncoming lanes of the road/highway
@ingroup sti
*/
typedef enum cms_sti_physical_road_separation_t {
    CMS_STI_PHYSICAL_ROAD_SEPARATION_NOT_PRESENT = 0,           /**< Physical separation is not present */
    CMS_STI_PHYSICAL_ROAD_SEPARATION_PRESENT                    /**< Physical separation is present */
} cms_sti_physical_road_separation_t;

/**
Possible values for RoadClass parameter
@ingroup sti
*/
typedef enum cms_sti_road_class_t {
    CMS_STI_ROAD_CLASS_UNKNOWN = 0,                             /**< Unknown road class */
    CMS_STI_ROAD_CLASS_MOTORWAY,                                /**< Motorway road class */
    CMS_STI_ROAD_CLASS_COUNTRY_ROAD,                            /**< Country road road class */
    CMS_STI_ROAD_CLASS_LOCAL_ROAD,                              /**< Local road road class */
} cms_sti_road_class_t;

/**
Possible values for StationInfo parameters which have two states
@ingroup sti
*/
typedef enum cms_sti_state_t {
    CMS_STI_STATE_OFF = 0,                                      /**< Off state */
    CMS_STI_STATE_ON                                            /**< On state */
} cms_sti_state_t;

/**
Possible values for StationInfo parameters which have three states
@ingroup sti
*/
typedef enum cms_sti_tristate_t {
    CMS_STI_TRISTATE_OFF = 0,                                   /**< Off state */
    CMS_STI_TRISTATE_ON,                                        /**< On or active (but not engaged) state */
    CMS_STI_TRISTATE_ENGAGED                                    /**< Engaged state */
} cms_sti_tristate_t;

/**
Possible values for setting/getting a StationInfo parameter.
@ingroup sti
*/
typedef enum cms_sti_type_t {

    /**
    Transmission state
    @type [enum]
    @see cms_sti_transmission_state_t
    */
    CMS_STI_TRANSMISSION_STATE = 0,

    /**
    Steering wheel angle.
    @note Do not confuse with driving wheel angle.
          The steering wheel is what the driver holds in their hands,
          the driving wheel is what touches the ground.
    @type [angle]
    @unit [0.001 degree] CCW positive
    */
    CMS_STI_STEERING_WHEEL_ANGLE,

    /**
    Steering wheel angle confidence
    @type [angle]
    @unit [0.001 degree]
    */
    CMS_STI_STEERING_WHEEL_ANGLE_CONF,

    /**
    Steering wheel angle rate
    @type [angular velocity]
    @unit [0.001 degree/second]
    */
    CMS_STI_STEERING_WHEEL_ANGLE_RATE,

    /**
    Driving wheel angle
    @note Do not confuse with steering wheel angle.
          The steering wheel is what the driver holds in their hands,
          the driving wheel is what touches the ground.
    @type [angle]
    @unit [0.001 degree]
    */
    CMS_STI_DRIVING_WHEEL_ANGLE,

    /**
    Longitudinal acceleration
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_LONG_ACC,

    /**
    Longitudinal acceleration confidence
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_LONG_ACC_CONF,

    /**
    Lateral acceleration
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_LAT_ACC,

    /**
    Lateral acceleration confidence
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_LAT_ACC_CONF,

    /**
    Vertical acceleration
    @note If the car is standing still, this should be 0, i.e.
          gravity is not included in this value.
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_VERT_ACC,

    /**
    Vertical acceleration confidence
    @type [acceleration]
    @unit [mm/s^2]
    */
    CMS_STI_VERT_ACC_CONF,

    /**
    Yaw rate
    @type [angular velocity]
    @unit [0.001 degree/second] CCW positive
    */
    CMS_STI_YAW_RATE,

    /**
    Yaw rate confidence
    @type [angular velocity]
    @unit [0.001 degree/second]
    */
    CMS_STI_YAW_RATE_CONF,

    /**
    Front left brake applied pressure
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_BRAKE_STATUS_LEFT_FRONT,

    /**
    Rear left brake applied pressure
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_BRAKE_STATUS_LEFT_REAR,

    /**
    Front right brake applied pressure
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_BRAKE_STATUS_RIGHT_FRONT,

    /**
    Rear right brake applied pressure
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_BRAKE_STATUS_RIGHT_REAR,

    /**
    Traction control status
    @type [enum]
    @see cms_sti_tristate_t
    */
    CMS_STI_TRACTION_CONTROL_STATUS,

    /**
    Anti-lock braking system status
    @type [enum]
    @see cms_sti_tristate_t
    */
    CMS_STI_ABS,

    /**
    Stability control status
    @type [enum]
    @see cms_sti_tristate_t
    */
    CMS_STI_STABILITY_CONTROL_STATUS,

    /**
    Emergency brake status, also known as 'Brake boost'
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EMERGENCY_BRAKE,

    /**
    Auxiliary brake status
    @type [enum]
    @see cms_sti_aux_brakes_t
    */
    CMS_STI_AUX_BRAKES,

    /**
    Vehicle width
    @note The width shall be the widest point of the vehicle with all
          factory installed equipment.
    @type [length]
    @unit [mm]
    */
    CMS_STI_VEHICLE_WIDTH,

    /**
    Vehicle length
    @note The length of the vehicle measured from the edge of the front
          bumper to the edge of the rear bumper.
    @type [length]
    @unit [mm]
    */
    CMS_STI_VEHICLE_LENGTH,

    /**
    Vehicle length confidence
    @type [enum]
    @see cms_sti_vehicle_length_conf_t
    */
    CMS_STI_VEHICLE_LENGTH_CONF,

    /**
    Vehicle height
    @type [length]
    @unit [mm]
    */
    CMS_STI_VEHICLE_HEIGHT,

    /**
    Vehicle mass
    @type [mass]
    @unit [gram]
    */
    CMS_STI_VEHICLE_MASS,

    /**
    Stop line violated
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EV_STOP_LINE_VIOLATED,

    /**
    Dangerous goods present
    @note Indicates the type of the dangerous goods being carried by a heavy vehicle.
    @type [enum]
    @see cms_sti_dangerous_good_t
    */
    CMS_STI_DANGEROUS_GOODS,

    /**
    Flat tire
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EV_FLAT_TIRE,

    /**
    Disabled vehicle
    @note An equipped vehicle that has self-declared that it is not
          performing all designed/intended functions and/or operations.
          Such a vehicle may be moving or may be stationary.
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EV_DISABLED_VEHICLE,

    /**
    Airbag deployed
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EV_AIRBAG_DEPLOYED,

    /**
    Low beam headlight status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_LOWBEAM_HEAD,

    /**
    High beam headlight status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_HIGHBEAM_HEAD,

    /**
    Left turn signal status
    @note Keep in mind that this signal should be
          a stable 'on', and should not be blinking
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_LEFT_TURN_SIGNAL,

    /**
    Right turn signal status
    @note Keep in mind that this signal should be
          a stable 'on', and should not be blinking
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_RIGHT_TURN_SIGNAL,

    /**
    Hazard light status status
    @note Keep in mind that this signal should be
          a stable 'on', and should not be blinking.
          This value is independent from the turn signal and signifies
          the button itself on the dashboard. If this value cannot be
          obtained independently, it should be manually set to 'on' if
          both turn signals are 'on'.
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_HAZARD_LIGHT,

    /**
    Auto light control status status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_AUTO_LIGHT_CONTROL,

    /**
    Daytime running light status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_DAYTIME_RUNNING,

    /**
    Fog light status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_FOG,

    /**
    Parking light status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_PARKING,

    /**
    Reverse light status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EXT_LIGHT_REVERSE,

    /**
    Station type
    @note Not to be confused with vehicle role. It may seem redundant,
          but while class doesn't change for the lifetime of the vehicle,
          role may change based on what it's doing currently.
    @type [enum]
    @see cms_sti_station_type_t
    */
    CMS_STI_STATION_TYPE,

    /**
    Vehicle role
    @note Not to be confused with vehicle class. It may seem redundant,
          but while class doesn't change for the lifetime of the vehicle,
          role may change based on what it's doing currently,
          e.g. simple passenger vehicle can be a safety car while it's in service,
          but a basic vehicle when it's not.
    @type [enum]
    @see cms_sti_vehicle_role_t
    */
    CMS_STI_VEHICLE_ROLE,

    /**
    Height of the front bumper
    @type [length]
    @unit [mm]
    */
    CMS_STI_BUMPER_HEIGHT_FRONT,

    /**
    Height of the rear bumper
    @type [length]
    @unit [mm]
    */
    CMS_STI_BUMPER_HEIGHT_REAR,

    /**
    Rain rate
    @type [rate]
    @unit [0.1 g/s/m^2]
    */
    CMS_STI_WEATHER_RAIN_RATE,

    /**
    Rain sensor
    @note Describes the current sensor reading normalized to a 0-1000 range,
          where 0 is completely dry, and 1000 is the highest rain value the
          sensor is capable of detecting.
          Note that this is independent from CMS_STI_WEATHER_PRECIP_SITUATON,
          which is more of a semantic value. If both are needed, they need to
          be set individually.
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_RAIN_SENSOR,

    /**
    Precipitation situation
    @note Describes the weather situation in terms of precipitation.
    @type [enum]
    @see cms_sti_weather_precip_situation_t
    */
    CMS_STI_WEATHER_PRECIP_SITUATON,

    /**
    Solar radiation
    @type [solar irradiance]
    @unit [J/m^2]
    */
    CMS_STI_WEATHER_SOLAR_RADIATION,

    /**
    Coefficient of friction
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_WEATHER_COEF_FRICTION,

    /**
    Ambient air temperature
    @type [temperature]
    @unit [0.1C°]
    */
    CMS_STI_WEATHER_AIR_TEMP,

    /**
    Ambient air pressure
    @type [pressure]
    @unit [Pascal]
    */
    CMS_STI_WEATHER_AIR_PRESSURE,

    /**
    Front wiper state
    @type [enum]
    @see cms_sti_wiper_state_t
    */
    CMS_STI_WIPER_STATE_FRONT,

    /**
    Rear wiper state
    @type [enum]
    @see cms_sti_wiper_state_t
    */
    CMS_STI_WIPER_STATE_REAR,

    /**
    Front wiper rate
    @note Usually only makes sense if the wiper state is also set to a
          correct value. This has to be done manually.
    @type [rate]
    @unit [sweeps/minute]
    */
    CMS_STI_WIPER_RATE_FRONT,

    /**
    Rear wiper rate
    @note Usually only makes sense if the wiper state is also set to a
          correct value. This has to be done manually.
    @type [rate]
    @unit [sweeps/minute]
    */
    CMS_STI_WIPER_RATE_REAR,

    /**
    Embarkation status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_EMBARKATION_STATUS,

    /**
    Lightbar status
    @note It is intended to inform if any sort of additional visible
          lighting-alerting system is currently in use by a vehicle.
    @type [enum]
    @see cms_sti_lightbar_t
    */
    CMS_STI_LIGHTBAR,

    /**
    Siren status
    @note A data element which is set if any sort of audible alarm is being
          emitted from the vehicle.
    @type [enum]
    @see cms_sti_siren_t
    */
    CMS_STI_SIREN,

    /**
    Accelerator pedal position
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_ACCELERATOR_PEDAL,

    /**
    Brake pedal position
    @type [thousandths]
    @unit [parts-per-thousand]
    */
    CMS_STI_BRAKE_PEDAL,

    /**
    Collision warning
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_COLLISION_WARNING,

    /**
    Adaptive cruise control status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_ADAPTIVE_CRUISE_CONTROL,

    /**
    Cruise control status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_CRUISE_CONTROL,

    /**
    Speed limiter status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_SPEED_LIMITER,

    /**
    Lane position
    @type [enum]
    @see cms_sti_lane_pos_t
    */
    CMS_STI_LANE_POSITION,

    /**
    Trailer weight
    @type [weight]
    @unit [g]
    */
    CMS_STI_TRAILER_WEIGHT,

    /**
    Front left door state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_FRONT_LEFT,

    /**
    Front right door state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_FRONT_RIGHT,

    /**
    Rear left door state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_REAR_LEFT,

    /**
    Rear right door state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_REAR_RIGHT,

    /**
    Bonnet state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_BONNET,

    /**
    Trunk state
    @type [enum]
    @see cms_sti_door_state_t
    */
    CMS_STI_DOOR_STATE_TRUNK,

    /**
    Fuel type
    @type [enum]
    @see cms_sti_fuel_type_t
    */
    CMS_STI_FUEL_TYPE,

    /**
    Road class
    @type [enum]
    @see cms_sti_road_class_t
    */
    CMS_STI_ROAD_CLASS,

    /**
    Area type
    @type [enum]
    @see cms_sti_area_type_t
    */
    CMS_STI_AREA_TYPE,

    /**
    Automatic emergency brake status
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_AUTOMATIC_EMERGENCY_BRAKE,

    /**
    Reversible Occupant Restraint System status (e.g. reversible belt-tightener)
    @note Activate when any of the applicable system is active.
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_REVERSIBLE_OCCUPANT_RESTRAINT_SYSTEM,

    /**
    A red warning light is active in the vehicle. (See ECE 121)
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_RED_WARNING_ACTIVE,

    /**
    First row belt buckle status on the driver side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW1_DRIVER,

    /**
    First row belt buckle status on the middle.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW1_MIDDLE,

    /**
    First row belt buckle status on the passenger side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW1_PASSENGER,

    /**
    Second row belt buckle status on the driver side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW2_DRIVER,

    /**
    Second row belt buckle status on the middle.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW2_MIDDLE,

    /**
    Second row belt buckle status on the passenger side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW2_PASSENGER,

    /**
    Third row belt buckle status on the driver side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW3_DRIVER,

    /**
    Third row belt buckle status on the middle.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW3_MIDDLE,

    /**
    Third row belt buckle status on the passenger side.
    @type [enum]
    @see cms_sti_belt_buckle_status_t
    */
    CMS_STI_BELT_BUCKLE_ROW3_PASSENGER,

    /**
    Ignition status of the vehicle. (terminal 15)
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_IGNITION,

    /**
    Type of the transmission. (automatic or manual)
    @type [enum]
    @see cms_sti_transmission_type_t
    */
    CMS_STI_TRANSMISSION_TYPE,

    /**
    Presence of physical separation between own and oncoming lanes of the road/highway
    @type [enum]
    @see cms_sti_physical_road_separation_t
    */
    CMS_STI_PHYSICAL_ROAD_SEPARATION,

    /**
    An eCall has been triggered manually by an occupant of the vehicle by the eCall
    button.
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_MANUAL_ECALL,

    /**
    Low severity crash crash is detected without the activation of an irreversible occupant
    restraint system. (e.g. high-voltage battery cut-off, door unlock)
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_LOW_SEVERITY_CRASH,

    /**
    Pedestrian collision is detected with the activation of at least one irreversible
    pedestrian-protection system. (e.g. pop-up bonnet, outside airbag)
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_PEDESTRIAN_COLLISION,

    /**
    High severity crash is detected with the activation of at least one irreversible
    occupant-restraint system. (e.g. pyrotechnic belt-tightener, airbag)
    @type [enum]
    @see cms_sti_state_t
    */
    CMS_STI_HIGH_SEVERITY_CRASH,

    /**
    Project specific STI values.
    @type [integer]
    @unit [varying]
    */
    CMS_STI_PROJECT_00 = 128,
    CMS_STI_PROJECT_01,
    CMS_STI_PROJECT_02,
    CMS_STI_PROJECT_03,
    CMS_STI_PROJECT_04,
    CMS_STI_PROJECT_05,
    CMS_STI_PROJECT_06,
    CMS_STI_PROJECT_07,
    CMS_STI_PROJECT_08,
    CMS_STI_PROJECT_09,
    CMS_STI_PROJECT_10,
    CMS_STI_PROJECT_11,
    CMS_STI_PROJECT_12,
    CMS_STI_PROJECT_13,
    CMS_STI_PROJECT_14,
    CMS_STI_PROJECT_15,
    CMS_STI_PROJECT_16,
    CMS_STI_PROJECT_17,
    CMS_STI_PROJECT_18,
    CMS_STI_PROJECT_19,
    CMS_STI_PROJECT_20,
    CMS_STI_PROJECT_21,
    CMS_STI_PROJECT_22,
    CMS_STI_PROJECT_23,
    CMS_STI_PROJECT_24,
    CMS_STI_PROJECT_25,
    CMS_STI_PROJECT_26,
    CMS_STI_PROJECT_27,
    CMS_STI_PROJECT_28,
    CMS_STI_PROJECT_29,
    CMS_STI_PROJECT_30,
    CMS_STI_PROJECT_31,

    /** Number of STI types, do not use as a parameter */
    CMS_STI_TYPE_LAST
} cms_sti_type_t;


/**
Value type for all station info parameters
@ingroup sti
*/
typedef int64_t cms_sti_value_t;

/**
Station info type-value pair
@ingroup sti
*/
typedef struct cms_sti_item_t {
    cms_sti_type_t type;                        /**< STI parameter type */
    cms_sti_value_t value;                      /**< STI parameter value */
} CMS_PACKED cms_sti_item_t;

/**
List of station info type-value pairs.
@ingroup sti
*/
typedef struct cms_sti_item_list_t {
    uint32_t length;                            /**< Length of item list */
    cms_sti_item_t items[CMS_STI_MAX_VALUES];   /**< Item list */
} CMS_PACKED cms_sti_item_list_t;

/**
Input structure for set station info
@ingroup sti
@see cms_sti_get
*/
typedef cms_sti_item_list_t cms_sti_set_items_t;

/**
List of station info types.
@ingroup sti
@see cms_sti_get
*/
typedef struct cms_sti_type_list_t {
    uint32_t length;                            /**< Length of type list */
    cms_sti_type_t types[CMS_STI_MAX_VALUES];   /**< Type list */
} CMS_PACKED cms_sti_type_list_t;

/**
Maximum length of stored station id
@ingroup sti
@see cms_sti_get_station_id
@see cms_sti_set_station_id
*/
#define CMS_STI_STATION_ID_LENGTH   16

/**
Station id
@ingroup sti
@see cms_sti_get_station_id
@see cms_sti_set_station_id
*/
struct cms_sti_station_id_t {
    uint8_t id[CMS_STI_STATION_ID_LENGTH];
};

/**
Set one or more station information data elements.
@ingroup sti
Any element can be N/A or out-of-range in both directions.
Setting a value to N/A usually means that the value will not be included in subsequent messages.

There is no implicit priority or expiration of data values:
always the last set value is used. The integration
(e.g. the module which does CAN translation) is expected to handle these
cases and set the values to N/A if needed.

Integration note: for CAN, or with other protocols where multiple values
arrive in the same packet, it is advised that a single sti_set call is made
for each incoming packet (e.g. CAN frame). This way the CAN frame content
need not be separated or collected.

@param session        Client session
@param[in] items      STI item list, includes the values to set
@param unused_out     Unused parameter
@return true in case of error
*/
bool cms_sti_set(const cms_session_t* session,
                 const cms_sti_set_items_t* items,
                 void* unused_out);

/**
Get one or more station information data elements.
@ingroup sti
Any of the returned values may be N/A or OOR.

@param session            Client session
@param[in] types          STI types input
@param[out] result_items  STI values output
@return true in case of error
*/
bool cms_sti_get(const cms_session_t* session,
                 const cms_sti_type_list_t* types,
                 cms_sti_item_list_t* result_items);

/**
Set station ID.
@ingroup sti
@param session        Client session
@param[in] station_id Station ID value to set
@param unused_out     Unused parameter
@return true in case of error
*/
bool cms_sti_set_station_id(const cms_session_t* session,
                            const cms_sti_station_id_t* station_id,
                            void* unused_out);

/**
Get station ID.
@ingroup sti
@param session            Client session
@param[in] unused_in      Unused parameter
@param[out] station_id    Station ID
@return true in case of error
*/
bool cms_sti_get_station_id(const cms_session_t* session,
                            void* unused_in,
                            cms_sti_station_id_t* station_id);

/**
Station info notification data.
@ingroup sti
*/
typedef cms_sti_set_items_t cms_sti_notif_data_t;

/**
Station info notify callback function.
@ingroup sti
When a notification is sent to a subscribed client the following
typed function callback is called. This callback function shall be registered
during cms_sti_subscribe function call.

@param data         Notification
@param ctx          User context, registered during subscribe
@see cms_sti_subscribe
*/
typedef void (*cms_sti_notify_f)(const cms_sti_notif_data_t* data,
                                 void* ctx);

/**
STI subscribe.
@ingroup sti
Upon successful subscription, when an STI set, the ITS software
stack sends a notification to the subscribed client with data.

@param session          Client session
@param callback_f       Notification callback function
@param ctx              User context
@param[out] subs_id     Subscription ID output
@return true in case of error
@see cms_sti_notify_f
*/
bool cms_sti_subscribe(const cms_session_t* session,
                       cms_sti_notify_f callback_f,
                       void* ctx,
                       cms_subs_id_t* subs_id);

/**
STI unsubscribe.
@ingroup sti
@param session      Client session
@param subs_id      Subscription ID, retrieved by cms_wsmp_subscribe
@return true in case of error
@see cms_sti_subscribe
*/
bool cms_sti_unsubscribe(const cms_session_t* session, cms_subs_id_t subs_id);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_STI_H_ */
