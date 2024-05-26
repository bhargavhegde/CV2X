/* Automatically generated file - do not edit */
#ifndef _FFASN1_V2X_CN_ASN_H
#define _FFASN1_V2X_CN_ASN_H

#include "asn1defs.h"

#ifdef  __cplusplus
extern "C" {
#endif

typedef int CN_MsgCount;

extern const ASN1CType asn1_type_CN_MsgCount[];

typedef int CN_DSecond;

extern const ASN1CType asn1_type_CN_DSecond[];

typedef enum CN_TimeConfidence {
  CN_TimeConfidence_unavailable,
  CN_TimeConfidence_time_100_000,
  CN_TimeConfidence_time_050_000,
  CN_TimeConfidence_time_020_000,
  CN_TimeConfidence_time_010_000,
  CN_TimeConfidence_time_002_000,
  CN_TimeConfidence_time_001_000,
  CN_TimeConfidence_time_000_500,
  CN_TimeConfidence_time_000_200,
  CN_TimeConfidence_time_000_100,
  CN_TimeConfidence_time_000_050,
  CN_TimeConfidence_time_000_020,
  CN_TimeConfidence_time_000_010,
  CN_TimeConfidence_time_000_005,
  CN_TimeConfidence_time_000_002,
  CN_TimeConfidence_time_000_001,
  CN_TimeConfidence_time_000_000_5,
  CN_TimeConfidence_time_000_000_2,
  CN_TimeConfidence_time_000_000_1,
  CN_TimeConfidence_time_000_000_05,
  CN_TimeConfidence_time_000_000_02,
  CN_TimeConfidence_time_000_000_01,
  CN_TimeConfidence_time_000_000_005,
  CN_TimeConfidence_time_000_000_002,
  CN_TimeConfidence_time_000_000_001,
  CN_TimeConfidence_time_000_000_000_5,
  CN_TimeConfidence_time_000_000_000_2,
  CN_TimeConfidence_time_000_000_000_1,
  CN_TimeConfidence_time_000_000_000_05,
  CN_TimeConfidence_time_000_000_000_02,
  CN_TimeConfidence_time_000_000_000_01,
  CN_TimeConfidence_time_000_000_000_005,
  CN_TimeConfidence_time_000_000_000_002,
  CN_TimeConfidence_time_000_000_000_001,
  CN_TimeConfidence_time_000_000_000_000_5,
  CN_TimeConfidence_time_000_000_000_000_2,
  CN_TimeConfidence_time_000_000_000_000_1,
  CN_TimeConfidence_time_000_000_000_000_05,
  CN_TimeConfidence_time_000_000_000_000_02,
  CN_TimeConfidence_time_000_000_000_000_01,
  CN_TimeConfidence__dummy = 2147483647,
} CN_TimeConfidence;

extern const ASN1CType asn1_type_CN_TimeConfidence[];

typedef int CN_Latitude;

extern const ASN1CType asn1_type_CN_Latitude[];

typedef int CN_Longitude;

extern const ASN1CType asn1_type_CN_Longitude[];

typedef int CN_Elevation;

extern const ASN1CType asn1_type_CN_Elevation[];

typedef struct CN_Position3D {
  CN_Latitude lat;
  CN_Longitude lon;
  BOOL elevation_option;
  CN_Elevation elevation;
} CN_Position3D;


extern const ASN1CType asn1_type_CN_Position3D[];

typedef int CN_SemiMajorAxisAccuracy;

extern const ASN1CType asn1_type_CN_SemiMajorAxisAccuracy[];

typedef int CN_SemiMinorAxisAccuracy;

extern const ASN1CType asn1_type_CN_SemiMinorAxisAccuracy[];

typedef int CN_SemiMajorAxisOrientation;

extern const ASN1CType asn1_type_CN_SemiMajorAxisOrientation[];

typedef struct CN_PositionalAccuracy {
  CN_SemiMajorAxisAccuracy semiMajor;
  CN_SemiMinorAxisAccuracy semiMinor;
  CN_SemiMajorAxisOrientation orientation;
} CN_PositionalAccuracy;


extern const ASN1CType asn1_type_CN_PositionalAccuracy[];

typedef enum CN_PositionConfidence {
  CN_PositionConfidence_unavailable,
  CN_PositionConfidence_a500m,
  CN_PositionConfidence_a200m,
  CN_PositionConfidence_a100m,
  CN_PositionConfidence_a50m,
  CN_PositionConfidence_a20m,
  CN_PositionConfidence_a10m,
  CN_PositionConfidence_a5m,
  CN_PositionConfidence_a2m,
  CN_PositionConfidence_a1m,
  CN_PositionConfidence_a50cm,
  CN_PositionConfidence_a20cm,
  CN_PositionConfidence_a10cm,
  CN_PositionConfidence_a5cm,
  CN_PositionConfidence_a2cm,
  CN_PositionConfidence_a1cm,
  CN_PositionConfidence__dummy = 2147483647,
} CN_PositionConfidence;

extern const ASN1CType asn1_type_CN_PositionConfidence[];

typedef enum CN_ElevationConfidence {
  CN_ElevationConfidence_unavailable,
  CN_ElevationConfidence_elev_500_00,
  CN_ElevationConfidence_elev_200_00,
  CN_ElevationConfidence_elev_100_00,
  CN_ElevationConfidence_elev_050_00,
  CN_ElevationConfidence_elev_020_00,
  CN_ElevationConfidence_elev_010_00,
  CN_ElevationConfidence_elev_005_00,
  CN_ElevationConfidence_elev_002_00,
  CN_ElevationConfidence_elev_001_00,
  CN_ElevationConfidence_elev_000_50,
  CN_ElevationConfidence_elev_000_20,
  CN_ElevationConfidence_elev_000_10,
  CN_ElevationConfidence_elev_000_05,
  CN_ElevationConfidence_elev_000_02,
  CN_ElevationConfidence_elev_000_01,
  CN_ElevationConfidence__dummy = 2147483647,
} CN_ElevationConfidence;

extern const ASN1CType asn1_type_CN_ElevationConfidence[];

typedef struct CN_PositionConfidenceSet {
  CN_PositionConfidence pos;
  BOOL elevation_option;
  CN_ElevationConfidence elevation;
} CN_PositionConfidenceSet;


extern const ASN1CType asn1_type_CN_PositionConfidenceSet[];

typedef enum CN_TransmissionState {
  CN_TransmissionState_neutral,
  CN_TransmissionState_park,
  CN_TransmissionState_forwardGears,
  CN_TransmissionState_reverseGears,
  CN_TransmissionState_reserved1,
  CN_TransmissionState_reserved2,
  CN_TransmissionState_reserved3,
  CN_TransmissionState_unavailable,
  CN_TransmissionState__dummy = 2147483647,
} CN_TransmissionState;

extern const ASN1CType asn1_type_CN_TransmissionState[];

typedef int CN_Speed;

extern const ASN1CType asn1_type_CN_Speed[];

typedef int CN_Heading;

extern const ASN1CType asn1_type_CN_Heading[];

typedef int CN_SteeringWheelAngle;

extern const ASN1CType asn1_type_CN_SteeringWheelAngle[];

typedef enum CN_SpeedConfidence {
  CN_SpeedConfidence_unavailable,
  CN_SpeedConfidence_prec100ms,
  CN_SpeedConfidence_prec10ms,
  CN_SpeedConfidence_prec5ms,
  CN_SpeedConfidence_prec1ms,
  CN_SpeedConfidence_prec0_1ms,
  CN_SpeedConfidence_prec0_05ms,
  CN_SpeedConfidence_prec0_01ms,
  CN_SpeedConfidence__dummy = 2147483647,
} CN_SpeedConfidence;

extern const ASN1CType asn1_type_CN_SpeedConfidence[];

typedef enum CN_HeadingConfidence {
  CN_HeadingConfidence_unavailable,
  CN_HeadingConfidence_prec10deg,
  CN_HeadingConfidence_prec05deg,
  CN_HeadingConfidence_prec01deg,
  CN_HeadingConfidence_prec0_1deg,
  CN_HeadingConfidence_prec0_05deg,
  CN_HeadingConfidence_prec0_01deg,
  CN_HeadingConfidence_prec0_0125deg,
  CN_HeadingConfidence__dummy = 2147483647,
} CN_HeadingConfidence;

extern const ASN1CType asn1_type_CN_HeadingConfidence[];

typedef enum CN_SteeringWheelAngleConfidence {
  CN_SteeringWheelAngleConfidence_unavailable,
  CN_SteeringWheelAngleConfidence_prec2deg,
  CN_SteeringWheelAngleConfidence_prec1deg,
  CN_SteeringWheelAngleConfidence_prec0_02deg,
  CN_SteeringWheelAngleConfidence__dummy = 2147483647,
} CN_SteeringWheelAngleConfidence;

extern const ASN1CType asn1_type_CN_SteeringWheelAngleConfidence[];

typedef struct CN_MotionConfidenceSet {
  BOOL speedCfd_option;
  CN_SpeedConfidence speedCfd;
  BOOL headingCfd_option;
  CN_HeadingConfidence headingCfd;
  BOOL steerCfd_option;
  CN_SteeringWheelAngleConfidence steerCfd;
} CN_MotionConfidenceSet;


extern const ASN1CType asn1_type_CN_MotionConfidenceSet[];

typedef int CN_Acceleration;

extern const ASN1CType asn1_type_CN_Acceleration[];

typedef int CN_VerticalAcceleration;

extern const ASN1CType asn1_type_CN_VerticalAcceleration[];

typedef int CN_YawRate;

extern const ASN1CType asn1_type_CN_YawRate[];

typedef struct CN_AccelerationSet4Way {
  CN_Acceleration lon;
  CN_Acceleration lat;
  CN_VerticalAcceleration vert;
  CN_YawRate yaw;
} CN_AccelerationSet4Way;


extern const ASN1CType asn1_type_CN_AccelerationSet4Way[];

typedef enum CN_BrakePedalStatus {
  CN_BrakePedalStatus_unavailable,
  CN_BrakePedalStatus_off,
  CN_BrakePedalStatus_on,
  CN_BrakePedalStatus__dummy = 2147483647,
} CN_BrakePedalStatus;

extern const ASN1CType asn1_type_CN_BrakePedalStatus[];

typedef ASN1BitString CN_BrakeAppliedStatus;

extern const ASN1CType asn1_type_CN_BrakeAppliedStatus[];

typedef enum CN_TractionControlStatus {
  CN_TractionControlStatus_unavailable,
  CN_TractionControlStatus_off,
  CN_TractionControlStatus_on,
  CN_TractionControlStatus_engaged,
  CN_TractionControlStatus__dummy = 2147483647,
} CN_TractionControlStatus;

extern const ASN1CType asn1_type_CN_TractionControlStatus[];

typedef enum CN_AntiLockBrakeStatus {
  CN_AntiLockBrakeStatus_unavailable,
  CN_AntiLockBrakeStatus_off,
  CN_AntiLockBrakeStatus_on,
  CN_AntiLockBrakeStatus_engaged,
  CN_AntiLockBrakeStatus__dummy = 2147483647,
} CN_AntiLockBrakeStatus;

extern const ASN1CType asn1_type_CN_AntiLockBrakeStatus[];

typedef enum CN_StabilityControlStatus {
  CN_StabilityControlStatus_unavailable,
  CN_StabilityControlStatus_off,
  CN_StabilityControlStatus_on,
  CN_StabilityControlStatus_engaged,
  CN_StabilityControlStatus__dummy = 2147483647,
} CN_StabilityControlStatus;

extern const ASN1CType asn1_type_CN_StabilityControlStatus[];

typedef enum CN_BrakeBoostApplied {
  CN_BrakeBoostApplied_unavailable,
  CN_BrakeBoostApplied_off,
  CN_BrakeBoostApplied_on,
  CN_BrakeBoostApplied__dummy = 2147483647,
} CN_BrakeBoostApplied;

extern const ASN1CType asn1_type_CN_BrakeBoostApplied[];

typedef enum CN_AuxiliaryBrakeStatus {
  CN_AuxiliaryBrakeStatus_unavailable,
  CN_AuxiliaryBrakeStatus_off,
  CN_AuxiliaryBrakeStatus_on,
  CN_AuxiliaryBrakeStatus_reserved,
  CN_AuxiliaryBrakeStatus__dummy = 2147483647,
} CN_AuxiliaryBrakeStatus;

extern const ASN1CType asn1_type_CN_AuxiliaryBrakeStatus[];

typedef struct CN_BrakeSystemStatus {
  BOOL brakePedal_option;
  CN_BrakePedalStatus brakePedal;
  BOOL wheelBrakes_option;
  CN_BrakeAppliedStatus wheelBrakes;
  BOOL traction_option;
  CN_TractionControlStatus traction;
  BOOL abs_option;
  CN_AntiLockBrakeStatus abs;
  BOOL scs_option;
  CN_StabilityControlStatus scs;
  BOOL brakeBoost_option;
  CN_BrakeBoostApplied brakeBoost;
  BOOL auxBrakes_option;
  CN_AuxiliaryBrakeStatus auxBrakes;
} CN_BrakeSystemStatus;


extern const ASN1CType asn1_type_CN_BrakeSystemStatus[];

typedef int CN_VehicleWidth;

extern const ASN1CType asn1_type_CN_VehicleWidth[];

typedef int CN_VehicleLength;

extern const ASN1CType asn1_type_CN_VehicleLength[];

typedef int CN_VehicleHeight;

extern const ASN1CType asn1_type_CN_VehicleHeight[];

typedef struct CN_VehicleSize {
  CN_VehicleWidth width;
  CN_VehicleLength length;
  BOOL height_option;
  CN_VehicleHeight height;
} CN_VehicleSize;


extern const ASN1CType asn1_type_CN_VehicleSize[];

typedef int CN_BasicVehicleClass;

extern const ASN1CType asn1_type_CN_BasicVehicleClass[];

typedef int CN_FuelType;

extern const ASN1CType asn1_type_CN_FuelType[];

typedef struct CN_VehicleClassification {
  CN_BasicVehicleClass classification;
  BOOL fuelType_option;
  CN_FuelType fuelType;
} CN_VehicleClassification;


extern const ASN1CType asn1_type_CN_VehicleClassification[];

typedef ASN1BitString CN_VehicleEventFlags;

extern const ASN1CType asn1_type_CN_VehicleEventFlags[];

typedef int CN_DYear;

extern const ASN1CType asn1_type_CN_DYear[];

typedef int CN_DMonth;

extern const ASN1CType asn1_type_CN_DMonth[];

typedef int CN_DDay;

extern const ASN1CType asn1_type_CN_DDay[];

typedef int CN_DHour;

extern const ASN1CType asn1_type_CN_DHour[];

typedef int CN_DMinute;

extern const ASN1CType asn1_type_CN_DMinute[];

typedef int CN_DTimeOffset;

extern const ASN1CType asn1_type_CN_DTimeOffset[];

typedef struct CN_DDateTime {
  BOOL year_option;
  CN_DYear year;
  BOOL month_option;
  CN_DMonth month;
  BOOL day_option;
  CN_DDay day;
  BOOL hour_option;
  CN_DHour hour;
  BOOL minute_option;
  CN_DMinute minute;
  BOOL second_option;
  CN_DSecond second;
  BOOL offset_option;
  CN_DTimeOffset offset;
} CN_DDateTime;


extern const ASN1CType asn1_type_CN_DDateTime[];

typedef struct CN_FullPositionVector {
  BOOL utcTime_option;
  CN_DDateTime utcTime;
  CN_Position3D pos;
  BOOL heading_option;
  CN_Heading heading;
  BOOL transmission_option;
  CN_TransmissionState transmission;
  BOOL speed_option;
  CN_Speed speed;
  BOOL posAccuracy_option;
  CN_PositionConfidenceSet posAccuracy;
  BOOL timeConfidence_option;
  CN_TimeConfidence timeConfidence;
  BOOL motionCfd_option;
  CN_MotionConfidenceSet motionCfd;
} CN_FullPositionVector;


extern const ASN1CType asn1_type_CN_FullPositionVector[];

typedef ASN1BitString CN_GNSSstatus;

extern const ASN1CType asn1_type_CN_GNSSstatus[];

typedef int CN_OffsetLL_B12;

extern const ASN1CType asn1_type_CN_OffsetLL_B12[];

typedef struct CN_Position_LL_24B {
  CN_OffsetLL_B12 lon;
  CN_OffsetLL_B12 lat;
} CN_Position_LL_24B;


extern const ASN1CType asn1_type_CN_Position_LL_24B[];

typedef int CN_OffsetLL_B14;

extern const ASN1CType asn1_type_CN_OffsetLL_B14[];

typedef struct CN_Position_LL_28B {
  CN_OffsetLL_B14 lon;
  CN_OffsetLL_B14 lat;
} CN_Position_LL_28B;


extern const ASN1CType asn1_type_CN_Position_LL_28B[];

typedef int CN_OffsetLL_B16;

extern const ASN1CType asn1_type_CN_OffsetLL_B16[];

typedef struct CN_Position_LL_32B {
  CN_OffsetLL_B16 lon;
  CN_OffsetLL_B16 lat;
} CN_Position_LL_32B;


extern const ASN1CType asn1_type_CN_Position_LL_32B[];

typedef int CN_OffsetLL_B18;

extern const ASN1CType asn1_type_CN_OffsetLL_B18[];

typedef struct CN_Position_LL_36B {
  CN_OffsetLL_B18 lon;
  CN_OffsetLL_B18 lat;
} CN_Position_LL_36B;


extern const ASN1CType asn1_type_CN_Position_LL_36B[];

typedef int CN_OffsetLL_B22;

extern const ASN1CType asn1_type_CN_OffsetLL_B22[];

typedef struct CN_Position_LL_44B {
  CN_OffsetLL_B22 lon;
  CN_OffsetLL_B22 lat;
} CN_Position_LL_44B;


extern const ASN1CType asn1_type_CN_Position_LL_44B[];

typedef int CN_OffsetLL_B24;

extern const ASN1CType asn1_type_CN_OffsetLL_B24[];

typedef struct CN_Position_LL_48B {
  CN_OffsetLL_B24 lon;
  CN_OffsetLL_B24 lat;
} CN_Position_LL_48B;


extern const ASN1CType asn1_type_CN_Position_LL_48B[];

typedef struct CN_Position_LLmD_64b {
  CN_Longitude lon;
  CN_Latitude lat;
} CN_Position_LLmD_64b;


extern const ASN1CType asn1_type_CN_Position_LLmD_64b[];

typedef enum {
  CN_PositionOffsetLL_position_LL1,
  CN_PositionOffsetLL_position_LL2,
  CN_PositionOffsetLL_position_LL3,
  CN_PositionOffsetLL_position_LL4,
  CN_PositionOffsetLL_position_LL5,
  CN_PositionOffsetLL_position_LL6,
  CN_PositionOffsetLL_position_LatLon,
  CN_PositionOffsetLL__dummy = 2147483647,
} CN_PositionOffsetLL_choice;

typedef struct CN_PositionOffsetLL {
  CN_PositionOffsetLL_choice choice;
  union {
    CN_Position_LL_24B position_LL1;
    CN_Position_LL_28B position_LL2;
    CN_Position_LL_32B position_LL3;
    CN_Position_LL_36B position_LL4;
    CN_Position_LL_44B position_LL5;
    CN_Position_LL_48B position_LL6;
    CN_Position_LLmD_64b position_LatLon;
  } u;
} CN_PositionOffsetLL;

extern const ASN1CType asn1_type_CN_PositionOffsetLL[];

typedef int CN_VertOffset_B07;

extern const ASN1CType asn1_type_CN_VertOffset_B07[];

typedef int CN_VertOffset_B08;

extern const ASN1CType asn1_type_CN_VertOffset_B08[];

typedef int CN_VertOffset_B09;

extern const ASN1CType asn1_type_CN_VertOffset_B09[];

typedef int CN_VertOffset_B10;

extern const ASN1CType asn1_type_CN_VertOffset_B10[];

typedef int CN_VertOffset_B11;

extern const ASN1CType asn1_type_CN_VertOffset_B11[];

typedef int CN_VertOffset_B12;

extern const ASN1CType asn1_type_CN_VertOffset_B12[];

typedef enum {
  CN_VerticalOffset_offset1,
  CN_VerticalOffset_offset2,
  CN_VerticalOffset_offset3,
  CN_VerticalOffset_offset4,
  CN_VerticalOffset_offset5,
  CN_VerticalOffset_offset6,
  CN_VerticalOffset_elevation,
  CN_VerticalOffset__dummy = 2147483647,
} CN_VerticalOffset_choice;

typedef struct CN_VerticalOffset {
  CN_VerticalOffset_choice choice;
  union {
    CN_VertOffset_B07 offset1;
    CN_VertOffset_B08 offset2;
    CN_VertOffset_B09 offset3;
    CN_VertOffset_B10 offset4;
    CN_VertOffset_B11 offset5;
    CN_VertOffset_B12 offset6;
    CN_Elevation elevation;
  } u;
} CN_VerticalOffset;

extern const ASN1CType asn1_type_CN_VerticalOffset[];

typedef struct CN_PositionOffsetLLV {
  CN_PositionOffsetLL offsetLL;
  BOOL offsetV_option;
  CN_VerticalOffset offsetV;
} CN_PositionOffsetLLV;


extern const ASN1CType asn1_type_CN_PositionOffsetLLV[];

typedef int CN_TimeOffset;

extern const ASN1CType asn1_type_CN_TimeOffset[];

typedef int CN_CoarseHeading;

extern const ASN1CType asn1_type_CN_CoarseHeading[];

typedef struct CN_PathHistoryPoint {
  CN_PositionOffsetLLV llvOffset;
  CN_TimeOffset timeOffset;
  BOOL speed_option;
  CN_Speed speed;
  BOOL posAccuracy_option;
  CN_PositionConfidenceSet posAccuracy;
  BOOL heading_option;
  CN_CoarseHeading heading;
} CN_PathHistoryPoint;


extern const ASN1CType asn1_type_CN_PathHistoryPoint[];

typedef struct CN_PathHistoryPointList {
  CN_PathHistoryPoint *tab;
  size_t count;
} CN_PathHistoryPointList;

extern const ASN1CType asn1_type_CN_PathHistoryPointList[];

typedef struct CN_PathHistory {
  BOOL initialPosition_option;
  CN_FullPositionVector initialPosition;
  BOOL currGNSSstatus_option;
  CN_GNSSstatus currGNSSstatus;
  CN_PathHistoryPointList crumbData;
} CN_PathHistory;


extern const ASN1CType asn1_type_CN_PathHistory[];

typedef int CN_RadiusOfCurvature;

extern const ASN1CType asn1_type_CN_RadiusOfCurvature[];

typedef int CN_Confidence;

extern const ASN1CType asn1_type_CN_Confidence[];

typedef struct CN_PathPrediction {
  CN_RadiusOfCurvature radiusOfCurve;
  CN_Confidence confidence;
} CN_PathPrediction;


extern const ASN1CType asn1_type_CN_PathPrediction[];

typedef ASN1BitString CN_ExteriorLights;

extern const ASN1CType asn1_type_CN_ExteriorLights[];

typedef struct CN_VehicleSafetyExtensions {
  BOOL events_option;
  CN_VehicleEventFlags events;
  BOOL pathHistory_option;
  CN_PathHistory pathHistory;
  BOOL pathPrediction_option;
  CN_PathPrediction pathPrediction;
  BOOL lights_option;
  CN_ExteriorLights lights;
} CN_VehicleSafetyExtensions;


extern const ASN1CType asn1_type_CN_VehicleSafetyExtensions[];

typedef enum CN_ResponseType {
  CN_ResponseType_notInUseOrNotEquipped,
  CN_ResponseType_emergency,
  CN_ResponseType_nonEmergency,
  CN_ResponseType_pursuit,
  CN_ResponseType_stationary,
  CN_ResponseType_slowMoving,
  CN_ResponseType_stopAndGoMovement,
  CN_ResponseType__dummy = 2147483647,
} CN_ResponseType;

extern const ASN1CType asn1_type_CN_ResponseType[];

typedef enum CN_SirenInUse {
  CN_SirenInUse_unavailable,
  CN_SirenInUse_notInUse,
  CN_SirenInUse_inUse,
  CN_SirenInUse_reserved,
  CN_SirenInUse__dummy = 2147483647,
} CN_SirenInUse;

extern const ASN1CType asn1_type_CN_SirenInUse[];

typedef enum CN_LightbarInUse {
  CN_LightbarInUse_unavailable,
  CN_LightbarInUse_notInUse,
  CN_LightbarInUse_inUse,
  CN_LightbarInUse_yellowCautionLights,
  CN_LightbarInUse_schooldBusLights,
  CN_LightbarInUse_arrowSignsActive,
  CN_LightbarInUse_slowMovingVehicle,
  CN_LightbarInUse_freqStops,
  CN_LightbarInUse__dummy = 2147483647,
} CN_LightbarInUse;

extern const ASN1CType asn1_type_CN_LightbarInUse[];

typedef struct CN_VehicleEmergencyExtensions {
  BOOL responseType_option;
  CN_ResponseType responseType;
  BOOL sirenUse_option;
  CN_SirenInUse sirenUse;
  BOOL lightsUse_option;
  CN_LightbarInUse lightsUse;
} CN_VehicleEmergencyExtensions;


extern const ASN1CType asn1_type_CN_VehicleEmergencyExtensions[];

typedef struct CN_BasicSafetyMessage {
  CN_MsgCount msgCnt;
  ASN1String id;
  CN_DSecond secMark;
  BOOL timeConfidence_option;
  CN_TimeConfidence timeConfidence;
  CN_Position3D pos;
  BOOL posAccuracy_option;
  CN_PositionalAccuracy posAccuracy;
  BOOL posConfidence_option;
  CN_PositionConfidenceSet posConfidence;
  CN_TransmissionState transmission;
  CN_Speed speed;
  CN_Heading heading;
  BOOL angle_option;
  CN_SteeringWheelAngle angle;
  BOOL motionCfd_option;
  CN_MotionConfidenceSet motionCfd;
  CN_AccelerationSet4Way accelSet;
  CN_BrakeSystemStatus brakes;
  CN_VehicleSize size;
  CN_VehicleClassification vehicleClass;
  BOOL safetyExt_option;
  CN_VehicleSafetyExtensions safetyExt;
  BOOL emergencyExt_option;
  CN_VehicleEmergencyExtensions emergencyExt;
} CN_BasicSafetyMessage;


extern const ASN1CType asn1_type_CN_BasicSafetyMessage[];

typedef int CN_MinuteOfTheYear;

extern const ASN1CType asn1_type_CN_MinuteOfTheYear[];

typedef int CN_TimeMark;

extern const ASN1CType asn1_type_CN_TimeMark[];

typedef ASN1String CN_DescriptiveName;

extern const ASN1CType asn1_type_CN_DescriptiveName[];

typedef int CN_RoadRegulatorID;

extern const ASN1CType asn1_type_CN_RoadRegulatorID[];

typedef int CN_NodeID;

extern const ASN1CType asn1_type_CN_NodeID[];

typedef struct CN_NodeReferenceID {
  BOOL region_option;
  CN_RoadRegulatorID region;
  CN_NodeID id;
} CN_NodeReferenceID;


extern const ASN1CType asn1_type_CN_NodeReferenceID[];

typedef enum CN_SpeedLimitType {
  CN_SpeedLimitType_unknown,
  CN_SpeedLimitType_maxSpeedInSchoolZone,
  CN_SpeedLimitType_maxSpeedInSchoolZoneWhenChildrenArePresent,
  CN_SpeedLimitType_maxSpeedInConstructionZone,
  CN_SpeedLimitType_vehicleMinSpeed,
  CN_SpeedLimitType_vehicleMaxSpeed,
  CN_SpeedLimitType_vehicleNightMaxSpeed,
  CN_SpeedLimitType_truckMinSpeed,
  CN_SpeedLimitType_truckMaxSpeed,
  CN_SpeedLimitType_truckNightMaxSpeed,
  CN_SpeedLimitType_vehiclesWithTrailersMinSpeed,
  CN_SpeedLimitType_vehiclesWithTrailersMaxSpeed,
  CN_SpeedLimitType_vehiclesWithTrailersNightMaxSpeed,
  CN_SpeedLimitType__dummy = 2147483647,
} CN_SpeedLimitType;

extern const ASN1CType asn1_type_CN_SpeedLimitType[];

typedef struct CN_RegulatorySpeedLimit {
  CN_SpeedLimitType type;
  CN_Speed speed;
} CN_RegulatorySpeedLimit;


extern const ASN1CType asn1_type_CN_RegulatorySpeedLimit[];

typedef struct CN_SpeedLimitList {
  CN_RegulatorySpeedLimit *tab;
  size_t count;
} CN_SpeedLimitList;

extern const ASN1CType asn1_type_CN_SpeedLimitList[];

typedef int CN_LaneWidth;

extern const ASN1CType asn1_type_CN_LaneWidth[];

typedef struct CN_RoadPoint {
  CN_PositionOffsetLLV posOffset;
} CN_RoadPoint;


extern const ASN1CType asn1_type_CN_RoadPoint[];

typedef struct CN_PointList {
  CN_RoadPoint *tab;
  size_t count;
} CN_PointList;

extern const ASN1CType asn1_type_CN_PointList[];

typedef int CN_PhaseID;

extern const ASN1CType asn1_type_CN_PhaseID[];

typedef struct CN_Movement {
  CN_NodeReferenceID remoteIntersection;
  BOOL phaseId_option;
  CN_PhaseID phaseId;
} CN_Movement;


extern const ASN1CType asn1_type_CN_Movement[];

typedef struct CN_MovementList {
  CN_Movement *tab;
  size_t count;
} CN_MovementList;

extern const ASN1CType asn1_type_CN_MovementList[];

typedef int CN_LaneID;

extern const ASN1CType asn1_type_CN_LaneID[];

typedef ASN1BitString CN_LaneSharing;

extern const ASN1CType asn1_type_CN_LaneSharing[];

typedef ASN1BitString CN_LaneAttributes_Vehicle;

extern const ASN1CType asn1_type_CN_LaneAttributes_Vehicle[];

typedef ASN1BitString CN_LaneAttributes_Crosswalk;

extern const ASN1CType asn1_type_CN_LaneAttributes_Crosswalk[];

typedef ASN1BitString CN_LaneAttributes_Bike;

extern const ASN1CType asn1_type_CN_LaneAttributes_Bike[];

typedef ASN1BitString CN_LaneAttributes_Sidewalk;

extern const ASN1CType asn1_type_CN_LaneAttributes_Sidewalk[];

typedef ASN1BitString CN_LaneAttributes_Barrier;

extern const ASN1CType asn1_type_CN_LaneAttributes_Barrier[];

typedef ASN1BitString CN_LaneAttributes_Striping;

extern const ASN1CType asn1_type_CN_LaneAttributes_Striping[];

typedef ASN1BitString CN_LaneAttributes_TrackedVehicle;

extern const ASN1CType asn1_type_CN_LaneAttributes_TrackedVehicle[];

typedef ASN1BitString CN_LaneAttributes_Parking;

extern const ASN1CType asn1_type_CN_LaneAttributes_Parking[];

typedef enum {
  CN_LaneTypeAttributes_vehicle,
  CN_LaneTypeAttributes_crosswalk,
  CN_LaneTypeAttributes_bikeLane,
  CN_LaneTypeAttributes_sidewalk,
  CN_LaneTypeAttributes_median,
  CN_LaneTypeAttributes_striping,
  CN_LaneTypeAttributes_trackedVehicle,
  CN_LaneTypeAttributes_parking,
  CN_LaneTypeAttributes__dummy = 2147483647,
} CN_LaneTypeAttributes_choice;

typedef struct CN_LaneTypeAttributes {
  CN_LaneTypeAttributes_choice choice;
  union {
    CN_LaneAttributes_Vehicle vehicle;
    CN_LaneAttributes_Crosswalk crosswalk;
    CN_LaneAttributes_Bike bikeLane;
    CN_LaneAttributes_Sidewalk sidewalk;
    CN_LaneAttributes_Barrier median;
    CN_LaneAttributes_Striping striping;
    CN_LaneAttributes_TrackedVehicle trackedVehicle;
    CN_LaneAttributes_Parking parking;
  } u;
} CN_LaneTypeAttributes;

extern const ASN1CType asn1_type_CN_LaneTypeAttributes[];

typedef struct CN_LaneAttributes {
  BOOL shareWith_option;
  CN_LaneSharing shareWith;
  CN_LaneTypeAttributes laneType;
} CN_LaneAttributes;


extern const ASN1CType asn1_type_CN_LaneAttributes[];

typedef ASN1BitString CN_AllowedManeuvers;

extern const ASN1CType asn1_type_CN_AllowedManeuvers[];

typedef struct CN_ConnectingLane {
  CN_LaneID lane;
  BOOL maneuver_option;
  CN_AllowedManeuvers maneuver;
} CN_ConnectingLane;


extern const ASN1CType asn1_type_CN_ConnectingLane[];

typedef struct CN_Connection {
  CN_NodeReferenceID remoteIntersection;
  BOOL connectingLane_option;
  CN_ConnectingLane connectingLane;
  BOOL phaseId_option;
  CN_PhaseID phaseId;
} CN_Connection;


extern const ASN1CType asn1_type_CN_Connection[];

typedef struct CN_ConnectsToList {
  CN_Connection *tab;
  size_t count;
} CN_ConnectsToList;

extern const ASN1CType asn1_type_CN_ConnectsToList[];

typedef struct CN_Lane {
  CN_LaneID laneID;
  BOOL laneWidth_option;
  CN_LaneWidth laneWidth;
  BOOL laneAttributes_option;
  CN_LaneAttributes laneAttributes;
  BOOL maneuvers_option;
  CN_AllowedManeuvers maneuvers;
  BOOL connectsTo_option;
  CN_ConnectsToList connectsTo;
  BOOL speedLimits_option;
  CN_SpeedLimitList speedLimits;
  BOOL points_option;
  CN_PointList points;
} CN_Lane;


extern const ASN1CType asn1_type_CN_Lane[];

typedef struct CN_LaneList {
  CN_Lane *tab;
  size_t count;
} CN_LaneList;

extern const ASN1CType asn1_type_CN_LaneList[];

typedef struct CN_Link {
  BOOL name_option;
  CN_DescriptiveName name;
  CN_NodeReferenceID upstreamNodeId;
  BOOL speedLimits_option;
  CN_SpeedLimitList speedLimits;
  CN_LaneWidth linkWidth;
  BOOL points_option;
  CN_PointList points;
  BOOL movements_option;
  CN_MovementList movements;
  CN_LaneList lanes;
} CN_Link;


extern const ASN1CType asn1_type_CN_Link[];

typedef struct CN_LinkList {
  CN_Link *tab;
  size_t count;
} CN_LinkList;

extern const ASN1CType asn1_type_CN_LinkList[];

typedef struct CN_Node {
  BOOL name_option;
  CN_DescriptiveName name;
  CN_NodeReferenceID id;
  CN_Position3D refPos;
  BOOL inLinks_option;
  CN_LinkList inLinks;
} CN_Node;


extern const ASN1CType asn1_type_CN_Node[];

typedef struct CN_NodeList {
  CN_Node *tab;
  size_t count;
} CN_NodeList;

extern const ASN1CType asn1_type_CN_NodeList[];

typedef struct CN_MapData {
  CN_MsgCount msgCnt;
  BOOL timeStamp_option;
  CN_MinuteOfTheYear timeStamp;
  CN_NodeList nodes;
} CN_MapData;


extern const ASN1CType asn1_type_CN_MapData[];

typedef enum CN_ParticipantType {
  CN_ParticipantType_unknown,
  CN_ParticipantType_motor,
  CN_ParticipantType_non_motor,
  CN_ParticipantType_pedestrian,
  CN_ParticipantType_rsu,
  CN_ParticipantType__dummy = 2147483647,
} CN_ParticipantType;

extern const ASN1CType asn1_type_CN_ParticipantType[];

typedef enum CN_SourceType {
  CN_SourceType_unknown,
  CN_SourceType_selfinfo,
  CN_SourceType_v2x,
  CN_SourceType_video,
  CN_SourceType_microwaveRadar,
  CN_SourceType_loop,
  CN_SourceType_lidar,
  CN_SourceType_integrated,
  CN_SourceType__dummy = 2147483647,
} CN_SourceType;

extern const ASN1CType asn1_type_CN_SourceType[];

typedef struct CN_ParticipantData {
  CN_ParticipantType ptcType;
  int ptcId;
  CN_SourceType source;
  BOOL id_option;
  ASN1String id;
  CN_DSecond secMark;
  CN_PositionOffsetLLV pos;
  CN_PositionConfidenceSet posConfidence;
  BOOL transmission_option;
  CN_TransmissionState transmission;
  CN_Speed speed;
  CN_Heading heading;
  BOOL angle_option;
  CN_SteeringWheelAngle angle;
  BOOL motionCfd_option;
  CN_MotionConfidenceSet motionCfd;
  BOOL accelSet_option;
  CN_AccelerationSet4Way accelSet;
  CN_VehicleSize size;
  BOOL vehicleClass_option;
  CN_VehicleClassification vehicleClass;
} CN_ParticipantData;


extern const ASN1CType asn1_type_CN_ParticipantData[];

typedef struct CN_ParticipantList {
  CN_ParticipantData *tab;
  size_t count;
} CN_ParticipantList;

extern const ASN1CType asn1_type_CN_ParticipantList[];

typedef struct CN_RoadsideSafetyMessage {
  CN_MsgCount msgCnt;
  ASN1String id;
  CN_Position3D refPos;
  CN_ParticipantList participants;
} CN_RoadsideSafetyMessage;


extern const ASN1CType asn1_type_CN_RoadsideSafetyMessage[];

typedef ASN1BitString CN_IntersectionStatusObject;

extern const ASN1CType asn1_type_CN_IntersectionStatusObject[];

typedef enum CN_LightState {
  CN_LightState_unavailable,
  CN_LightState_dark,
  CN_LightState_flashing_red,
  CN_LightState_red,
  CN_LightState_flashing_green,
  CN_LightState_permissive_green,
  CN_LightState_protected_green,
  CN_LightState_yellow,
  CN_LightState_flashing_yellow,
  CN_LightState__dummy = 2147483647,
} CN_LightState;

extern const ASN1CType asn1_type_CN_LightState[];

typedef struct CN_TimeCountingDown {
  CN_TimeMark startTime;
  BOOL minEndTime_option;
  CN_TimeMark minEndTime;
  BOOL maxEndTime_option;
  CN_TimeMark maxEndTime;
  CN_TimeMark likelyEndTime;
  BOOL timeConfidence_option;
  CN_Confidence timeConfidence;
  BOOL nextStartTime_option;
  CN_TimeMark nextStartTime;
  BOOL nextDuration_option;
  CN_TimeMark nextDuration;
} CN_TimeCountingDown;


extern const ASN1CType asn1_type_CN_TimeCountingDown[];

typedef struct CN_UTCTiming {
  CN_TimeMark startUTCTime;
  BOOL minEndUTCTime_option;
  CN_TimeMark minEndUTCTime;
  BOOL maxEndUTCTime_option;
  CN_TimeMark maxEndUTCTime;
  CN_TimeMark likelyEndUTCTime;
  BOOL timeConfidence_option;
  CN_Confidence timeConfidence;
  BOOL nextStartUTCTime_option;
  CN_TimeMark nextStartUTCTime;
  BOOL nextEndUTCTime_option;
  CN_TimeMark nextEndUTCTime;
} CN_UTCTiming;


extern const ASN1CType asn1_type_CN_UTCTiming[];

typedef enum {
  CN_TimeChangeDetails_counting,
  CN_TimeChangeDetails_utcTiming,
  CN_TimeChangeDetails__dummy = 2147483647,
} CN_TimeChangeDetails_choice;

typedef struct CN_TimeChangeDetails {
  CN_TimeChangeDetails_choice choice;
  union {
    CN_TimeCountingDown counting;
    CN_UTCTiming utcTiming;
  } u;
} CN_TimeChangeDetails;

extern const ASN1CType asn1_type_CN_TimeChangeDetails[];

typedef struct CN_PhaseState {
  CN_LightState light;
  BOOL timing_option;
  CN_TimeChangeDetails timing;
} CN_PhaseState;


extern const ASN1CType asn1_type_CN_PhaseState[];

typedef struct CN_PhaseStateList {
  CN_PhaseState *tab;
  size_t count;
} CN_PhaseStateList;

extern const ASN1CType asn1_type_CN_PhaseStateList[];

typedef struct CN_Phase {
  CN_PhaseID id;
  CN_PhaseStateList phaseStates;
} CN_Phase;


extern const ASN1CType asn1_type_CN_Phase[];

typedef struct CN_PhaseList {
  CN_Phase *tab;
  size_t count;
} CN_PhaseList;

extern const ASN1CType asn1_type_CN_PhaseList[];

typedef struct CN_IntersectionState {
  CN_NodeReferenceID intersectionId;
  CN_IntersectionStatusObject status;
  BOOL moy_option;
  CN_MinuteOfTheYear moy;
  BOOL timeStamp_option;
  CN_DSecond timeStamp;
  BOOL timeConfidence_option;
  CN_TimeConfidence timeConfidence;
  CN_PhaseList phases;
} CN_IntersectionState;


extern const ASN1CType asn1_type_CN_IntersectionState[];

typedef struct CN_IntersectionStateList {
  CN_IntersectionState *tab;
  size_t count;
} CN_IntersectionStateList;

extern const ASN1CType asn1_type_CN_IntersectionStateList[];

typedef struct CN_SPAT {
  CN_MsgCount msgCnt;
  BOOL moy_option;
  CN_MinuteOfTheYear moy;
  BOOL timeStamp_option;
  CN_DSecond timeStamp;
  BOOL name_option;
  CN_DescriptiveName name;
  CN_IntersectionStateList intersections;
} CN_SPAT;


extern const ASN1CType asn1_type_CN_SPAT[];

typedef int CN_EventType;

extern const ASN1CType asn1_type_CN_EventType[];

typedef enum CN_EventSource {
  CN_EventSource_unknown,
  CN_EventSource_police,
  CN_EventSource_government,
  CN_EventSource_meteorological,
  CN_EventSource_internet,
  CN_EventSource_detection,
  CN_EventSource__dummy = 2147483647,
} CN_EventSource;

extern const ASN1CType asn1_type_CN_EventSource[];

typedef int CN_Radius;

extern const ASN1CType asn1_type_CN_Radius[];

typedef enum {
  CN_Description_textString,
  CN_Description_textGB2312,
  CN_Description__dummy = 2147483647,
} CN_Description_choice;

typedef struct CN_Description {
  CN_Description_choice choice;
  union {
    ASN1String textString;
    ASN1String textGB2312;
  } u;
} CN_Description;

extern const ASN1CType asn1_type_CN_Description[];

typedef struct CN_RSITimeDetails {
  BOOL startTime_option;
  CN_MinuteOfTheYear startTime;
  BOOL endTime_option;
  CN_MinuteOfTheYear endTime;
  BOOL endTimeConfidence_option;
  CN_TimeConfidence endTimeConfidence;
} CN_RSITimeDetails;


extern const ASN1CType asn1_type_CN_RSITimeDetails[];

typedef ASN1String CN_RSIPriority;

extern const ASN1CType asn1_type_CN_RSIPriority[];

typedef struct CN_PathPointList {
  CN_PositionOffsetLLV *tab;
  size_t count;
} CN_PathPointList;

extern const ASN1CType asn1_type_CN_PathPointList[];

typedef struct CN_ReferencePath {
  CN_PathPointList activePath;
  CN_Radius pathRadius;
} CN_ReferencePath;


extern const ASN1CType asn1_type_CN_ReferencePath[];

typedef struct CN_ReferencePathList {
  CN_ReferencePath *tab;
  size_t count;
} CN_ReferencePathList;

extern const ASN1CType asn1_type_CN_ReferencePathList[];

typedef ASN1BitString CN_ReferenceLanes;

extern const ASN1CType asn1_type_CN_ReferenceLanes[];

typedef struct CN_ReferenceLink {
  CN_NodeReferenceID upstreamNodeId;
  CN_NodeReferenceID downstreamNodeId;
  BOOL referenceLanes_option;
  CN_ReferenceLanes referenceLanes;
} CN_ReferenceLink;


extern const ASN1CType asn1_type_CN_ReferenceLink[];

typedef struct CN_ReferenceLinkList {
  CN_ReferenceLink *tab;
  size_t count;
} CN_ReferenceLinkList;

extern const ASN1CType asn1_type_CN_ReferenceLinkList[];

typedef struct CN_RTEData {
  int rteId;
  CN_EventType eventType;
  CN_EventSource eventSource;
  BOOL eventPos_option;
  CN_PositionOffsetLLV eventPos;
  BOOL eventRadius_option;
  CN_Radius eventRadius;
  BOOL description_option;
  CN_Description description;
  BOOL timeDetails_option;
  CN_RSITimeDetails timeDetails;
  BOOL priority_option;
  CN_RSIPriority priority;
  BOOL referencePaths_option;
  CN_ReferencePathList referencePaths;
  BOOL referenceLinks_option;
  CN_ReferenceLinkList referenceLinks;
  BOOL eventConfidence_option;
  CN_Confidence eventConfidence;
} CN_RTEData;


extern const ASN1CType asn1_type_CN_RTEData[];

typedef struct CN_RTEList {
  CN_RTEData *tab;
  size_t count;
} CN_RTEList;

extern const ASN1CType asn1_type_CN_RTEList[];

typedef int CN_SignType;

extern const ASN1CType asn1_type_CN_SignType[];

typedef struct CN_RTSData {
  int rtsId;
  CN_SignType signType;
  BOOL signPos_option;
  CN_PositionOffsetLLV signPos;
  BOOL description_option;
  CN_Description description;
  BOOL timeDetails_option;
  CN_RSITimeDetails timeDetails;
  BOOL priority_option;
  CN_RSIPriority priority;
  BOOL referencePaths_option;
  CN_ReferencePathList referencePaths;
  BOOL referenceLinks_option;
  CN_ReferenceLinkList referenceLinks;
} CN_RTSData;


extern const ASN1CType asn1_type_CN_RTSData[];

typedef struct CN_RTSList {
  CN_RTSData *tab;
  size_t count;
} CN_RTSList;

extern const ASN1CType asn1_type_CN_RTSList[];

typedef struct CN_RoadSideInformation {
  CN_MsgCount msgCnt;
  BOOL moy_option;
  CN_MinuteOfTheYear moy;
  ASN1String id;
  CN_Position3D refPos;
  BOOL rtes_option;
  CN_RTEList rtes;
  BOOL rtss_option;
  CN_RTSList rtss;
} CN_RoadSideInformation;


extern const ASN1CType asn1_type_CN_RoadSideInformation[];

typedef enum {
  CN_MessageFrame_bsmFrame,
  CN_MessageFrame_mapFrame,
  CN_MessageFrame_rsmFrame,
  CN_MessageFrame_spatFrame,
  CN_MessageFrame_rsiFrame,
  CN_MessageFrame__dummy = 2147483647,
} CN_MessageFrame_choice;

typedef struct CN_MessageFrame {
  CN_MessageFrame_choice choice;
  union {
    CN_BasicSafetyMessage bsmFrame;
    CN_MapData mapFrame;
    CN_RoadsideSafetyMessage rsmFrame;
    CN_SPAT spatFrame;
    CN_RoadSideInformation rsiFrame;
  } u;
} CN_MessageFrame;

extern const ASN1CType asn1_type_CN_MessageFrame[];

#define CN_unknownVehicleClass 0

#define CN_specialVehicleClass 1

#define CN_passenger_Vehicle_TypeUnknown 10

#define CN_passenger_Vehicle_TypeOther 11

#define CN_lightTruck_Vehicle_TypeUnknown 20

#define CN_lightTruck_Vehicle_TypeOther 21

#define CN_truck_Vehicle_TypeUnknown 25

#define CN_truck_Vehicle_TypeOther 26

#define CN_truck_axleCnt2 27

#define CN_truck_axleCnt3 28

#define CN_truck_axleCnt4 29

#define CN_truck_axleCnt4Trailer 30

#define CN_truck_axleCnt5Trailer 31

#define CN_truck_axleCnt6Trailer 32

#define CN_truck_axleCnt5MultiTrailer 33

#define CN_truck_axleCnt6MultiTrailer 34

#define CN_truck_axleCnt7MultiTrailer 35

#define CN_motorcycle_TypeUnknown 40

#define CN_motorcycle_TypeOther 41

#define CN_motorcycle_Cruiser_Standard 42

#define CN_motorcycle_SportUnclad 43

#define CN_motorcycle_SportTouring 44

#define CN_motorcycle_SuperSport 45

#define CN_motorcycle_Touring 46

#define CN_motorcycle_Trike 47

#define CN_motorcycle_wPassengers 48

#define CN_transit_TypeUnknown 50

#define CN_transit_TypeOther 51

#define CN_transit_BRT 52

#define CN_transit_ExpressBus 53

#define CN_transit_LocalBus 54

#define CN_transit_SchoolBus 55

#define CN_transit_FixedGuideway 56

#define CN_transit_Paratransit 57

#define CN_transit_Paratransit_Ambulance 58

#define CN_emergency_TypeUnknown 60

#define CN_emergency_TypeOther 61

#define CN_emergency_Fire_Light_Vehicle 62

#define CN_emergency_Fire_Heavy_Vehicle 63

#define CN_emergency_Fire_Paramedic_Vehicle 64

#define CN_emergency_Fire_Ambulance_Vehicle 65

#define CN_emergency_Police_Light_Vehicle 66

#define CN_emergency_Police_Heavy_Vehicle 67

#define CN_emergency_Other_Responder 68

#define CN_emergency_Other_Ambulance 69

#define CN_otherTraveler_TypeUnknown 80

#define CN_otherTraveler_TypeOther 81

#define CN_otherTraveler_Pedestrian 82

#define CN_otherTraveler_Visually_Disabled 83

#define CN_otherTraveler_Physically_Disabled 84

#define CN_otherTraveler_Bicycle 85

#define CN_otherTraveler_Vulnerable_Roadworker 86

#define CN_infrastructure_TypeUnknown 90

#define CN_infrastructure_Fixed 91

#define CN_infrastructure_Movable 92

#define CN_equipped_CargoTrailer 93

#define CN_unknownFuel 0

#define CN_gasoline 1

#define CN_ethanol 2

#define CN_diesel 3

#define CN_electric 4

#define CN_hybrid 5

#define CN_hydrogen 6

#define CN_natGasLiquid 7

#define CN_natGasComp 8

#define CN_propane 9

#ifdef  __cplusplus
}
#endif

#endif /* _FFASN1_V2X_CN_ASN_H */
