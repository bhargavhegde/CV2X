/* Automatically generated file - do not edit */
#ifndef _FFASN1_V2X_EU_ASN_H
#define _FFASN1_V2X_EU_ASN_H

#include "asn1defs.h"

#ifdef  __cplusplus
extern "C" {
#endif

typedef unsigned int EU_StationID;

extern const ASN1CType asn1_type_EU_StationID[];

typedef struct EU_ItsPduHeader {
  int protocolVersion;
  int messageID;
  EU_StationID stationID;
} EU_ItsPduHeader;


extern const ASN1CType asn1_type_EU_ItsPduHeader[];

typedef int EU_GenerationDeltaTime;

enum {
  EU_GenerationDeltaTime_oneMilliSec = 1,
};

extern const ASN1CType asn1_type_EU_GenerationDeltaTime[];

typedef int EU_StationType;

enum {
  EU_StationType_unknown = 0,
  EU_StationType_pedestrian = 1,
  EU_StationType_cyclist = 2,
  EU_StationType_moped = 3,
  EU_StationType_motorcycle = 4,
  EU_StationType_passengerCar = 5,
  EU_StationType_bus = 6,
  EU_StationType_lightTruck = 7,
  EU_StationType_heavyTruck = 8,
  EU_StationType_trailer = 9,
  EU_StationType_specialVehicles = 10,
  EU_StationType_tram = 11,
  EU_StationType_lightVruVehicle = 12,
  EU_StationType_animal = 13,
  EU_StationType_roadSideUnit = 15,
};

extern const ASN1CType asn1_type_EU_StationType[];

typedef int EU_Latitude;

enum {
  EU_Latitude_oneMicrodegreeNorth = 10,
  EU_Latitude_oneMicrodegreeSouth = -10,
  EU_Latitude_unavailable = 900000001,
};

extern const ASN1CType asn1_type_EU_Latitude[];

typedef int EU_Longitude;

enum {
  EU_Longitude_oneMicrodegreeEast = 10,
  EU_Longitude_oneMicrodegreeWest = -10,
  EU_Longitude_unavailable = 1800000001,
};

extern const ASN1CType asn1_type_EU_Longitude[];

typedef int EU_SemiAxisLength;

enum {
  EU_SemiAxisLength_oneCentimeter = 1,
  EU_SemiAxisLength_outOfRange = 4094,
  EU_SemiAxisLength_unavailable = 4095,
};

extern const ASN1CType asn1_type_EU_SemiAxisLength[];

typedef int EU_HeadingValue;

enum {
  EU_HeadingValue_wgs84North = 0,
  EU_HeadingValue_wgs84East = 900,
  EU_HeadingValue_wgs84South = 1800,
  EU_HeadingValue_wgs84West = 2700,
  EU_HeadingValue_unavailable = 3601,
};

extern const ASN1CType asn1_type_EU_HeadingValue[];

typedef struct EU_PosConfidenceEllipse {
  EU_SemiAxisLength semiMajorConfidence;
  EU_SemiAxisLength semiMinorConfidence;
  EU_HeadingValue semiMajorOrientation;
} EU_PosConfidenceEllipse;


extern const ASN1CType asn1_type_EU_PosConfidenceEllipse[];

typedef int EU_AltitudeValue;

enum {
  EU_AltitudeValue_referenceEllipsoidSurface = 0,
  EU_AltitudeValue_oneCentimeter = 1,
  EU_AltitudeValue_unavailable = 800001,
};

extern const ASN1CType asn1_type_EU_AltitudeValue[];

typedef enum EU_AltitudeConfidence {
  EU_AltitudeConfidence_alt_000_01,
  EU_AltitudeConfidence_alt_000_02,
  EU_AltitudeConfidence_alt_000_05,
  EU_AltitudeConfidence_alt_000_10,
  EU_AltitudeConfidence_alt_000_20,
  EU_AltitudeConfidence_alt_000_50,
  EU_AltitudeConfidence_alt_001_00,
  EU_AltitudeConfidence_alt_002_00,
  EU_AltitudeConfidence_alt_005_00,
  EU_AltitudeConfidence_alt_010_00,
  EU_AltitudeConfidence_alt_020_00,
  EU_AltitudeConfidence_alt_050_00,
  EU_AltitudeConfidence_alt_100_00,
  EU_AltitudeConfidence_alt_200_00,
  EU_AltitudeConfidence_outOfRange,
  EU_AltitudeConfidence_unavailable,
  EU_AltitudeConfidence__dummy = 2147483647,
} EU_AltitudeConfidence;

extern const ASN1CType asn1_type_EU_AltitudeConfidence[];

typedef struct EU_Altitude {
  EU_AltitudeValue altitudeValue;
  EU_AltitudeConfidence altitudeConfidence;
} EU_Altitude;


extern const ASN1CType asn1_type_EU_Altitude[];

typedef struct EU_ReferencePosition {
  EU_Latitude latitude;
  EU_Longitude longitude;
  EU_PosConfidenceEllipse positionConfidenceEllipse;
  EU_Altitude altitude;
} EU_ReferencePosition;


extern const ASN1CType asn1_type_EU_ReferencePosition[];

typedef struct EU_BasicContainer {
  EU_StationType stationType;
  EU_ReferencePosition referencePosition;
} EU_BasicContainer;


extern const ASN1CType asn1_type_EU_BasicContainer[];

typedef int EU_HeadingConfidence;

enum {
  EU_HeadingConfidence_equalOrWithinZeroPointOneDegree = 1,
  EU_HeadingConfidence_equalOrWithinOneDegree = 10,
  EU_HeadingConfidence_outOfRange = 126,
  EU_HeadingConfidence_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_HeadingConfidence[];

typedef struct EU_Heading {
  EU_HeadingValue headingValue;
  EU_HeadingConfidence headingConfidence;
} EU_Heading;


extern const ASN1CType asn1_type_EU_Heading[];

typedef int EU_SpeedValue;

enum {
  EU_SpeedValue_standstill = 0,
  EU_SpeedValue_oneCentimeterPerSec = 1,
  EU_SpeedValue_unavailable = 16383,
};

extern const ASN1CType asn1_type_EU_SpeedValue[];

typedef int EU_SpeedConfidence;

enum {
  EU_SpeedConfidence_equalOrWithinOneCentimeterPerSec = 1,
  EU_SpeedConfidence_equalOrWithinOneMeterPerSec = 100,
  EU_SpeedConfidence_outOfRange = 126,
  EU_SpeedConfidence_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_SpeedConfidence[];

typedef struct EU_Speed {
  EU_SpeedValue speedValue;
  EU_SpeedConfidence speedConfidence;
} EU_Speed;


extern const ASN1CType asn1_type_EU_Speed[];

typedef enum EU_DriveDirection {
  EU_DriveDirection_forward,
  EU_DriveDirection_backward,
  EU_DriveDirection_unavailable,
  EU_DriveDirection__dummy = 2147483647,
} EU_DriveDirection;

extern const ASN1CType asn1_type_EU_DriveDirection[];

typedef int EU_VehicleLengthValue;

enum {
  EU_VehicleLengthValue_tenCentimeters = 1,
  EU_VehicleLengthValue_outOfRange = 1022,
  EU_VehicleLengthValue_unavailable = 1023,
};

extern const ASN1CType asn1_type_EU_VehicleLengthValue[];

typedef enum EU_VehicleLengthConfidenceIndication {
  EU_VehicleLengthConfidenceIndication_noTrailerPresent,
  EU_VehicleLengthConfidenceIndication_trailerPresentWithKnownLength,
  EU_VehicleLengthConfidenceIndication_trailerPresentWithUnknownLength,
  EU_VehicleLengthConfidenceIndication_trailerPresenceIsUnknown,
  EU_VehicleLengthConfidenceIndication_unavailable,
  EU_VehicleLengthConfidenceIndication__dummy = 2147483647,
} EU_VehicleLengthConfidenceIndication;

extern const ASN1CType asn1_type_EU_VehicleLengthConfidenceIndication[];

typedef struct EU_VehicleLength {
  EU_VehicleLengthValue vehicleLengthValue;
  EU_VehicleLengthConfidenceIndication vehicleLengthConfidenceIndication;
} EU_VehicleLength;


extern const ASN1CType asn1_type_EU_VehicleLength[];

typedef int EU_VehicleWidth;

enum {
  EU_VehicleWidth_tenCentimeters = 1,
  EU_VehicleWidth_outOfRange = 61,
  EU_VehicleWidth_unavailable = 62,
};

extern const ASN1CType asn1_type_EU_VehicleWidth[];

typedef int EU_LongitudinalAccelerationValue;

enum {
  EU_LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward = 1,
  EU_LongitudinalAccelerationValue_pointOneMeterPerSecSquaredBackward = -1,
  EU_LongitudinalAccelerationValue_unavailable = 161,
};

extern const ASN1CType asn1_type_EU_LongitudinalAccelerationValue[];

typedef int EU_AccelerationConfidence;

enum {
  EU_AccelerationConfidence_pointOneMeterPerSecSquared = 1,
  EU_AccelerationConfidence_outOfRange = 101,
  EU_AccelerationConfidence_unavailable = 102,
};

extern const ASN1CType asn1_type_EU_AccelerationConfidence[];

typedef struct EU_LongitudinalAcceleration {
  EU_LongitudinalAccelerationValue longitudinalAccelerationValue;
  EU_AccelerationConfidence longitudinalAccelerationConfidence;
} EU_LongitudinalAcceleration;


extern const ASN1CType asn1_type_EU_LongitudinalAcceleration[];

typedef int EU_CurvatureValue;

enum {
  EU_CurvatureValue_straight = 0,
  EU_CurvatureValue_unavailable = 1023,
};

extern const ASN1CType asn1_type_EU_CurvatureValue[];

typedef enum EU_CurvatureConfidence {
  EU_CurvatureConfidence_onePerMeter_0_00002,
  EU_CurvatureConfidence_onePerMeter_0_0001,
  EU_CurvatureConfidence_onePerMeter_0_0005,
  EU_CurvatureConfidence_onePerMeter_0_002,
  EU_CurvatureConfidence_onePerMeter_0_01,
  EU_CurvatureConfidence_onePerMeter_0_1,
  EU_CurvatureConfidence_outOfRange,
  EU_CurvatureConfidence_unavailable,
  EU_CurvatureConfidence__dummy = 2147483647,
} EU_CurvatureConfidence;

extern const ASN1CType asn1_type_EU_CurvatureConfidence[];

typedef struct EU_Curvature {
  EU_CurvatureValue curvatureValue;
  EU_CurvatureConfidence curvatureConfidence;
} EU_Curvature;


extern const ASN1CType asn1_type_EU_Curvature[];

typedef enum EU_CurvatureCalculationMode {
  EU_CurvatureCalculationMode_yawRateUsed,
  EU_CurvatureCalculationMode_yawRateNotUsed,
  EU_CurvatureCalculationMode_unavailable,
  EU_CurvatureCalculationMode__dummy = 2147483647,
} EU_CurvatureCalculationMode;

extern const ASN1CType asn1_type_EU_CurvatureCalculationMode[];

typedef int EU_YawRateValue;

enum {
  EU_YawRateValue_straight = 0,
  EU_YawRateValue_degSec_000_01ToRight = -1,
  EU_YawRateValue_degSec_000_01ToLeft = 1,
  EU_YawRateValue_unavailable = 32767,
};

extern const ASN1CType asn1_type_EU_YawRateValue[];

typedef enum EU_YawRateConfidence {
  EU_YawRateConfidence_degSec_000_01,
  EU_YawRateConfidence_degSec_000_05,
  EU_YawRateConfidence_degSec_000_10,
  EU_YawRateConfidence_degSec_001_00,
  EU_YawRateConfidence_degSec_005_00,
  EU_YawRateConfidence_degSec_010_00,
  EU_YawRateConfidence_degSec_100_00,
  EU_YawRateConfidence_outOfRange,
  EU_YawRateConfidence_unavailable,
  EU_YawRateConfidence__dummy = 2147483647,
} EU_YawRateConfidence;

extern const ASN1CType asn1_type_EU_YawRateConfidence[];

typedef struct EU_YawRate {
  EU_YawRateValue yawRateValue;
  EU_YawRateConfidence yawRateConfidence;
} EU_YawRate;


extern const ASN1CType asn1_type_EU_YawRate[];

typedef ASN1BitString EU_AccelerationControl;

extern const ASN1CType asn1_type_EU_AccelerationControl[];

typedef int EU_LanePosition;

enum {
  EU_LanePosition_offTheRoad = -1,
  EU_LanePosition_innerHardShoulder = 0,
  EU_LanePosition_innermostDrivingLane = 1,
  EU_LanePosition_secondLaneFromInside = 2,
  EU_LanePosition_outerHardShoulder = 14,
};

extern const ASN1CType asn1_type_EU_LanePosition[];

typedef int EU_SteeringWheelAngleValue;

enum {
  EU_SteeringWheelAngleValue_straight = 0,
  EU_SteeringWheelAngleValue_onePointFiveDegreesToRight = -1,
  EU_SteeringWheelAngleValue_onePointFiveDegreesToLeft = 1,
  EU_SteeringWheelAngleValue_unavailable = 512,
};

extern const ASN1CType asn1_type_EU_SteeringWheelAngleValue[];

typedef int EU_SteeringWheelAngleConfidence;

enum {
  EU_SteeringWheelAngleConfidence_equalOrWithinOnePointFiveDegree = 1,
  EU_SteeringWheelAngleConfidence_outOfRange = 126,
  EU_SteeringWheelAngleConfidence_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_SteeringWheelAngleConfidence[];

typedef struct EU_SteeringWheelAngle {
  EU_SteeringWheelAngleValue steeringWheelAngleValue;
  EU_SteeringWheelAngleConfidence steeringWheelAngleConfidence;
} EU_SteeringWheelAngle;


extern const ASN1CType asn1_type_EU_SteeringWheelAngle[];

typedef int EU_LateralAccelerationValue;

enum {
  EU_LateralAccelerationValue_pointOneMeterPerSecSquaredToRight = -1,
  EU_LateralAccelerationValue_pointOneMeterPerSecSquaredToLeft = 1,
  EU_LateralAccelerationValue_unavailable = 161,
};

extern const ASN1CType asn1_type_EU_LateralAccelerationValue[];

typedef struct EU_LateralAcceleration {
  EU_LateralAccelerationValue lateralAccelerationValue;
  EU_AccelerationConfidence lateralAccelerationConfidence;
} EU_LateralAcceleration;


extern const ASN1CType asn1_type_EU_LateralAcceleration[];

typedef int EU_VerticalAccelerationValue;

enum {
  EU_VerticalAccelerationValue_pointOneMeterPerSecSquaredUp = 1,
  EU_VerticalAccelerationValue_pointOneMeterPerSecSquaredDown = -1,
  EU_VerticalAccelerationValue_unavailable = 161,
};

extern const ASN1CType asn1_type_EU_VerticalAccelerationValue[];

typedef struct EU_VerticalAcceleration {
  EU_VerticalAccelerationValue verticalAccelerationValue;
  EU_AccelerationConfidence verticalAccelerationConfidence;
} EU_VerticalAcceleration;


extern const ASN1CType asn1_type_EU_VerticalAcceleration[];

typedef int EU_PerformanceClass;

enum {
  EU_PerformanceClass_unavailable = 0,
  EU_PerformanceClass_performanceClassA = 1,
  EU_PerformanceClass_performanceClassB = 2,
};

extern const ASN1CType asn1_type_EU_PerformanceClass[];

typedef int EU_ProtectedZoneID;

extern const ASN1CType asn1_type_EU_ProtectedZoneID[];

typedef EU_ProtectedZoneID EU_CenDsrcTollingZoneID;

extern const ASN1CType asn1_type_EU_CenDsrcTollingZoneID[];

typedef struct EU_CenDsrcTollingZone {
  EU_Latitude protectedZoneLatitude;
  EU_Longitude protectedZoneLongitude;
  BOOL cenDsrcTollingZoneID_option;
  EU_CenDsrcTollingZoneID cenDsrcTollingZoneID;
} EU_CenDsrcTollingZone;


extern const ASN1CType asn1_type_EU_CenDsrcTollingZone[];

typedef struct EU_BasicVehicleContainerHighFrequency {
  EU_Heading heading;
  EU_Speed speed;
  EU_DriveDirection driveDirection;
  EU_VehicleLength vehicleLength;
  EU_VehicleWidth vehicleWidth;
  EU_LongitudinalAcceleration longitudinalAcceleration;
  EU_Curvature curvature;
  EU_CurvatureCalculationMode curvatureCalculationMode;
  EU_YawRate yawRate;
  BOOL accelerationControl_option;
  EU_AccelerationControl accelerationControl;
  BOOL lanePosition_option;
  EU_LanePosition lanePosition;
  BOOL steeringWheelAngle_option;
  EU_SteeringWheelAngle steeringWheelAngle;
  BOOL lateralAcceleration_option;
  EU_LateralAcceleration lateralAcceleration;
  BOOL verticalAcceleration_option;
  EU_VerticalAcceleration verticalAcceleration;
  BOOL performanceClass_option;
  EU_PerformanceClass performanceClass;
  BOOL cenDsrcTollingZone_option;
  EU_CenDsrcTollingZone cenDsrcTollingZone;
} EU_BasicVehicleContainerHighFrequency;


extern const ASN1CType asn1_type_EU_BasicVehicleContainerHighFrequency[];

typedef enum EU_ProtectedZoneType {
  EU_ProtectedZoneType_permanentCenDsrcTolling,
  EU_ProtectedZoneType_temporaryCenDsrcTolling,
  EU_ProtectedZoneType__dummy = 2147483647,
} EU_ProtectedZoneType;

extern const ASN1CType asn1_type_EU_ProtectedZoneType[];

typedef ASN1Integer EU_TimestampIts;

enum {
  EU_TimestampIts_utcStartOf2004 = 0,
  EU_TimestampIts_oneMillisecAfterUTCStartOf2004 = 1,
};

extern const ASN1CType asn1_type_EU_TimestampIts[];

typedef int EU_ProtectedZoneRadius;

enum {
  EU_ProtectedZoneRadius_oneMeter = 1,
};

extern const ASN1CType asn1_type_EU_ProtectedZoneRadius[];

typedef struct EU_ProtectedCommunicationZone {
  EU_ProtectedZoneType protectedZoneType;
  BOOL expiryTime_option;
  EU_TimestampIts expiryTime;
  EU_Latitude protectedZoneLatitude;
  EU_Longitude protectedZoneLongitude;
  BOOL protectedZoneRadius_option;
  EU_ProtectedZoneRadius protectedZoneRadius;
  BOOL protectedZoneID_option;
  EU_ProtectedZoneID protectedZoneID;
} EU_ProtectedCommunicationZone;


extern const ASN1CType asn1_type_EU_ProtectedCommunicationZone[];

typedef struct EU_ProtectedCommunicationZonesRSU {
  EU_ProtectedCommunicationZone *tab;
  size_t count;
} EU_ProtectedCommunicationZonesRSU;

extern const ASN1CType asn1_type_EU_ProtectedCommunicationZonesRSU[];

typedef struct EU_RSUContainerHighFrequency {
  BOOL protectedCommunicationZonesRSU_option;
  EU_ProtectedCommunicationZonesRSU protectedCommunicationZonesRSU;
} EU_RSUContainerHighFrequency;


extern const ASN1CType asn1_type_EU_RSUContainerHighFrequency[];

typedef enum {
  EU_HighFrequencyContainer_basicVehicleContainerHighFrequency,
  EU_HighFrequencyContainer_rsuContainerHighFrequency,
  EU_HighFrequencyContainer__dummy = 2147483647,
} EU_HighFrequencyContainer_choice;

typedef struct EU_HighFrequencyContainer {
  EU_HighFrequencyContainer_choice choice;
  union {
    EU_BasicVehicleContainerHighFrequency basicVehicleContainerHighFrequency;
    EU_RSUContainerHighFrequency rsuContainerHighFrequency;
  } u;
} EU_HighFrequencyContainer;

extern const ASN1CType asn1_type_EU_HighFrequencyContainer[];

typedef enum EU_VehicleRole {
  EU_VehicleRole_Default,
  EU_VehicleRole_publicTransport,
  EU_VehicleRole_specialTransport,
  EU_VehicleRole_dangerousGoods,
  EU_VehicleRole_roadWork,
  EU_VehicleRole_rescue,
  EU_VehicleRole_emergency,
  EU_VehicleRole_safetyCar,
  EU_VehicleRole_agriculture,
  EU_VehicleRole_commercial,
  EU_VehicleRole_military,
  EU_VehicleRole_roadOperator,
  EU_VehicleRole_taxi,
  EU_VehicleRole_reserved1,
  EU_VehicleRole_reserved2,
  EU_VehicleRole_reserved3,
  EU_VehicleRole__dummy = 2147483647,
} EU_VehicleRole;

extern const ASN1CType asn1_type_EU_VehicleRole[];

typedef ASN1BitString EU_ExteriorLights;

extern const ASN1CType asn1_type_EU_ExteriorLights[];

typedef int EU_DeltaLatitude;

enum {
  EU_DeltaLatitude_oneMicrodegreeNorth = 10,
  EU_DeltaLatitude_oneMicrodegreeSouth = -10,
  EU_DeltaLatitude_unavailable = 131072,
};

extern const ASN1CType asn1_type_EU_DeltaLatitude[];

typedef int EU_DeltaLongitude;

enum {
  EU_DeltaLongitude_oneMicrodegreeEast = 10,
  EU_DeltaLongitude_oneMicrodegreeWest = -10,
  EU_DeltaLongitude_unavailable = 131072,
};

extern const ASN1CType asn1_type_EU_DeltaLongitude[];

typedef int EU_DeltaAltitude;

enum {
  EU_DeltaAltitude_oneCentimeterUp = 1,
  EU_DeltaAltitude_oneCentimeterDown = -1,
  EU_DeltaAltitude_unavailable = 12800,
};

extern const ASN1CType asn1_type_EU_DeltaAltitude[];

typedef struct EU_DeltaReferencePosition {
  EU_DeltaLatitude deltaLatitude;
  EU_DeltaLongitude deltaLongitude;
  EU_DeltaAltitude deltaAltitude;
} EU_DeltaReferencePosition;


extern const ASN1CType asn1_type_EU_DeltaReferencePosition[];

typedef int EU_PathDeltaTime;

enum {
  EU_PathDeltaTime_tenMilliSecondsInPast = 1,
};

extern const ASN1CType asn1_type_EU_PathDeltaTime[];

typedef struct EU_PathPoint {
  EU_DeltaReferencePosition pathPosition;
  BOOL pathDeltaTime_option;
  EU_PathDeltaTime pathDeltaTime;
} EU_PathPoint;


extern const ASN1CType asn1_type_EU_PathPoint[];

typedef struct EU_PathHistory {
  EU_PathPoint *tab;
  size_t count;
} EU_PathHistory;

extern const ASN1CType asn1_type_EU_PathHistory[];

typedef struct EU_BasicVehicleContainerLowFrequency {
  EU_VehicleRole vehicleRole;
  EU_ExteriorLights exteriorLights;
  EU_PathHistory pathHistory;
} EU_BasicVehicleContainerLowFrequency;


extern const ASN1CType asn1_type_EU_BasicVehicleContainerLowFrequency[];

typedef enum {
  EU_LowFrequencyContainer_basicVehicleContainerLowFrequency,
  EU_LowFrequencyContainer__dummy = 2147483647,
} EU_LowFrequencyContainer_choice;

typedef struct EU_LowFrequencyContainer {
  EU_LowFrequencyContainer_choice choice;
  union {
    EU_BasicVehicleContainerLowFrequency basicVehicleContainerLowFrequency;
  } u;
} EU_LowFrequencyContainer;

extern const ASN1CType asn1_type_EU_LowFrequencyContainer[];

typedef BOOL EU_EmbarkationStatus;

extern const ASN1CType asn1_type_EU_EmbarkationStatus[];

typedef int EU_PtActivationType;

enum {
  EU_PtActivationType_undefinedCodingType = 0,
  EU_PtActivationType_r09_16CodingType = 1,
  EU_PtActivationType_vdv_50149CodingType = 2,
};

extern const ASN1CType asn1_type_EU_PtActivationType[];

typedef ASN1String EU_PtActivationData;

extern const ASN1CType asn1_type_EU_PtActivationData[];

typedef struct EU_PtActivation {
  EU_PtActivationType ptActivationType;
  EU_PtActivationData ptActivationData;
} EU_PtActivation;


extern const ASN1CType asn1_type_EU_PtActivation[];

typedef struct EU_PublicTransportContainer {
  EU_EmbarkationStatus embarkationStatus;
  BOOL ptActivation_option;
  EU_PtActivation ptActivation;
} EU_PublicTransportContainer;


extern const ASN1CType asn1_type_EU_PublicTransportContainer[];

typedef ASN1BitString EU_SpecialTransportType;

extern const ASN1CType asn1_type_EU_SpecialTransportType[];

typedef ASN1BitString EU_LightBarSirenInUse;

extern const ASN1CType asn1_type_EU_LightBarSirenInUse[];

typedef struct EU_SpecialTransportContainer {
  EU_SpecialTransportType specialTransportType;
  EU_LightBarSirenInUse lightBarSirenInUse;
} EU_SpecialTransportContainer;


extern const ASN1CType asn1_type_EU_SpecialTransportContainer[];

typedef enum EU_DangerousGoodsBasic {
  EU_DangerousGoodsBasic_explosives1,
  EU_DangerousGoodsBasic_explosives2,
  EU_DangerousGoodsBasic_explosives3,
  EU_DangerousGoodsBasic_explosives4,
  EU_DangerousGoodsBasic_explosives5,
  EU_DangerousGoodsBasic_explosives6,
  EU_DangerousGoodsBasic_flammableGases,
  EU_DangerousGoodsBasic_nonFlammableGases,
  EU_DangerousGoodsBasic_toxicGases,
  EU_DangerousGoodsBasic_flammableLiquids,
  EU_DangerousGoodsBasic_flammableSolids,
  EU_DangerousGoodsBasic_substancesLiableToSpontaneousCombustion,
  EU_DangerousGoodsBasic_substancesEmittingFlammableGasesUponContactWithWater,
  EU_DangerousGoodsBasic_oxidizingSubstances,
  EU_DangerousGoodsBasic_organicPeroxides,
  EU_DangerousGoodsBasic_toxicSubstances,
  EU_DangerousGoodsBasic_infectiousSubstances,
  EU_DangerousGoodsBasic_radioactiveMaterial,
  EU_DangerousGoodsBasic_corrosiveSubstances,
  EU_DangerousGoodsBasic_miscellaneousDangerousSubstances,
  EU_DangerousGoodsBasic__dummy = 2147483647,
} EU_DangerousGoodsBasic;

extern const ASN1CType asn1_type_EU_DangerousGoodsBasic[];

typedef struct EU_DangerousGoodsContainer {
  EU_DangerousGoodsBasic dangerousGoodsBasic;
} EU_DangerousGoodsContainer;


extern const ASN1CType asn1_type_EU_DangerousGoodsContainer[];

typedef int EU_RoadworksSubCauseCode;

enum {
  EU_RoadworksSubCauseCode_unavailable = 0,
  EU_RoadworksSubCauseCode_majorRoadworks = 1,
  EU_RoadworksSubCauseCode_roadMarkingWork = 2,
  EU_RoadworksSubCauseCode_slowMovingRoadMaintenance = 3,
  EU_RoadworksSubCauseCode_shortTermStationaryRoadworks = 4,
  EU_RoadworksSubCauseCode_streetCleaning = 5,
  EU_RoadworksSubCauseCode_winterService = 6,
};

extern const ASN1CType asn1_type_EU_RoadworksSubCauseCode[];

typedef enum EU_HardShoulderStatus {
  EU_HardShoulderStatus_availableForStopping,
  EU_HardShoulderStatus_closed,
  EU_HardShoulderStatus_availableForDriving,
  EU_HardShoulderStatus__dummy = 2147483647,
} EU_HardShoulderStatus;

extern const ASN1CType asn1_type_EU_HardShoulderStatus[];

typedef ASN1BitString EU_DrivingLaneStatus;

extern const ASN1CType asn1_type_EU_DrivingLaneStatus[];

typedef struct EU_ClosedLanes {
  BOOL innerhardShoulderStatus_option;
  EU_HardShoulderStatus innerhardShoulderStatus;
  BOOL outerhardShoulderStatus_option;
  EU_HardShoulderStatus outerhardShoulderStatus;
  BOOL drivingLaneStatus_option;
  EU_DrivingLaneStatus drivingLaneStatus;
} EU_ClosedLanes;


extern const ASN1CType asn1_type_EU_ClosedLanes[];

typedef struct EU_RoadWorksContainerBasic {
  BOOL roadworksSubCauseCode_option;
  EU_RoadworksSubCauseCode roadworksSubCauseCode;
  EU_LightBarSirenInUse lightBarSirenInUse;
  BOOL closedLanes_option;
  EU_ClosedLanes closedLanes;
} EU_RoadWorksContainerBasic;


extern const ASN1CType asn1_type_EU_RoadWorksContainerBasic[];

typedef struct EU_RescueContainer {
  EU_LightBarSirenInUse lightBarSirenInUse;
} EU_RescueContainer;


extern const ASN1CType asn1_type_EU_RescueContainer[];

typedef int EU_CauseCodeType;

enum {
  EU_CauseCodeType_reserved = 0,
  EU_CauseCodeType_trafficCondition = 1,
  EU_CauseCodeType_accident = 2,
  EU_CauseCodeType_roadworks = 3,
  EU_CauseCodeType_impassability = 5,
  EU_CauseCodeType_adverseWeatherCondition_Adhesion = 6,
  EU_CauseCodeType_aquaplannning = 7,
  EU_CauseCodeType_hazardousLocation_SurfaceCondition = 9,
  EU_CauseCodeType_hazardousLocation_ObstacleOnTheRoad = 10,
  EU_CauseCodeType_hazardousLocation_AnimalOnTheRoad = 11,
  EU_CauseCodeType_humanPresenceOnTheRoad = 12,
  EU_CauseCodeType_wrongWayDriving = 14,
  EU_CauseCodeType_rescueAndRecoveryWorkInProgress = 15,
  EU_CauseCodeType_adverseWeatherCondition_ExtremeWeatherCondition = 17,
  EU_CauseCodeType_adverseWeatherCondition_Visibility = 18,
  EU_CauseCodeType_adverseWeatherCondition_Precipitation = 19,
  EU_CauseCodeType_slowVehicle = 26,
  EU_CauseCodeType_dangerousEndOfQueue = 27,
  EU_CauseCodeType_vehicleBreakdown = 91,
  EU_CauseCodeType_postCrash = 92,
  EU_CauseCodeType_humanProblem = 93,
  EU_CauseCodeType_stationaryVehicle = 94,
  EU_CauseCodeType_emergencyVehicleApproaching = 95,
  EU_CauseCodeType_hazardousLocation_DangerousCurve = 96,
  EU_CauseCodeType_collisionRisk = 97,
  EU_CauseCodeType_signalViolation = 98,
  EU_CauseCodeType_dangerousSituation = 99,
};

extern const ASN1CType asn1_type_EU_CauseCodeType[];

typedef int EU_SubCauseCodeType;

extern const ASN1CType asn1_type_EU_SubCauseCodeType[];

typedef struct EU_CauseCode {
  EU_CauseCodeType causeCode;
  EU_SubCauseCodeType subCauseCode;
} EU_CauseCode;


extern const ASN1CType asn1_type_EU_CauseCode[];

typedef ASN1BitString EU_EmergencyPriority;

extern const ASN1CType asn1_type_EU_EmergencyPriority[];

typedef struct EU_EmergencyContainer {
  EU_LightBarSirenInUse lightBarSirenInUse;
  BOOL incidentIndication_option;
  EU_CauseCode incidentIndication;
  BOOL emergencyPriority_option;
  EU_EmergencyPriority emergencyPriority;
} EU_EmergencyContainer;


extern const ASN1CType asn1_type_EU_EmergencyContainer[];

typedef enum EU_TrafficRule {
  EU_TrafficRule_noPassing,
  EU_TrafficRule_noPassingForTrucks,
  EU_TrafficRule_passToRight,
  EU_TrafficRule_passToLeft,
  EU_TrafficRule__dummy = 2147483647,
} EU_TrafficRule;

extern const ASN1CType asn1_type_EU_TrafficRule[];

typedef int EU_SpeedLimit;

enum {
  EU_SpeedLimit_oneKmPerHour = 1,
};

extern const ASN1CType asn1_type_EU_SpeedLimit[];

typedef struct EU_SafetyCarContainer {
  EU_LightBarSirenInUse lightBarSirenInUse;
  BOOL incidentIndication_option;
  EU_CauseCode incidentIndication;
  BOOL trafficRule_option;
  EU_TrafficRule trafficRule;
  BOOL speedLimit_option;
  EU_SpeedLimit speedLimit;
} EU_SafetyCarContainer;


extern const ASN1CType asn1_type_EU_SafetyCarContainer[];

typedef enum {
  EU_SpecialVehicleContainer_publicTransportContainer,
  EU_SpecialVehicleContainer_specialTransportContainer,
  EU_SpecialVehicleContainer_dangerousGoodsContainer,
  EU_SpecialVehicleContainer_roadWorksContainerBasic,
  EU_SpecialVehicleContainer_rescueContainer,
  EU_SpecialVehicleContainer_emergencyContainer,
  EU_SpecialVehicleContainer_safetyCarContainer,
  EU_SpecialVehicleContainer__dummy = 2147483647,
} EU_SpecialVehicleContainer_choice;

typedef struct EU_SpecialVehicleContainer {
  EU_SpecialVehicleContainer_choice choice;
  union {
    EU_PublicTransportContainer publicTransportContainer;
    EU_SpecialTransportContainer specialTransportContainer;
    EU_DangerousGoodsContainer dangerousGoodsContainer;
    EU_RoadWorksContainerBasic roadWorksContainerBasic;
    EU_RescueContainer rescueContainer;
    EU_EmergencyContainer emergencyContainer;
    EU_SafetyCarContainer safetyCarContainer;
  } u;
} EU_SpecialVehicleContainer;

extern const ASN1CType asn1_type_EU_SpecialVehicleContainer[];

typedef struct EU_CamParameters {
  EU_BasicContainer basicContainer;
  EU_HighFrequencyContainer highFrequencyContainer;
  BOOL lowFrequencyContainer_option;
  EU_LowFrequencyContainer lowFrequencyContainer;
  BOOL specialVehicleContainer_option;
  EU_SpecialVehicleContainer specialVehicleContainer;
} EU_CamParameters;


extern const ASN1CType asn1_type_EU_CamParameters[];

typedef struct EU_CoopAwareness {
  EU_GenerationDeltaTime generationDeltaTime;
  EU_CamParameters camParameters;
} EU_CoopAwareness;


extern const ASN1CType asn1_type_EU_CoopAwareness[];

typedef struct EU_CAM {
  EU_ItsPduHeader header;
  EU_CoopAwareness cam;
} EU_CAM;


extern const ASN1CType asn1_type_EU_CAM[];

typedef int EU_CpmStationDataId;

extern const ASN1CType asn1_type_EU_CpmStationDataId[];

typedef struct EU_CPM_STATIONDATA_ID_AND_TYPE { /* object class definition */
  ASN1CType id;
  ASN1CType Type;
} EU_CPM_STATIONDATA_ID_AND_TYPE;


extern const ASN1CType asn1_type_EU_CPM_STATIONDATA_ID_AND_TYPE[];

typedef int EU_CpmContainerId;

extern const ASN1CType asn1_type_EU_CpmContainerId[];

typedef struct EU_CPM_CONTAINER_ID_AND_TYPE { /* object class definition */
  ASN1CType id;
  ASN1CType Type;
} EU_CPM_CONTAINER_ID_AND_TYPE;


extern const ASN1CType asn1_type_EU_CPM_CONTAINER_ID_AND_TYPE[];

typedef int EU_WGS84AngleValue;

enum {
  EU_WGS84AngleValue_wgs84North = 0,
  EU_WGS84AngleValue_wgs84East = 900,
  EU_WGS84AngleValue_wgs84South = 1800,
  EU_WGS84AngleValue_wgs84West = 2700,
  EU_WGS84AngleValue_unavailable = 3601,
};

extern const ASN1CType asn1_type_EU_WGS84AngleValue[];

typedef int EU_AngleConfidence;

enum {
  EU_AngleConfidence_zeroPointOneDegree = 1,
  EU_AngleConfidence_oneDegree = 10,
  EU_AngleConfidence_outOfRange = 126,
  EU_AngleConfidence_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_AngleConfidence[];

typedef struct EU_WGS84Angle {
  EU_WGS84AngleValue value;
  EU_AngleConfidence confidence;
} EU_WGS84Angle;


extern const ASN1CType asn1_type_EU_WGS84Angle[];

typedef int EU_CartesianAngleValue;

enum {
  EU_CartesianAngleValue_zeroPointOneDegree = 1,
  EU_CartesianAngleValue_oneDegree = 10,
  EU_CartesianAngleValue_unavailable = 3601,
};

extern const ASN1CType asn1_type_EU_CartesianAngleValue[];

typedef struct EU_CartesianAngle {
  EU_CartesianAngleValue value;
  EU_AngleConfidence confidence;
} EU_CartesianAngle;


extern const ASN1CType asn1_type_EU_CartesianAngle[];

typedef int EU_VehicleHeightCPM;

extern const ASN1CType asn1_type_EU_VehicleHeightCPM[];

typedef int EU_RefPointId;

extern const ASN1CType asn1_type_EU_RefPointId[];

typedef int EU_HitchPointOffset;

enum {
  EU_HitchPointOffset_zeroPointOneMeter = 1,
  EU_HitchPointOffset_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_HitchPointOffset[];

typedef int EU_FrontOverhang;

enum {
  EU_FrontOverhang_noOverhang = 0,
  EU_FrontOverhang_zeroPointOneMeter = 1,
  EU_FrontOverhang_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_FrontOverhang[];

typedef int EU_RearOverhang;

enum {
  EU_RearOverhang_zeroPointOneMeter = 1,
  EU_RearOverhang_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_RearOverhang[];

typedef struct EU_TrailerDataCPM {
  EU_RefPointId refPointId;
  EU_HitchPointOffset hitchPointOffset;
  EU_FrontOverhang frontOverhang;
  EU_RearOverhang rearOverhang;
  BOOL trailerWidth_option;
  EU_VehicleWidth trailerWidth;
  BOOL hitchAngle_option;
  EU_CartesianAngle hitchAngle;
} EU_TrailerDataCPM;


extern const ASN1CType asn1_type_EU_TrailerDataCPM[];

typedef struct EU_TrailerDataContainer {
  EU_TrailerDataCPM *tab;
  size_t count;
} EU_TrailerDataContainer;

extern const ASN1CType asn1_type_EU_TrailerDataContainer[];

typedef struct EU_OriginatingVehicleContainer {
  EU_Heading heading;
  EU_Speed speed;
  BOOL vehicleOrientationAngle_option;
  EU_WGS84Angle vehicleOrientationAngle;
  EU_DriveDirection driveDirection;
  BOOL longitudinalAcceleration_option;
  EU_LongitudinalAcceleration longitudinalAcceleration;
  BOOL lateralAcceleration_option;
  EU_LateralAcceleration lateralAcceleration;
  BOOL verticalAcceleration_option;
  EU_VerticalAcceleration verticalAcceleration;
  BOOL yawRate_option;
  EU_YawRate yawRate;
  BOOL pitchAngle_option;
  EU_CartesianAngle pitchAngle;
  BOOL rollAngle_option;
  EU_CartesianAngle rollAngle;
  BOOL vehicleLength_option;
  EU_VehicleLength vehicleLength;
  BOOL vehicleWidth_option;
  EU_VehicleWidth vehicleWidth;
  BOOL vehicleHeight_option;
  EU_VehicleHeightCPM vehicleHeight;
  BOOL trailerDataContainer_option;
  EU_TrailerDataContainer trailerDataContainer;
} EU_OriginatingVehicleContainer;


extern const ASN1CType asn1_type_EU_OriginatingVehicleContainer[];

typedef int EU_RoadRegulatorID;

extern const ASN1CType asn1_type_EU_RoadRegulatorID[];

typedef int EU_IntersectionID;

extern const ASN1CType asn1_type_EU_IntersectionID[];

typedef struct EU_IntersectionReferenceID {
  BOOL region_option;
  EU_RoadRegulatorID region;
  EU_IntersectionID id;
} EU_IntersectionReferenceID;


extern const ASN1CType asn1_type_EU_IntersectionReferenceID[];

typedef int EU_RoadSegmentID;

extern const ASN1CType asn1_type_EU_RoadSegmentID[];

typedef struct EU_RoadSegmentReferenceID {
  BOOL region_option;
  EU_RoadRegulatorID region;
  EU_RoadSegmentID id;
} EU_RoadSegmentReferenceID;


extern const ASN1CType asn1_type_EU_RoadSegmentReferenceID[];

typedef enum {
  EU_OriginatingRSUContainer_intersectionReferenceId,
  EU_OriginatingRSUContainer_roadSegmentReferenceId,
  EU_OriginatingRSUContainer__dummy = 2147483647,
} EU_OriginatingRSUContainer_choice;

typedef struct EU_OriginatingRSUContainer {
  EU_OriginatingRSUContainer_choice choice;
  union {
    EU_IntersectionReferenceID intersectionReferenceId;
    EU_RoadSegmentReferenceID roadSegmentReferenceId;
  } u;
} EU_OriginatingRSUContainer;

extern const ASN1CType asn1_type_EU_OriginatingRSUContainer[];

typedef int EU_Identifier;

extern const ASN1CType asn1_type_EU_Identifier[];

typedef int EU_SensorType;

enum {
  EU_SensorType_undefined = 0,
  EU_SensorType_radar = 1,
  EU_SensorType_lidar = 2,
  EU_SensorType_monovideo = 3,
  EU_SensorType_stereovision = 4,
  EU_SensorType_nightvision = 5,
  EU_SensorType_ultrasonic = 6,
  EU_SensorType_pmd = 7,
  EU_SensorType_fusion = 8,
  EU_SensorType_inductionloop = 9,
  EU_SensorType_sphericalCamera = 10,
  EU_SensorType_itssaggregation = 11,
};

extern const ASN1CType asn1_type_EU_SensorType[];

typedef int EU_XSensorOffset;

enum {
  EU_XSensorOffset_negativeZeroPointZeroOneMeter = -1,
  EU_XSensorOffset_negativeOneMeter = -100,
};

extern const ASN1CType asn1_type_EU_XSensorOffset[];

typedef int EU_YSensorOffset;

enum {
  EU_YSensorOffset_zeroPointZeroOneMeter = 1,
  EU_YSensorOffset_oneMeter = 100,
};

extern const ASN1CType asn1_type_EU_YSensorOffset[];

typedef int EU_ZSensorOffset;

enum {
  EU_ZSensorOffset_zeroPointZeroOneMeter = 1,
  EU_ZSensorOffset_oneMeter = 100,
};

extern const ASN1CType asn1_type_EU_ZSensorOffset[];

typedef int EU_Range;

enum {
  EU_Range_zeroPointOneMeter = 1,
  EU_Range_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_Range[];

typedef struct EU_VehicleSensorProperties {
  EU_Range range;
  EU_CartesianAngleValue horizontalOpeningAngleStart;
  EU_CartesianAngleValue horizontalOpeningAngleEnd;
  BOOL verticalOpeningAngleStart_option;
  EU_CartesianAngleValue verticalOpeningAngleStart;
  BOOL verticalOpeningAngleEnd_option;
  EU_CartesianAngleValue verticalOpeningAngleEnd;
} EU_VehicleSensorProperties;


extern const ASN1CType asn1_type_EU_VehicleSensorProperties[];

typedef struct EU_VehicleSensorPropertyList {
  EU_VehicleSensorProperties *tab;
  size_t count;
} EU_VehicleSensorPropertyList;

extern const ASN1CType asn1_type_EU_VehicleSensorPropertyList[];

typedef struct EU_VehicleSensor {
  EU_RefPointId refPointId;
  EU_XSensorOffset xSensorOffset;
  EU_YSensorOffset ySensorOffset;
  BOOL zSensorOffset_option;
  EU_ZSensorOffset zSensorOffset;
  EU_VehicleSensorPropertyList vehicleSensorPropertyList;
} EU_VehicleSensor;


extern const ASN1CType asn1_type_EU_VehicleSensor[];

typedef int EU_Offset_B10;

extern const ASN1CType asn1_type_EU_Offset_B10[];

typedef struct EU_Node_XY_20b {
  EU_Offset_B10 x;
  EU_Offset_B10 y;
} EU_Node_XY_20b;


extern const ASN1CType asn1_type_EU_Node_XY_20b[];

typedef int EU_Offset_B11;

extern const ASN1CType asn1_type_EU_Offset_B11[];

typedef struct EU_Node_XY_22b {
  EU_Offset_B11 x;
  EU_Offset_B11 y;
} EU_Node_XY_22b;


extern const ASN1CType asn1_type_EU_Node_XY_22b[];

typedef int EU_Offset_B12;

extern const ASN1CType asn1_type_EU_Offset_B12[];

typedef struct EU_Node_XY_24b {
  EU_Offset_B12 x;
  EU_Offset_B12 y;
} EU_Node_XY_24b;


extern const ASN1CType asn1_type_EU_Node_XY_24b[];

typedef int EU_Offset_B13;

extern const ASN1CType asn1_type_EU_Offset_B13[];

typedef struct EU_Node_XY_26b {
  EU_Offset_B13 x;
  EU_Offset_B13 y;
} EU_Node_XY_26b;


extern const ASN1CType asn1_type_EU_Node_XY_26b[];

typedef int EU_Offset_B14;

extern const ASN1CType asn1_type_EU_Offset_B14[];

typedef struct EU_Node_XY_28b {
  EU_Offset_B14 x;
  EU_Offset_B14 y;
} EU_Node_XY_28b;


extern const ASN1CType asn1_type_EU_Node_XY_28b[];

typedef int EU_Offset_B16;

extern const ASN1CType asn1_type_EU_Offset_B16[];

typedef struct EU_Node_XY_32b {
  EU_Offset_B16 x;
  EU_Offset_B16 y;
} EU_Node_XY_32b;


extern const ASN1CType asn1_type_EU_Node_XY_32b[];

typedef struct EU_Node_LLmD_64b {
  EU_Longitude lon;
  EU_Latitude lat;
} EU_Node_LLmD_64b;


extern const ASN1CType asn1_type_EU_Node_LLmD_64b[];

typedef int EU_RegionId;

extern const ASN1CType asn1_type_EU_RegionId[];

typedef struct EU_RegionalExtension_17 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_17;


extern const ASN1CType asn1_type_EU_RegionalExtension_17[];

typedef enum {
  EU_NodeOffsetPointXY_node_XY1,
  EU_NodeOffsetPointXY_node_XY2,
  EU_NodeOffsetPointXY_node_XY3,
  EU_NodeOffsetPointXY_node_XY4,
  EU_NodeOffsetPointXY_node_XY5,
  EU_NodeOffsetPointXY_node_XY6,
  EU_NodeOffsetPointXY_node_LatLon,
  EU_NodeOffsetPointXY_regional,
  EU_NodeOffsetPointXY__dummy = 2147483647,
} EU_NodeOffsetPointXY_choice;

typedef struct EU_NodeOffsetPointXY {
  EU_NodeOffsetPointXY_choice choice;
  union {
    EU_Node_XY_20b node_XY1;
    EU_Node_XY_22b node_XY2;
    EU_Node_XY_24b node_XY3;
    EU_Node_XY_26b node_XY4;
    EU_Node_XY_28b node_XY5;
    EU_Node_XY_32b node_XY6;
    EU_Node_LLmD_64b node_LatLon;
    EU_RegionalExtension_17 regional;
  } u;
} EU_NodeOffsetPointXY;

extern const ASN1CType asn1_type_EU_NodeOffsetPointXY[];

typedef enum {
  EU_NodeOffsetPointZ_node_Z1,
  EU_NodeOffsetPointZ_node_Z2,
  EU_NodeOffsetPointZ_node_Z3,
  EU_NodeOffsetPointZ_node_Z4,
  EU_NodeOffsetPointZ_node_Z5,
  EU_NodeOffsetPointZ_node_Z6,
  EU_NodeOffsetPointZ__dummy = 2147483647,
} EU_NodeOffsetPointZ_choice;

typedef struct EU_NodeOffsetPointZ {
  EU_NodeOffsetPointZ_choice choice;
  union {
    EU_Offset_B10 node_Z1;
    EU_Offset_B11 node_Z2;
    EU_Offset_B12 node_Z3;
    EU_Offset_B13 node_Z4;
    EU_Offset_B14 node_Z5;
    EU_Offset_B16 node_Z6;
  } u;
} EU_NodeOffsetPointZ;

extern const ASN1CType asn1_type_EU_NodeOffsetPointZ[];

typedef struct EU_OffsetPoint {
  EU_NodeOffsetPointXY nodeOffsetPointxy;
  BOOL nodeOffsetPointZ_option;
  EU_NodeOffsetPointZ nodeOffsetPointZ;
} EU_OffsetPoint;


extern const ASN1CType asn1_type_EU_OffsetPoint[];

typedef int EU_SensorHeight;

enum {
  EU_SensorHeight_zeroPointZeroOneMeter = 1,
};

extern const ASN1CType asn1_type_EU_SensorHeight[];

typedef struct EU_AreaRadial {
  EU_Range range;
  EU_WGS84AngleValue stationaryHorizontalOpeningAngleStart;
  EU_WGS84AngleValue stationaryHorizontalOpeningAngleEnd;
  BOOL verticalOpeningAngleStart_option;
  EU_CartesianAngleValue verticalOpeningAngleStart;
  BOOL verticalOpeningAngleEnd_option;
  EU_CartesianAngleValue verticalOpeningAngleEnd;
  BOOL sensorPositionOffset_option;
  EU_OffsetPoint sensorPositionOffset;
  BOOL sensorHeight_option;
  EU_SensorHeight sensorHeight;
} EU_AreaRadial;


extern const ASN1CType asn1_type_EU_AreaRadial[];

typedef struct EU_PolyPointList {
  EU_OffsetPoint *tab;
  size_t count;
} EU_PolyPointList;

extern const ASN1CType asn1_type_EU_PolyPointList[];

typedef struct EU_AreaPolygon {
  EU_PolyPointList polyPointList;
} EU_AreaPolygon;


extern const ASN1CType asn1_type_EU_AreaPolygon[];

typedef int EU_Radius;

enum {
  EU_Radius_zeroPointOneMeter = 1,
  EU_Radius_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_Radius[];

typedef struct EU_AreaCircular {
  BOOL nodeCenterPoint_option;
  EU_OffsetPoint nodeCenterPoint;
  EU_Radius radius;
} EU_AreaCircular;


extern const ASN1CType asn1_type_EU_AreaCircular[];

typedef int EU_SemiRangeLength;

enum {
  EU_SemiRangeLength_zeroPointOneMeter = 1,
  EU_SemiRangeLength_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_SemiRangeLength[];

typedef struct EU_AreaEllipse {
  BOOL nodeCenterPoint_option;
  EU_OffsetPoint nodeCenterPoint;
  EU_SemiRangeLength semiMinorRangeLength;
  EU_SemiRangeLength semiMajorRangeLength;
  EU_WGS84AngleValue semiMajorRangeOrientation;
  BOOL semiHeight_option;
  EU_SemiRangeLength semiHeight;
} EU_AreaEllipse;


extern const ASN1CType asn1_type_EU_AreaEllipse[];

typedef struct EU_AreaRectangle {
  BOOL nodeCenterPoint_option;
  EU_OffsetPoint nodeCenterPoint;
  EU_SemiRangeLength semiMajorRangeLength;
  EU_SemiRangeLength semiMinorRangeLength;
  EU_WGS84AngleValue semiMajorRangeOrientation;
  BOOL semiHeight_option;
  EU_SemiRangeLength semiHeight;
} EU_AreaRectangle;


extern const ASN1CType asn1_type_EU_AreaRectangle[];

typedef enum {
  EU_DetectionArea_vehicleSensor,
  EU_DetectionArea_stationarySensorRadial,
  EU_DetectionArea_stationarySensorPolygon,
  EU_DetectionArea_stationarySensorCircular,
  EU_DetectionArea_stationarySensorEllipse,
  EU_DetectionArea_stationarySensorRectangle,
  EU_DetectionArea__dummy = 2147483647,
} EU_DetectionArea_choice;

typedef struct EU_DetectionArea {
  EU_DetectionArea_choice choice;
  union {
    EU_VehicleSensor vehicleSensor;
    EU_AreaRadial stationarySensorRadial;
    EU_AreaPolygon stationarySensorPolygon;
    EU_AreaCircular stationarySensorCircular;
    EU_AreaEllipse stationarySensorEllipse;
    EU_AreaRectangle stationarySensorRectangle;
  } u;
} EU_DetectionArea;

extern const ASN1CType asn1_type_EU_DetectionArea[];

typedef int EU_FreeSpaceConfidence;

enum {
  EU_FreeSpaceConfidence_unknown = 0,
  EU_FreeSpaceConfidence_onePercent = 1,
  EU_FreeSpaceConfidence_oneHundredPercent = 100,
  EU_FreeSpaceConfidence_unavailable = 101,
};

extern const ASN1CType asn1_type_EU_FreeSpaceConfidence[];

typedef struct EU_SensorInformation {
  EU_Identifier sensorID;
  EU_SensorType type;
  EU_DetectionArea detectionArea;
  BOOL freeSpaceConfidence_option;
  EU_FreeSpaceConfidence freeSpaceConfidence;
} EU_SensorInformation;


extern const ASN1CType asn1_type_EU_SensorInformation[];

typedef struct EU_SensorInformationContainer {
  EU_SensorInformation *tab;
  size_t count;
} EU_SensorInformationContainer;

extern const ASN1CType asn1_type_EU_SensorInformationContainer[];

typedef int EU_NumberOfPerceivedObjects;

extern const ASN1CType asn1_type_EU_NumberOfPerceivedObjects[];

typedef int EU_TimeOfMeasurement;

enum {
  EU_TimeOfMeasurement_oneMilliSecond = 1,
};

extern const ASN1CType asn1_type_EU_TimeOfMeasurement[];

typedef int EU_ObjectConfidence;

enum {
  EU_ObjectConfidence_unknown = 0,
  EU_ObjectConfidence_onePercent = 1,
  EU_ObjectConfidence_oneHundredPercent = 100,
  EU_ObjectConfidence_unavailable = 101,
};

extern const ASN1CType asn1_type_EU_ObjectConfidence[];

typedef int EU_DistanceValue;

enum {
  EU_DistanceValue_zeroPointZeroOneMeter = 1,
  EU_DistanceValue_oneMeter = 100,
};

extern const ASN1CType asn1_type_EU_DistanceValue[];

typedef int EU_DistanceConfidence;

enum {
  EU_DistanceConfidence_zeroPointZeroOneMeter = 1,
  EU_DistanceConfidence_oneMeter = 100,
  EU_DistanceConfidence_outOfRange = 101,
  EU_DistanceConfidence_unavailable = 102,
};

extern const ASN1CType asn1_type_EU_DistanceConfidence[];

typedef struct EU_ObjectDistance {
  EU_DistanceValue value;
  EU_DistanceConfidence confidence;
} EU_ObjectDistance;


extern const ASN1CType asn1_type_EU_ObjectDistance[];

typedef int EU_SpeedValueExtended;

enum {
  EU_SpeedValueExtended_standstill = 0,
  EU_SpeedValueExtended_oneCentimeterPerSec = 1,
  EU_SpeedValueExtended_speedMaximum = 16382,
  EU_SpeedValueExtended_unavailable = 16383,
};

extern const ASN1CType asn1_type_EU_SpeedValueExtended[];

typedef struct EU_SpeedExtended {
  EU_SpeedValueExtended value;
  EU_SpeedConfidence confidence;
} EU_SpeedExtended;


extern const ASN1CType asn1_type_EU_SpeedExtended[];

typedef int EU_ObjectDimensionValue;

enum {
  EU_ObjectDimensionValue_zeroPointOneMeter = 1,
  EU_ObjectDimensionValue_oneMeter = 10,
};

extern const ASN1CType asn1_type_EU_ObjectDimensionValue[];

typedef int EU_ObjectDimensionConfidence;

enum {
  EU_ObjectDimensionConfidence_zeroPointZeroOneMeter = 1,
  EU_ObjectDimensionConfidence_oneMeter = 100,
  EU_ObjectDimensionConfidence_outOfRange = 101,
  EU_ObjectDimensionConfidence_unavailable = 102,
};

extern const ASN1CType asn1_type_EU_ObjectDimensionConfidence[];

typedef struct EU_ObjectDimension {
  EU_ObjectDimensionValue value;
  EU_ObjectDimensionConfidence confidence;
} EU_ObjectDimension;


extern const ASN1CType asn1_type_EU_ObjectDimension[];

typedef int EU_ObjectRefPoint;

enum {
  EU_ObjectRefPoint_mid = 0,
  EU_ObjectRefPoint_bottomLeft = 1,
  EU_ObjectRefPoint_midLeft = 2,
  EU_ObjectRefPoint_topLeft = 3,
  EU_ObjectRefPoint_bottomMid = 4,
  EU_ObjectRefPoint_topMid = 5,
  EU_ObjectRefPoint_bottomRight = 6,
  EU_ObjectRefPoint_midRight = 7,
  EU_ObjectRefPoint_topRight = 8,
};

extern const ASN1CType asn1_type_EU_ObjectRefPoint[];

typedef int EU_ObjectAge;

enum {
  EU_ObjectAge_oneMiliSec = 1,
  EU_ObjectAge_moreThan1Point5Second = 1500,
};

extern const ASN1CType asn1_type_EU_ObjectAge[];

typedef struct EU_SensorIdList {
  EU_Identifier *tab;
  size_t count;
} EU_SensorIdList;

extern const ASN1CType asn1_type_EU_SensorIdList[];

typedef int EU_DynamicStatus;

enum {
  EU_DynamicStatus_dynamic = 0,
  EU_DynamicStatus_hasBeenDynamic = 1,
  EU_DynamicStatus_Static = 2,
};

extern const ASN1CType asn1_type_EU_DynamicStatus[];

typedef int EU_UnknownSubclassType;

enum {
  EU_UnknownSubclassType_unknown = 0,
};

extern const ASN1CType asn1_type_EU_UnknownSubclassType[];

typedef int EU_VehicleSubclassType;

enum {
  EU_VehicleSubclassType_unknown = 0,
  EU_VehicleSubclassType_moped = 1,
  EU_VehicleSubclassType_motorcycle = 2,
  EU_VehicleSubclassType_passengerCar = 3,
  EU_VehicleSubclassType_bus = 4,
  EU_VehicleSubclassType_lightTruck = 5,
  EU_VehicleSubclassType_heavyTruck = 6,
  EU_VehicleSubclassType_trailer = 7,
  EU_VehicleSubclassType_specialVehicles = 8,
  EU_VehicleSubclassType_tram = 9,
  EU_VehicleSubclassType_emergencyVehicle = 10,
  EU_VehicleSubclassType_agricultural = 11,
};

extern const ASN1CType asn1_type_EU_VehicleSubclassType[];

typedef int EU_PersonSubclassType;

enum {
  EU_PersonSubclassType_unknown = 0,
  EU_PersonSubclassType_pedestrian = 1,
  EU_PersonSubclassType_personInWheelchair = 2,
  EU_PersonSubclassType_cyclist = 3,
  EU_PersonSubclassType_personWithStroller = 4,
  EU_PersonSubclassType_personOnSkates = 5,
  EU_PersonSubclassType_personGroup = 6,
};

extern const ASN1CType asn1_type_EU_PersonSubclassType[];

typedef int EU_AnimalSubclassType;

enum {
  EU_AnimalSubclassType_unknown = 0,
};

extern const ASN1CType asn1_type_EU_AnimalSubclassType[];

typedef int EU_OtherSubclassType;

enum {
  EU_OtherSubclassType_unknown = 0,
  EU_OtherSubclassType_roadSideUnit = 1,
};

extern const ASN1CType asn1_type_EU_OtherSubclassType[];

typedef enum {
  EU_ObjectClass_unknownSubclass,
  EU_ObjectClass_vehicleSubclass,
  EU_ObjectClass_personSubclass,
  EU_ObjectClass_animalSubclass,
  EU_ObjectClass_otherSubclass,
  EU_ObjectClass__dummy = 2147483647,
} EU_ObjectClass_choice;

typedef struct EU_ObjectClass {
  EU_ObjectClass_choice choice;
  union {
    EU_UnknownSubclassType unknownSubclass;
    EU_VehicleSubclassType vehicleSubclass;
    EU_PersonSubclassType personSubclass;
    EU_AnimalSubclassType animalSubclass;
    EU_OtherSubclassType otherSubclass;
  } u;
} EU_ObjectClass;

extern const ASN1CType asn1_type_EU_ObjectClass[];

typedef int EU_ClassConfidence;

enum {
  EU_ClassConfidence_unknown = 0,
  EU_ClassConfidence_onePercent = 1,
  EU_ClassConfidence_oneHundredPercent = 100,
  EU_ClassConfidence_unavailable = 101,
};

extern const ASN1CType asn1_type_EU_ClassConfidence[];

typedef struct EU_ObjectClassWithConfidence {
  EU_ObjectClass objectClass;
  EU_ClassConfidence confidence;
} EU_ObjectClassWithConfidence;


extern const ASN1CType asn1_type_EU_ObjectClassWithConfidence[];

typedef struct EU_ObjectClassDescription {
  EU_ObjectClassWithConfidence *tab;
  size_t count;
} EU_ObjectClassDescription;

extern const ASN1CType asn1_type_EU_ObjectClassDescription[];

typedef int EU_LaneID;

extern const ASN1CType asn1_type_EU_LaneID[];

typedef int EU_LongitudinalLanePositionValue;

enum {
  EU_LongitudinalLanePositionValue_zeroPointOneMeter = 1,
};

extern const ASN1CType asn1_type_EU_LongitudinalLanePositionValue[];

typedef int EU_LongitudinalLanePositionConfidence;

enum {
  EU_LongitudinalLanePositionConfidence_zeroPointZeroOneMeter = 1,
  EU_LongitudinalLanePositionConfidence_oneMeter = 100,
  EU_LongitudinalLanePositionConfidence_outOfRange = 101,
  EU_LongitudinalLanePositionConfidence_unavailable = 102,
};

extern const ASN1CType asn1_type_EU_LongitudinalLanePositionConfidence[];

typedef struct EU_LongitudinalLanePosition {
  EU_LongitudinalLanePositionValue longitudinalLanePositionValue;
  EU_LongitudinalLanePositionConfidence longitudinalLanePositionConfidence;
} EU_LongitudinalLanePosition;


extern const ASN1CType asn1_type_EU_LongitudinalLanePosition[];

typedef struct EU_MatchedPosition {
  BOOL laneID_option;
  EU_LaneID laneID;
  BOOL longitudinalLanePosition_option;
  EU_LongitudinalLanePosition longitudinalLanePosition;
} EU_MatchedPosition;


extern const ASN1CType asn1_type_EU_MatchedPosition[];

typedef struct EU_PerceivedObject {
  EU_Identifier objectID;
  EU_TimeOfMeasurement timeOfMeasurement;
  EU_ObjectConfidence objectConfidence;
  EU_ObjectDistance xDistance;
  EU_ObjectDistance yDistance;
  BOOL zDistance_option;
  EU_ObjectDistance zDistance;
  EU_SpeedExtended xSpeed;
  EU_SpeedExtended ySpeed;
  BOOL zSpeed_option;
  EU_SpeedExtended zSpeed;
  BOOL xAcceleration_option;
  EU_LongitudinalAcceleration xAcceleration;
  BOOL yAcceleration_option;
  EU_LateralAcceleration yAcceleration;
  BOOL zAcceleration_option;
  EU_VerticalAcceleration zAcceleration;
  BOOL yawAngle_option;
  EU_CartesianAngle yawAngle;
  BOOL planarObjectDimension1_option;
  EU_ObjectDimension planarObjectDimension1;
  BOOL planarObjectDimension2_option;
  EU_ObjectDimension planarObjectDimension2;
  BOOL verticalObjectDimension_option;
  EU_ObjectDimension verticalObjectDimension;
  EU_ObjectRefPoint objectRefPoint;
  BOOL objectAge_option;
  EU_ObjectAge objectAge;
  BOOL sensorIDList_option;
  EU_SensorIdList sensorIDList;
  BOOL dynamicStatus_option;
  EU_DynamicStatus dynamicStatus;
  BOOL classification_option;
  EU_ObjectClassDescription classification;
  BOOL matchedPosition_option;
  EU_MatchedPosition matchedPosition;
} EU_PerceivedObject;


extern const ASN1CType asn1_type_EU_PerceivedObject[];

typedef struct EU_PerceivedObjectContainer_1 {
  EU_PerceivedObject *tab;
  size_t count;
} EU_PerceivedObjectContainer_1;

extern const ASN1CType asn1_type_EU_PerceivedObjectContainer_1[];

typedef struct EU_PerceivedObjectContainer {
  EU_NumberOfPerceivedObjects numberOfPerceivedObjects;
  EU_PerceivedObjectContainer_1 perceivedObjects;
} EU_PerceivedObjectContainer;


extern const ASN1CType asn1_type_EU_PerceivedObjectContainer[];

typedef enum {
  EU_FreeSpaceArea_freeSpacePolygon,
  EU_FreeSpaceArea_freeSpaceCircular,
  EU_FreeSpaceArea_freeSpaceEllipse,
  EU_FreeSpaceArea_freeSpaceRectangle,
  EU_FreeSpaceArea__dummy = 2147483647,
} EU_FreeSpaceArea_choice;

typedef struct EU_FreeSpaceArea {
  EU_FreeSpaceArea_choice choice;
  union {
    EU_AreaPolygon freeSpacePolygon;
    EU_AreaCircular freeSpaceCircular;
    EU_AreaEllipse freeSpaceEllipse;
    EU_AreaRectangle freeSpaceRectangle;
  } u;
} EU_FreeSpaceArea;

extern const ASN1CType asn1_type_EU_FreeSpaceArea[];

typedef BOOL EU_ShadowingApplies;

extern const ASN1CType asn1_type_EU_ShadowingApplies[];

typedef struct EU_FreeSpaceAddendum {
  EU_FreeSpaceConfidence freeSpaceConfidence;
  EU_FreeSpaceArea freeSpaceArea;
  BOOL sensorIDList_option;
  EU_SensorIdList sensorIDList;
  EU_ShadowingApplies shadowingApplies;
} EU_FreeSpaceAddendum;


extern const ASN1CType asn1_type_EU_FreeSpaceAddendum[];

typedef struct EU_FreeSpaceAddendumContainer {
  EU_FreeSpaceAddendum *tab;
  size_t count;
} EU_FreeSpaceAddendumContainer;

extern const ASN1CType asn1_type_EU_FreeSpaceAddendumContainer[];

#define EU_originatingStationVehicleContainer 1

#define EU_originatingStationRSUContainer 2

#define EU_sensorInformationCpmContainer 1

#define EU_perceivedObjectCpmContainer 2

#define EU_freeSpaceAdddendumCpmContainer 3

typedef int EU_SegmentCount;

extern const ASN1CType asn1_type_EU_SegmentCount[];

typedef struct EU_MessageSegmentInfo {
  EU_SegmentCount totalMsgSegments;
  EU_SegmentCount thisSegmentNum;
} EU_MessageSegmentInfo;


extern const ASN1CType asn1_type_EU_MessageSegmentInfo[];

typedef struct EU_CpmManagementContainer {
  EU_StationType stationType;
  BOOL messageSegmentInfo_option;
  EU_MessageSegmentInfo messageSegmentInfo;
  EU_ReferencePosition referencePosition;
} EU_CpmManagementContainer;


extern const ASN1CType asn1_type_EU_CpmManagementContainer[];

typedef struct EU_CpmStationDataContainer_1 {
  EU_CpmStationDataId containerId;
  ASN1OpenType containerData;
} EU_CpmStationDataContainer_1;


extern const ASN1CType asn1_type_EU_CpmStationDataContainer_1[];

typedef struct EU_CpmPerceptionDataContainer_1 {
  EU_CpmContainerId containerId;
  ASN1OpenType containerData;
} EU_CpmPerceptionDataContainer_1;


extern const ASN1CType asn1_type_EU_CpmPerceptionDataContainer_1[];

typedef struct EU_CpmParameters_1 {
  EU_CpmPerceptionDataContainer_1 *tab;
  size_t count;
} EU_CpmParameters_1;

extern const ASN1CType asn1_type_EU_CpmParameters_1[];

typedef struct EU_CpmParameters {
  EU_CpmManagementContainer managementContainer;
  BOOL stationDataContainer_option;
  EU_CpmStationDataContainer_1 stationDataContainer;
  BOOL perceptionData_option;
  EU_CpmParameters_1 perceptionData;
} EU_CpmParameters;


extern const ASN1CType asn1_type_EU_CpmParameters[];

typedef struct EU_CollectivePerceptionMessage {
  EU_GenerationDeltaTime generationDeltaTime;
  EU_CpmParameters cpmParameters;
} EU_CollectivePerceptionMessage;


extern const ASN1CType asn1_type_EU_CollectivePerceptionMessage[];

typedef struct EU_CPM {
  EU_ItsPduHeader header;
  EU_CollectivePerceptionMessage cpm;
} EU_CPM;


extern const ASN1CType asn1_type_EU_CPM[];

typedef int EU_SequenceNumber;

extern const ASN1CType asn1_type_EU_SequenceNumber[];

typedef struct EU_ActionID {
  EU_StationID originatingStationID;
  EU_SequenceNumber sequenceNumber;
} EU_ActionID;


extern const ASN1CType asn1_type_EU_ActionID[];

typedef enum EU_Termination {
  EU_Termination_isCancellation,
  EU_Termination_isNegation,
  EU_Termination__dummy = 2147483647,
} EU_Termination;

extern const ASN1CType asn1_type_EU_Termination[];

typedef enum EU_RelevanceDistance {
  EU_RelevanceDistance_lessThan50m,
  EU_RelevanceDistance_lessThan100m,
  EU_RelevanceDistance_lessThan200m,
  EU_RelevanceDistance_lessThan500m,
  EU_RelevanceDistance_lessThan1000m,
  EU_RelevanceDistance_lessThan5km,
  EU_RelevanceDistance_lessThan10km,
  EU_RelevanceDistance_over10km,
  EU_RelevanceDistance__dummy = 2147483647,
} EU_RelevanceDistance;

extern const ASN1CType asn1_type_EU_RelevanceDistance[];

typedef enum EU_RelevanceTrafficDirection {
  EU_RelevanceTrafficDirection_allTrafficDirections,
  EU_RelevanceTrafficDirection_upstreamTraffic,
  EU_RelevanceTrafficDirection_downstreamTraffic,
  EU_RelevanceTrafficDirection_oppositeTraffic,
  EU_RelevanceTrafficDirection__dummy = 2147483647,
} EU_RelevanceTrafficDirection;

extern const ASN1CType asn1_type_EU_RelevanceTrafficDirection[];

typedef int EU_ValidityDuration;

enum {
  EU_ValidityDuration_timeOfDetection = 0,
  EU_ValidityDuration_oneSecondAfterDetection = 1,
};

extern const ASN1CType asn1_type_EU_ValidityDuration[];

typedef int EU_TransmissionInterval;

enum {
  EU_TransmissionInterval_oneMilliSecond = 1,
  EU_TransmissionInterval_tenSeconds = 10000,
};

extern const ASN1CType asn1_type_EU_TransmissionInterval[];

typedef struct EU_ManagementContainer {
  EU_ActionID actionID;
  EU_TimestampIts detectionTime;
  EU_TimestampIts referenceTime;
  BOOL termination_option;
  EU_Termination termination;
  EU_ReferencePosition eventPosition;
  BOOL relevanceDistance_option;
  EU_RelevanceDistance relevanceDistance;
  BOOL relevanceTrafficDirection_option;
  EU_RelevanceTrafficDirection relevanceTrafficDirection;
  EU_ValidityDuration validityDuration;
  BOOL transmissionInterval_option;
  EU_TransmissionInterval transmissionInterval;
  EU_StationType stationType;
} EU_ManagementContainer;


extern const ASN1CType asn1_type_EU_ManagementContainer[];

typedef int EU_InformationQuality;

enum {
  EU_InformationQuality_unavailable = 0,
  EU_InformationQuality_lowest = 1,
  EU_InformationQuality_highest = 7,
};

extern const ASN1CType asn1_type_EU_InformationQuality[];

typedef struct EU_EventPoint {
  EU_DeltaReferencePosition eventPosition;
  BOOL eventDeltaTime_option;
  EU_PathDeltaTime eventDeltaTime;
  EU_InformationQuality informationQuality;
} EU_EventPoint;


extern const ASN1CType asn1_type_EU_EventPoint[];

typedef struct EU_EventHistory {
  EU_EventPoint *tab;
  size_t count;
} EU_EventHistory;

extern const ASN1CType asn1_type_EU_EventHistory[];

typedef struct EU_SituationContainer {
  EU_InformationQuality informationQuality;
  EU_CauseCode eventType;
  BOOL linkedCause_option;
  EU_CauseCode linkedCause;
  BOOL eventHistory_option;
  EU_EventHistory eventHistory;
} EU_SituationContainer;


extern const ASN1CType asn1_type_EU_SituationContainer[];

typedef struct EU_Traces {
  EU_PathHistory *tab;
  size_t count;
} EU_Traces;

extern const ASN1CType asn1_type_EU_Traces[];

typedef enum EU_RoadType {
  EU_RoadType_urban_NoStructuralSeparationToOppositeLanes,
  EU_RoadType_urban_WithStructuralSeparationToOppositeLanes,
  EU_RoadType_nonUrban_NoStructuralSeparationToOppositeLanes,
  EU_RoadType_nonUrban_WithStructuralSeparationToOppositeLanes,
  EU_RoadType__dummy = 2147483647,
} EU_RoadType;

extern const ASN1CType asn1_type_EU_RoadType[];

typedef struct EU_LocationContainer {
  BOOL eventSpeed_option;
  EU_Speed eventSpeed;
  BOOL eventPositionHeading_option;
  EU_Heading eventPositionHeading;
  EU_Traces traces;
  BOOL roadType_option;
  EU_RoadType roadType;
} EU_LocationContainer;


extern const ASN1CType asn1_type_EU_LocationContainer[];

typedef int EU_HeightLonCarr;

enum {
  EU_HeightLonCarr_oneCentimeter = 1,
  EU_HeightLonCarr_unavailable = 100,
};

extern const ASN1CType asn1_type_EU_HeightLonCarr[];

typedef int EU_PosLonCarr;

enum {
  EU_PosLonCarr_oneCentimeter = 1,
  EU_PosLonCarr_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_PosLonCarr[];

typedef int EU_PosPillar;

enum {
  EU_PosPillar_tenCentimeters = 1,
  EU_PosPillar_unavailable = 30,
};

extern const ASN1CType asn1_type_EU_PosPillar[];

typedef struct EU_PositionOfPillars {
  EU_PosPillar *tab;
  size_t count;
} EU_PositionOfPillars;

extern const ASN1CType asn1_type_EU_PositionOfPillars[];

typedef int EU_PosCentMass;

enum {
  EU_PosCentMass_tenCentimeters = 1,
  EU_PosCentMass_unavailable = 63,
};

extern const ASN1CType asn1_type_EU_PosCentMass[];

typedef int EU_WheelBaseVehicle;

enum {
  EU_WheelBaseVehicle_tenCentimeters = 1,
  EU_WheelBaseVehicle_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_WheelBaseVehicle[];

typedef int EU_TurningRadius;

enum {
  EU_TurningRadius_point4Meters = 1,
  EU_TurningRadius_unavailable = 255,
};

extern const ASN1CType asn1_type_EU_TurningRadius[];

typedef int EU_PosFrontAx;

enum {
  EU_PosFrontAx_tenCentimeters = 1,
  EU_PosFrontAx_unavailable = 20,
};

extern const ASN1CType asn1_type_EU_PosFrontAx[];

typedef ASN1BitString EU_PositionOfOccupants;

extern const ASN1CType asn1_type_EU_PositionOfOccupants[];

typedef int EU_VehicleMass;

enum {
  EU_VehicleMass_hundredKg = 1,
  EU_VehicleMass_unavailable = 1024,
};

extern const ASN1CType asn1_type_EU_VehicleMass[];

typedef enum EU_RequestResponseIndication {
  EU_RequestResponseIndication_request,
  EU_RequestResponseIndication_response,
  EU_RequestResponseIndication__dummy = 2147483647,
} EU_RequestResponseIndication;

extern const ASN1CType asn1_type_EU_RequestResponseIndication[];

typedef struct EU_ImpactReductionContainer {
  EU_HeightLonCarr heightLonCarrLeft;
  EU_HeightLonCarr heightLonCarrRight;
  EU_PosLonCarr posLonCarrLeft;
  EU_PosLonCarr posLonCarrRight;
  EU_PositionOfPillars positionOfPillars;
  EU_PosCentMass posCentMass;
  EU_WheelBaseVehicle wheelBaseVehicle;
  EU_TurningRadius turningRadius;
  EU_PosFrontAx posFrontAx;
  EU_PositionOfOccupants positionOfOccupants;
  EU_VehicleMass vehicleMass;
  EU_RequestResponseIndication requestResponseIndication;
} EU_ImpactReductionContainer;


extern const ASN1CType asn1_type_EU_ImpactReductionContainer[];

typedef int EU_Temperature;

enum {
  EU_Temperature_equalOrSmallerThanMinus60Deg = -60,
  EU_Temperature_oneDegreeCelsius = 1,
  EU_Temperature_equalOrGreaterThan67Deg = 67,
};

extern const ASN1CType asn1_type_EU_Temperature[];

typedef struct EU_RestrictedTypes {
  EU_StationType *tab;
  size_t count;
} EU_RestrictedTypes;

extern const ASN1CType asn1_type_EU_RestrictedTypes[];

typedef struct EU_ItineraryPath {
  EU_ReferencePosition *tab;
  size_t count;
} EU_ItineraryPath;

extern const ASN1CType asn1_type_EU_ItineraryPath[];

typedef struct EU_ReferenceDenms {
  EU_ActionID *tab;
  size_t count;
} EU_ReferenceDenms;

extern const ASN1CType asn1_type_EU_ReferenceDenms[];

typedef struct EU_RoadWorksContainerExtended {
  BOOL lightBarSirenInUse_option;
  EU_LightBarSirenInUse lightBarSirenInUse;
  BOOL closedLanes_option;
  EU_ClosedLanes closedLanes;
  BOOL restriction_option;
  EU_RestrictedTypes restriction;
  BOOL speedLimit_option;
  EU_SpeedLimit speedLimit;
  BOOL incidentIndication_option;
  EU_CauseCode incidentIndication;
  BOOL recommendedPath_option;
  EU_ItineraryPath recommendedPath;
  BOOL startingPointSpeedLimit_option;
  EU_DeltaReferencePosition startingPointSpeedLimit;
  BOOL trafficFlowRule_option;
  EU_TrafficRule trafficFlowRule;
  BOOL referenceDenms_option;
  EU_ReferenceDenms referenceDenms;
} EU_RoadWorksContainerExtended;


extern const ASN1CType asn1_type_EU_RoadWorksContainerExtended[];

typedef enum EU_PositioningSolutionType {
  EU_PositioningSolutionType_noPositioningSolution,
  EU_PositioningSolutionType_sGNSS,
  EU_PositioningSolutionType_dGNSS,
  EU_PositioningSolutionType_sGNSSplusDR,
  EU_PositioningSolutionType_dGNSSplusDR,
  EU_PositioningSolutionType_dR,
  EU_PositioningSolutionType__dummy = 2147483647,
} EU_PositioningSolutionType;

extern const ASN1CType asn1_type_EU_PositioningSolutionType[];

typedef enum EU_StationarySince {
  EU_StationarySince_lessThan1Minute,
  EU_StationarySince_lessThan2Minutes,
  EU_StationarySince_lessThan15Minutes,
  EU_StationarySince_equalOrGreater15Minutes,
  EU_StationarySince__dummy = 2147483647,
} EU_StationarySince;

extern const ASN1CType asn1_type_EU_StationarySince[];

typedef ASN1String EU_PhoneNumber;

extern const ASN1CType asn1_type_EU_PhoneNumber[];

typedef struct EU_DangerousGoodsExtended {
  EU_DangerousGoodsBasic dangerousGoodsType;
  int unNumber;
  BOOL elevatedTemperature;
  BOOL tunnelsRestricted;
  BOOL limitedQuantity;
  BOOL emergencyActionCode_option;
  ASN1String emergencyActionCode;
  BOOL phoneNumber_option;
  EU_PhoneNumber phoneNumber;
  BOOL companyName_option;
  ASN1String companyName;
} EU_DangerousGoodsExtended;


extern const ASN1CType asn1_type_EU_DangerousGoodsExtended[];

typedef int EU_NumberOfOccupants;

enum {
  EU_NumberOfOccupants_oneOccupant = 1,
  EU_NumberOfOccupants_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_NumberOfOccupants[];

typedef ASN1String EU_WMInumber;

extern const ASN1CType asn1_type_EU_WMInumber[];

typedef ASN1String EU_VDS;

extern const ASN1CType asn1_type_EU_VDS[];

typedef struct EU_VehicleIdentification {
  BOOL wMInumber_option;
  EU_WMInumber wMInumber;
  BOOL vDS_option;
  EU_VDS vDS;
} EU_VehicleIdentification;


extern const ASN1CType asn1_type_EU_VehicleIdentification[];

typedef ASN1BitString EU_EnergyStorageType;

extern const ASN1CType asn1_type_EU_EnergyStorageType[];

typedef struct EU_StationaryVehicleContainer {
  BOOL stationarySince_option;
  EU_StationarySince stationarySince;
  BOOL stationaryCause_option;
  EU_CauseCode stationaryCause;
  BOOL carryingDangerousGoods_option;
  EU_DangerousGoodsExtended carryingDangerousGoods;
  BOOL numberOfOccupants_option;
  EU_NumberOfOccupants numberOfOccupants;
  BOOL vehicleIdentification_option;
  EU_VehicleIdentification vehicleIdentification;
  BOOL energyStorageType_option;
  EU_EnergyStorageType energyStorageType;
} EU_StationaryVehicleContainer;


extern const ASN1CType asn1_type_EU_StationaryVehicleContainer[];

typedef struct EU_AlacarteContainer {
  BOOL lanePosition_option;
  EU_LanePosition lanePosition;
  BOOL impactReduction_option;
  EU_ImpactReductionContainer impactReduction;
  BOOL externalTemperature_option;
  EU_Temperature externalTemperature;
  BOOL roadWorks_option;
  EU_RoadWorksContainerExtended roadWorks;
  BOOL positioningSolution_option;
  EU_PositioningSolutionType positioningSolution;
  BOOL stationaryVehicle_option;
  EU_StationaryVehicleContainer stationaryVehicle;
} EU_AlacarteContainer;


extern const ASN1CType asn1_type_EU_AlacarteContainer[];

typedef struct EU_DecentralizedEnvironmentalNotificationMessage {
  EU_ManagementContainer management;
  BOOL situation_option;
  EU_SituationContainer situation;
  BOOL location_option;
  EU_LocationContainer location;
  BOOL alacarte_option;
  EU_AlacarteContainer alacarte;
} EU_DecentralizedEnvironmentalNotificationMessage;


extern const ASN1CType asn1_type_EU_DecentralizedEnvironmentalNotificationMessage[];

typedef struct EU_DENM {
  EU_ItsPduHeader header;
  EU_DecentralizedEnvironmentalNotificationMessage denm;
} EU_DENM;


extern const ASN1CType asn1_type_EU_DENM[];

#define EU_defaultValidity 600

typedef int EU_MinuteOfTheYear;

extern const ASN1CType asn1_type_EU_MinuteOfTheYear[];

typedef ASN1String EU_DescriptiveName;

extern const ASN1CType asn1_type_EU_DescriptiveName[];

typedef int EU_MsgCount;

extern const ASN1CType asn1_type_EU_MsgCount[];

typedef ASN1BitString EU_IntersectionStatusObject;

extern const ASN1CType asn1_type_EU_IntersectionStatusObject[];

typedef int EU_DSecond;

extern const ASN1CType asn1_type_EU_DSecond[];

typedef struct EU_EnabledLaneList {
  EU_LaneID *tab;
  size_t count;
} EU_EnabledLaneList;

extern const ASN1CType asn1_type_EU_EnabledLaneList[];

typedef int EU_SignalGroupID;

extern const ASN1CType asn1_type_EU_SignalGroupID[];

typedef enum EU_MovementPhaseState {
  EU_MovementPhaseState_unavailable,
  EU_MovementPhaseState_dark,
  EU_MovementPhaseState_stop_Then_Proceed,
  EU_MovementPhaseState_stop_And_Remain,
  EU_MovementPhaseState_pre_Movement,
  EU_MovementPhaseState_permissive_Movement_Allowed,
  EU_MovementPhaseState_protected_Movement_Allowed,
  EU_MovementPhaseState_permissive_clearance,
  EU_MovementPhaseState_protected_clearance,
  EU_MovementPhaseState_caution_Conflicting_Traffic,
  EU_MovementPhaseState__dummy = 2147483647,
} EU_MovementPhaseState;

extern const ASN1CType asn1_type_EU_MovementPhaseState[];

typedef int EU_TimeMark;

extern const ASN1CType asn1_type_EU_TimeMark[];

typedef int EU_TimeIntervalConfidence;

extern const ASN1CType asn1_type_EU_TimeIntervalConfidence[];

typedef struct EU_TimeChangeDetails {
  BOOL startTime_option;
  EU_TimeMark startTime;
  EU_TimeMark minEndTime;
  BOOL maxEndTime_option;
  EU_TimeMark maxEndTime;
  BOOL likelyTime_option;
  EU_TimeMark likelyTime;
  BOOL confidence_option;
  EU_TimeIntervalConfidence confidence;
  BOOL nextTime_option;
  EU_TimeMark nextTime;
} EU_TimeChangeDetails;


extern const ASN1CType asn1_type_EU_TimeChangeDetails[];

typedef enum EU_AdvisorySpeedType {
  EU_AdvisorySpeedType_none,
  EU_AdvisorySpeedType_greenwave,
  EU_AdvisorySpeedType_ecoDrive,
  EU_AdvisorySpeedType_transit,
  EU_AdvisorySpeedType__dummy = 2147483647,
} EU_AdvisorySpeedType;

extern const ASN1CType asn1_type_EU_AdvisorySpeedType[];

typedef int EU_SpeedAdvice;

extern const ASN1CType asn1_type_EU_SpeedAdvice[];

typedef int EU_ZoneLength;

extern const ASN1CType asn1_type_EU_ZoneLength[];

typedef int EU_RestrictionClassID;

extern const ASN1CType asn1_type_EU_RestrictionClassID[];

typedef struct EU_RegionalExtension_6 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_6;


extern const ASN1CType asn1_type_EU_RegionalExtension_6[];

typedef struct EU_AdvisorySpeed_1 {
  EU_RegionalExtension_6 *tab;
  size_t count;
} EU_AdvisorySpeed_1;

extern const ASN1CType asn1_type_EU_AdvisorySpeed_1[];

typedef struct EU_AdvisorySpeed {
  EU_AdvisorySpeedType type;
  BOOL speed_option;
  EU_SpeedAdvice speed;
  BOOL confidence_option;
  EU_SpeedConfidence confidence;
  BOOL distance_option;
  EU_ZoneLength distance;
  BOOL Class_option;
  EU_RestrictionClassID Class;
  BOOL regional_option;
  EU_AdvisorySpeed_1 regional;
} EU_AdvisorySpeed;


extern const ASN1CType asn1_type_EU_AdvisorySpeed[];

typedef struct EU_AdvisorySpeedList {
  EU_AdvisorySpeed *tab;
  size_t count;
} EU_AdvisorySpeedList;

extern const ASN1CType asn1_type_EU_AdvisorySpeedList[];

typedef struct EU_RegionalExtension_14 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_14;


extern const ASN1CType asn1_type_EU_RegionalExtension_14[];

typedef struct EU_MovementEvent_1 {
  EU_RegionalExtension_14 *tab;
  size_t count;
} EU_MovementEvent_1;

extern const ASN1CType asn1_type_EU_MovementEvent_1[];

typedef struct EU_MovementEvent {
  EU_MovementPhaseState eventState;
  BOOL timing_option;
  EU_TimeChangeDetails timing;
  BOOL speeds_option;
  EU_AdvisorySpeedList speeds;
  BOOL regional_option;
  EU_MovementEvent_1 regional;
} EU_MovementEvent;


extern const ASN1CType asn1_type_EU_MovementEvent[];

typedef struct EU_MovementEventList {
  EU_MovementEvent *tab;
  size_t count;
} EU_MovementEventList;

extern const ASN1CType asn1_type_EU_MovementEventList[];

typedef int EU_LaneConnectionID;

extern const ASN1CType asn1_type_EU_LaneConnectionID[];

typedef BOOL EU_WaitOnStopline;

extern const ASN1CType asn1_type_EU_WaitOnStopline[];

typedef BOOL EU_PedestrianBicycleDetect;

extern const ASN1CType asn1_type_EU_PedestrianBicycleDetect[];

typedef struct EU_RegionalExtension_8 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_8;


extern const ASN1CType asn1_type_EU_RegionalExtension_8[];

typedef struct EU_ConnectionManeuverAssist_1 {
  EU_RegionalExtension_8 *tab;
  size_t count;
} EU_ConnectionManeuverAssist_1;

extern const ASN1CType asn1_type_EU_ConnectionManeuverAssist_1[];

typedef struct EU_ConnectionManeuverAssist {
  EU_LaneConnectionID connectionID;
  BOOL queueLength_option;
  EU_ZoneLength queueLength;
  BOOL availableStorageLength_option;
  EU_ZoneLength availableStorageLength;
  BOOL waitOnStop_option;
  EU_WaitOnStopline waitOnStop;
  BOOL pedBicycleDetect_option;
  EU_PedestrianBicycleDetect pedBicycleDetect;
  BOOL regional_option;
  EU_ConnectionManeuverAssist_1 regional;
} EU_ConnectionManeuverAssist;


extern const ASN1CType asn1_type_EU_ConnectionManeuverAssist[];

typedef struct EU_ManeuverAssistList {
  EU_ConnectionManeuverAssist *tab;
  size_t count;
} EU_ManeuverAssistList;

extern const ASN1CType asn1_type_EU_ManeuverAssistList[];

typedef struct EU_RegionalExtension_15 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_15;


extern const ASN1CType asn1_type_EU_RegionalExtension_15[];

typedef struct EU_MovementState_1 {
  EU_RegionalExtension_15 *tab;
  size_t count;
} EU_MovementState_1;

extern const ASN1CType asn1_type_EU_MovementState_1[];

typedef struct EU_MovementState {
  BOOL movementName_option;
  EU_DescriptiveName movementName;
  EU_SignalGroupID signalGroup;
  EU_MovementEventList state_time_speed;
  BOOL maneuverAssistList_option;
  EU_ManeuverAssistList maneuverAssistList;
  BOOL regional_option;
  EU_MovementState_1 regional;
} EU_MovementState;


extern const ASN1CType asn1_type_EU_MovementState[];

typedef struct EU_MovementList {
  EU_MovementState *tab;
  size_t count;
} EU_MovementList;

extern const ASN1CType asn1_type_EU_MovementList[];

typedef struct EU_RegionalExtension_11 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_11;


extern const ASN1CType asn1_type_EU_RegionalExtension_11[];

typedef struct EU_IntersectionState_1 {
  EU_RegionalExtension_11 *tab;
  size_t count;
} EU_IntersectionState_1;

extern const ASN1CType asn1_type_EU_IntersectionState_1[];

typedef struct EU_IntersectionState {
  BOOL name_option;
  EU_DescriptiveName name;
  EU_IntersectionReferenceID id;
  EU_MsgCount revision;
  EU_IntersectionStatusObject status;
  BOOL moy_option;
  EU_MinuteOfTheYear moy;
  BOOL timeStamp_option;
  EU_DSecond timeStamp;
  BOOL enabledLanes_option;
  EU_EnabledLaneList enabledLanes;
  EU_MovementList states;
  BOOL maneuverAssistList_option;
  EU_ManeuverAssistList maneuverAssistList;
  BOOL regional_option;
  EU_IntersectionState_1 regional;
} EU_IntersectionState;


extern const ASN1CType asn1_type_EU_IntersectionState[];

typedef struct EU_IntersectionStateList {
  EU_IntersectionState *tab;
  size_t count;
} EU_IntersectionStateList;

extern const ASN1CType asn1_type_EU_IntersectionStateList[];

typedef struct EU_RegionalExtension_1 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_1;


extern const ASN1CType asn1_type_EU_RegionalExtension_1[];

typedef struct EU_DSRC_SPAT_1 {
  EU_RegionalExtension_1 *tab;
  size_t count;
} EU_DSRC_SPAT_1;

extern const ASN1CType asn1_type_EU_DSRC_SPAT_1[];

typedef struct EU_DSRC_SPAT {
  BOOL timeStamp_option;
  EU_MinuteOfTheYear timeStamp;
  BOOL name_option;
  EU_DescriptiveName name;
  EU_IntersectionStateList intersections;
  BOOL regional_option;
  EU_DSRC_SPAT_1 regional;
} EU_DSRC_SPAT;


extern const ASN1CType asn1_type_EU_DSRC_SPAT[];

typedef EU_DSRC_SPAT EU_SpatData;

extern const ASN1CType asn1_type_EU_SpatData[];

typedef struct EU_SPATEM {
  EU_ItsPduHeader header;
  EU_SpatData spat;
} EU_SPATEM;


extern const ASN1CType asn1_type_EU_SPATEM[];

typedef enum EU_LayerType {
  EU_LayerType_none,
  EU_LayerType_mixedContent,
  EU_LayerType_generalMapData,
  EU_LayerType_intersectionData,
  EU_LayerType_curveData,
  EU_LayerType_roadwaySectionData,
  EU_LayerType_parkingAreaData,
  EU_LayerType_sharedLaneData,
  EU_LayerType__dummy = 2147483647,
} EU_LayerType;

extern const ASN1CType asn1_type_EU_LayerType[];

typedef int EU_LayerID;

extern const ASN1CType asn1_type_EU_LayerID[];

typedef int EU_Elevation;

extern const ASN1CType asn1_type_EU_Elevation[];

typedef struct EU_RegionalExtension_18 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_18;


extern const ASN1CType asn1_type_EU_RegionalExtension_18[];

typedef struct EU_Position3D_1 {
  EU_RegionalExtension_18 *tab;
  size_t count;
} EU_Position3D_1;

extern const ASN1CType asn1_type_EU_Position3D_1[];

typedef struct EU_Position3D {
  EU_Latitude lat;
  EU_Longitude Long;
  BOOL elevation_option;
  EU_Elevation elevation;
  BOOL regional_option;
  EU_Position3D_1 regional;
} EU_Position3D;


extern const ASN1CType asn1_type_EU_Position3D[];

typedef int EU_LaneWidth;

extern const ASN1CType asn1_type_EU_LaneWidth[];

typedef enum EU_SpeedLimitType {
  EU_SpeedLimitType_unknown,
  EU_SpeedLimitType_maxSpeedInSchoolZone,
  EU_SpeedLimitType_maxSpeedInSchoolZoneWhenChildrenArePresent,
  EU_SpeedLimitType_maxSpeedInConstructionZone,
  EU_SpeedLimitType_vehicleMinSpeed,
  EU_SpeedLimitType_vehicleMaxSpeed,
  EU_SpeedLimitType_vehicleNightMaxSpeed,
  EU_SpeedLimitType_truckMinSpeed,
  EU_SpeedLimitType_truckMaxSpeed,
  EU_SpeedLimitType_truckNightMaxSpeed,
  EU_SpeedLimitType_vehiclesWithTrailersMinSpeed,
  EU_SpeedLimitType_vehiclesWithTrailersMaxSpeed,
  EU_SpeedLimitType_vehiclesWithTrailersNightMaxSpeed,
  EU_SpeedLimitType__dummy = 2147483647,
} EU_SpeedLimitType;

extern const ASN1CType asn1_type_EU_SpeedLimitType[];

typedef int EU_Velocity;

extern const ASN1CType asn1_type_EU_Velocity[];

typedef struct EU_RegulatorySpeedLimit {
  EU_SpeedLimitType type;
  EU_Velocity speed;
} EU_RegulatorySpeedLimit;


extern const ASN1CType asn1_type_EU_RegulatorySpeedLimit[];

typedef struct EU_SpeedLimitList {
  EU_RegulatorySpeedLimit *tab;
  size_t count;
} EU_SpeedLimitList;

extern const ASN1CType asn1_type_EU_SpeedLimitList[];

typedef int EU_ApproachID;

extern const ASN1CType asn1_type_EU_ApproachID[];

typedef ASN1BitString EU_LaneDirection;

extern const ASN1CType asn1_type_EU_LaneDirection[];

typedef ASN1BitString EU_LaneSharing;

extern const ASN1CType asn1_type_EU_LaneSharing[];

typedef ASN1BitString EU_LaneAttributes_Vehicle;

extern const ASN1CType asn1_type_EU_LaneAttributes_Vehicle[];

typedef ASN1BitString EU_LaneAttributes_Crosswalk;

extern const ASN1CType asn1_type_EU_LaneAttributes_Crosswalk[];

typedef ASN1BitString EU_LaneAttributes_Bike;

extern const ASN1CType asn1_type_EU_LaneAttributes_Bike[];

typedef ASN1BitString EU_LaneAttributes_Sidewalk;

extern const ASN1CType asn1_type_EU_LaneAttributes_Sidewalk[];

typedef ASN1BitString EU_LaneAttributes_Barrier;

extern const ASN1CType asn1_type_EU_LaneAttributes_Barrier[];

typedef ASN1BitString EU_LaneAttributes_Striping;

extern const ASN1CType asn1_type_EU_LaneAttributes_Striping[];

typedef ASN1BitString EU_LaneAttributes_TrackedVehicle;

extern const ASN1CType asn1_type_EU_LaneAttributes_TrackedVehicle[];

typedef ASN1BitString EU_LaneAttributes_Parking;

extern const ASN1CType asn1_type_EU_LaneAttributes_Parking[];

typedef enum {
  EU_LaneTypeAttributes_vehicle,
  EU_LaneTypeAttributes_crosswalk,
  EU_LaneTypeAttributes_bikeLane,
  EU_LaneTypeAttributes_sidewalk,
  EU_LaneTypeAttributes_median,
  EU_LaneTypeAttributes_striping,
  EU_LaneTypeAttributes_trackedVehicle,
  EU_LaneTypeAttributes_parking,
  EU_LaneTypeAttributes__dummy = 2147483647,
} EU_LaneTypeAttributes_choice;

typedef struct EU_LaneTypeAttributes {
  EU_LaneTypeAttributes_choice choice;
  union {
    EU_LaneAttributes_Vehicle vehicle;
    EU_LaneAttributes_Crosswalk crosswalk;
    EU_LaneAttributes_Bike bikeLane;
    EU_LaneAttributes_Sidewalk sidewalk;
    EU_LaneAttributes_Barrier median;
    EU_LaneAttributes_Striping striping;
    EU_LaneAttributes_TrackedVehicle trackedVehicle;
    EU_LaneAttributes_Parking parking;
  } u;
} EU_LaneTypeAttributes;

extern const ASN1CType asn1_type_EU_LaneTypeAttributes[];

typedef struct EU_RegionalExtension_12 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_12;


extern const ASN1CType asn1_type_EU_RegionalExtension_12[];

typedef struct EU_LaneAttributes {
  EU_LaneDirection directionalUse;
  EU_LaneSharing sharedWith;
  EU_LaneTypeAttributes laneType;
  BOOL regional_option;
  EU_RegionalExtension_12 regional;
} EU_LaneAttributes;


extern const ASN1CType asn1_type_EU_LaneAttributes[];

typedef ASN1BitString EU_AllowedManeuvers;

extern const ASN1CType asn1_type_EU_AllowedManeuvers[];

typedef enum EU_NodeAttributeXY {
  EU_NodeAttributeXY_reserved,
  EU_NodeAttributeXY_stopLine,
  EU_NodeAttributeXY_roundedCapStyleA,
  EU_NodeAttributeXY_roundedCapStyleB,
  EU_NodeAttributeXY_mergePoint,
  EU_NodeAttributeXY_divergePoint,
  EU_NodeAttributeXY_downstreamStopLine,
  EU_NodeAttributeXY_downstreamStartNode,
  EU_NodeAttributeXY_closedToTraffic,
  EU_NodeAttributeXY_safeIsland,
  EU_NodeAttributeXY_curbPresentAtStepOff,
  EU_NodeAttributeXY_hydrantPresent,
  EU_NodeAttributeXY__dummy = 2147483647,
} EU_NodeAttributeXY;

extern const ASN1CType asn1_type_EU_NodeAttributeXY[];

typedef struct EU_NodeAttributeXYList {
  EU_NodeAttributeXY *tab;
  size_t count;
} EU_NodeAttributeXYList;

extern const ASN1CType asn1_type_EU_NodeAttributeXYList[];

typedef enum EU_SegmentAttributeXY {
  EU_SegmentAttributeXY_reserved,
  EU_SegmentAttributeXY_doNotBlock,
  EU_SegmentAttributeXY_whiteLine,
  EU_SegmentAttributeXY_mergingLaneLeft,
  EU_SegmentAttributeXY_mergingLaneRight,
  EU_SegmentAttributeXY_curbOnLeft,
  EU_SegmentAttributeXY_curbOnRight,
  EU_SegmentAttributeXY_loadingzoneOnLeft,
  EU_SegmentAttributeXY_loadingzoneOnRight,
  EU_SegmentAttributeXY_turnOutPointOnLeft,
  EU_SegmentAttributeXY_turnOutPointOnRight,
  EU_SegmentAttributeXY_adjacentParkingOnLeft,
  EU_SegmentAttributeXY_adjacentParkingOnRight,
  EU_SegmentAttributeXY_adjacentBikeLaneOnLeft,
  EU_SegmentAttributeXY_adjacentBikeLaneOnRight,
  EU_SegmentAttributeXY_sharedBikeLane,
  EU_SegmentAttributeXY_bikeBoxInFront,
  EU_SegmentAttributeXY_transitStopOnLeft,
  EU_SegmentAttributeXY_transitStopOnRight,
  EU_SegmentAttributeXY_transitStopInLane,
  EU_SegmentAttributeXY_sharedWithTrackedVehicle,
  EU_SegmentAttributeXY_safeIsland,
  EU_SegmentAttributeXY_lowCurbsPresent,
  EU_SegmentAttributeXY_rumbleStripPresent,
  EU_SegmentAttributeXY_audibleSignalingPresent,
  EU_SegmentAttributeXY_adaptiveTimingPresent,
  EU_SegmentAttributeXY_rfSignalRequestPresent,
  EU_SegmentAttributeXY_partialCurbIntrusion,
  EU_SegmentAttributeXY_taperToLeft,
  EU_SegmentAttributeXY_taperToRight,
  EU_SegmentAttributeXY_taperToCenterLine,
  EU_SegmentAttributeXY_parallelParking,
  EU_SegmentAttributeXY_headInParking,
  EU_SegmentAttributeXY_freeParking,
  EU_SegmentAttributeXY_timeRestrictionsOnParking,
  EU_SegmentAttributeXY_costToPark,
  EU_SegmentAttributeXY_midBlockCurbPresent,
  EU_SegmentAttributeXY_unEvenPavementPresent,
  EU_SegmentAttributeXY__dummy = 2147483647,
} EU_SegmentAttributeXY;

extern const ASN1CType asn1_type_EU_SegmentAttributeXY[];

typedef struct EU_SegmentAttributeXYList {
  EU_SegmentAttributeXY *tab;
  size_t count;
} EU_SegmentAttributeXYList;

extern const ASN1CType asn1_type_EU_SegmentAttributeXYList[];

typedef int EU_DeltaAngle;

extern const ASN1CType asn1_type_EU_DeltaAngle[];

typedef int EU_RoadwayCrownAngle;

extern const ASN1CType asn1_type_EU_RoadwayCrownAngle[];

typedef int EU_MergeDivergeNodeAngle;

extern const ASN1CType asn1_type_EU_MergeDivergeNodeAngle[];

typedef struct EU_RegionalExtension_13 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_13;


extern const ASN1CType asn1_type_EU_RegionalExtension_13[];

typedef struct EU_LaneDataAttribute_1 {
  EU_RegionalExtension_13 *tab;
  size_t count;
} EU_LaneDataAttribute_1;

extern const ASN1CType asn1_type_EU_LaneDataAttribute_1[];

typedef enum {
  EU_LaneDataAttribute_pathEndPointAngle,
  EU_LaneDataAttribute_laneCrownPointCenter,
  EU_LaneDataAttribute_laneCrownPointLeft,
  EU_LaneDataAttribute_laneCrownPointRight,
  EU_LaneDataAttribute_laneAngle,
  EU_LaneDataAttribute_speedLimits,
  EU_LaneDataAttribute_regional,
  EU_LaneDataAttribute__dummy = 2147483647,
} EU_LaneDataAttribute_choice;

typedef struct EU_LaneDataAttribute {
  EU_LaneDataAttribute_choice choice;
  union {
    EU_DeltaAngle pathEndPointAngle;
    EU_RoadwayCrownAngle laneCrownPointCenter;
    EU_RoadwayCrownAngle laneCrownPointLeft;
    EU_RoadwayCrownAngle laneCrownPointRight;
    EU_MergeDivergeNodeAngle laneAngle;
    EU_SpeedLimitList speedLimits;
    EU_LaneDataAttribute_1 regional;
  } u;
} EU_LaneDataAttribute;

extern const ASN1CType asn1_type_EU_LaneDataAttribute[];

typedef struct EU_LaneDataAttributeList {
  EU_LaneDataAttribute *tab;
  size_t count;
} EU_LaneDataAttributeList;

extern const ASN1CType asn1_type_EU_LaneDataAttributeList[];

typedef struct EU_RegionalExtension_16 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_16;


extern const ASN1CType asn1_type_EU_RegionalExtension_16[];

typedef struct EU_NodeAttributeSetXY_1 {
  EU_RegionalExtension_16 *tab;
  size_t count;
} EU_NodeAttributeSetXY_1;

extern const ASN1CType asn1_type_EU_NodeAttributeSetXY_1[];

typedef struct EU_NodeAttributeSetXY {
  BOOL localNode_option;
  EU_NodeAttributeXYList localNode;
  BOOL disabled_option;
  EU_SegmentAttributeXYList disabled;
  BOOL enabled_option;
  EU_SegmentAttributeXYList enabled;
  BOOL data_option;
  EU_LaneDataAttributeList data;
  BOOL dWidth_option;
  EU_Offset_B10 dWidth;
  BOOL dElevation_option;
  EU_Offset_B10 dElevation;
  BOOL regional_option;
  EU_NodeAttributeSetXY_1 regional;
} EU_NodeAttributeSetXY;


extern const ASN1CType asn1_type_EU_NodeAttributeSetXY[];

typedef struct EU_NodeXY {
  EU_NodeOffsetPointXY delta;
  BOOL attributes_option;
  EU_NodeAttributeSetXY attributes;
} EU_NodeXY;


extern const ASN1CType asn1_type_EU_NodeXY[];

typedef struct EU_NodeSetXY {
  EU_NodeXY *tab;
  size_t count;
} EU_NodeSetXY;

extern const ASN1CType asn1_type_EU_NodeSetXY[];

typedef int EU_DrivenLineOffsetSm;

extern const ASN1CType asn1_type_EU_DrivenLineOffsetSm[];

typedef int EU_DrivenLineOffsetLg;

extern const ASN1CType asn1_type_EU_DrivenLineOffsetLg[];

typedef enum {
  EU_ComputedLane_1_small,
  EU_ComputedLane_1_large,
  EU_ComputedLane_1__dummy = 2147483647,
} EU_ComputedLane_1_choice;

typedef struct EU_ComputedLane_1 {
  EU_ComputedLane_1_choice choice;
  union {
    EU_DrivenLineOffsetSm small;
    EU_DrivenLineOffsetLg large;
  } u;
} EU_ComputedLane_1;

extern const ASN1CType asn1_type_EU_ComputedLane_1[];

typedef enum {
  EU_ComputedLane_2_small,
  EU_ComputedLane_2_large,
  EU_ComputedLane_2__dummy = 2147483647,
} EU_ComputedLane_2_choice;

typedef struct EU_ComputedLane_2 {
  EU_ComputedLane_2_choice choice;
  union {
    EU_DrivenLineOffsetSm small;
    EU_DrivenLineOffsetLg large;
  } u;
} EU_ComputedLane_2;

extern const ASN1CType asn1_type_EU_ComputedLane_2[];

typedef int EU_Angle;

extern const ASN1CType asn1_type_EU_Angle[];

typedef int EU_Scale_B12;

extern const ASN1CType asn1_type_EU_Scale_B12[];

typedef struct EU_RegionalExtension_7 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_7;


extern const ASN1CType asn1_type_EU_RegionalExtension_7[];

typedef struct EU_ComputedLane_3 {
  EU_RegionalExtension_7 *tab;
  size_t count;
} EU_ComputedLane_3;

extern const ASN1CType asn1_type_EU_ComputedLane_3[];

typedef struct EU_ComputedLane {
  EU_LaneID referenceLaneId;
  EU_ComputedLane_1 offsetXaxis;
  EU_ComputedLane_2 offsetYaxis;
  BOOL rotateXY_option;
  EU_Angle rotateXY;
  BOOL scaleXaxis_option;
  EU_Scale_B12 scaleXaxis;
  BOOL scaleYaxis_option;
  EU_Scale_B12 scaleYaxis;
  BOOL regional_option;
  EU_ComputedLane_3 regional;
} EU_ComputedLane;


extern const ASN1CType asn1_type_EU_ComputedLane[];

typedef enum {
  EU_NodeListXY_nodes,
  EU_NodeListXY_computed,
  EU_NodeListXY__dummy = 2147483647,
} EU_NodeListXY_choice;

typedef struct EU_NodeListXY {
  EU_NodeListXY_choice choice;
  union {
    EU_NodeSetXY nodes;
    EU_ComputedLane computed;
  } u;
} EU_NodeListXY;

extern const ASN1CType asn1_type_EU_NodeListXY[];

typedef struct EU_ConnectingLane {
  EU_LaneID lane;
  BOOL maneuver_option;
  EU_AllowedManeuvers maneuver;
} EU_ConnectingLane;


extern const ASN1CType asn1_type_EU_ConnectingLane[];

typedef struct EU_Connection {
  EU_ConnectingLane connectingLane;
  BOOL remoteIntersection_option;
  EU_IntersectionReferenceID remoteIntersection;
  BOOL signalGroup_option;
  EU_SignalGroupID signalGroup;
  BOOL userClass_option;
  EU_RestrictionClassID userClass;
  BOOL connectionID_option;
  EU_LaneConnectionID connectionID;
} EU_Connection;


extern const ASN1CType asn1_type_EU_Connection[];

typedef struct EU_ConnectsToList {
  EU_Connection *tab;
  size_t count;
} EU_ConnectsToList;

extern const ASN1CType asn1_type_EU_ConnectsToList[];

typedef struct EU_OverlayLaneList {
  EU_LaneID *tab;
  size_t count;
} EU_OverlayLaneList;

extern const ASN1CType asn1_type_EU_OverlayLaneList[];

typedef struct EU_RegionalExtension_9 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_9;


extern const ASN1CType asn1_type_EU_RegionalExtension_9[];

typedef struct EU_GenericLane_1 {
  EU_RegionalExtension_9 *tab;
  size_t count;
} EU_GenericLane_1;

extern const ASN1CType asn1_type_EU_GenericLane_1[];

typedef struct EU_GenericLane {
  EU_LaneID laneID;
  BOOL name_option;
  EU_DescriptiveName name;
  BOOL ingressApproach_option;
  EU_ApproachID ingressApproach;
  BOOL egressApproach_option;
  EU_ApproachID egressApproach;
  EU_LaneAttributes laneAttributes;
  BOOL maneuvers_option;
  EU_AllowedManeuvers maneuvers;
  EU_NodeListXY nodeList;
  BOOL connectsTo_option;
  EU_ConnectsToList connectsTo;
  BOOL overlays_option;
  EU_OverlayLaneList overlays;
  BOOL regional_option;
  EU_GenericLane_1 regional;
} EU_GenericLane;


extern const ASN1CType asn1_type_EU_GenericLane[];

typedef struct EU_LaneList {
  EU_GenericLane *tab;
  size_t count;
} EU_LaneList;

extern const ASN1CType asn1_type_EU_LaneList[];

typedef struct EU_RegionalExtension_23 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_23;


extern const ASN1CType asn1_type_EU_RegionalExtension_23[];

typedef struct EU_SignalControlZone {
  EU_RegionalExtension_23 zone;
} EU_SignalControlZone;


extern const ASN1CType asn1_type_EU_SignalControlZone[];

typedef struct EU_PreemptPriorityList {
  EU_SignalControlZone *tab;
  size_t count;
} EU_PreemptPriorityList;

extern const ASN1CType asn1_type_EU_PreemptPriorityList[];

typedef struct EU_RegionalExtension_10 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_10;


extern const ASN1CType asn1_type_EU_RegionalExtension_10[];

typedef struct EU_IntersectionGeometry_1 {
  EU_RegionalExtension_10 *tab;
  size_t count;
} EU_IntersectionGeometry_1;

extern const ASN1CType asn1_type_EU_IntersectionGeometry_1[];

typedef struct EU_IntersectionGeometry {
  BOOL name_option;
  EU_DescriptiveName name;
  EU_IntersectionReferenceID id;
  EU_MsgCount revision;
  EU_Position3D refPoint;
  BOOL laneWidth_option;
  EU_LaneWidth laneWidth;
  BOOL speedLimits_option;
  EU_SpeedLimitList speedLimits;
  EU_LaneList laneSet;
  BOOL preemptPriorityData_option;
  EU_PreemptPriorityList preemptPriorityData;
  BOOL regional_option;
  EU_IntersectionGeometry_1 regional;
} EU_IntersectionGeometry;


extern const ASN1CType asn1_type_EU_IntersectionGeometry[];

typedef struct EU_IntersectionGeometryList {
  EU_IntersectionGeometry *tab;
  size_t count;
} EU_IntersectionGeometryList;

extern const ASN1CType asn1_type_EU_IntersectionGeometryList[];

typedef struct EU_RoadLaneSetList {
  EU_GenericLane *tab;
  size_t count;
} EU_RoadLaneSetList;

extern const ASN1CType asn1_type_EU_RoadLaneSetList[];

typedef struct EU_RegionalExtension_22 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_22;


extern const ASN1CType asn1_type_EU_RegionalExtension_22[];

typedef struct EU_RoadSegment_1 {
  EU_RegionalExtension_22 *tab;
  size_t count;
} EU_RoadSegment_1;

extern const ASN1CType asn1_type_EU_RoadSegment_1[];

typedef struct EU_RoadSegment {
  BOOL name_option;
  EU_DescriptiveName name;
  EU_RoadSegmentReferenceID id;
  EU_MsgCount revision;
  EU_Position3D refPoint;
  BOOL laneWidth_option;
  EU_LaneWidth laneWidth;
  BOOL speedLimits_option;
  EU_SpeedLimitList speedLimits;
  EU_RoadLaneSetList roadLaneSet;
  BOOL regional_option;
  EU_RoadSegment_1 regional;
} EU_RoadSegment;


extern const ASN1CType asn1_type_EU_RoadSegment[];

typedef struct EU_RoadSegmentList {
  EU_RoadSegment *tab;
  size_t count;
} EU_RoadSegmentList;

extern const ASN1CType asn1_type_EU_RoadSegmentList[];

typedef struct EU_DataParameters {
  BOOL processMethod_option;
  ASN1String processMethod;
  BOOL processAgency_option;
  ASN1String processAgency;
  BOOL lastCheckedDate_option;
  ASN1String lastCheckedDate;
  BOOL geoidUsed_option;
  ASN1String geoidUsed;
} EU_DataParameters;


extern const ASN1CType asn1_type_EU_DataParameters[];

typedef enum EU_RestrictionAppliesTo {
  EU_RestrictionAppliesTo_none,
  EU_RestrictionAppliesTo_equippedTransit,
  EU_RestrictionAppliesTo_equippedTaxis,
  EU_RestrictionAppliesTo_equippedOther,
  EU_RestrictionAppliesTo_emissionCompliant,
  EU_RestrictionAppliesTo_equippedBicycle,
  EU_RestrictionAppliesTo_weightCompliant,
  EU_RestrictionAppliesTo_heightCompliant,
  EU_RestrictionAppliesTo_pedestrians,
  EU_RestrictionAppliesTo_slowMovingPersons,
  EU_RestrictionAppliesTo_wheelchairUsers,
  EU_RestrictionAppliesTo_visualDisabilities,
  EU_RestrictionAppliesTo_audioDisabilities,
  EU_RestrictionAppliesTo_otherUnknownDisabilities,
  EU_RestrictionAppliesTo__dummy = 2147483647,
} EU_RestrictionAppliesTo;

extern const ASN1CType asn1_type_EU_RestrictionAppliesTo[];

typedef struct EU_RegionalExtension_21 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_21;


extern const ASN1CType asn1_type_EU_RegionalExtension_21[];

typedef struct EU_RestrictionUserType_1 {
  EU_RegionalExtension_21 *tab;
  size_t count;
} EU_RestrictionUserType_1;

extern const ASN1CType asn1_type_EU_RestrictionUserType_1[];

typedef enum {
  EU_RestrictionUserType_basicType,
  EU_RestrictionUserType_regional,
  EU_RestrictionUserType__dummy = 2147483647,
} EU_RestrictionUserType_choice;

typedef struct EU_RestrictionUserType {
  EU_RestrictionUserType_choice choice;
  union {
    EU_RestrictionAppliesTo basicType;
    EU_RestrictionUserType_1 regional;
  } u;
} EU_RestrictionUserType;

extern const ASN1CType asn1_type_EU_RestrictionUserType[];

typedef struct EU_RestrictionUserTypeList {
  EU_RestrictionUserType *tab;
  size_t count;
} EU_RestrictionUserTypeList;

extern const ASN1CType asn1_type_EU_RestrictionUserTypeList[];

typedef struct EU_RestrictionClassAssignment {
  EU_RestrictionClassID id;
  EU_RestrictionUserTypeList users;
} EU_RestrictionClassAssignment;


extern const ASN1CType asn1_type_EU_RestrictionClassAssignment[];

typedef struct EU_RestrictionClassList {
  EU_RestrictionClassAssignment *tab;
  size_t count;
} EU_RestrictionClassList;

extern const ASN1CType asn1_type_EU_RestrictionClassList[];

typedef struct EU_RegionalExtension_4 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_4;


extern const ASN1CType asn1_type_EU_RegionalExtension_4[];

typedef struct EU_MapData_1 {
  EU_RegionalExtension_4 *tab;
  size_t count;
} EU_MapData_1;

extern const ASN1CType asn1_type_EU_MapData_1[];

typedef struct EU_MapData {
  BOOL timeStamp_option;
  EU_MinuteOfTheYear timeStamp;
  EU_MsgCount msgIssueRevision;
  BOOL layerType_option;
  EU_LayerType layerType;
  BOOL layerID_option;
  EU_LayerID layerID;
  BOOL intersections_option;
  EU_IntersectionGeometryList intersections;
  BOOL roadSegments_option;
  EU_RoadSegmentList roadSegments;
  BOOL dataParameters_option;
  EU_DataParameters dataParameters;
  BOOL restrictionList_option;
  EU_RestrictionClassList restrictionList;
  BOOL regional_option;
  EU_MapData_1 regional;
} EU_MapData;


extern const ASN1CType asn1_type_EU_MapData[];

typedef struct EU_MAPEM {
  EU_ItsPduHeader header;
  EU_MapData map;
} EU_MAPEM;


extern const ASN1CType asn1_type_EU_MAPEM[];

typedef ASN1BitString EU_CountryCode;

extern const ASN1CType asn1_type_EU_CountryCode[];

typedef int EU_IssuerIdentifier;

extern const ASN1CType asn1_type_EU_IssuerIdentifier[];

typedef struct EU_Provider {
  EU_CountryCode countryCode;
  EU_IssuerIdentifier providerIdentifier;
} EU_Provider;


extern const ASN1CType asn1_type_EU_Provider[];

typedef int EU_IviIdentificationNumber;

extern const ASN1CType asn1_type_EU_IviIdentificationNumber[];

typedef struct EU_IviIdentificationNumbers {
  EU_IviIdentificationNumber *tab;
  size_t count;
} EU_IviIdentificationNumbers;

extern const ASN1CType asn1_type_EU_IviIdentificationNumbers[];

typedef int EU_IviStatus;

enum {
  EU_IviStatus_New = 0,
  EU_IviStatus_update = 1,
  EU_IviStatus_cancellation = 2,
  EU_IviStatus_negation = 3,
};

extern const ASN1CType asn1_type_EU_IviStatus[];

typedef struct EU_ConnectedDenms {
  EU_ActionID *tab;
  size_t count;
} EU_ConnectedDenms;

extern const ASN1CType asn1_type_EU_ConnectedDenms[];

typedef struct EU_IviManagementContainer {
  EU_Provider serviceProviderId;
  EU_IviIdentificationNumber iviIdentificationNumber;
  BOOL timeStamp_option;
  EU_TimestampIts timeStamp;
  BOOL validFrom_option;
  EU_TimestampIts validFrom;
  BOOL validTo_option;
  EU_TimestampIts validTo;
  BOOL connectedIviStructures_option;
  EU_IviIdentificationNumbers connectedIviStructures;
  EU_IviStatus iviStatus;
  BOOL connectedDenms_option;
  EU_ConnectedDenms connectedDenms;
} EU_IviManagementContainer;


extern const ASN1CType asn1_type_EU_IviManagementContainer[];

typedef int EU_Zid;

extern const ASN1CType asn1_type_EU_Zid[];

typedef struct EU_DeltaPosition {
  EU_DeltaLatitude deltaLatitude;
  EU_DeltaLongitude deltaLongitude;
} EU_DeltaPosition;


extern const ASN1CType asn1_type_EU_DeltaPosition[];

typedef struct EU_DeltaPositions {
  EU_DeltaPosition *tab;
  size_t count;
} EU_DeltaPositions;

extern const ASN1CType asn1_type_EU_DeltaPositions[];

typedef struct EU_DeltaReferencePositions {
  EU_DeltaReferencePosition *tab;
  size_t count;
} EU_DeltaReferencePositions;

extern const ASN1CType asn1_type_EU_DeltaReferencePositions[];

typedef struct EU_AbsolutePosition {
  EU_Latitude latitude;
  EU_Longitude longitude;
} EU_AbsolutePosition;


extern const ASN1CType asn1_type_EU_AbsolutePosition[];

typedef struct EU_AbsolutePositions {
  EU_AbsolutePosition *tab;
  size_t count;
} EU_AbsolutePositions;

extern const ASN1CType asn1_type_EU_AbsolutePositions[];

typedef struct EU_AbsolutePositionWAltitude {
  EU_Latitude latitude;
  EU_Longitude longitude;
  EU_Altitude altitude;
} EU_AbsolutePositionWAltitude;


extern const ASN1CType asn1_type_EU_AbsolutePositionWAltitude[];

typedef struct EU_AbsolutePositionsWAltitude {
  EU_AbsolutePositionWAltitude *tab;
  size_t count;
} EU_AbsolutePositionsWAltitude;

extern const ASN1CType asn1_type_EU_AbsolutePositionsWAltitude[];

typedef enum {
  EU_PolygonalLine_deltaPositions,
  EU_PolygonalLine_deltaPositionsWithAltitude,
  EU_PolygonalLine_absolutePositions,
  EU_PolygonalLine_absolutePositionsWithAltitude,
  EU_PolygonalLine__dummy = 2147483647,
} EU_PolygonalLine_choice;

typedef struct EU_PolygonalLine {
  EU_PolygonalLine_choice choice;
  union {
    EU_DeltaPositions deltaPositions;
    EU_DeltaReferencePositions deltaPositionsWithAltitude;
    EU_AbsolutePositions absolutePositions;
    EU_AbsolutePositionsWAltitude absolutePositionsWithAltitude;
  } u;
} EU_PolygonalLine;

extern const ASN1CType asn1_type_EU_PolygonalLine[];

typedef int EU_IviLaneWidth;

extern const ASN1CType asn1_type_EU_IviLaneWidth[];

typedef struct EU_Segment {
  EU_PolygonalLine line;
  BOOL laneWidth_option;
  EU_IviLaneWidth laneWidth;
} EU_Segment;


extern const ASN1CType asn1_type_EU_Segment[];

typedef struct EU_ComputedSegment {
  EU_Zid zoneId;
  EU_LanePosition laneNumber;
  EU_IviLaneWidth laneWidth;
  BOOL offsetDistance_option;
  int offsetDistance;
  BOOL offsetPosition_option;
  EU_DeltaReferencePosition offsetPosition;
} EU_ComputedSegment;


extern const ASN1CType asn1_type_EU_ComputedSegment[];

typedef enum {
  EU_Zone_segment,
  EU_Zone_area,
  EU_Zone_computedSegment,
  EU_Zone__dummy = 2147483647,
} EU_Zone_choice;

typedef struct EU_Zone {
  EU_Zone_choice choice;
  union {
    EU_Segment segment;
    EU_PolygonalLine area;
    EU_ComputedSegment computedSegment;
  } u;
} EU_Zone;

extern const ASN1CType asn1_type_EU_Zone[];

typedef struct EU_GlcPart {
  EU_Zid zoneId;
  BOOL laneNumber_option;
  EU_LanePosition laneNumber;
  BOOL zoneExtension_option;
  int zoneExtension;
  BOOL zoneHeading_option;
  EU_HeadingValue zoneHeading;
  BOOL zone_option;
  EU_Zone zone;
} EU_GlcPart;


extern const ASN1CType asn1_type_EU_GlcPart[];

typedef struct EU_GlcParts {
  EU_GlcPart *tab;
  size_t count;
} EU_GlcParts;

extern const ASN1CType asn1_type_EU_GlcParts[];

typedef struct EU_GeographicLocationContainer {
  EU_ReferencePosition referencePosition;
  BOOL referencePositionTime_option;
  EU_TimestampIts referencePositionTime;
  BOOL referencePositionHeading_option;
  EU_Heading referencePositionHeading;
  BOOL referencePositionSpeed_option;
  EU_Speed referencePositionSpeed;
  EU_GlcParts parts;
} EU_GeographicLocationContainer;


extern const ASN1CType asn1_type_EU_GeographicLocationContainer[];

typedef struct EU_ZoneIds {
  EU_Zid *tab;
  size_t count;
} EU_ZoneIds;

extern const ASN1CType asn1_type_EU_ZoneIds[];

typedef int EU_Ext3;

extern const ASN1CType asn1_type_EU_Ext3[];

typedef enum {
  EU_Ext2_content,
  EU_Ext2_extension,
  EU_Ext2__dummy = 2147483647,
} EU_Ext2_choice;

typedef struct EU_Ext2 {
  EU_Ext2_choice choice;
  union {
    int content;
    EU_Ext3 extension;
  } u;
} EU_Ext2;

extern const ASN1CType asn1_type_EU_Ext2[];

typedef enum {
  EU_Ext1_content,
  EU_Ext1_extension,
  EU_Ext1__dummy = 2147483647,
} EU_Ext1_choice;

typedef struct EU_Ext1 {
  EU_Ext1_choice choice;
  union {
    int content;
    EU_Ext2 extension;
  } u;
} EU_Ext1;

extern const ASN1CType asn1_type_EU_Ext1[];

typedef enum {
  EU_VarLengthNumber_content,
  EU_VarLengthNumber_extension,
  EU_VarLengthNumber__dummy = 2147483647,
} EU_VarLengthNumber_choice;

typedef struct EU_VarLengthNumber {
  EU_VarLengthNumber_choice choice;
  union {
    int content;
    EU_Ext1 extension;
  } u;
} EU_VarLengthNumber;

extern const ASN1CType asn1_type_EU_VarLengthNumber[];

typedef int EU_Direction;

enum {
  EU_Direction_sameDirection = 0,
  EU_Direction_oppositeDirection = 1,
  EU_Direction_bothDirections = 2,
  EU_Direction_valueNotUsed = 3,
};

extern const ASN1CType asn1_type_EU_Direction[];

typedef struct EU_LanePositions {
  EU_LanePosition *tab;
  size_t count;
} EU_LanePositions;

extern const ASN1CType asn1_type_EU_LanePositions[];

typedef int EU_IviType;

enum {
  EU_IviType_immediateDangerWarningMessages = 0,
  EU_IviType_regulatoryMessages = 1,
  EU_IviType_trafficRelatedInformationMessages = 2,
  EU_IviType_pollutionMessages = 3,
  EU_IviType_notTrafficRelatedInformationMessages = 4,
};

extern const ASN1CType asn1_type_EU_IviType[];

typedef int EU_IviPurpose;

enum {
  EU_IviPurpose_safety = 0,
  EU_IviPurpose_environmental = 1,
  EU_IviPurpose_trafficOptimisation = 2,
};

extern const ASN1CType asn1_type_EU_IviPurpose[];

typedef int EU_LaneStatus;

enum {
  EU_LaneStatus_open = 0,
  EU_LaneStatus_closed = 1,
  EU_LaneStatus_mergeR = 2,
  EU_LaneStatus_mergeL = 3,
  EU_LaneStatus_mergeLR = 4,
  EU_LaneStatus_provisionallyOpen = 5,
  EU_LaneStatus_diverging = 6,
};

extern const ASN1CType asn1_type_EU_LaneStatus[];

typedef enum EU_EuVehicleCategoryL {
  EU_EuVehicleCategoryL_l1,
  EU_EuVehicleCategoryL_l2,
  EU_EuVehicleCategoryL_l3,
  EU_EuVehicleCategoryL_l4,
  EU_EuVehicleCategoryL_l5,
  EU_EuVehicleCategoryL_l6,
  EU_EuVehicleCategoryL_l7,
  EU_EuVehicleCategoryL__dummy = 2147483647,
} EU_EuVehicleCategoryL;

extern const ASN1CType asn1_type_EU_EuVehicleCategoryL[];

typedef enum EU_EuVehicleCategoryM {
  EU_EuVehicleCategoryM_m1,
  EU_EuVehicleCategoryM_m2,
  EU_EuVehicleCategoryM_m3,
  EU_EuVehicleCategoryM__dummy = 2147483647,
} EU_EuVehicleCategoryM;

extern const ASN1CType asn1_type_EU_EuVehicleCategoryM[];

typedef enum EU_EuVehicleCategoryN {
  EU_EuVehicleCategoryN_n1,
  EU_EuVehicleCategoryN_n2,
  EU_EuVehicleCategoryN_n3,
  EU_EuVehicleCategoryN__dummy = 2147483647,
} EU_EuVehicleCategoryN;

extern const ASN1CType asn1_type_EU_EuVehicleCategoryN[];

typedef enum EU_EuVehicleCategoryO {
  EU_EuVehicleCategoryO_o1,
  EU_EuVehicleCategoryO_o2,
  EU_EuVehicleCategoryO_o3,
  EU_EuVehicleCategoryO_o4,
  EU_EuVehicleCategoryO__dummy = 2147483647,
} EU_EuVehicleCategoryO;

extern const ASN1CType asn1_type_EU_EuVehicleCategoryO[];

typedef enum {
  EU_EuVehicleCategoryCode_euVehicleCategoryL,
  EU_EuVehicleCategoryCode_euVehicleCategoryM,
  EU_EuVehicleCategoryCode_euVehicleCategoryN,
  EU_EuVehicleCategoryCode_euVehicleCategoryO,
  EU_EuVehicleCategoryCode_euVehilcleCategoryT,
  EU_EuVehicleCategoryCode_euVehilcleCategoryG,
  EU_EuVehicleCategoryCode__dummy = 2147483647,
} EU_EuVehicleCategoryCode_choice;

typedef struct EU_EuVehicleCategoryCode {
  EU_EuVehicleCategoryCode_choice choice;
  union {
    EU_EuVehicleCategoryL euVehicleCategoryL;
    EU_EuVehicleCategoryM euVehicleCategoryM;
    EU_EuVehicleCategoryN euVehicleCategoryN;
    EU_EuVehicleCategoryO euVehicleCategoryO;
  } u;
} EU_EuVehicleCategoryCode;

extern const ASN1CType asn1_type_EU_EuVehicleCategoryCode[];

typedef int EU_Iso3833VehicleType;

enum {
  EU_Iso3833VehicleType_passengerCar = 0,
  EU_Iso3833VehicleType_saloon = 1,
  EU_Iso3833VehicleType_convertibleSaloon = 2,
  EU_Iso3833VehicleType_pullmanSaloon = 3,
  EU_Iso3833VehicleType_stationWagon = 4,
  EU_Iso3833VehicleType_truckStationWagon = 5,
  EU_Iso3833VehicleType_coupe = 6,
  EU_Iso3833VehicleType_convertible = 7,
  EU_Iso3833VehicleType_multipurposePassengerCar = 8,
  EU_Iso3833VehicleType_forwardControlPassengerCar = 9,
  EU_Iso3833VehicleType_specialPassengerCar = 10,
  EU_Iso3833VehicleType_bus = 11,
  EU_Iso3833VehicleType_minibus = 12,
  EU_Iso3833VehicleType_urbanBus = 13,
  EU_Iso3833VehicleType_interurbanCoach = 14,
  EU_Iso3833VehicleType_longDistanceCoach = 15,
  EU_Iso3833VehicleType_articulatedBus = 16,
  EU_Iso3833VehicleType_trolleyBus = 17,
  EU_Iso3833VehicleType_specialBus = 18,
  EU_Iso3833VehicleType_commercialVehicle = 19,
  EU_Iso3833VehicleType_specialCommercialVehicle = 20,
  EU_Iso3833VehicleType_specialVehicle = 21,
  EU_Iso3833VehicleType_trailingTowingVehicle = 22,
  EU_Iso3833VehicleType_semiTrailerTowingVehicle = 23,
  EU_Iso3833VehicleType_trailer = 24,
  EU_Iso3833VehicleType_busTrailer = 25,
  EU_Iso3833VehicleType_generalPurposeTrailer = 26,
  EU_Iso3833VehicleType_caravan = 27,
  EU_Iso3833VehicleType_specialTrailer = 28,
  EU_Iso3833VehicleType_semiTrailer = 29,
  EU_Iso3833VehicleType_busSemiTrailer = 30,
  EU_Iso3833VehicleType_generalPurposeSemiTrailer = 31,
  EU_Iso3833VehicleType_specialSemiTrailer = 32,
  EU_Iso3833VehicleType_roadTrain = 33,
  EU_Iso3833VehicleType_passengerRoadTrain = 34,
  EU_Iso3833VehicleType_articulatedRoadTrain = 35,
  EU_Iso3833VehicleType_doubleRoadTrain = 36,
  EU_Iso3833VehicleType_compositeRoadTrain = 37,
  EU_Iso3833VehicleType_specialRoadTrain = 38,
  EU_Iso3833VehicleType_moped = 39,
  EU_Iso3833VehicleType_motorCycle = 40,
};

extern const ASN1CType asn1_type_EU_Iso3833VehicleType[];

typedef enum EU_EuroValue {
  EU_EuroValue_noEntry,
  EU_EuroValue_euro_1,
  EU_EuroValue_euro_2,
  EU_EuroValue_euro_3,
  EU_EuroValue_euro_4,
  EU_EuroValue_euro_5,
  EU_EuroValue_euro_6,
  EU_EuroValue_reservedForUse1,
  EU_EuroValue_reservedForUse2,
  EU_EuroValue_reservedForUse3,
  EU_EuroValue_reservedForUse4,
  EU_EuroValue_reservedForUse5,
  EU_EuroValue_reservedForUse6,
  EU_EuroValue_reservedForUse7,
  EU_EuroValue_reservedForUse8,
  EU_EuroValue_eev,
  EU_EuroValue__dummy = 2147483647,
} EU_EuroValue;

extern const ASN1CType asn1_type_EU_EuroValue[];

typedef enum EU_CopValue {
  EU_CopValue_noEntry,
  EU_CopValue_co2class1,
  EU_CopValue_co2class2,
  EU_CopValue_co2class3,
  EU_CopValue_co2class4,
  EU_CopValue_co2class5,
  EU_CopValue_co2class6,
  EU_CopValue_co2class7,
  EU_CopValue_reservedforUse,
  EU_CopValue__dummy = 2147483647,
} EU_CopValue;

extern const ASN1CType asn1_type_EU_CopValue[];

typedef struct EU_EnvironmentalCharacteristics {
  EU_EuroValue euroValue;
  EU_CopValue copValue;
} EU_EnvironmentalCharacteristics;


extern const ASN1CType asn1_type_EU_EnvironmentalCharacteristics[];

typedef int EU_EngineCharacteristics;

enum {
  EU_EngineCharacteristics_noEntry = 0,
  EU_EngineCharacteristics_noEngine = 1,
  EU_EngineCharacteristics_petrolUnleaded = 2,
  EU_EngineCharacteristics_petrolLeaded = 3,
  EU_EngineCharacteristics_diesel = 4,
  EU_EngineCharacteristics_lPG = 5,
  EU_EngineCharacteristics_battery = 6,
  EU_EngineCharacteristics_solar = 7,
  EU_EngineCharacteristics_hybrid = 8,
  EU_EngineCharacteristics_hydrogen = 9,
};

extern const ASN1CType asn1_type_EU_EngineCharacteristics[];

typedef int EU_GoodsType;

enum {
  EU_GoodsType_ammunition = 0,
  EU_GoodsType_chemicals = 1,
  EU_GoodsType_empty = 2,
  EU_GoodsType_fuel = 3,
  EU_GoodsType_glass = 4,
  EU_GoodsType_dangerous = 5,
  EU_GoodsType_liquid = 6,
  EU_GoodsType_liveStock = 7,
  EU_GoodsType_dangerousForPeople = 8,
  EU_GoodsType_dangerousForTheEnvironment = 9,
  EU_GoodsType_dangerousForWater = 10,
  EU_GoodsType_perishableProducts = 11,
  EU_GoodsType_pharmaceutical = 12,
  EU_GoodsType_vehicles = 13,
};

extern const ASN1CType asn1_type_EU_GoodsType[];

typedef struct EU_LoadType {
  EU_GoodsType goodsType;
  EU_DangerousGoodsBasic dangerousGoodsType;
  EU_SpecialTransportType specialTransportType;
} EU_LoadType;


extern const ASN1CType asn1_type_EU_LoadType[];

typedef enum {
  EU_VehicleCharacteristicsFixValues_simpleVehicleType,
  EU_VehicleCharacteristicsFixValues_euVehicleCategoryCode,
  EU_VehicleCharacteristicsFixValues_iso3833VehicleType,
  EU_VehicleCharacteristicsFixValues_euroAndCo2value,
  EU_VehicleCharacteristicsFixValues_engineCharacteristics,
  EU_VehicleCharacteristicsFixValues_loadType,
  EU_VehicleCharacteristicsFixValues_usage,
  EU_VehicleCharacteristicsFixValues__dummy = 2147483647,
} EU_VehicleCharacteristicsFixValues_choice;

typedef struct EU_VehicleCharacteristicsFixValues {
  EU_VehicleCharacteristicsFixValues_choice choice;
  union {
    EU_StationType simpleVehicleType;
    EU_EuVehicleCategoryCode euVehicleCategoryCode;
    EU_Iso3833VehicleType iso3833VehicleType;
    EU_EnvironmentalCharacteristics euroAndCo2value;
    EU_EngineCharacteristics engineCharacteristics;
    EU_LoadType loadType;
    EU_VehicleRole usage;
  } u;
} EU_VehicleCharacteristicsFixValues;

extern const ASN1CType asn1_type_EU_VehicleCharacteristicsFixValues[];

typedef struct EU_VehicleCharacteristicsFixValuesList {
  EU_VehicleCharacteristicsFixValues *tab;
  size_t count;
} EU_VehicleCharacteristicsFixValuesList;

extern const ASN1CType asn1_type_EU_VehicleCharacteristicsFixValuesList[];

typedef int EU_ComparisonOperator;

enum {
  EU_ComparisonOperator_greaterThan = 0,
  EU_ComparisonOperator_greaterThanOrEqualTo = 1,
  EU_ComparisonOperator_lessThan = 2,
  EU_ComparisonOperator_lessThanOrEqualTo = 3,
};

extern const ASN1CType asn1_type_EU_ComparisonOperator[];

typedef int EU_Int1;

extern const ASN1CType asn1_type_EU_Int1[];

typedef struct EU_VehicleDimensions {
  EU_Int1 vehicleLengthOverall;
  EU_Int1 vehicleHeigthOverall;
  EU_Int1 vehicleWidthOverall;
} EU_VehicleDimensions;


extern const ASN1CType asn1_type_EU_VehicleDimensions[];

typedef int EU_Int2;

extern const ASN1CType asn1_type_EU_Int2[];

typedef struct EU_VehicleWeightLimits {
  EU_Int2 vehicleMaxLadenWeight;
  EU_Int2 vehicleTrainMaximumWeight;
  EU_Int2 vehicleWeightUnladen;
} EU_VehicleWeightLimits;


extern const ASN1CType asn1_type_EU_VehicleWeightLimits[];

typedef struct EU_AxleWeightLimits {
  EU_Int2 maxLadenweightOnAxle1;
  EU_Int2 maxLadenweightOnAxle2;
  EU_Int2 maxLadenweightOnAxle3;
  EU_Int2 maxLadenweightOnAxle4;
  EU_Int2 maxLadenweightOnAxle5;
} EU_AxleWeightLimits;


extern const ASN1CType asn1_type_EU_AxleWeightLimits[];

typedef struct EU_PassengerCapacity {
  EU_Int1 numberOfSeats;
  EU_Int1 numberOfStandingPlaces;
} EU_PassengerCapacity;


extern const ASN1CType asn1_type_EU_PassengerCapacity[];

typedef enum EU_UnitType {
  EU_UnitType_mg_km,
  EU_UnitType_mg_kWh,
  EU_UnitType__dummy = 2147483647,
} EU_UnitType;

extern const ASN1CType asn1_type_EU_UnitType[];

typedef struct EU_ExhaustEmissionValues {
  EU_UnitType unitType;
  int emissionCO;
  EU_Int2 emissionHC;
  EU_Int2 emissionNOX;
  EU_Int2 emissionHCNOX;
} EU_ExhaustEmissionValues;


extern const ASN1CType asn1_type_EU_ExhaustEmissionValues[];

typedef struct EU_DieselEmissionValues_1 {
  EU_UnitType unitType;
  int value;
} EU_DieselEmissionValues_1;


extern const ASN1CType asn1_type_EU_DieselEmissionValues_1[];

typedef struct EU_DieselEmissionValues {
  EU_DieselEmissionValues_1 particulate;
  EU_Int2 absorptionCoeff;
} EU_DieselEmissionValues;


extern const ASN1CType asn1_type_EU_DieselEmissionValues[];

typedef struct EU_SoundLevel {
  EU_Int1 soundstationary;
  EU_Int1 sounddriveby;
} EU_SoundLevel;


extern const ASN1CType asn1_type_EU_SoundLevel[];

typedef enum {
  EU_VehicleCharacteristicsRanges_1_numberOfAxles,
  EU_VehicleCharacteristicsRanges_1_vehicleDimensions,
  EU_VehicleCharacteristicsRanges_1_vehicleWeightLimits,
  EU_VehicleCharacteristicsRanges_1_axleWeightLimits,
  EU_VehicleCharacteristicsRanges_1_passengerCapacity,
  EU_VehicleCharacteristicsRanges_1_exhaustEmissionValues,
  EU_VehicleCharacteristicsRanges_1_dieselEmissionValues,
  EU_VehicleCharacteristicsRanges_1_soundLevel,
  EU_VehicleCharacteristicsRanges_1__dummy = 2147483647,
} EU_VehicleCharacteristicsRanges_1_choice;

typedef struct EU_VehicleCharacteristicsRanges_1 {
  EU_VehicleCharacteristicsRanges_1_choice choice;
  union {
    int numberOfAxles;
    EU_VehicleDimensions vehicleDimensions;
    EU_VehicleWeightLimits vehicleWeightLimits;
    EU_AxleWeightLimits axleWeightLimits;
    EU_PassengerCapacity passengerCapacity;
    EU_ExhaustEmissionValues exhaustEmissionValues;
    EU_DieselEmissionValues dieselEmissionValues;
    EU_SoundLevel soundLevel;
  } u;
} EU_VehicleCharacteristicsRanges_1;

extern const ASN1CType asn1_type_EU_VehicleCharacteristicsRanges_1[];

typedef struct EU_VehicleCharacteristicsRanges {
  EU_ComparisonOperator comparisonOperator;
  EU_VehicleCharacteristicsRanges_1 limits;
} EU_VehicleCharacteristicsRanges;


extern const ASN1CType asn1_type_EU_VehicleCharacteristicsRanges[];

typedef struct EU_VehicleCharacteristicsRangesList {
  EU_VehicleCharacteristicsRanges *tab;
  size_t count;
} EU_VehicleCharacteristicsRangesList;

extern const ASN1CType asn1_type_EU_VehicleCharacteristicsRangesList[];

typedef struct EU_TractorCharacteristics {
  BOOL equalTo_option;
  EU_VehicleCharacteristicsFixValuesList equalTo;
  BOOL notEqualTo_option;
  EU_VehicleCharacteristicsFixValuesList notEqualTo;
  BOOL ranges_option;
  EU_VehicleCharacteristicsRangesList ranges;
} EU_TractorCharacteristics;


extern const ASN1CType asn1_type_EU_TractorCharacteristics[];

typedef struct EU_TrailerCharacteristicsFixValuesList {
  EU_VehicleCharacteristicsFixValues *tab;
  size_t count;
} EU_TrailerCharacteristicsFixValuesList;

extern const ASN1CType asn1_type_EU_TrailerCharacteristicsFixValuesList[];

typedef struct EU_TrailerCharacteristicsRangesList {
  EU_VehicleCharacteristicsRanges *tab;
  size_t count;
} EU_TrailerCharacteristicsRangesList;

extern const ASN1CType asn1_type_EU_TrailerCharacteristicsRangesList[];

typedef struct EU_TrailerCharacteristics {
  BOOL equalTo_option;
  EU_TrailerCharacteristicsFixValuesList equalTo;
  BOOL notEqualTo_option;
  EU_TrailerCharacteristicsFixValuesList notEqualTo;
  BOOL ranges_option;
  EU_TrailerCharacteristicsRangesList ranges;
} EU_TrailerCharacteristics;


extern const ASN1CType asn1_type_EU_TrailerCharacteristics[];

typedef struct EU_TrailerCharacteristicsList {
  EU_TrailerCharacteristics *tab;
  size_t count;
} EU_TrailerCharacteristicsList;

extern const ASN1CType asn1_type_EU_TrailerCharacteristicsList[];

typedef EU_TractorCharacteristics EU_TrainCharacteristics;

extern const ASN1CType asn1_type_EU_TrainCharacteristics[];

typedef struct EU_CompleteVehicleCharacteristics {
  BOOL tractor_option;
  EU_TractorCharacteristics tractor;
  BOOL trailer_option;
  EU_TrailerCharacteristicsList trailer;
  BOOL train_option;
  EU_TrainCharacteristics train;
} EU_CompleteVehicleCharacteristics;


extern const ASN1CType asn1_type_EU_CompleteVehicleCharacteristics[];

typedef struct EU_VehicleCharacteristicsList {
  EU_CompleteVehicleCharacteristics *tab;
  size_t count;
} EU_VehicleCharacteristicsList;

extern const ASN1CType asn1_type_EU_VehicleCharacteristicsList[];

typedef int EU_DriverCharacteristics;

enum {
  EU_DriverCharacteristics_unexperiencedDrivers = 0,
  EU_DriverCharacteristics_experiencedDrivers = 1,
  EU_DriverCharacteristics_rfu1 = 2,
  EU_DriverCharacteristics_rfu2 = 3,
};

extern const ASN1CType asn1_type_EU_DriverCharacteristics[];

typedef int EU_VcClass;

enum {
  EU_VcClass_classA = 0,
  EU_VcClass_classB = 1,
  EU_VcClass_classC = 2,
  EU_VcClass_classD = 3,
  EU_VcClass_classE = 4,
  EU_VcClass_classF = 5,
  EU_VcClass_classG = 6,
  EU_VcClass_classH = 7,
};

extern const ASN1CType asn1_type_EU_VcClass[];

typedef int EU_VcOption;

enum {
  EU_VcOption_none = 0,
  EU_VcOption_a = 1,
  EU_VcOption_b = 2,
  EU_VcOption_c = 3,
  EU_VcOption_d = 4,
  EU_VcOption_e = 5,
  EU_VcOption_f = 6,
  EU_VcOption_g = 7,
};

extern const ASN1CType asn1_type_EU_VcOption[];

typedef struct EU_InternationalSign_applicablePeriod_1 {
  int yearRangeStartYear;
  int yearRangeEndYear;
} EU_InternationalSign_applicablePeriod_1;


extern const ASN1CType asn1_type_EU_InternationalSign_applicablePeriod_1[];

typedef struct EU_MonthDay {
  int month;
  int day;
} EU_MonthDay;


extern const ASN1CType asn1_type_EU_MonthDay[];

typedef struct EU_InternationalSign_applicablePeriod_2 {
  EU_MonthDay dateRangeStartMonthDay;
  EU_MonthDay dateRangeEndMonthDay;
} EU_InternationalSign_applicablePeriod_2;


extern const ASN1CType asn1_type_EU_InternationalSign_applicablePeriod_2[];

typedef ASN1BitString EU_RepeatingPeriodDayTypes;

extern const ASN1CType asn1_type_EU_RepeatingPeriodDayTypes[];

typedef struct EU_HoursMinutes {
  int hours;
  int mins;
} EU_HoursMinutes;


extern const ASN1CType asn1_type_EU_HoursMinutes[];

typedef struct EU_InternationalSign_applicablePeriod_3 {
  EU_HoursMinutes timeRangeStartTime;
  EU_HoursMinutes timeRangeEndTime;
} EU_InternationalSign_applicablePeriod_3;


extern const ASN1CType asn1_type_EU_InternationalSign_applicablePeriod_3[];

typedef ASN1BitString EU_DayOfWeek;

extern const ASN1CType asn1_type_EU_DayOfWeek[];

typedef struct EU_InternationalSign_applicablePeriod {
  BOOL year_option;
  EU_InternationalSign_applicablePeriod_1 year;
  BOOL month_day_option;
  EU_InternationalSign_applicablePeriod_2 month_day;
  BOOL repeatingPeriodDayTypes_option;
  EU_RepeatingPeriodDayTypes repeatingPeriodDayTypes;
  BOOL hourMinutes_option;
  EU_InternationalSign_applicablePeriod_3 hourMinutes;
  BOOL dateRangeOfWeek_option;
  EU_DayOfWeek dateRangeOfWeek;
  BOOL durationHourMinute_option;
  EU_HoursMinutes durationHourMinute;
} EU_InternationalSign_applicablePeriod;


extern const ASN1CType asn1_type_EU_InternationalSign_applicablePeriod[];

typedef struct EU_ValidityPeriods {
  EU_InternationalSign_applicablePeriod *tab;
  size_t count;
} EU_ValidityPeriods;

extern const ASN1CType asn1_type_EU_ValidityPeriods[];

typedef int EU_RSCUnit;

enum {
  EU_RSCUnit_kmperh = 0,
  EU_RSCUnit_milesperh = 1,
  EU_RSCUnit_kilometer = 2,
  EU_RSCUnit_meter = 3,
  EU_RSCUnit_decimeter = 4,
  EU_RSCUnit_centimeter = 5,
  EU_RSCUnit_mile = 6,
  EU_RSCUnit_yard = 7,
  EU_RSCUnit_foot = 8,
  EU_RSCUnit_minutesOfTime = 9,
  EU_RSCUnit_tonnes = 10,
  EU_RSCUnit_hundredkg = 11,
  EU_RSCUnit_pound = 12,
  EU_RSCUnit_rateOfIncline = 13,
};

extern const ASN1CType asn1_type_EU_RSCUnit[];

typedef struct EU_VcCode {
  EU_VcClass roadSignClass;
  int roadSignCode;
  EU_VcOption vcOption;
  BOOL validity_option;
  EU_ValidityPeriods validity;
  BOOL value_option;
  int value;
  BOOL unit_option;
  EU_RSCUnit unit;
} EU_VcCode;


extern const ASN1CType asn1_type_EU_VcCode[];

typedef enum EU_ISO14823Code_1 {
  EU_ISO14823Code_1_dangerWarning,
  EU_ISO14823Code_1_regulatory,
  EU_ISO14823Code_1_informative,
  EU_ISO14823Code_1__dummy = 2147483647,
} EU_ISO14823Code_1;

extern const ASN1CType asn1_type_EU_ISO14823Code_1[];

typedef enum EU_ISO14823Code_2 {
  EU_ISO14823Code_2_publicFacilities,
  EU_ISO14823Code_2__dummy = 2147483647,
} EU_ISO14823Code_2;

extern const ASN1CType asn1_type_EU_ISO14823Code_2[];

typedef enum EU_ISO14823Code_3 {
  EU_ISO14823Code_3_ambientCondition,
  EU_ISO14823Code_3_roadCondition,
  EU_ISO14823Code_3__dummy = 2147483647,
} EU_ISO14823Code_3;

extern const ASN1CType asn1_type_EU_ISO14823Code_3[];

typedef enum {
  EU_ISO14823Code_4_trafficSignPictogram,
  EU_ISO14823Code_4_publicFacilitiesPictogram,
  EU_ISO14823Code_4_ambientOrRoadConditionPictogram,
  EU_ISO14823Code_4__dummy = 2147483647,
} EU_ISO14823Code_4_choice;

typedef struct EU_ISO14823Code_4 {
  EU_ISO14823Code_4_choice choice;
  union {
    EU_ISO14823Code_1 trafficSignPictogram;
    EU_ISO14823Code_2 publicFacilitiesPictogram;
    EU_ISO14823Code_3 ambientOrRoadConditionPictogram;
  } u;
} EU_ISO14823Code_4;

extern const ASN1CType asn1_type_EU_ISO14823Code_4[];

typedef struct EU_ISO14823Code_5 {
  int nature;
  int serialNumber;
} EU_ISO14823Code_5;


extern const ASN1CType asn1_type_EU_ISO14823Code_5[];

typedef struct EU_ISO14823Code_6 {
  BOOL countryCode_option;
  ASN1String countryCode;
  EU_ISO14823Code_4 serviceCategoryCode;
  EU_ISO14823Code_5 pictogramCategoryCode;
} EU_ISO14823Code_6;


extern const ASN1CType asn1_type_EU_ISO14823Code_6[];

typedef EU_InternationalSign_applicablePeriod EU_InternationalSign_exemptedApplicablePeriod;

extern const ASN1CType asn1_type_EU_InternationalSign_exemptedApplicablePeriod[];

typedef int EU_InternationalSign_directionalFlowOfLane;

enum {
  EU_InternationalSign_directionalFlowOfLane_sDL = 1,
  EU_InternationalSign_directionalFlowOfLane_sLT = 2,
  EU_InternationalSign_directionalFlowOfLane_sRT = 3,
  EU_InternationalSign_directionalFlowOfLane_lTO = 4,
  EU_InternationalSign_directionalFlowOfLane_rTO = 5,
  EU_InternationalSign_directionalFlowOfLane_cLL = 6,
  EU_InternationalSign_directionalFlowOfLane_cRI = 7,
  EU_InternationalSign_directionalFlowOfLane_oVL = 8,
};

extern const ASN1CType asn1_type_EU_InternationalSign_directionalFlowOfLane[];

typedef int EU_Code_Units_2;

enum {
  EU_Code_Units_2_kmperh = 0,
  EU_Code_Units_2_milesperh = 1,
  EU_Code_Units_2_kilometre = 2,
  EU_Code_Units_2_metre = 3,
  EU_Code_Units_2_decimetre = 4,
  EU_Code_Units_2_centimetre = 5,
  EU_Code_Units_2_mile = 6,
  EU_Code_Units_2_yard = 7,
  EU_Code_Units_2_foot = 8,
  EU_Code_Units_2_minutesOfTime = 9,
  EU_Code_Units_2_tonnes = 10,
  EU_Code_Units_2_hundredkg = 11,
  EU_Code_Units_2_pound = 12,
  EU_Code_Units_2_rateOfIncline = 13,
  EU_Code_Units_2_durationinminutes = 14,
};

extern const ASN1CType asn1_type_EU_Code_Units_2[];

typedef struct EU_Distance {
  int value;
  EU_Code_Units_2 unit;
} EU_Distance;


extern const ASN1CType asn1_type_EU_Distance[];

typedef int EU_Code_Units_4;

enum {
  EU_Code_Units_4_kmperh = 0,
  EU_Code_Units_4_milesperh = 1,
  EU_Code_Units_4_kilometre = 2,
  EU_Code_Units_4_metre = 3,
  EU_Code_Units_4_decimetre = 4,
  EU_Code_Units_4_centimetre = 5,
  EU_Code_Units_4_mile = 6,
  EU_Code_Units_4_yard = 7,
  EU_Code_Units_4_foot = 8,
  EU_Code_Units_4_minutesOfTime = 9,
  EU_Code_Units_4_tonnes = 10,
  EU_Code_Units_4_hundredkg = 11,
  EU_Code_Units_4_pound = 12,
  EU_Code_Units_4_rateOfIncline = 13,
  EU_Code_Units_4_durationinminutes = 14,
};

extern const ASN1CType asn1_type_EU_Code_Units_4[];

typedef struct EU_Weight {
  int value;
  EU_Code_Units_4 unit;
} EU_Weight;


extern const ASN1CType asn1_type_EU_Weight[];

typedef struct EU_InternationalSign_applicableVehicleDimensions {
  BOOL vehicleHeight_option;
  EU_Distance vehicleHeight;
  BOOL vehicleWidth_option;
  EU_Distance vehicleWidth;
  BOOL vehicleLength_option;
  EU_Distance vehicleLength;
  BOOL vehicleWeight_option;
  EU_Weight vehicleWeight;
} EU_InternationalSign_applicableVehicleDimensions;


extern const ASN1CType asn1_type_EU_InternationalSign_applicableVehicleDimensions[];

typedef int EU_Code_Units_1;

enum {
  EU_Code_Units_1_kmperh = 0,
  EU_Code_Units_1_milesperh = 1,
  EU_Code_Units_1_kilometre = 2,
  EU_Code_Units_1_metre = 3,
  EU_Code_Units_1_decimetre = 4,
  EU_Code_Units_1_centimetre = 5,
  EU_Code_Units_1_mile = 6,
  EU_Code_Units_1_yard = 7,
  EU_Code_Units_1_foot = 8,
  EU_Code_Units_1_minutesOfTime = 9,
  EU_Code_Units_1_tonnes = 10,
  EU_Code_Units_1_hundredkg = 11,
  EU_Code_Units_1_pound = 12,
  EU_Code_Units_1_rateOfIncline = 13,
  EU_Code_Units_1_durationinminutes = 14,
};

extern const ASN1CType asn1_type_EU_Code_Units_1[];

typedef struct EU_InternationalSign_speedLimits {
  BOOL speedLimitMax_option;
  int speedLimitMax;
  BOOL speedLimitMin_option;
  int speedLimitMin;
  EU_Code_Units_1 unit;
} EU_InternationalSign_speedLimits;


extern const ASN1CType asn1_type_EU_InternationalSign_speedLimits[];

typedef int EU_InternationalSign_rateOfIncline;

extern const ASN1CType asn1_type_EU_InternationalSign_rateOfIncline[];

typedef EU_Distance EU_InternationalSign_distanceBetweenVehicles;

extern const ASN1CType asn1_type_EU_InternationalSign_distanceBetweenVehicles[];

typedef int EU_DestinationType;

enum {
  EU_DestinationType_none = 0,
  EU_DestinationType_importantArea = 1,
  EU_DestinationType_principalArea = 2,
  EU_DestinationType_generalArea = 3,
  EU_DestinationType_wellKnownPoint = 4,
  EU_DestinationType_country = 5,
  EU_DestinationType_city = 6,
  EU_DestinationType_street = 7,
  EU_DestinationType_industrialArea = 8,
  EU_DestinationType_historicArea = 9,
  EU_DestinationType_touristicArea = 10,
  EU_DestinationType_culturalArea = 11,
  EU_DestinationType_touristicRoute = 12,
  EU_DestinationType_recommendedRoute = 13,
  EU_DestinationType_touristicAttraction = 14,
  EU_DestinationType_geographicArea = 15,
};

extern const ASN1CType asn1_type_EU_DestinationType[];

typedef enum EU_GddStructure_1 {
  EU_GddStructure_1_dangerWarning,
  EU_GddStructure_1_regulatory,
  EU_GddStructure_1_informative,
  EU_GddStructure_1__dummy = 2147483647,
} EU_GddStructure_1;

extern const ASN1CType asn1_type_EU_GddStructure_1[];

typedef enum EU_GddStructure_2 {
  EU_GddStructure_2_publicFacilities,
  EU_GddStructure_2__dummy = 2147483647,
} EU_GddStructure_2;

extern const ASN1CType asn1_type_EU_GddStructure_2[];

typedef enum EU_GddStructure_3 {
  EU_GddStructure_3_ambientCondition,
  EU_GddStructure_3_roadCondition,
  EU_GddStructure_3__dummy = 2147483647,
} EU_GddStructure_3;

extern const ASN1CType asn1_type_EU_GddStructure_3[];

typedef enum {
  EU_GddStructure_4_trafficSignPictogram,
  EU_GddStructure_4_publicFacilitiesPictogram,
  EU_GddStructure_4_ambientOrRoadConditionPictogram,
  EU_GddStructure_4__dummy = 2147483647,
} EU_GddStructure_4_choice;

typedef struct EU_GddStructure_4 {
  EU_GddStructure_4_choice choice;
  union {
    EU_GddStructure_1 trafficSignPictogram;
    EU_GddStructure_2 publicFacilitiesPictogram;
    EU_GddStructure_3 ambientOrRoadConditionPictogram;
  } u;
} EU_GddStructure_4;

extern const ASN1CType asn1_type_EU_GddStructure_4[];

typedef struct EU_GddStructure_5 {
  int nature;
  int serialNumber;
} EU_GddStructure_5;


extern const ASN1CType asn1_type_EU_GddStructure_5[];

typedef struct EU_GddStructure_6 {
  BOOL countryCode_option;
  ASN1String countryCode;
  EU_GddStructure_4 serviceCategoryCode;
  EU_GddStructure_5 pictogramCategoryCode;
} EU_GddStructure_6;


extern const ASN1CType asn1_type_EU_GddStructure_6[];

typedef struct EU_InternationalSign_section {
  BOOL startingPointLength_option;
  EU_Distance startingPointLength;
  BOOL continuityLength_option;
  EU_Distance continuityLength;
} EU_InternationalSign_section;


extern const ASN1CType asn1_type_EU_InternationalSign_section[];

typedef int EU_InternationalSign_numberOfLane;

extern const ASN1CType asn1_type_EU_InternationalSign_numberOfLane[];

typedef enum {
  EU_GddAttribute_dtm,
  EU_GddAttribute_edt,
  EU_GddAttribute_dfl,
  EU_GddAttribute_ved,
  EU_GddAttribute_spe,
  EU_GddAttribute_roi,
  EU_GddAttribute_dbv,
  EU_GddAttribute_ddd,
  EU_GddAttribute_set,
  EU_GddAttribute_nol,
  EU_GddAttribute__dummy = 2147483647,
} EU_GddAttribute_choice;

typedef struct EU_GddAttribute {
  EU_GddAttribute_choice choice;
  union {
    EU_InternationalSign_applicablePeriod dtm;
    EU_InternationalSign_exemptedApplicablePeriod edt;
    EU_InternationalSign_directionalFlowOfLane dfl;
    EU_InternationalSign_applicableVehicleDimensions ved;
    EU_InternationalSign_speedLimits spe;
    EU_InternationalSign_rateOfIncline roi;
    EU_InternationalSign_distanceBetweenVehicles dbv;
    struct EU_InternationalSign_destinationInformation * ddd;
    EU_InternationalSign_section set;
    EU_InternationalSign_numberOfLane nol;
  } u;
} EU_GddAttribute;

extern const ASN1CType asn1_type_EU_GddAttribute[];

typedef struct EU_GddAttributes {
  EU_GddAttribute *tab;
  size_t count;
} EU_GddAttributes;

extern const ASN1CType asn1_type_EU_GddAttributes[];

typedef struct EU_GddStructure {
  EU_GddStructure_6 pictogramCode;
  BOOL attributes_option;
  EU_GddAttributes attributes;
} EU_GddStructure;


extern const ASN1CType asn1_type_EU_GddStructure[];

typedef struct EU_DestinationPlace {
  EU_DestinationType destType;
  BOOL destRSCode_option;
  EU_GddStructure destRSCode;
  BOOL destBlob_option;
  ASN1String destBlob;
  BOOL placeNameIdentification_option;
  int placeNameIdentification;
  BOOL placeNameText_option;
  ASN1String placeNameText;
} EU_DestinationPlace;


extern const ASN1CType asn1_type_EU_DestinationPlace[];

typedef struct EU_DestinationPlaces {
  EU_DestinationPlace *tab;
  size_t count;
} EU_DestinationPlaces;

extern const ASN1CType asn1_type_EU_DestinationPlaces[];

typedef int EU_DestinationRoadType;

enum {
  EU_DestinationRoadType_none = 0,
  EU_DestinationRoadType_nationalHighway = 1,
  EU_DestinationRoadType_localHighway = 2,
  EU_DestinationRoadType_tollExpresswayMotorway = 3,
  EU_DestinationRoadType_internationalHighway = 4,
  EU_DestinationRoadType_highway = 5,
  EU_DestinationRoadType_expressway = 6,
  EU_DestinationRoadType_nationalRoad = 7,
  EU_DestinationRoadType_regionalProvincialRoad = 8,
  EU_DestinationRoadType_localRoad = 9,
  EU_DestinationRoadType_motorwayJunction = 10,
  EU_DestinationRoadType_diversion = 11,
  EU_DestinationRoadType_rfu1 = 12,
  EU_DestinationRoadType_rfu2 = 13,
  EU_DestinationRoadType_rfu3 = 14,
  EU_DestinationRoadType_rfu4 = 15,
};

extern const ASN1CType asn1_type_EU_DestinationRoadType[];

typedef struct EU_DestinationRoad {
  EU_DestinationRoadType derType;
  BOOL roadNumberIdentifier_option;
  int roadNumberIdentifier;
  BOOL roadNumberText_option;
  ASN1String roadNumberText;
} EU_DestinationRoad;


extern const ASN1CType asn1_type_EU_DestinationRoad[];

typedef struct EU_DestinationRoads {
  EU_DestinationRoad *tab;
  size_t count;
} EU_DestinationRoads;

extern const ASN1CType asn1_type_EU_DestinationRoads[];

typedef int EU_Code_Units_3;

enum {
  EU_Code_Units_3_kmperh = 0,
  EU_Code_Units_3_milesperh = 1,
  EU_Code_Units_3_kilometre = 2,
  EU_Code_Units_3_metre = 3,
  EU_Code_Units_3_decimetre = 4,
  EU_Code_Units_3_centimetre = 5,
  EU_Code_Units_3_mile = 6,
  EU_Code_Units_3_yard = 7,
  EU_Code_Units_3_foot = 8,
  EU_Code_Units_3_minutesOfTime = 9,
  EU_Code_Units_3_tonnes = 10,
  EU_Code_Units_3_hundredkg = 11,
  EU_Code_Units_3_pound = 12,
  EU_Code_Units_3_rateOfIncline = 13,
  EU_Code_Units_3_durationinminutes = 14,
};

extern const ASN1CType asn1_type_EU_Code_Units_3[];

typedef struct EU_DistanceOrDuration {
  int value;
  EU_Code_Units_3 unit;
} EU_DistanceOrDuration;


extern const ASN1CType asn1_type_EU_DistanceOrDuration[];

typedef struct EU_DDD_IO {
  int arrowDirection;
  BOOL destPlace_option;
  EU_DestinationPlaces destPlace;
  BOOL destRoad_option;
  EU_DestinationRoads destRoad;
  BOOL roadNumberIdentifier_option;
  int roadNumberIdentifier;
  BOOL streetName_option;
  int streetName;
  BOOL streetNameText_option;
  ASN1String streetNameText;
  BOOL distanceToDivergingPoint_option;
  EU_DistanceOrDuration distanceToDivergingPoint;
  BOOL distanceToDestinationPlace_option;
  EU_DistanceOrDuration distanceToDestinationPlace;
} EU_DDD_IO;


extern const ASN1CType asn1_type_EU_DDD_IO[];

typedef struct EU_DDD_IO_LIST {
  EU_DDD_IO *tab;
  size_t count;
} EU_DDD_IO_LIST;

extern const ASN1CType asn1_type_EU_DDD_IO_LIST[];

typedef struct EU_InternationalSign_destinationInformation {
  BOOL junctionDirection_option;
  int junctionDirection;
  BOOL roundaboutCwDirection_option;
  int roundaboutCwDirection;
  BOOL roundaboutCcwDirection_option;
  int roundaboutCcwDirection;
  EU_DDD_IO_LIST ioList;
} EU_InternationalSign_destinationInformation;


extern const ASN1CType asn1_type_EU_InternationalSign_destinationInformation[];

typedef enum {
  EU_ISO14823Attribute_dtm,
  EU_ISO14823Attribute_edt,
  EU_ISO14823Attribute_dfl,
  EU_ISO14823Attribute_ved,
  EU_ISO14823Attribute_spe,
  EU_ISO14823Attribute_roi,
  EU_ISO14823Attribute_dbv,
  EU_ISO14823Attribute_ddd,
  EU_ISO14823Attribute__dummy = 2147483647,
} EU_ISO14823Attribute_choice;

typedef struct EU_ISO14823Attribute {
  EU_ISO14823Attribute_choice choice;
  union {
    EU_InternationalSign_applicablePeriod dtm;
    EU_InternationalSign_exemptedApplicablePeriod edt;
    EU_InternationalSign_directionalFlowOfLane dfl;
    EU_InternationalSign_applicableVehicleDimensions ved;
    EU_InternationalSign_speedLimits spe;
    EU_InternationalSign_rateOfIncline roi;
    EU_InternationalSign_distanceBetweenVehicles dbv;
    EU_InternationalSign_destinationInformation ddd;
  } u;
} EU_ISO14823Attribute;

extern const ASN1CType asn1_type_EU_ISO14823Attribute[];

typedef struct EU_ISO14823Attributes {
  EU_ISO14823Attribute *tab;
  size_t count;
} EU_ISO14823Attributes;

extern const ASN1CType asn1_type_EU_ISO14823Attributes[];

typedef struct EU_ISO14823Code {
  EU_ISO14823Code_6 pictogramCode;
  BOOL attributes_option;
  EU_ISO14823Attributes attributes;
} EU_ISO14823Code;


extern const ASN1CType asn1_type_EU_ISO14823Code[];

typedef struct EU_AnyCatalogue {
  EU_Provider owner;
  int version;
  int pictogramCode;
  BOOL value_option;
  int value;
  BOOL unit_option;
  EU_RSCUnit unit;
  BOOL attributes_option;
  EU_ISO14823Attributes attributes;
} EU_AnyCatalogue;


extern const ASN1CType asn1_type_EU_AnyCatalogue[];

typedef enum {
  EU_RSCode_1_viennaConvention,
  EU_RSCode_1_iso14823,
  EU_RSCode_1_itisCodes,
  EU_RSCode_1_anyCatalogue,
  EU_RSCode_1__dummy = 2147483647,
} EU_RSCode_1_choice;

typedef struct EU_RSCode_1 {
  EU_RSCode_1_choice choice;
  union {
    EU_VcCode viennaConvention;
    EU_ISO14823Code iso14823;
    int itisCodes;
    EU_AnyCatalogue anyCatalogue;
  } u;
} EU_RSCode_1;

extern const ASN1CType asn1_type_EU_RSCode_1[];

typedef struct EU_RSCode {
  BOOL layoutComponentId_option;
  int layoutComponentId;
  EU_RSCode_1 code;
} EU_RSCode;


extern const ASN1CType asn1_type_EU_RSCode[];

typedef struct EU_RoadSignCodes {
  EU_RSCode *tab;
  size_t count;
} EU_RoadSignCodes;

extern const ASN1CType asn1_type_EU_RoadSignCodes[];

typedef struct EU_Text {
  BOOL layoutComponentId_option;
  int layoutComponentId;
  ASN1BitString language;
  ASN1String textContent;
} EU_Text;


extern const ASN1CType asn1_type_EU_Text[];

typedef struct EU_ConstraintTextLines1 {
  EU_Text *tab;
  size_t count;
} EU_ConstraintTextLines1;

extern const ASN1CType asn1_type_EU_ConstraintTextLines1[];

typedef struct EU_GicPart {
  BOOL detectionZoneIds_option;
  EU_ZoneIds detectionZoneIds;
  BOOL its_Rrid_option;
  EU_VarLengthNumber its_Rrid;
  BOOL relevanceZoneIds_option;
  EU_ZoneIds relevanceZoneIds;
  BOOL direction_option;
  EU_Direction direction;
  BOOL driverAwarenessZoneIds_option;
  EU_ZoneIds driverAwarenessZoneIds;
  BOOL minimumAwarenessTime_option;
  int minimumAwarenessTime;
  BOOL applicableLanes_option;
  EU_LanePositions applicableLanes;
  EU_IviType iviType;
  BOOL iviPurpose_option;
  EU_IviPurpose iviPurpose;
  BOOL laneStatus_option;
  EU_LaneStatus laneStatus;
  BOOL vehicleCharacteristics_option;
  EU_VehicleCharacteristicsList vehicleCharacteristics;
  BOOL driverCharacteristics_option;
  EU_DriverCharacteristics driverCharacteristics;
  BOOL layoutId_option;
  int layoutId;
  BOOL preStoredlayoutId_option;
  int preStoredlayoutId;
  EU_RoadSignCodes roadSignCodes;
  BOOL extraText_option;
  EU_ConstraintTextLines1 extraText;
} EU_GicPart;


extern const ASN1CType asn1_type_EU_GicPart[];

typedef struct EU_GeneralIviContainer {
  EU_GicPart *tab;
  size_t count;
} EU_GeneralIviContainer;

extern const ASN1CType asn1_type_EU_GeneralIviContainer[];

typedef int EU_LaneType;

enum {
  EU_LaneType_traffic = 0,
  EU_LaneType_through = 1,
  EU_LaneType_reversible = 2,
  EU_LaneType_acceleration = 3,
  EU_LaneType_deceleration = 4,
  EU_LaneType_leftHandTurning = 5,
  EU_LaneType_rightHandTurning = 6,
  EU_LaneType_dedicatedVehicle = 7,
  EU_LaneType_bus = 8,
  EU_LaneType_taxi = 9,
  EU_LaneType_hov = 10,
  EU_LaneType_hot = 11,
  EU_LaneType_pedestrian = 12,
  EU_LaneType_bikeLane = 13,
  EU_LaneType_median = 14,
  EU_LaneType_striping = 15,
  EU_LaneType_trackedVehicle = 16,
  EU_LaneType_parking = 17,
  EU_LaneType_emergency = 18,
  EU_LaneType_verge = 19,
  EU_LaneType_minimumRiskManoeuvre = 20,
};

extern const ASN1CType asn1_type_EU_LaneType[];

typedef int EU_DefinitionAccuracy;

enum {
  EU_DefinitionAccuracy_oneCm = 0,
  EU_DefinitionAccuracy_twoCm = 1,
  EU_DefinitionAccuracy_fiveCm = 2,
  EU_DefinitionAccuracy_tenCm = 3,
  EU_DefinitionAccuracy_twentyCm = 4,
  EU_DefinitionAccuracy_fiftyCm = 5,
  EU_DefinitionAccuracy_oneMeter = 6,
  EU_DefinitionAccuracy_unavailable = 7,
};

extern const ASN1CType asn1_type_EU_DefinitionAccuracy[];

typedef BOOL EU_LaneMarkingStatus;

extern const ASN1CType asn1_type_EU_LaneMarkingStatus[];

typedef int EU_MarkingColour;

enum {
  EU_MarkingColour_white = 0,
  EU_MarkingColour_yellow = 1,
  EU_MarkingColour_orange = 2,
  EU_MarkingColour_red = 3,
  EU_MarkingColour_blue = 4,
  EU_MarkingColour_unavailable = 7,
};

extern const ASN1CType asn1_type_EU_MarkingColour[];

typedef int EU_LaneDelimitation;

enum {
  EU_LaneDelimitation_noDelimitation = 0,
  EU_LaneDelimitation_lowLaneSeparator = 1,
  EU_LaneDelimitation_highLaneSeparator = 2,
  EU_LaneDelimitation_wall = 3,
  EU_LaneDelimitation_curb = 4,
  EU_LaneDelimitation_unpaved = 5,
  EU_LaneDelimitation_guardrail = 6,
};

extern const ASN1CType asn1_type_EU_LaneDelimitation[];

typedef struct EU_LaneCharacteristics {
  EU_DefinitionAccuracy zoneDefinitionAccuracy;
  EU_LaneMarkingStatus existinglaneMarkingStatus;
  EU_MarkingColour newlaneMarkingColour;
  EU_LaneDelimitation laneDelimitationLeft;
  EU_LaneDelimitation laneDelimitationRight;
  EU_Zid mergingWith;
} EU_LaneCharacteristics;


extern const ASN1CType asn1_type_EU_LaneCharacteristics[];

typedef int EU_FrictionCoefficient;

extern const ASN1CType asn1_type_EU_FrictionCoefficient[];

typedef int EU_MaterialType;

enum {
  EU_MaterialType_asphalt = 0,
  EU_MaterialType_concrete = 1,
  EU_MaterialType_cobblestone = 2,
  EU_MaterialType_gravel = 3,
  EU_MaterialType_unavailable = 7,
};

extern const ASN1CType asn1_type_EU_MaterialType[];

typedef int EU_WearLevel;

enum {
  EU_WearLevel_New = 0,
  EU_WearLevel_good = 1,
  EU_WearLevel_bad = 2,
  EU_WearLevel_hasPotholes = 3,
  EU_WearLevel_unavailable = 7,
};

extern const ASN1CType asn1_type_EU_WearLevel[];

typedef int EU_BankingAngle;

extern const ASN1CType asn1_type_EU_BankingAngle[];

typedef struct EU_RoadSurfaceStaticCharacteristics {
  EU_FrictionCoefficient frictionCoefficient;
  EU_MaterialType material;
  EU_WearLevel wear;
  EU_BankingAngle avBankingAngle;
} EU_RoadSurfaceStaticCharacteristics;


extern const ASN1CType asn1_type_EU_RoadSurfaceStaticCharacteristics[];

typedef int EU_Condition;

enum {
  EU_Condition_dry = 0,
  EU_Condition_moist = 1,
  EU_Condition_wet = 2,
  EU_Condition_standingWater = 3,
  EU_Condition_frost = 4,
  EU_Condition_ice = 5,
  EU_Condition_snow = 6,
  EU_Condition_slush = 7,
  EU_Condition_unvailable = 8,
};

extern const ASN1CType asn1_type_EU_Condition[];

typedef int EU_IviTemperature;

extern const ASN1CType asn1_type_EU_IviTemperature[];

typedef int EU_Depth;

extern const ASN1CType asn1_type_EU_Depth[];

typedef int EU_TreatmentType;

enum {
  EU_TreatmentType_no = 0,
  EU_TreatmentType_antiskid = 1,
  EU_TreatmentType_anti_icing = 2,
  EU_TreatmentType_de_icing = 3,
  EU_TreatmentType_unavailable = 7,
};

extern const ASN1CType asn1_type_EU_TreatmentType[];

typedef struct EU_RoadSurfaceDynamicCharacteristics {
  EU_Condition condition;
  EU_IviTemperature temperature;
  EU_Depth iceOrWaterDepth;
  EU_TreatmentType treatment;
} EU_RoadSurfaceDynamicCharacteristics;


extern const ASN1CType asn1_type_EU_RoadSurfaceDynamicCharacteristics[];

typedef struct EU_LaneInformation {
  EU_LanePosition laneNumber;
  EU_Direction direction;
  BOOL validity_option;
  EU_InternationalSign_applicablePeriod validity;
  EU_LaneType laneType;
  BOOL laneTypeQualifier_option;
  EU_CompleteVehicleCharacteristics laneTypeQualifier;
  EU_LaneStatus laneStatus;
  BOOL laneWidth_option;
  EU_IviLaneWidth laneWidth;
  BOOL detectionZoneIds_option;
  EU_ZoneIds detectionZoneIds;
  BOOL relevanceZoneIds_option;
  EU_ZoneIds relevanceZoneIds;
  BOOL laneCharacteristics_option;
  EU_LaneCharacteristics laneCharacteristics;
  BOOL laneSurfaceStaticCharacteristics_option;
  EU_RoadSurfaceStaticCharacteristics laneSurfaceStaticCharacteristics;
  BOOL laneSurfaceDynamicCharacteristics_option;
  EU_RoadSurfaceDynamicCharacteristics laneSurfaceDynamicCharacteristics;
} EU_LaneInformation;


extern const ASN1CType asn1_type_EU_LaneInformation[];

typedef struct EU_LaneConfiguration {
  EU_LaneInformation *tab;
  size_t count;
} EU_LaneConfiguration;

extern const ASN1CType asn1_type_EU_LaneConfiguration[];

typedef struct EU_RccPart {
  EU_ZoneIds relevanceZoneIds;
  EU_RoadType roadType;
  EU_LaneConfiguration laneConfiguration;
} EU_RccPart;


extern const ASN1CType asn1_type_EU_RccPart[];

typedef struct EU_RoadConfigurationContainer {
  EU_RccPart *tab;
  size_t count;
} EU_RoadConfigurationContainer;

extern const ASN1CType asn1_type_EU_RoadConfigurationContainer[];

typedef struct EU_TextLines {
  EU_Text *tab;
  size_t count;
} EU_TextLines;

extern const ASN1CType asn1_type_EU_TextLines[];

typedef struct EU_TcPart {
  BOOL detectionZoneIds_option;
  EU_ZoneIds detectionZoneIds;
  EU_ZoneIds relevanceZoneIds;
  BOOL direction_option;
  EU_Direction direction;
  BOOL driverAwarenessZoneIds_option;
  EU_ZoneIds driverAwarenessZoneIds;
  BOOL minimumAwarenessTime_option;
  int minimumAwarenessTime;
  BOOL applicableLanes_option;
  EU_LanePositions applicableLanes;
  BOOL layoutId_option;
  int layoutId;
  BOOL preStoredlayoutId_option;
  int preStoredlayoutId;
  BOOL text_option;
  EU_TextLines text;
  ASN1String data;
  BOOL iviType_option;
  EU_IviType iviType;
  BOOL laneStatus_option;
  EU_LaneStatus laneStatus;
  BOOL vehicleCharacteristics_option;
  EU_VehicleCharacteristicsList vehicleCharacteristics;
} EU_TcPart;


extern const ASN1CType asn1_type_EU_TcPart[];

typedef struct EU_TextContainer {
  EU_TcPart *tab;
  size_t count;
} EU_TextContainer;

extern const ASN1CType asn1_type_EU_TextContainer[];

typedef struct EU_LayoutComponent {
  int layoutComponentId;
  int height;
  int width;
  int x;
  int y;
  int textScripting;
} EU_LayoutComponent;


extern const ASN1CType asn1_type_EU_LayoutComponent[];

typedef struct EU_LayoutComponents {
  EU_LayoutComponent *tab;
  size_t count;
} EU_LayoutComponents;

extern const ASN1CType asn1_type_EU_LayoutComponents[];

typedef struct EU_LayoutContainer {
  int layoutId;
  BOOL height_option;
  int height;
  BOOL width_option;
  int width;
  EU_LayoutComponents layoutComponents;
} EU_LayoutContainer;


extern const ASN1CType asn1_type_EU_LayoutContainer[];

typedef int EU_PriorityLevel;

extern const ASN1CType asn1_type_EU_PriorityLevel[];

typedef int EU_SaeAutomationLevel;

extern const ASN1CType asn1_type_EU_SaeAutomationLevel[];

typedef struct EU_SaeAutomationLevels {
  EU_SaeAutomationLevel *tab;
  size_t count;
} EU_SaeAutomationLevels;

extern const ASN1CType asn1_type_EU_SaeAutomationLevels[];

typedef int EU_GapBetweenVehicles;

extern const ASN1CType asn1_type_EU_GapBetweenVehicles[];

typedef struct EU_ConstraintTextLines2 {
  EU_Text *tab;
  size_t count;
} EU_ConstraintTextLines2;

extern const ASN1CType asn1_type_EU_ConstraintTextLines2[];

typedef struct EU_AutomatedVehicleRule {
  EU_PriorityLevel priority;
  EU_SaeAutomationLevels allowedSaeAutomationLevels;
  BOOL minGapBetweenVehicles_option;
  EU_GapBetweenVehicles minGapBetweenVehicles;
  BOOL recGapBetweenVehicles_option;
  EU_GapBetweenVehicles recGapBetweenVehicles;
  BOOL automatedVehicleMaxSpeedLimit_option;
  EU_SpeedValue automatedVehicleMaxSpeedLimit;
  BOOL automatedVehicleMinSpeedLimit_option;
  EU_SpeedValue automatedVehicleMinSpeedLimit;
  BOOL automatedVehicleSpeedRecommendation_option;
  EU_SpeedValue automatedVehicleSpeedRecommendation;
  BOOL roadSignCodes_option;
  EU_RoadSignCodes roadSignCodes;
  BOOL extraText_option;
  EU_ConstraintTextLines2 extraText;
} EU_AutomatedVehicleRule;


extern const ASN1CType asn1_type_EU_AutomatedVehicleRule[];

typedef struct EU_AutomatedVehicleRules {
  EU_AutomatedVehicleRule *tab;
  size_t count;
} EU_AutomatedVehicleRules;

extern const ASN1CType asn1_type_EU_AutomatedVehicleRules[];

typedef int EU_MaxNoOfVehicles;

extern const ASN1CType asn1_type_EU_MaxNoOfVehicles[];

typedef int EU_MaxLenghtOfPlatoon;

extern const ASN1CType asn1_type_EU_MaxLenghtOfPlatoon[];

typedef struct EU_PlatooningRule {
  EU_PriorityLevel priority;
  EU_SaeAutomationLevels allowedSaeAutomationLevels;
  BOOL maxNoOfVehicles_option;
  EU_MaxNoOfVehicles maxNoOfVehicles;
  BOOL maxLenghtOfPlatoon_option;
  EU_MaxLenghtOfPlatoon maxLenghtOfPlatoon;
  BOOL minGapBetweenVehicles_option;
  EU_GapBetweenVehicles minGapBetweenVehicles;
  BOOL platoonMaxSpeedLimit_option;
  EU_SpeedValue platoonMaxSpeedLimit;
  BOOL platoonMinSpeedLimit_option;
  EU_SpeedValue platoonMinSpeedLimit;
  BOOL platoonSpeedRecommendation_option;
  EU_SpeedValue platoonSpeedRecommendation;
  BOOL roadSignCodes_option;
  EU_RoadSignCodes roadSignCodes;
  BOOL extraText_option;
  EU_ConstraintTextLines2 extraText;
} EU_PlatooningRule;


extern const ASN1CType asn1_type_EU_PlatooningRule[];

typedef struct EU_PlatooningRules {
  EU_PlatooningRule *tab;
  size_t count;
} EU_PlatooningRules;

extern const ASN1CType asn1_type_EU_PlatooningRules[];

typedef struct EU_AvcPart {
  BOOL detectionZoneIds_option;
  EU_ZoneIds detectionZoneIds;
  EU_ZoneIds relevanceZoneIds;
  BOOL direction_option;
  EU_Direction direction;
  BOOL applicableLanes_option;
  EU_LanePositions applicableLanes;
  BOOL vehicleCharacteristics_option;
  EU_VehicleCharacteristicsList vehicleCharacteristics;
  BOOL automatedVehicleRules_option;
  EU_AutomatedVehicleRules automatedVehicleRules;
  BOOL platooningRules_option;
  EU_PlatooningRules platooningRules;
} EU_AvcPart;


extern const ASN1CType asn1_type_EU_AvcPart[];

typedef struct EU_AutomatedVehicleContainer {
  EU_AvcPart *tab;
  size_t count;
} EU_AutomatedVehicleContainer;

extern const ASN1CType asn1_type_EU_AutomatedVehicleContainer[];

typedef enum {
  EU_MapReference_roadsegment,
  EU_MapReference_intersection,
  EU_MapReference__dummy = 2147483647,
} EU_MapReference_choice;

typedef struct EU_MapReference {
  EU_MapReference_choice choice;
  union {
    EU_RoadSegmentReferenceID roadsegment;
    EU_IntersectionReferenceID intersection;
  } u;
} EU_MapReference;

extern const ASN1CType asn1_type_EU_MapReference[];

typedef struct EU_LaneIds {
  EU_LaneID *tab;
  size_t count;
} EU_LaneIds;

extern const ASN1CType asn1_type_EU_LaneIds[];

typedef struct EU_MlcPart {
  EU_Zid zoneId;
  BOOL laneIds_option;
  EU_LaneIds laneIds;
} EU_MlcPart;


extern const ASN1CType asn1_type_EU_MlcPart[];

typedef struct EU_MlcParts {
  EU_MlcPart *tab;
  size_t count;
} EU_MlcParts;

extern const ASN1CType asn1_type_EU_MlcParts[];

typedef struct EU_MapLocationContainer {
  EU_MapReference reference;
  EU_MlcParts parts;
} EU_MapLocationContainer;


extern const ASN1CType asn1_type_EU_MapLocationContainer[];

typedef struct EU_RscPart {
  BOOL detectionZoneIds_option;
  EU_ZoneIds detectionZoneIds;
  EU_ZoneIds relevanceZoneIds;
  BOOL direction_option;
  EU_Direction direction;
  BOOL roadSurfaceStaticCharacteristics_option;
  EU_RoadSurfaceStaticCharacteristics roadSurfaceStaticCharacteristics;
  BOOL roadSurfaceDynamicCharacteristics_option;
  EU_RoadSurfaceDynamicCharacteristics roadSurfaceDynamicCharacteristics;
} EU_RscPart;


extern const ASN1CType asn1_type_EU_RscPart[];

typedef struct EU_RoadSurfaceContainer {
  EU_RscPart *tab;
  size_t count;
} EU_RoadSurfaceContainer;

extern const ASN1CType asn1_type_EU_RoadSurfaceContainer[];

typedef enum {
  EU_IviContainer_glc,
  EU_IviContainer_giv,
  EU_IviContainer_rcc,
  EU_IviContainer_tc,
  EU_IviContainer_lac,
  EU_IviContainer_avc,
  EU_IviContainer_mlc,
  EU_IviContainer_rsc,
  EU_IviContainer__dummy = 2147483647,
} EU_IviContainer_choice;

typedef struct EU_IviContainer {
  EU_IviContainer_choice choice;
  union {
    EU_GeographicLocationContainer glc;
    EU_GeneralIviContainer giv;
    EU_RoadConfigurationContainer rcc;
    EU_TextContainer tc;
    EU_LayoutContainer lac;
    EU_AutomatedVehicleContainer avc;
    EU_MapLocationContainer mlc;
    EU_RoadSurfaceContainer rsc;
  } u;
} EU_IviContainer;

extern const ASN1CType asn1_type_EU_IviContainer[];

typedef struct EU_IviContainers {
  EU_IviContainer *tab;
  size_t count;
} EU_IviContainers;

extern const ASN1CType asn1_type_EU_IviContainers[];

typedef struct EU_IviStructure {
  EU_IviManagementContainer mandatory;
  BOOL optional_option;
  EU_IviContainers optional;
} EU_IviStructure;


extern const ASN1CType asn1_type_EU_IviStructure[];

typedef struct EU_IVIM {
  EU_ItsPduHeader header;
  EU_IviStructure ivi;
} EU_IVIM;


extern const ASN1CType asn1_type_EU_IVIM[];

typedef int EU_RequestID;

extern const ASN1CType asn1_type_EU_RequestID[];

typedef enum EU_PriorityRequestType {
  EU_PriorityRequestType_priorityRequestTypeReserved,
  EU_PriorityRequestType_priorityRequest,
  EU_PriorityRequestType_priorityRequestUpdate,
  EU_PriorityRequestType_priorityCancellation,
  EU_PriorityRequestType__dummy = 2147483647,
} EU_PriorityRequestType;

extern const ASN1CType asn1_type_EU_PriorityRequestType[];

typedef enum {
  EU_IntersectionAccessPoint_lane,
  EU_IntersectionAccessPoint_approach,
  EU_IntersectionAccessPoint_connection,
  EU_IntersectionAccessPoint__dummy = 2147483647,
} EU_IntersectionAccessPoint_choice;

typedef struct EU_IntersectionAccessPoint {
  EU_IntersectionAccessPoint_choice choice;
  union {
    EU_LaneID lane;
    EU_ApproachID approach;
    EU_LaneConnectionID connection;
  } u;
} EU_IntersectionAccessPoint;

extern const ASN1CType asn1_type_EU_IntersectionAccessPoint[];

typedef struct EU_RegionalExtension_25 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_25;


extern const ASN1CType asn1_type_EU_RegionalExtension_25[];

typedef struct EU_SignalRequest_1 {
  EU_RegionalExtension_25 *tab;
  size_t count;
} EU_SignalRequest_1;

extern const ASN1CType asn1_type_EU_SignalRequest_1[];

typedef struct EU_SignalRequest {
  EU_IntersectionReferenceID id;
  EU_RequestID requestID;
  EU_PriorityRequestType requestType;
  EU_IntersectionAccessPoint inBoundLane;
  BOOL outBoundLane_option;
  EU_IntersectionAccessPoint outBoundLane;
  BOOL regional_option;
  EU_SignalRequest_1 regional;
} EU_SignalRequest;


extern const ASN1CType asn1_type_EU_SignalRequest[];

typedef struct EU_RegionalExtension_24 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_24;


extern const ASN1CType asn1_type_EU_RegionalExtension_24[];

typedef struct EU_SignalRequestPackage_1 {
  EU_RegionalExtension_24 *tab;
  size_t count;
} EU_SignalRequestPackage_1;

extern const ASN1CType asn1_type_EU_SignalRequestPackage_1[];

typedef struct EU_SignalRequestPackage {
  EU_SignalRequest request;
  BOOL minute_option;
  EU_MinuteOfTheYear minute;
  BOOL second_option;
  EU_DSecond second;
  BOOL duration_option;
  EU_DSecond duration;
  BOOL regional_option;
  EU_SignalRequestPackage_1 regional;
} EU_SignalRequestPackage;


extern const ASN1CType asn1_type_EU_SignalRequestPackage[];

typedef struct EU_SignalRequestList {
  EU_SignalRequestPackage *tab;
  size_t count;
} EU_SignalRequestList;

extern const ASN1CType asn1_type_EU_SignalRequestList[];

typedef ASN1String EU_TemporaryID;

extern const ASN1CType asn1_type_EU_TemporaryID[];

typedef enum {
  EU_VehicleID_entityID,
  EU_VehicleID_stationID,
  EU_VehicleID__dummy = 2147483647,
} EU_VehicleID_choice;

typedef struct EU_VehicleID {
  EU_VehicleID_choice choice;
  union {
    EU_TemporaryID entityID;
    EU_StationID stationID;
  } u;
} EU_VehicleID;

extern const ASN1CType asn1_type_EU_VehicleID[];

typedef enum EU_BasicVehicleRole {
  EU_BasicVehicleRole_basicVehicle,
  EU_BasicVehicleRole_publicTransport,
  EU_BasicVehicleRole_specialTransport,
  EU_BasicVehicleRole_dangerousGoods,
  EU_BasicVehicleRole_roadWork,
  EU_BasicVehicleRole_roadRescue,
  EU_BasicVehicleRole_emergency,
  EU_BasicVehicleRole_safetyCar,
  EU_BasicVehicleRole_none_unknown,
  EU_BasicVehicleRole_truck,
  EU_BasicVehicleRole_motorcycle,
  EU_BasicVehicleRole_roadSideSource,
  EU_BasicVehicleRole_police,
  EU_BasicVehicleRole_fire,
  EU_BasicVehicleRole_ambulance,
  EU_BasicVehicleRole_dot,
  EU_BasicVehicleRole_transit,
  EU_BasicVehicleRole_slowMoving,
  EU_BasicVehicleRole_stopNgo,
  EU_BasicVehicleRole_cyclist,
  EU_BasicVehicleRole_pedestrian,
  EU_BasicVehicleRole_nonMotorized,
  EU_BasicVehicleRole_military,
  EU_BasicVehicleRole__dummy = 2147483647,
} EU_BasicVehicleRole;

extern const ASN1CType asn1_type_EU_BasicVehicleRole[];

typedef enum EU_RequestSubRole {
  EU_RequestSubRole_requestSubRoleUnKnown,
  EU_RequestSubRole_requestSubRole1,
  EU_RequestSubRole_requestSubRole2,
  EU_RequestSubRole_requestSubRole3,
  EU_RequestSubRole_requestSubRole4,
  EU_RequestSubRole_requestSubRole5,
  EU_RequestSubRole_requestSubRole6,
  EU_RequestSubRole_requestSubRole7,
  EU_RequestSubRole_requestSubRole8,
  EU_RequestSubRole_requestSubRole9,
  EU_RequestSubRole_requestSubRole10,
  EU_RequestSubRole_requestSubRole11,
  EU_RequestSubRole_requestSubRole12,
  EU_RequestSubRole_requestSubRole13,
  EU_RequestSubRole_requestSubRole14,
  EU_RequestSubRole_requestSubRoleReserved,
  EU_RequestSubRole__dummy = 2147483647,
} EU_RequestSubRole;

extern const ASN1CType asn1_type_EU_RequestSubRole[];

typedef enum EU_RequestImportanceLevel {
  EU_RequestImportanceLevel_requestImportanceLevelUnKnown,
  EU_RequestImportanceLevel_requestImportanceLevel1,
  EU_RequestImportanceLevel_requestImportanceLevel2,
  EU_RequestImportanceLevel_requestImportanceLevel3,
  EU_RequestImportanceLevel_requestImportanceLevel4,
  EU_RequestImportanceLevel_requestImportanceLevel5,
  EU_RequestImportanceLevel_requestImportanceLevel6,
  EU_RequestImportanceLevel_requestImportanceLevel7,
  EU_RequestImportanceLevel_requestImportanceLevel8,
  EU_RequestImportanceLevel_requestImportanceLevel9,
  EU_RequestImportanceLevel_requestImportanceLevel10,
  EU_RequestImportanceLevel_requestImportanceLevel11,
  EU_RequestImportanceLevel_requestImportanceLevel12,
  EU_RequestImportanceLevel_requestImportanceLevel13,
  EU_RequestImportanceLevel_requestImportanceLevel14,
  EU_RequestImportanceLevel_requestImportanceReserved,
  EU_RequestImportanceLevel__dummy = 2147483647,
} EU_RequestImportanceLevel;

extern const ASN1CType asn1_type_EU_RequestImportanceLevel[];

typedef enum EU_VehicleType {
  EU_VehicleType_none,
  EU_VehicleType_unknown,
  EU_VehicleType_special,
  EU_VehicleType_moto,
  EU_VehicleType_car,
  EU_VehicleType_carOther,
  EU_VehicleType_bus,
  EU_VehicleType_axleCnt2,
  EU_VehicleType_axleCnt3,
  EU_VehicleType_axleCnt4,
  EU_VehicleType_axleCnt4Trailer,
  EU_VehicleType_axleCnt5Trailer,
  EU_VehicleType_axleCnt6Trailer,
  EU_VehicleType_axleCnt5MultiTrailer,
  EU_VehicleType_axleCnt6MultiTrailer,
  EU_VehicleType_axleCnt7MultiTrailer,
  EU_VehicleType__dummy = 2147483647,
} EU_VehicleType;

extern const ASN1CType asn1_type_EU_VehicleType[];

typedef struct EU_RegionalExtension_20 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_20;


extern const ASN1CType asn1_type_EU_RegionalExtension_20[];

typedef struct EU_RequestorType {
  EU_BasicVehicleRole role;
  BOOL subrole_option;
  EU_RequestSubRole subrole;
  BOOL request_option;
  EU_RequestImportanceLevel request;
  BOOL iso3883_option;
  EU_Iso3833VehicleType iso3883;
  BOOL hpmsType_option;
  EU_VehicleType hpmsType;
  BOOL regional_option;
  EU_RegionalExtension_20 regional;
} EU_RequestorType;


extern const ASN1CType asn1_type_EU_RequestorType[];

typedef enum EU_TransmissionState {
  EU_TransmissionState_neutral,
  EU_TransmissionState_park,
  EU_TransmissionState_forwardGears,
  EU_TransmissionState_reverseGears,
  EU_TransmissionState_reserved1,
  EU_TransmissionState_reserved2,
  EU_TransmissionState_reserved3,
  EU_TransmissionState_unavailable,
  EU_TransmissionState__dummy = 2147483647,
} EU_TransmissionState;

extern const ASN1CType asn1_type_EU_TransmissionState[];

typedef struct EU_TransmissionAndSpeed {
  EU_TransmissionState transmisson;
  EU_Velocity speed;
} EU_TransmissionAndSpeed;


extern const ASN1CType asn1_type_EU_TransmissionAndSpeed[];

typedef struct EU_RequestorPositionVector {
  EU_Position3D position;
  BOOL heading_option;
  EU_Angle heading;
  BOOL speed_option;
  EU_TransmissionAndSpeed speed;
} EU_RequestorPositionVector;


extern const ASN1CType asn1_type_EU_RequestorPositionVector[];

typedef ASN1BitString EU_TransitVehicleStatus;

extern const ASN1CType asn1_type_EU_TransitVehicleStatus[];

typedef enum EU_TransitVehicleOccupancy {
  EU_TransitVehicleOccupancy_occupancyUnknown,
  EU_TransitVehicleOccupancy_occupancyEmpty,
  EU_TransitVehicleOccupancy_occupancyVeryLow,
  EU_TransitVehicleOccupancy_occupancyLow,
  EU_TransitVehicleOccupancy_occupancyMed,
  EU_TransitVehicleOccupancy_occupancyHigh,
  EU_TransitVehicleOccupancy_occupancyNearlyFull,
  EU_TransitVehicleOccupancy_occupancyFull,
  EU_TransitVehicleOccupancy__dummy = 2147483647,
} EU_TransitVehicleOccupancy;

extern const ASN1CType asn1_type_EU_TransitVehicleOccupancy[];

typedef int EU_DeltaTime;

extern const ASN1CType asn1_type_EU_DeltaTime[];

typedef struct EU_RegionalExtension_19 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_19;


extern const ASN1CType asn1_type_EU_RegionalExtension_19[];

typedef struct EU_RequestorDescription_1 {
  EU_RegionalExtension_19 *tab;
  size_t count;
} EU_RequestorDescription_1;

extern const ASN1CType asn1_type_EU_RequestorDescription_1[];

typedef struct EU_RequestorDescription {
  EU_VehicleID id;
  BOOL type_option;
  EU_RequestorType type;
  BOOL position_option;
  EU_RequestorPositionVector position;
  BOOL name_option;
  EU_DescriptiveName name;
  BOOL routeName_option;
  EU_DescriptiveName routeName;
  BOOL transitStatus_option;
  EU_TransitVehicleStatus transitStatus;
  BOOL transitOccupancy_option;
  EU_TransitVehicleOccupancy transitOccupancy;
  BOOL transitSchedule_option;
  EU_DeltaTime transitSchedule;
  BOOL regional_option;
  EU_RequestorDescription_1 regional;
} EU_RequestorDescription;


extern const ASN1CType asn1_type_EU_RequestorDescription[];

typedef struct EU_RegionalExtension_2 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_2;


extern const ASN1CType asn1_type_EU_RegionalExtension_2[];

typedef struct EU_SignalRequestMessage_1 {
  EU_RegionalExtension_2 *tab;
  size_t count;
} EU_SignalRequestMessage_1;

extern const ASN1CType asn1_type_EU_SignalRequestMessage_1[];

typedef struct EU_SignalRequestMessage {
  BOOL timeStamp_option;
  EU_MinuteOfTheYear timeStamp;
  EU_DSecond second;
  BOOL sequenceNumber_option;
  EU_MsgCount sequenceNumber;
  BOOL requests_option;
  EU_SignalRequestList requests;
  EU_RequestorDescription requestor;
  BOOL regional_option;
  EU_SignalRequestMessage_1 regional;
} EU_SignalRequestMessage;


extern const ASN1CType asn1_type_EU_SignalRequestMessage[];

typedef struct EU_SREM {
  EU_ItsPduHeader header;
  EU_SignalRequestMessage srm;
} EU_SREM;


extern const ASN1CType asn1_type_EU_SREM[];

typedef struct EU_SignalRequesterInfo {
  EU_VehicleID id;
  EU_RequestID request;
  EU_MsgCount sequenceNumber;
  BOOL role_option;
  EU_BasicVehicleRole role;
  BOOL typeData_option;
  EU_RequestorType typeData;
} EU_SignalRequesterInfo;


extern const ASN1CType asn1_type_EU_SignalRequesterInfo[];

typedef enum EU_PrioritizationResponseStatus {
  EU_PrioritizationResponseStatus_unknown,
  EU_PrioritizationResponseStatus_requested,
  EU_PrioritizationResponseStatus_processing,
  EU_PrioritizationResponseStatus_watchOtherTraffic,
  EU_PrioritizationResponseStatus_granted,
  EU_PrioritizationResponseStatus_rejected,
  EU_PrioritizationResponseStatus_maxPresence,
  EU_PrioritizationResponseStatus_reserviceLocked,
  EU_PrioritizationResponseStatus__dummy = 2147483647,
} EU_PrioritizationResponseStatus;

extern const ASN1CType asn1_type_EU_PrioritizationResponseStatus[];

typedef struct EU_RegionalExtension_26 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_26;


extern const ASN1CType asn1_type_EU_RegionalExtension_26[];

typedef struct EU_SignalStatusPackage_1 {
  EU_RegionalExtension_26 *tab;
  size_t count;
} EU_SignalStatusPackage_1;

extern const ASN1CType asn1_type_EU_SignalStatusPackage_1[];

typedef struct EU_SignalStatusPackage {
  BOOL requester_option;
  EU_SignalRequesterInfo requester;
  EU_IntersectionAccessPoint inboundOn;
  BOOL outboundOn_option;
  EU_IntersectionAccessPoint outboundOn;
  BOOL minute_option;
  EU_MinuteOfTheYear minute;
  BOOL second_option;
  EU_DSecond second;
  BOOL duration_option;
  EU_DSecond duration;
  EU_PrioritizationResponseStatus status;
  BOOL regional_option;
  EU_SignalStatusPackage_1 regional;
} EU_SignalStatusPackage;


extern const ASN1CType asn1_type_EU_SignalStatusPackage[];

typedef struct EU_SignalStatusPackageList {
  EU_SignalStatusPackage *tab;
  size_t count;
} EU_SignalStatusPackageList;

extern const ASN1CType asn1_type_EU_SignalStatusPackageList[];

typedef struct EU_RegionalExtension_27 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_27;


extern const ASN1CType asn1_type_EU_RegionalExtension_27[];

typedef struct EU_SignalStatus_1 {
  EU_RegionalExtension_27 *tab;
  size_t count;
} EU_SignalStatus_1;

extern const ASN1CType asn1_type_EU_SignalStatus_1[];

typedef struct EU_SignalStatus {
  EU_MsgCount sequenceNumber;
  EU_IntersectionReferenceID id;
  EU_SignalStatusPackageList sigStatus;
  BOOL regional_option;
  EU_SignalStatus_1 regional;
} EU_SignalStatus;


extern const ASN1CType asn1_type_EU_SignalStatus[];

typedef struct EU_SignalStatusList {
  EU_SignalStatus *tab;
  size_t count;
} EU_SignalStatusList;

extern const ASN1CType asn1_type_EU_SignalStatusList[];

typedef struct EU_RegionalExtension_3 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_3;


extern const ASN1CType asn1_type_EU_RegionalExtension_3[];

typedef struct EU_SignalStatusMessage_1 {
  EU_RegionalExtension_3 *tab;
  size_t count;
} EU_SignalStatusMessage_1;

extern const ASN1CType asn1_type_EU_SignalStatusMessage_1[];

typedef struct EU_SignalStatusMessage {
  BOOL timeStamp_option;
  EU_MinuteOfTheYear timeStamp;
  EU_DSecond second;
  BOOL sequenceNumber_option;
  EU_MsgCount sequenceNumber;
  EU_SignalStatusList status;
  BOOL regional_option;
  EU_SignalStatusMessage_1 regional;
} EU_SignalStatusMessage;


extern const ASN1CType asn1_type_EU_SignalStatusMessage[];

typedef struct EU_SSEM {
  EU_ItsPduHeader header;
  EU_SignalStatusMessage ssm;
} EU_SSEM;


extern const ASN1CType asn1_type_EU_SSEM[];

typedef enum EU_RTCM_Revision {
  EU_RTCM_Revision_unknown,
  EU_RTCM_Revision_rtcmRev2,
  EU_RTCM_Revision_rtcmRev3,
  EU_RTCM_Revision_reserved,
  EU_RTCM_Revision__dummy = 2147483647,
} EU_RTCM_Revision;

extern const ASN1CType asn1_type_EU_RTCM_Revision[];

typedef int EU_DYear;

extern const ASN1CType asn1_type_EU_DYear[];

typedef int EU_DMonth;

extern const ASN1CType asn1_type_EU_DMonth[];

typedef int EU_DDay;

extern const ASN1CType asn1_type_EU_DDay[];

typedef int EU_DHour;

extern const ASN1CType asn1_type_EU_DHour[];

typedef int EU_DMinute;

extern const ASN1CType asn1_type_EU_DMinute[];

typedef int EU_DOffset;

extern const ASN1CType asn1_type_EU_DOffset[];

typedef struct EU_DDateTime {
  BOOL year_option;
  EU_DYear year;
  BOOL month_option;
  EU_DMonth month;
  BOOL day_option;
  EU_DDay day;
  BOOL hour_option;
  EU_DHour hour;
  BOOL minute_option;
  EU_DMinute minute;
  BOOL second_option;
  EU_DSecond second;
  BOOL offset_option;
  EU_DOffset offset;
} EU_DDateTime;


extern const ASN1CType asn1_type_EU_DDateTime[];

typedef int EU_SemiMajorAxisAccuracy;

extern const ASN1CType asn1_type_EU_SemiMajorAxisAccuracy[];

typedef int EU_SemiMinorAxisAccuracy;

extern const ASN1CType asn1_type_EU_SemiMinorAxisAccuracy[];

typedef int EU_SemiMajorAxisOrientation;

extern const ASN1CType asn1_type_EU_SemiMajorAxisOrientation[];

typedef struct EU_PositionalAccuracy {
  EU_SemiMajorAxisAccuracy semiMajor;
  EU_SemiMinorAxisAccuracy semiMinor;
  EU_SemiMajorAxisOrientation orientation;
} EU_PositionalAccuracy;


extern const ASN1CType asn1_type_EU_PositionalAccuracy[];

typedef enum EU_TimeConfidence {
  EU_TimeConfidence_unavailable,
  EU_TimeConfidence_time_100_000,
  EU_TimeConfidence_time_050_000,
  EU_TimeConfidence_time_020_000,
  EU_TimeConfidence_time_010_000,
  EU_TimeConfidence_time_002_000,
  EU_TimeConfidence_time_001_000,
  EU_TimeConfidence_time_000_500,
  EU_TimeConfidence_time_000_200,
  EU_TimeConfidence_time_000_100,
  EU_TimeConfidence_time_000_050,
  EU_TimeConfidence_time_000_020,
  EU_TimeConfidence_time_000_010,
  EU_TimeConfidence_time_000_005,
  EU_TimeConfidence_time_000_002,
  EU_TimeConfidence_time_000_001,
  EU_TimeConfidence_time_000_000_5,
  EU_TimeConfidence_time_000_000_2,
  EU_TimeConfidence_time_000_000_1,
  EU_TimeConfidence_time_000_000_05,
  EU_TimeConfidence_time_000_000_02,
  EU_TimeConfidence_time_000_000_01,
  EU_TimeConfidence_time_000_000_005,
  EU_TimeConfidence_time_000_000_002,
  EU_TimeConfidence_time_000_000_001,
  EU_TimeConfidence_time_000_000_000_5,
  EU_TimeConfidence_time_000_000_000_2,
  EU_TimeConfidence_time_000_000_000_1,
  EU_TimeConfidence_time_000_000_000_05,
  EU_TimeConfidence_time_000_000_000_02,
  EU_TimeConfidence_time_000_000_000_01,
  EU_TimeConfidence_time_000_000_000_005,
  EU_TimeConfidence_time_000_000_000_002,
  EU_TimeConfidence_time_000_000_000_001,
  EU_TimeConfidence_time_000_000_000_000_5,
  EU_TimeConfidence_time_000_000_000_000_2,
  EU_TimeConfidence_time_000_000_000_000_1,
  EU_TimeConfidence_time_000_000_000_000_05,
  EU_TimeConfidence_time_000_000_000_000_02,
  EU_TimeConfidence_time_000_000_000_000_01,
  EU_TimeConfidence__dummy = 2147483647,
} EU_TimeConfidence;

extern const ASN1CType asn1_type_EU_TimeConfidence[];

typedef enum EU_PositionConfidence {
  EU_PositionConfidence_unavailable,
  EU_PositionConfidence_a500m,
  EU_PositionConfidence_a200m,
  EU_PositionConfidence_a100m,
  EU_PositionConfidence_a50m,
  EU_PositionConfidence_a20m,
  EU_PositionConfidence_a10m,
  EU_PositionConfidence_a5m,
  EU_PositionConfidence_a2m,
  EU_PositionConfidence_a1m,
  EU_PositionConfidence_a50cm,
  EU_PositionConfidence_a20cm,
  EU_PositionConfidence_a10cm,
  EU_PositionConfidence_a5cm,
  EU_PositionConfidence_a2cm,
  EU_PositionConfidence_a1cm,
  EU_PositionConfidence__dummy = 2147483647,
} EU_PositionConfidence;

extern const ASN1CType asn1_type_EU_PositionConfidence[];

typedef enum EU_ElevationConfidence {
  EU_ElevationConfidence_unavailable,
  EU_ElevationConfidence_elev_500_00,
  EU_ElevationConfidence_elev_200_00,
  EU_ElevationConfidence_elev_100_00,
  EU_ElevationConfidence_elev_050_00,
  EU_ElevationConfidence_elev_020_00,
  EU_ElevationConfidence_elev_010_00,
  EU_ElevationConfidence_elev_005_00,
  EU_ElevationConfidence_elev_002_00,
  EU_ElevationConfidence_elev_001_00,
  EU_ElevationConfidence_elev_000_50,
  EU_ElevationConfidence_elev_000_20,
  EU_ElevationConfidence_elev_000_10,
  EU_ElevationConfidence_elev_000_05,
  EU_ElevationConfidence_elev_000_02,
  EU_ElevationConfidence_elev_000_01,
  EU_ElevationConfidence__dummy = 2147483647,
} EU_ElevationConfidence;

extern const ASN1CType asn1_type_EU_ElevationConfidence[];

typedef struct EU_PositionConfidenceSet {
  EU_PositionConfidence pos;
  EU_ElevationConfidence elevation;
} EU_PositionConfidenceSet;


extern const ASN1CType asn1_type_EU_PositionConfidenceSet[];

typedef enum EU_ThrottleConfidence {
  EU_ThrottleConfidence_unavailable,
  EU_ThrottleConfidence_prec10percent,
  EU_ThrottleConfidence_prec1percent,
  EU_ThrottleConfidence_prec0_5percent,
  EU_ThrottleConfidence__dummy = 2147483647,
} EU_ThrottleConfidence;

extern const ASN1CType asn1_type_EU_ThrottleConfidence[];

typedef struct EU_SpeedandHeadingandThrottleConfidence {
  EU_HeadingConfidence heading;
  EU_SpeedConfidence speed;
  EU_ThrottleConfidence throttle;
} EU_SpeedandHeadingandThrottleConfidence;


extern const ASN1CType asn1_type_EU_SpeedandHeadingandThrottleConfidence[];

typedef struct EU_FullPositionVector {
  BOOL utcTime_option;
  EU_DDateTime utcTime;
  EU_Longitude Long;
  EU_Latitude lat;
  BOOL elevation_option;
  EU_Elevation elevation;
  BOOL heading_option;
  EU_Heading heading;
  BOOL speed_option;
  EU_TransmissionAndSpeed speed;
  BOOL posAccuracy_option;
  EU_PositionalAccuracy posAccuracy;
  BOOL timeConfidence_option;
  EU_TimeConfidence timeConfidence;
  BOOL posConfidence_option;
  EU_PositionConfidenceSet posConfidence;
  BOOL speedConfidence_option;
  EU_SpeedandHeadingandThrottleConfidence speedConfidence;
} EU_FullPositionVector;


extern const ASN1CType asn1_type_EU_FullPositionVector[];

typedef ASN1BitString EU_GNSSstatus;

extern const ASN1CType asn1_type_EU_GNSSstatus[];

typedef int EU_Offset_B09;

extern const ASN1CType asn1_type_EU_Offset_B09[];

typedef struct EU_AntennaOffsetSet {
  EU_Offset_B12 antOffsetX;
  EU_Offset_B09 antOffsetY;
  EU_Offset_B10 antOffsetZ;
} EU_AntennaOffsetSet;


extern const ASN1CType asn1_type_EU_AntennaOffsetSet[];

typedef struct EU_RTCMheader {
  EU_GNSSstatus status;
  EU_AntennaOffsetSet offsetSet;
} EU_RTCMheader;


extern const ASN1CType asn1_type_EU_RTCMheader[];

typedef ASN1String EU_RTCMmessage;

extern const ASN1CType asn1_type_EU_RTCMmessage[];

typedef struct EU_RTCMmessageList {
  EU_RTCMmessage *tab;
  size_t count;
} EU_RTCMmessageList;

extern const ASN1CType asn1_type_EU_RTCMmessageList[];

typedef struct EU_RegionalExtension_5 {
  EU_RegionId regionId;
  ASN1OpenType regExtValue;
} EU_RegionalExtension_5;


extern const ASN1CType asn1_type_EU_RegionalExtension_5[];

typedef struct EU_RTCMcorrections_1 {
  EU_RegionalExtension_5 *tab;
  size_t count;
} EU_RTCMcorrections_1;

extern const ASN1CType asn1_type_EU_RTCMcorrections_1[];

typedef struct EU_RTCMcorrections {
  EU_MsgCount msgCnt;
  EU_RTCM_Revision rev;
  BOOL timeStamp_option;
  EU_MinuteOfTheYear timeStamp;
  BOOL anchorPoint_option;
  EU_FullPositionVector anchorPoint;
  BOOL rtcmHeader_option;
  EU_RTCMheader rtcmHeader;
  EU_RTCMmessageList msgs;
  BOOL regional_option;
  EU_RTCMcorrections_1 regional;
} EU_RTCMcorrections;


extern const ASN1CType asn1_type_EU_RTCMcorrections[];

typedef struct EU_RTCMEM {
  EU_ItsPduHeader header;
  EU_RTCMcorrections rtcmc;
} EU_RTCMEM;


extern const ASN1CType asn1_type_EU_RTCMEM[];

typedef int EU_Code_Units;

enum {
  EU_Code_Units_kmperh = 0,
  EU_Code_Units_milesperh = 1,
  EU_Code_Units_kilometre = 2,
  EU_Code_Units_metre = 3,
  EU_Code_Units_decimetre = 4,
  EU_Code_Units_centimetre = 5,
  EU_Code_Units_mile = 6,
  EU_Code_Units_yard = 7,
  EU_Code_Units_foot = 8,
  EU_Code_Units_minutesOfTime = 9,
  EU_Code_Units_tonnes = 10,
  EU_Code_Units_hundredkg = 11,
  EU_Code_Units_pound = 12,
  EU_Code_Units_rateOfIncline = 13,
  EU_Code_Units_durationinminutes = 14,
};

extern const ASN1CType asn1_type_EU_Code_Units[];

typedef int EU_TimeReference;

enum {
  EU_TimeReference_oneMilliSec = 1,
};

extern const ASN1CType asn1_type_EU_TimeReference[];

typedef struct EU_ItsStationPosition {
  EU_StationID stationID;
  BOOL laneID_option;
  EU_LaneID laneID;
  BOOL nodeXY_option;
  EU_NodeOffsetPointXY nodeXY;
  BOOL timeReference_option;
  EU_TimeReference timeReference;
} EU_ItsStationPosition;


extern const ASN1CType asn1_type_EU_ItsStationPosition[];

typedef struct EU_ItsStationPositionList {
  EU_ItsStationPosition *tab;
  size_t count;
} EU_ItsStationPositionList;

extern const ASN1CType asn1_type_EU_ItsStationPositionList[];

typedef struct EU_ConnectionManeuverAssist_addGrpC {
  BOOL itsStationPosition_option;
  EU_ItsStationPositionList itsStationPosition;
} EU_ConnectionManeuverAssist_addGrpC;


extern const ASN1CType asn1_type_EU_ConnectionManeuverAssist_addGrpC[];

typedef struct EU_ConnectionTrajectory_addGrpC {
  EU_NodeSetXY nodes;
  EU_LaneConnectionID connectionID;
} EU_ConnectionTrajectory_addGrpC;


extern const ASN1CType asn1_type_EU_ConnectionTrajectory_addGrpC[];

typedef int EU_VehicleHeight;

extern const ASN1CType asn1_type_EU_VehicleHeight[];

typedef int EU_DSRC_VehicleMass;

extern const ASN1CType asn1_type_EU_DSRC_VehicleMass[];

typedef struct EU_LaneAttributes_addGrpC {
  BOOL maxVehicleHeight_option;
  EU_VehicleHeight maxVehicleHeight;
  BOOL maxVehicleWeight_option;
  EU_DSRC_VehicleMass maxVehicleWeight;
} EU_LaneAttributes_addGrpC;


extern const ASN1CType asn1_type_EU_LaneAttributes_addGrpC[];

typedef struct EU_PrioritizationResponse {
  EU_StationID stationID;
  EU_PrioritizationResponseStatus priorState;
  EU_SignalGroupID signalGroup;
} EU_PrioritizationResponse;


extern const ASN1CType asn1_type_EU_PrioritizationResponse[];

typedef struct EU_PrioritizationResponseList {
  EU_PrioritizationResponse *tab;
  size_t count;
} EU_PrioritizationResponseList;

extern const ASN1CType asn1_type_EU_PrioritizationResponseList[];

typedef struct EU_IntersectionState_addGrpC {
  BOOL activePrioritizations_option;
  EU_PrioritizationResponseList activePrioritizations;
} EU_IntersectionState_addGrpC;


extern const ASN1CType asn1_type_EU_IntersectionState_addGrpC[];

typedef struct EU_SignalHeadLocation {
  EU_NodeOffsetPointXY nodeXY;
  EU_DeltaAltitude nodeZ;
  EU_SignalGroupID signalGroupID;
} EU_SignalHeadLocation;


extern const ASN1CType asn1_type_EU_SignalHeadLocation[];

typedef struct EU_SignalHeadLocationList {
  EU_SignalHeadLocation *tab;
  size_t count;
} EU_SignalHeadLocationList;

extern const ASN1CType asn1_type_EU_SignalHeadLocationList[];

typedef struct EU_MapData_addGrpC {
  BOOL signalHeadLocations_option;
  EU_SignalHeadLocationList signalHeadLocations;
} EU_MapData_addGrpC;


extern const ASN1CType asn1_type_EU_MapData_addGrpC[];

typedef struct EU_Position3D_addGrpC {
  EU_Altitude altitude;
} EU_Position3D_addGrpC;


extern const ASN1CType asn1_type_EU_Position3D_addGrpC[];

typedef enum EU_ExceptionalCondition {
  EU_ExceptionalCondition_unknown,
  EU_ExceptionalCondition_publicTransportPriority,
  EU_ExceptionalCondition_emergencyVehiclePriority,
  EU_ExceptionalCondition_trainPriority,
  EU_ExceptionalCondition_bridgeOpen,
  EU_ExceptionalCondition_vehicleHeight,
  EU_ExceptionalCondition_weather,
  EU_ExceptionalCondition_trafficJam,
  EU_ExceptionalCondition_tunnelClosure,
  EU_ExceptionalCondition_meteringActive,
  EU_ExceptionalCondition_truckPriority,
  EU_ExceptionalCondition_bicyclePlatoonPriority,
  EU_ExceptionalCondition__dummy = 2147483647,
} EU_ExceptionalCondition;

extern const ASN1CType asn1_type_EU_ExceptionalCondition[];

typedef struct EU_MovementEvent_addGrpC {
  BOOL stateChangeReason_option;
  EU_ExceptionalCondition stateChangeReason;
} EU_MovementEvent_addGrpC;


extern const ASN1CType asn1_type_EU_MovementEvent_addGrpC[];

typedef enum EU_PtvRequestType {
  EU_PtvRequestType_preRequest,
  EU_PtvRequestType_mainRequest,
  EU_PtvRequestType_doorCloseRequest,
  EU_PtvRequestType_cancelRequest,
  EU_PtvRequestType_emergencyRequest,
  EU_PtvRequestType__dummy = 2147483647,
} EU_PtvRequestType;

extern const ASN1CType asn1_type_EU_PtvRequestType[];

typedef struct EU_NodeID {
  ASN1Integer id;
  BOOL lane_option;
  EU_LaneID lane;
} EU_NodeID;


extern const ASN1CType asn1_type_EU_NodeID[];

typedef struct EU_NodeIDList {
  EU_NodeID *tab;
  size_t count;
} EU_NodeIDList;

extern const ASN1CType asn1_type_EU_NodeIDList[];

typedef struct EU_NodeAttributeSet_addGrpC {
  BOOL ptvRequest_option;
  EU_PtvRequestType ptvRequest;
  BOOL nodeIDList_option;
  EU_NodeIDList nodeIDList;
  BOOL nodeID_option;
  EU_NodeID nodeID;
} EU_NodeAttributeSet_addGrpC;


extern const ASN1CType asn1_type_EU_NodeAttributeSet_addGrpC[];

typedef enum EU_EmissionType {
  EU_EmissionType_euro1,
  EU_EmissionType_euro2,
  EU_EmissionType_euro3,
  EU_EmissionType_euro4,
  EU_EmissionType_euro5,
  EU_EmissionType_euro6,
  EU_EmissionType__dummy = 2147483647,
} EU_EmissionType;

extern const ASN1CType asn1_type_EU_EmissionType[];

typedef int EU_FuelType;

extern const ASN1CType asn1_type_EU_FuelType[];

typedef struct EU_RestrictionUserType_addGrpC {
  BOOL emission_option;
  EU_EmissionType emission;
  BOOL fuel_option;
  EU_FuelType fuel;
} EU_RestrictionUserType_addGrpC;


extern const ASN1CType asn1_type_EU_RestrictionUserType_addGrpC[];

typedef enum EU_BatteryStatus {
  EU_BatteryStatus_unknown,
  EU_BatteryStatus_critical,
  EU_BatteryStatus_low,
  EU_BatteryStatus_good,
  EU_BatteryStatus__dummy = 2147483647,
} EU_BatteryStatus;

extern const ASN1CType asn1_type_EU_BatteryStatus[];

typedef struct EU_RequestorDescription_addGrpC {
  BOOL fuel_option;
  EU_FuelType fuel;
  BOOL batteryStatus_option;
  EU_BatteryStatus batteryStatus;
} EU_RequestorDescription_addGrpC;


extern const ASN1CType asn1_type_EU_RequestorDescription_addGrpC[];

typedef enum EU_RejectedReason {
  EU_RejectedReason_unknown,
  EU_RejectedReason_emergencyVehiclePriority,
  EU_RejectedReason_maxWaitingTimeExceeded,
  EU_RejectedReason_ptPriorityDisabled,
  EU_RejectedReason_higherPTPriorityGranted,
  EU_RejectedReason_vehicleTrackingUnknown,
  EU_RejectedReason__dummy = 2147483647,
} EU_RejectedReason;

extern const ASN1CType asn1_type_EU_RejectedReason[];

typedef struct EU_SignalStatusPackage_addGrpC {
  BOOL synchToSchedule_option;
  EU_DeltaTime synchToSchedule;
  BOOL rejectedReason_option;
  EU_RejectedReason rejectedReason;
} EU_SignalStatusPackage_addGrpC;


extern const ASN1CType asn1_type_EU_SignalStatusPackage_addGrpC[];

typedef struct EU_REG_EXT_ID_AND_TYPE { /* object class definition */
  ASN1CType id;
  ASN1CType Type;
} EU_REG_EXT_ID_AND_TYPE;


extern const ASN1CType asn1_type_EU_REG_EXT_ID_AND_TYPE[];

typedef ASN1Null EU_RegionalSPAT;

extern const ASN1CType asn1_type_EU_RegionalSPAT[];

typedef ASN1Null EU_RegionalMapData;

extern const ASN1CType asn1_type_EU_RegionalMapData[];

typedef EU_IntersectionGeometry EU_Intersection;

#define asn1_type_EU_Intersection asn1_type_EU_IntersectionGeometry

typedef int EU_DSRCmsgID;

extern const ASN1CType asn1_type_EU_DSRCmsgID[];

#define EU_mapDataP 18

#define EU_rtcmCorrections 28

#define EU_signalPhaseAndTimingMessage 19

#define EU_signalRequestMessage 29

#define EU_signalStatusMessage 30

#define EU_unknownFuel 0

#define EU_gasoline 1

#define EU_ethanol 2

#define EU_diesel 3

#define EU_electric 4

#define EU_hybrid 5

#define EU_hydrogen 6

#define EU_natGasLiquid 7

#define EU_natGasComp 8

#define EU_propane 9

#define EU_noRegion 0

#define EU_addGrpA 1

#define EU_addGrpB 2

#define EU_addGrpC 3

typedef int EU_TrafficConditionSubCauseCode;

enum {
  EU_TrafficConditionSubCauseCode_unavailable = 0,
  EU_TrafficConditionSubCauseCode_increasedVolumeOfTraffic = 1,
  EU_TrafficConditionSubCauseCode_trafficJamSlowlyIncreasing = 2,
  EU_TrafficConditionSubCauseCode_trafficJamIncreasing = 3,
  EU_TrafficConditionSubCauseCode_trafficJamStronglyIncreasing = 4,
  EU_TrafficConditionSubCauseCode_trafficStationary = 5,
  EU_TrafficConditionSubCauseCode_trafficJamSlightlyDecreasing = 6,
  EU_TrafficConditionSubCauseCode_trafficJamDecreasing = 7,
  EU_TrafficConditionSubCauseCode_trafficJamStronglyDecreasing = 8,
};

extern const ASN1CType asn1_type_EU_TrafficConditionSubCauseCode[];

typedef int EU_AccidentSubCauseCode;

enum {
  EU_AccidentSubCauseCode_unavailable = 0,
  EU_AccidentSubCauseCode_multiVehicleAccident = 1,
  EU_AccidentSubCauseCode_heavyAccident = 2,
  EU_AccidentSubCauseCode_accidentInvolvingLorry = 3,
  EU_AccidentSubCauseCode_accidentInvolvingBus = 4,
  EU_AccidentSubCauseCode_accidentInvolvingHazardousMaterials = 5,
  EU_AccidentSubCauseCode_accidentOnOppositeLane = 6,
  EU_AccidentSubCauseCode_unsecuredAccident = 7,
  EU_AccidentSubCauseCode_assistanceRequested = 8,
};

extern const ASN1CType asn1_type_EU_AccidentSubCauseCode[];

typedef int EU_HumanPresenceOnTheRoadSubCauseCode;

enum {
  EU_HumanPresenceOnTheRoadSubCauseCode_unavailable = 0,
  EU_HumanPresenceOnTheRoadSubCauseCode_childrenOnRoadway = 1,
  EU_HumanPresenceOnTheRoadSubCauseCode_cyclistOnRoadway = 2,
  EU_HumanPresenceOnTheRoadSubCauseCode_motorcyclistOnRoadway = 3,
};

extern const ASN1CType asn1_type_EU_HumanPresenceOnTheRoadSubCauseCode[];

typedef int EU_WrongWayDrivingSubCauseCode;

enum {
  EU_WrongWayDrivingSubCauseCode_unavailable = 0,
  EU_WrongWayDrivingSubCauseCode_wrongLane = 1,
  EU_WrongWayDrivingSubCauseCode_wrongDirection = 2,
};

extern const ASN1CType asn1_type_EU_WrongWayDrivingSubCauseCode[];

typedef int EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode;

enum {
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_unavailable = 0,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_strongWinds = 1,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_damagingHail = 2,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_hurricane = 3,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_thunderstorm = 4,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_tornado = 5,
  EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_blizzard = 6,
};

extern const ASN1CType asn1_type_EU_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode[];

typedef int EU_AdverseWeatherCondition_AdhesionSubCauseCode;

enum {
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_unavailable = 0,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_heavyFrostOnRoad = 1,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_fuelOnRoad = 2,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_mudOnRoad = 3,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_snowOnRoad = 4,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_iceOnRoad = 5,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_blackIceOnRoad = 6,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_oilOnRoad = 7,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_looseChippings = 8,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_instantBlackIce = 9,
  EU_AdverseWeatherCondition_AdhesionSubCauseCode_roadsSalted = 10,
};

extern const ASN1CType asn1_type_EU_AdverseWeatherCondition_AdhesionSubCauseCode[];

typedef int EU_AdverseWeatherCondition_VisibilitySubCauseCode;

enum {
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_unavailable = 0,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_fog = 1,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_smoke = 2,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_heavySnowfall = 3,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_heavyRain = 4,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_heavyHail = 5,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_lowSunGlare = 6,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_sandstorms = 7,
  EU_AdverseWeatherCondition_VisibilitySubCauseCode_swarmsOfInsects = 8,
};

extern const ASN1CType asn1_type_EU_AdverseWeatherCondition_VisibilitySubCauseCode[];

typedef int EU_AdverseWeatherCondition_PrecipitationSubCauseCode;

enum {
  EU_AdverseWeatherCondition_PrecipitationSubCauseCode_unavailable = 0,
  EU_AdverseWeatherCondition_PrecipitationSubCauseCode_heavyRain = 1,
  EU_AdverseWeatherCondition_PrecipitationSubCauseCode_heavySnowfall = 2,
  EU_AdverseWeatherCondition_PrecipitationSubCauseCode_softHail = 3,
};

extern const ASN1CType asn1_type_EU_AdverseWeatherCondition_PrecipitationSubCauseCode[];

typedef int EU_SlowVehicleSubCauseCode;

enum {
  EU_SlowVehicleSubCauseCode_unavailable = 0,
  EU_SlowVehicleSubCauseCode_maintenanceVehicle = 1,
  EU_SlowVehicleSubCauseCode_vehiclesSlowingToLookAtAccident = 2,
  EU_SlowVehicleSubCauseCode_abnormalLoad = 3,
  EU_SlowVehicleSubCauseCode_abnormalWideLoad = 4,
  EU_SlowVehicleSubCauseCode_convoy = 5,
  EU_SlowVehicleSubCauseCode_snowplough = 6,
  EU_SlowVehicleSubCauseCode_deicing = 7,
  EU_SlowVehicleSubCauseCode_saltingVehicles = 8,
};

extern const ASN1CType asn1_type_EU_SlowVehicleSubCauseCode[];

typedef int EU_StationaryVehicleSubCauseCode;

enum {
  EU_StationaryVehicleSubCauseCode_unavailable = 0,
  EU_StationaryVehicleSubCauseCode_humanProblem = 1,
  EU_StationaryVehicleSubCauseCode_vehicleBreakdown = 2,
  EU_StationaryVehicleSubCauseCode_postCrash = 3,
  EU_StationaryVehicleSubCauseCode_publicTransportStop = 4,
  EU_StationaryVehicleSubCauseCode_carryingDangerousGoods = 5,
};

extern const ASN1CType asn1_type_EU_StationaryVehicleSubCauseCode[];

typedef int EU_HumanProblemSubCauseCode;

enum {
  EU_HumanProblemSubCauseCode_unavailable = 0,
  EU_HumanProblemSubCauseCode_glycemiaProblem = 1,
  EU_HumanProblemSubCauseCode_heartProblem = 2,
};

extern const ASN1CType asn1_type_EU_HumanProblemSubCauseCode[];

typedef int EU_EmergencyVehicleApproachingSubCauseCode;

enum {
  EU_EmergencyVehicleApproachingSubCauseCode_unavailable = 0,
  EU_EmergencyVehicleApproachingSubCauseCode_emergencyVehicleApproaching = 1,
  EU_EmergencyVehicleApproachingSubCauseCode_prioritizedVehicleApproaching = 2,
};

extern const ASN1CType asn1_type_EU_EmergencyVehicleApproachingSubCauseCode[];

typedef int EU_HazardousLocation_DangerousCurveSubCauseCode;

enum {
  EU_HazardousLocation_DangerousCurveSubCauseCode_unavailable = 0,
  EU_HazardousLocation_DangerousCurveSubCauseCode_dangerousLeftTurnCurve = 1,
  EU_HazardousLocation_DangerousCurveSubCauseCode_dangerousRightTurnCurve = 2,
  EU_HazardousLocation_DangerousCurveSubCauseCode_multipleCurvesStartingWithUnknownTurningDirection = 3,
  EU_HazardousLocation_DangerousCurveSubCauseCode_multipleCurvesStartingWithLeftTurn = 4,
  EU_HazardousLocation_DangerousCurveSubCauseCode_multipleCurvesStartingWithRightTurn = 5,
};

extern const ASN1CType asn1_type_EU_HazardousLocation_DangerousCurveSubCauseCode[];

typedef int EU_HazardousLocation_SurfaceConditionSubCauseCode;

enum {
  EU_HazardousLocation_SurfaceConditionSubCauseCode_unavailable = 0,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_rockfalls = 1,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_earthquakeDamage = 2,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_sewerCollapse = 3,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_subsidence = 4,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_snowDrifts = 5,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_stormDamage = 6,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_burstPipe = 7,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_volcanoEruption = 8,
  EU_HazardousLocation_SurfaceConditionSubCauseCode_fallingIce = 9,
};

extern const ASN1CType asn1_type_EU_HazardousLocation_SurfaceConditionSubCauseCode[];

typedef int EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode;

enum {
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_unavailable = 0,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_shedLoad = 1,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_partsOfVehicles = 2,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_partsOfTyres = 3,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_bigObjects = 4,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_fallenTrees = 5,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_hubCaps = 6,
  EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode_waitingVehicles = 7,
};

extern const ASN1CType asn1_type_EU_HazardousLocation_ObstacleOnTheRoadSubCauseCode[];

typedef int EU_HazardousLocation_AnimalOnTheRoadSubCauseCode;

enum {
  EU_HazardousLocation_AnimalOnTheRoadSubCauseCode_unavailable = 0,
  EU_HazardousLocation_AnimalOnTheRoadSubCauseCode_wildAnimals = 1,
  EU_HazardousLocation_AnimalOnTheRoadSubCauseCode_herdOfAnimals = 2,
  EU_HazardousLocation_AnimalOnTheRoadSubCauseCode_smallAnimals = 3,
  EU_HazardousLocation_AnimalOnTheRoadSubCauseCode_largeAnimals = 4,
};

extern const ASN1CType asn1_type_EU_HazardousLocation_AnimalOnTheRoadSubCauseCode[];

typedef int EU_CollisionRiskSubCauseCode;

enum {
  EU_CollisionRiskSubCauseCode_unavailable = 0,
  EU_CollisionRiskSubCauseCode_longitudinalCollisionRisk = 1,
  EU_CollisionRiskSubCauseCode_crossingCollisionRisk = 2,
  EU_CollisionRiskSubCauseCode_lateralCollisionRisk = 3,
  EU_CollisionRiskSubCauseCode_vulnerableRoadUser = 4,
};

extern const ASN1CType asn1_type_EU_CollisionRiskSubCauseCode[];

typedef int EU_SignalViolationSubCauseCode;

enum {
  EU_SignalViolationSubCauseCode_unavailable = 0,
  EU_SignalViolationSubCauseCode_stopSignViolation = 1,
  EU_SignalViolationSubCauseCode_trafficLightViolation = 2,
  EU_SignalViolationSubCauseCode_turningRegulationViolation = 3,
};

extern const ASN1CType asn1_type_EU_SignalViolationSubCauseCode[];

typedef int EU_RescueAndRecoveryWorkInProgressSubCauseCode;

enum {
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_unavailable = 0,
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_emergencyVehicles = 1,
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_rescueHelicopterLanding = 2,
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_policeActivityOngoing = 3,
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_medicalEmergencyOngoing = 4,
  EU_RescueAndRecoveryWorkInProgressSubCauseCode_childAbductionInProgress = 5,
};

extern const ASN1CType asn1_type_EU_RescueAndRecoveryWorkInProgressSubCauseCode[];

typedef int EU_DangerousEndOfQueueSubCauseCode;

enum {
  EU_DangerousEndOfQueueSubCauseCode_unavailable = 0,
  EU_DangerousEndOfQueueSubCauseCode_suddenEndOfQueue = 1,
  EU_DangerousEndOfQueueSubCauseCode_queueOverHill = 2,
  EU_DangerousEndOfQueueSubCauseCode_queueAroundBend = 3,
  EU_DangerousEndOfQueueSubCauseCode_queueInTunnel = 4,
};

extern const ASN1CType asn1_type_EU_DangerousEndOfQueueSubCauseCode[];

typedef int EU_DangerousSituationSubCauseCode;

enum {
  EU_DangerousSituationSubCauseCode_unavailable = 0,
  EU_DangerousSituationSubCauseCode_emergencyElectronicBrakeEngaged = 1,
  EU_DangerousSituationSubCauseCode_preCrashSystemEngaged = 2,
  EU_DangerousSituationSubCauseCode_espEngaged = 3,
  EU_DangerousSituationSubCauseCode_absEngaged = 4,
  EU_DangerousSituationSubCauseCode_aebEngaged = 5,
  EU_DangerousSituationSubCauseCode_brakeWarningEngaged = 6,
  EU_DangerousSituationSubCauseCode_collisionRiskWarningEngaged = 7,
};

extern const ASN1CType asn1_type_EU_DangerousSituationSubCauseCode[];

typedef int EU_VehicleBreakdownSubCauseCode;

enum {
  EU_VehicleBreakdownSubCauseCode_unavailable = 0,
  EU_VehicleBreakdownSubCauseCode_lackOfFuel = 1,
  EU_VehicleBreakdownSubCauseCode_lackOfBatteryPower = 2,
  EU_VehicleBreakdownSubCauseCode_engineProblem = 3,
  EU_VehicleBreakdownSubCauseCode_transmissionProblem = 4,
  EU_VehicleBreakdownSubCauseCode_engineCoolingProblem = 5,
  EU_VehicleBreakdownSubCauseCode_brakingSystemProblem = 6,
  EU_VehicleBreakdownSubCauseCode_steeringProblem = 7,
  EU_VehicleBreakdownSubCauseCode_tyrePuncture = 8,
  EU_VehicleBreakdownSubCauseCode_tyrePressureProblem = 9,
};

extern const ASN1CType asn1_type_EU_VehicleBreakdownSubCauseCode[];

typedef int EU_PostCrashSubCauseCode;

enum {
  EU_PostCrashSubCauseCode_unavailable = 0,
  EU_PostCrashSubCauseCode_accidentWithoutECallTriggered = 1,
  EU_PostCrashSubCauseCode_accidentWithECallManuallyTriggered = 2,
  EU_PostCrashSubCauseCode_accidentWithECallAutomaticallyTriggered = 3,
  EU_PostCrashSubCauseCode_accidentWithECallTriggeredWithoutAccessToCellularNetwork = 4,
};

extern const ASN1CType asn1_type_EU_PostCrashSubCauseCode[];

typedef struct EU_DigitalMap {
  EU_ReferencePosition *tab;
  size_t count;
} EU_DigitalMap;

extern const ASN1CType asn1_type_EU_DigitalMap[];

typedef ASN1String EU_OpeningDaysHours;

extern const ASN1CType asn1_type_EU_OpeningDaysHours[];

typedef struct EU_IVI {
  EU_ItsPduHeader header;
  EU_IviStructure ivi;
} EU_IVI;


extern const ASN1CType asn1_type_EU_IVI[];

typedef EU_MAPEM EU_MAP;

#define asn1_type_EU_MAP asn1_type_EU_MAPEM

typedef EU_SPATEM EU_SPAT;

#define asn1_type_EU_SPAT asn1_type_EU_SPATEM

typedef EU_SREM EU_SRM;

#define asn1_type_EU_SRM asn1_type_EU_SREM

typedef EU_SSEM EU_SSM;

#define asn1_type_EU_SSM asn1_type_EU_SSEM

typedef EU_RTCMEM EU_RTCM;

#define asn1_type_EU_RTCM asn1_type_EU_RTCMEM

typedef EU_ItsPduHeader EU_ItsPduHeaderVam;

extern const ASN1CType asn1_type_EU_ItsPduHeaderVam[];

typedef enum EU_OffRoadLanePosition {
  EU_OffRoadLanePosition_unavailable,
  EU_OffRoadLanePosition_sidewalk,
  EU_OffRoadLanePosition_parkingLane,
  EU_OffRoadLanePosition_bikeLane,
  EU_OffRoadLanePosition_max,
  EU_OffRoadLanePosition__dummy = 2147483647,
} EU_OffRoadLanePosition;

extern const ASN1CType asn1_type_EU_OffRoadLanePosition[];

typedef struct EU_MapPosition {
  EU_IntersectionReferenceID intersectionId;
  EU_LaneID lane;
} EU_MapPosition;


extern const ASN1CType asn1_type_EU_MapPosition[];

typedef enum {
  EU_NonIslandLanePosition_offRoadLanePosition,
  EU_NonIslandLanePosition_vehicularLanePosition,
  EU_NonIslandLanePosition_mapPosition,
  EU_NonIslandLanePosition__dummy = 2147483647,
} EU_NonIslandLanePosition_choice;

typedef struct EU_NonIslandLanePosition {
  EU_NonIslandLanePosition_choice choice;
  union {
    EU_OffRoadLanePosition offRoadLanePosition;
    EU_LanePosition vehicularLanePosition;
    EU_MapPosition mapPosition;
  } u;
} EU_NonIslandLanePosition;

extern const ASN1CType asn1_type_EU_NonIslandLanePosition[];

typedef struct EU_TrafficIslandPosition {
  EU_NonIslandLanePosition oneSide;
  EU_NonIslandLanePosition otherSide;
} EU_TrafficIslandPosition;


extern const ASN1CType asn1_type_EU_TrafficIslandPosition[];

typedef enum {
  EU_VruLanePosition_offRoadLanePosition,
  EU_VruLanePosition_vehicularLanePosition,
  EU_VruLanePosition_trafficIslandPosition,
  EU_VruLanePosition_mapPosition,
  EU_VruLanePosition__dummy = 2147483647,
} EU_VruLanePosition_choice;

typedef struct EU_VruLanePosition {
  EU_VruLanePosition_choice choice;
  union {
    EU_OffRoadLanePosition offRoadLanePosition;
    EU_LanePosition vehicularLanePosition;
    EU_TrafficIslandPosition trafficIslandPosition;
    EU_MapPosition mapPosition;
  } u;
} EU_VruLanePosition;

extern const ASN1CType asn1_type_EU_VruLanePosition[];

typedef enum EU_VruEnvironment {
  EU_VruEnvironment_unavailable,
  EU_VruEnvironment_intersectionCrossing,
  EU_VruEnvironment_zebraCrossing,
  EU_VruEnvironment_sidewalk,
  EU_VruEnvironment_onVehicleRoad,
  EU_VruEnvironment_protectedGeographicArea,
  EU_VruEnvironment_max,
  EU_VruEnvironment__dummy = 2147483647,
} EU_VruEnvironment;

extern const ASN1CType asn1_type_EU_VruEnvironment[];

typedef enum EU_VruMovementControl {
  EU_VruMovementControl_unavailable,
  EU_VruMovementControl_braking,
  EU_VruMovementControl_hardBraking,
  EU_VruMovementControl_stopPedaling,
  EU_VruMovementControl_brakingAndStopPedaling,
  EU_VruMovementControl_hardBrakingAndStopPedaling,
  EU_VruMovementControl_noReaction,
  EU_VruMovementControl_max,
  EU_VruMovementControl__dummy = 2147483647,
} EU_VruMovementControl;

extern const ASN1CType asn1_type_EU_VruMovementControl[];

typedef EU_Heading EU_VruOrientation;

extern const ASN1CType asn1_type_EU_VruOrientation[];

typedef EU_Heading EU_VruRollAngle;

extern const ASN1CType asn1_type_EU_VruRollAngle[];

typedef enum EU_VruDeviceUsage {
  EU_VruDeviceUsage_unavailable,
  EU_VruDeviceUsage_other,
  EU_VruDeviceUsage_idle,
  EU_VruDeviceUsage_listeningToAudio,
  EU_VruDeviceUsage_typing,
  EU_VruDeviceUsage_calling,
  EU_VruDeviceUsage_playingGames,
  EU_VruDeviceUsage_reading,
  EU_VruDeviceUsage_viewing,
  EU_VruDeviceUsage_max,
  EU_VruDeviceUsage__dummy = 2147483647,
} EU_VruDeviceUsage;

extern const ASN1CType asn1_type_EU_VruDeviceUsage[];

typedef struct EU_VruHighFrequencyContainer {
  EU_Heading heading;
  EU_Speed speed;
  EU_LongitudinalAcceleration longitudinalAcceleration;
  BOOL curvature_option;
  EU_Curvature curvature;
  BOOL curvatureCalculationMode_option;
  EU_CurvatureCalculationMode curvatureCalculationMode;
  BOOL yawRate_option;
  EU_YawRate yawRate;
  BOOL lateralAcceleration_option;
  EU_LateralAcceleration lateralAcceleration;
  BOOL verticalAcceleration_option;
  EU_VerticalAcceleration verticalAcceleration;
  BOOL vruLanePosition_option;
  EU_VruLanePosition vruLanePosition;
  BOOL environment_option;
  EU_VruEnvironment environment;
  BOOL movementControl_option;
  EU_VruMovementControl movementControl;
  BOOL orientation_option;
  EU_VruOrientation orientation;
  BOOL rollAngle_option;
  EU_VruRollAngle rollAngle;
  BOOL deviceUsage_option;
  EU_VruDeviceUsage deviceUsage;
} EU_VruHighFrequencyContainer;


extern const ASN1CType asn1_type_EU_VruHighFrequencyContainer[];

typedef enum EU_VruSubProfilePedestrian {
  EU_VruSubProfilePedestrian_unavailable,
  EU_VruSubProfilePedestrian_ordinary_pedestrian,
  EU_VruSubProfilePedestrian_road_worker,
  EU_VruSubProfilePedestrian_first_responder,
  EU_VruSubProfilePedestrian_max,
  EU_VruSubProfilePedestrian__dummy = 2147483647,
} EU_VruSubProfilePedestrian;

extern const ASN1CType asn1_type_EU_VruSubProfilePedestrian[];

typedef enum EU_VruSubProfileBicyclist {
  EU_VruSubProfileBicyclist_unavailable,
  EU_VruSubProfileBicyclist_bicyclist,
  EU_VruSubProfileBicyclist_wheelchair_user,
  EU_VruSubProfileBicyclist_horse_and_rider,
  EU_VruSubProfileBicyclist_rollerskater,
  EU_VruSubProfileBicyclist_e_scooter,
  EU_VruSubProfileBicyclist_personal_transporter,
  EU_VruSubProfileBicyclist_pedelec,
  EU_VruSubProfileBicyclist_speed_pedelec,
  EU_VruSubProfileBicyclist_max,
  EU_VruSubProfileBicyclist__dummy = 2147483647,
} EU_VruSubProfileBicyclist;

extern const ASN1CType asn1_type_EU_VruSubProfileBicyclist[];

typedef enum EU_VruSubProfileMotorcyclist {
  EU_VruSubProfileMotorcyclist_unavailable,
  EU_VruSubProfileMotorcyclist_moped,
  EU_VruSubProfileMotorcyclist_motorcycle,
  EU_VruSubProfileMotorcyclist_motorcycle_and_sidecar_right,
  EU_VruSubProfileMotorcyclist_motorcycle_and_sidecar_left,
  EU_VruSubProfileMotorcyclist_max,
  EU_VruSubProfileMotorcyclist__dummy = 2147483647,
} EU_VruSubProfileMotorcyclist;

extern const ASN1CType asn1_type_EU_VruSubProfileMotorcyclist[];

typedef enum EU_VruSubProfileAnimal {
  EU_VruSubProfileAnimal_unavailable,
  EU_VruSubProfileAnimal_wild_animal,
  EU_VruSubProfileAnimal_farm_animal,
  EU_VruSubProfileAnimal_service_animal,
  EU_VruSubProfileAnimal_max,
  EU_VruSubProfileAnimal__dummy = 2147483647,
} EU_VruSubProfileAnimal;

extern const ASN1CType asn1_type_EU_VruSubProfileAnimal[];

typedef enum {
  EU_VruProfileAndSubprofile_pedestrian,
  EU_VruProfileAndSubprofile_bicyclist,
  EU_VruProfileAndSubprofile_motorcylist,
  EU_VruProfileAndSubprofile_animal,
  EU_VruProfileAndSubprofile__dummy = 2147483647,
} EU_VruProfileAndSubprofile_choice;

typedef struct EU_VruProfileAndSubprofile {
  EU_VruProfileAndSubprofile_choice choice;
  union {
    EU_VruSubProfilePedestrian pedestrian;
    EU_VruSubProfileBicyclist bicyclist;
    EU_VruSubProfileMotorcyclist motorcylist;
    EU_VruSubProfileAnimal animal;
  } u;
} EU_VruProfileAndSubprofile;

extern const ASN1CType asn1_type_EU_VruProfileAndSubprofile[];

typedef ASN1BitString EU_VruSpecificExteriorLights;

extern const ASN1CType asn1_type_EU_VruSpecificExteriorLights[];

typedef struct EU_VruExteriorLights {
  EU_VruSpecificExteriorLights vruSpecific;
  EU_ExteriorLights vehicular;
} EU_VruExteriorLights;


extern const ASN1CType asn1_type_EU_VruExteriorLights[];

typedef enum EU_VruSizeClass {
  EU_VruSizeClass_unavailable,
  EU_VruSizeClass_low,
  EU_VruSizeClass_medium,
  EU_VruSizeClass_high,
  EU_VruSizeClass_max,
  EU_VruSizeClass__dummy = 2147483647,
} EU_VruSizeClass;

extern const ASN1CType asn1_type_EU_VruSizeClass[];

typedef struct EU_VruLowFrequencyContainer {
  BOOL profileAndSubprofile_option;
  EU_VruProfileAndSubprofile profileAndSubprofile;
  BOOL exteriorLights_option;
  EU_VruExteriorLights exteriorLights;
  BOOL sizeClass_option;
  EU_VruSizeClass sizeClass;
} EU_VruLowFrequencyContainer;


extern const ASN1CType asn1_type_EU_VruLowFrequencyContainer[];

typedef int EU_ClusterId;

extern const ASN1CType asn1_type_EU_ClusterId[];

typedef enum {
  EU_ClusterBoundingBoxShape_clusterRectangle,
  EU_ClusterBoundingBoxShape_clusterCircle,
  EU_ClusterBoundingBoxShape_clusterPolygon,
  EU_ClusterBoundingBoxShape__dummy = 2147483647,
} EU_ClusterBoundingBoxShape_choice;

typedef struct EU_ClusterBoundingBoxShape {
  EU_ClusterBoundingBoxShape_choice choice;
  union {
    EU_AreaRectangle clusterRectangle;
    EU_AreaCircular clusterCircle;
    EU_AreaPolygon clusterPolygon;
  } u;
} EU_ClusterBoundingBoxShape;

extern const ASN1CType asn1_type_EU_ClusterBoundingBoxShape[];

typedef int EU_ClusterCardinalitySize;

enum {
  EU_ClusterCardinalitySize_unavailable = 0,
  EU_ClusterCardinalitySize_onlyLeader = 1,
};

extern const ASN1CType asn1_type_EU_ClusterCardinalitySize[];

typedef ASN1BitString EU_ClusterProfiles;

extern const ASN1CType asn1_type_EU_ClusterProfiles[];

typedef struct EU_VruClusterInformationContainer {
  EU_ClusterId clusterId;
  EU_ClusterBoundingBoxShape clusterBoundingBoxShape;
  EU_ClusterCardinalitySize clusterCardinalitySize;
  EU_ClusterProfiles clusterProfiles;
} EU_VruClusterInformationContainer;


extern const ASN1CType asn1_type_EU_VruClusterInformationContainer[];

typedef int EU_VruClusterOpTimestamp;

extern const ASN1CType asn1_type_EU_VruClusterOpTimestamp[];

typedef struct EU_ClusterJoinInfo {
  EU_ClusterId clusterId;
  EU_VruClusterOpTimestamp joinTime;
} EU_ClusterJoinInfo;


extern const ASN1CType asn1_type_EU_ClusterJoinInfo[];

typedef enum EU_ClusterLeaveReason {
  EU_ClusterLeaveReason_notProvided,
  EU_ClusterLeaveReason_clusterLeaderLost,
  EU_ClusterLeaveReason_clusterDisbandedByLeader,
  EU_ClusterLeaveReason_outOfClusterBoundingBox,
  EU_ClusterLeaveReason_outOfClusterSpeedRange,
  EU_ClusterLeaveReason_joiningAnotherCluster,
  EU_ClusterLeaveReason_cancelledJoin,
  EU_ClusterLeaveReason_failedJoin,
  EU_ClusterLeaveReason_safetyCondition,
  EU_ClusterLeaveReason_max,
  EU_ClusterLeaveReason__dummy = 2147483647,
} EU_ClusterLeaveReason;

extern const ASN1CType asn1_type_EU_ClusterLeaveReason[];

typedef struct EU_ClusterLeaveInfo {
  EU_ClusterId clusterId;
  EU_ClusterLeaveReason clusterLeaveReason;
} EU_ClusterLeaveInfo;


extern const ASN1CType asn1_type_EU_ClusterLeaveInfo[];

typedef enum EU_ClusterBreakupReason {
  EU_ClusterBreakupReason_notProvided,
  EU_ClusterBreakupReason_clusteringPurposeCompleted,
  EU_ClusterBreakupReason_leaderMovedOutOfClusterBoundingBox,
  EU_ClusterBreakupReason_joiningAnotherCluster,
  EU_ClusterBreakupReason_enteringLowRiskAreaBasedOnMaps,
  EU_ClusterBreakupReason_receptionOfCpmContainingCluster,
  EU_ClusterBreakupReason_max,
  EU_ClusterBreakupReason__dummy = 2147483647,
} EU_ClusterBreakupReason;

extern const ASN1CType asn1_type_EU_ClusterBreakupReason[];

typedef struct EU_ClusterBreakupInfo {
  EU_ClusterBreakupReason clusterBreakupReason;
  EU_VruClusterOpTimestamp breakupTime;
} EU_ClusterBreakupInfo;


extern const ASN1CType asn1_type_EU_ClusterBreakupInfo[];

typedef struct EU_VruClusterOperationContainer {
  BOOL clusterJoinInfo_option;
  EU_ClusterJoinInfo clusterJoinInfo;
  BOOL clusterLeaveInfo_option;
  EU_ClusterLeaveInfo clusterLeaveInfo;
  BOOL clusterBreakupInfo_option;
  EU_ClusterBreakupInfo clusterBreakupInfo;
  BOOL clusterIdChangeTimeInfo_option;
  EU_VruClusterOpTimestamp clusterIdChangeTimeInfo;
} EU_VruClusterOperationContainer;


extern const ASN1CType asn1_type_EU_VruClusterOperationContainer[];

typedef struct EU_VruPathPoint {
  EU_ReferencePosition pathPosition;
  BOOL pathDeltaTime_option;
  EU_PathDeltaTime pathDeltaTime;
} EU_VruPathPoint;


extern const ASN1CType asn1_type_EU_VruPathPoint[];

typedef struct EU_SequenceOfVruPathPoint {
  EU_VruPathPoint *tab;
  size_t count;
} EU_SequenceOfVruPathPoint;

extern const ASN1CType asn1_type_EU_SequenceOfVruPathPoint[];

typedef BOOL EU_StationSafeDistanceIndication;

extern const ASN1CType asn1_type_EU_StationSafeDistanceIndication[];

typedef int EU_ActionDeltaTime;

enum {
  EU_ActionDeltaTime_zero = 0,
  EU_ActionDeltaTime_hundredMs = 1,
  EU_ActionDeltaTime_twoHundredMs = 2,
  EU_ActionDeltaTime_unavailable = 127,
};

extern const ASN1CType asn1_type_EU_ActionDeltaTime[];

typedef struct EU_VruSafeDistanceIndication {
  BOOL subjectStation_option;
  EU_StationID subjectStation;
  EU_StationSafeDistanceIndication stationSafeDistanceIndication;
  BOOL timeToCollision_option;
  EU_ActionDeltaTime timeToCollision;
} EU_VruSafeDistanceIndication;


extern const ASN1CType asn1_type_EU_VruSafeDistanceIndication[];

typedef struct EU_SequenceOfVruSafeDistanceIndication {
  EU_VruSafeDistanceIndication *tab;
  size_t count;
} EU_SequenceOfVruSafeDistanceIndication;

extern const ASN1CType asn1_type_EU_SequenceOfVruSafeDistanceIndication[];

typedef int EU_TrajectoryInterceptionProbability;

enum {
  EU_TrajectoryInterceptionProbability_zero = 0,
  EU_TrajectoryInterceptionProbability_twoPercent = 1,
  EU_TrajectoryInterceptionProbability_fourPercent = 2,
  EU_TrajectoryInterceptionProbability_oneHundredPercent = 50,
  EU_TrajectoryInterceptionProbability_unavailable = 63,
};

extern const ASN1CType asn1_type_EU_TrajectoryInterceptionProbability[];

typedef int EU_TrajectoryInterceptionConfidence;

enum {
  EU_TrajectoryInterceptionConfidence_lessthan50percent = 0,
  EU_TrajectoryInterceptionConfidence_between50and70Percent = 1,
  EU_TrajectoryInterceptionConfidence_between70and90Percent = 2,
  EU_TrajectoryInterceptionConfidence_above90Percent = 3,
};

extern const ASN1CType asn1_type_EU_TrajectoryInterceptionConfidence[];

typedef struct EU_TrajectoryInterceptionIndication {
  BOOL subjectStation_option;
  EU_StationID subjectStation;
  EU_TrajectoryInterceptionProbability trajectoryInterceptionProbability;
  BOOL trajectoryInterceptionConfidence_option;
  EU_TrajectoryInterceptionConfidence trajectoryInterceptionConfidence;
} EU_TrajectoryInterceptionIndication;


extern const ASN1CType asn1_type_EU_TrajectoryInterceptionIndication[];

typedef struct EU_SequenceOfTrajectoryInterceptionIndication {
  EU_TrajectoryInterceptionIndication *tab;
  size_t count;
} EU_SequenceOfTrajectoryInterceptionIndication;

extern const ASN1CType asn1_type_EU_SequenceOfTrajectoryInterceptionIndication[];

typedef enum EU_AccelOrDecel {
  EU_AccelOrDecel_accelerate,
  EU_AccelOrDecel_decelerate,
  EU_AccelOrDecel__dummy = 2147483647,
} EU_AccelOrDecel;

extern const ASN1CType asn1_type_EU_AccelOrDecel[];

typedef struct EU_AccelerationChangeIndication {
  EU_AccelOrDecel accelOrDecel;
  EU_ActionDeltaTime actionDeltaTime;
} EU_AccelerationChangeIndication;


extern const ASN1CType asn1_type_EU_AccelerationChangeIndication[];

typedef enum EU_LeftOrRight {
  EU_LeftOrRight_left,
  EU_LeftOrRight_right,
  EU_LeftOrRight__dummy = 2147483647,
} EU_LeftOrRight;

extern const ASN1CType asn1_type_EU_LeftOrRight[];

typedef struct EU_HeadingChangeIndication {
  EU_LeftOrRight direction;
  EU_ActionDeltaTime actionDeltaTime;
} EU_HeadingChangeIndication;


extern const ASN1CType asn1_type_EU_HeadingChangeIndication[];

typedef int EU_StabilityLossProbability;

enum {
  EU_StabilityLossProbability_zero = 0,
  EU_StabilityLossProbability_twoPercent = 1,
  EU_StabilityLossProbability_fourPercent = 2,
  EU_StabilityLossProbability_unavailable = 63,
};

extern const ASN1CType asn1_type_EU_StabilityLossProbability[];

typedef struct EU_StabilityChangeIndication {
  EU_StabilityLossProbability lossProbability;
  EU_ActionDeltaTime actionDeltaTime;
} EU_StabilityChangeIndication;


extern const ASN1CType asn1_type_EU_StabilityChangeIndication[];

typedef struct EU_VruMotionPredictionContainer {
  BOOL pathHistory_option;
  EU_PathHistory pathHistory;
  BOOL pathPrediction_option;
  EU_SequenceOfVruPathPoint pathPrediction;
  BOOL safeDistance_option;
  EU_SequenceOfVruSafeDistanceIndication safeDistance;
  BOOL trajectoryInterceptionIndication_option;
  EU_SequenceOfTrajectoryInterceptionIndication trajectoryInterceptionIndication;
  BOOL accelerationChangeIndication_option;
  EU_AccelerationChangeIndication accelerationChangeIndication;
  BOOL headingChangeIndication_option;
  EU_HeadingChangeIndication headingChangeIndication;
  BOOL stabilityChangeIndication_option;
  EU_StabilityChangeIndication stabilityChangeIndication;
} EU_VruMotionPredictionContainer;


extern const ASN1CType asn1_type_EU_VruMotionPredictionContainer[];

typedef struct EU_VamParameters {
  EU_BasicContainer basicContainer;
  BOOL vruHighFrequencyContainer_option;
  EU_VruHighFrequencyContainer vruHighFrequencyContainer;
  BOOL vruLowFrequencyContainer_option;
  EU_VruLowFrequencyContainer vruLowFrequencyContainer;
  BOOL vruClusterInformationContainer_option;
  EU_VruClusterInformationContainer vruClusterInformationContainer;
  BOOL vruClusterOperationContainer_option;
  EU_VruClusterOperationContainer vruClusterOperationContainer;
  BOOL vruMotionPredictionContainer_option;
  EU_VruMotionPredictionContainer vruMotionPredictionContainer;
} EU_VamParameters;


extern const ASN1CType asn1_type_EU_VamParameters[];

typedef struct EU_VruAwareness {
  EU_GenerationDeltaTime generationDeltaTime;
  EU_VamParameters vamParameters;
} EU_VruAwareness;


extern const ASN1CType asn1_type_EU_VruAwareness[];

typedef struct EU_VAM {
  EU_ItsPduHeaderVam header;
  EU_VruAwareness vam;
} EU_VAM;


extern const ASN1CType asn1_type_EU_VAM[];

typedef enum EU_VruProfile {
  EU_VruProfile_unavailable,
  EU_VruProfile_pedestrian,
  EU_VruProfile_cyclist,
  EU_VruProfile_motorcyclist,
  EU_VruProfile_animal,
  EU_VruProfile_max,
  EU_VruProfile__dummy = 2147483647,
} EU_VruProfile;

extern const ASN1CType asn1_type_EU_VruProfile[];

typedef ASN1Null EU_DUMMY;

extern const ASN1CType asn1_type_EU_DUMMY[];

#ifdef  __cplusplus
}
#endif

#endif /* _FFASN1_V2X_EU_ASN_H */
