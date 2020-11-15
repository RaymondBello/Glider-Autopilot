#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Simulink_Simulation_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "Simulink_Simulation.h"
#include "Simulink_Simulation_capi.h"
#include "Simulink_Simulation_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 16 , TARGET_STRING
( "Simulink_Simulation/Environment" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0
, 0 } , { 1 , 16 , TARGET_STRING ( "Simulink_Simulation/Environment" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 2 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 3 , 16 , TARGET_STRING ( "Simulink_Simulation/Environment" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 17 , TARGET_STRING (
"Simulink_Simulation/Pilot" ) , TARGET_STRING ( "AltCmd" ) , 0 , 0 , 0 , 0 ,
0 } , { 5 , 16 , TARGET_STRING ( "Simulink_Simulation/Environment/z-->h" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 6 , 17 , TARGET_STRING (
"Simulink_Simulation/Pilot/Rate Limiter" ) , TARGET_STRING ( "AltCmd" ) , 0 ,
0 , 0 , 0 , 0 } , { 7 , 17 , TARGET_STRING ( "Simulink_Simulation/Pilot/Add"
) , TARGET_STRING ( "AltCmd" ) , 0 , 0 , 0 , 0 , 1 } , { 8 , 16 ,
TARGET_STRING (
"Simulink_Simulation/Environment/COESA Atmosphere Model/S-Function" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 9 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment/COESA Atmosphere Model/S-Function" ) ,
TARGET_STRING ( "" ) , 1 , 0 , 0 , 0 , 0 } , { 10 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment/COESA Atmosphere Model/S-Function" ) ,
TARGET_STRING ( "" ) , 2 , 0 , 0 , 0 , 0 } , { 11 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment/COESA Atmosphere Model/S-Function" ) ,
TARGET_STRING ( "" ) , 3 , 0 , 0 , 0 , 0 } , { 12 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/vt" ) , TARGET_STRING (
"Airspeed" ) , 0 , 0 , 0 , 0 , 0 } , { 13 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Air Data Computer" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 14 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Air Data Computer" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 15 , 18 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Air Data Computer" ) ,
TARGET_STRING ( "h_sensed" ) , 0 , 0 , 0 , 0 , 3 } , { 16 , 0 , TARGET_STRING
( "Simulink_Simulation/Vehicle System Model/Flight Sensors/Sqrt" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 17 , 1 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 18 , 2 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 19 , 3 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 20 , 16 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Time" )
, TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 21 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Logical Operator1"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 22 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Logical Operator2"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 23 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Logical Operator3"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 24 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Wind Shear Model/Wind speed at reference height"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 2 } , { 25 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Wind Shear Model/ln(ref_height//z0)"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 26 , 18 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Air Data Computer/cast8" )
, TARGET_STRING ( "h_sensed" ) , 0 , 0 , 0 , 0 , 3 } , { 27 , 0 ,
TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Theta Controller"
) , TARGET_STRING ( "Elevator" ) , 0 , 0 , 0 , 0 , 3 } , { 28 , 0 ,
TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Add" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 29 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Add1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 3 } , { 30 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Pressure Altitude/S-Function"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 31 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 32 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/Sum1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 33 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/Thrust_LMN" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 1 } , { 34 , 0 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/Thrust_YZ" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 1 } , { 35 , 1 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)/Distance into Gust (x) (Limited to gust length d)"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 36 , 2 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 37 , 3 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 38 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Angle Conversion/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 39 , 5 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 40 , 6 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 41 , 7 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 42 , 8 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 43 , 9 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 44 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Length Conversion/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 45 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Length Conversion1/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 46 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise/Divide"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 2 } , { 47 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise/Product"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 4 } , { 48 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Wind Shear Model/Length Conversion/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 49 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Multiply"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 50 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 51 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 52 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Merge1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 53 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Position"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 54 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Theta"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 55 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/q"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 56 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Sum1"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 57 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant6"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 58 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant7"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 59 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Math Function"
) , TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 2 } , { 60 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator/Demand limits"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 61 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator/Sum2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 62 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator/Sum3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 63 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1/Demand limits"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 3 } , { 64 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1/Sum2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 65 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1/Sum3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 66 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2/Demand limits"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 67 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2/Sum2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 68 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2/Sum3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 69 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 70 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Switch1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 71 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/y*cos"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 72 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/y*sin"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 73 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/SinCos"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 74 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/SinCos"
) , TARGET_STRING ( "" ) , 1 , 0 , 0 , 0 , 2 } , { 75 , 4 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/u^1//6"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 76 , 4 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 77 , 4 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/w4"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 78 , 5 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 79 , 6 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 80 , 7 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 81 , 7 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)/w1"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 82 , 8 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 83 , 8 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/w "
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 84 , 8 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/w 1"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 85 , 9 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/Lwg//V 1"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 86 , 9 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/w"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 87 , 9 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/w "
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 88 , 9 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/sqrt"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 89 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/PreLook-Up Index Search  (prob of exceed)"
) , TARGET_STRING ( "" ) , 0 , 2 , 0 , 0 , 2 } , { 90 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/PreLook-Up Index Search  (prob of exceed)"
) , TARGET_STRING ( "" ) , 1 , 0 , 0 , 0 , 2 } , { 91 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/Low Altitude Intensity/sigma_wg "
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 92 , 10 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 93 , 13 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 94 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Relative Ratio/Math Function1"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 2 } , { 95 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Compute Mach at Sea Level/Subsystem"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 96 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Compute Mach at Sea Level/Subsystem1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 97 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Compute Mach at Sea Level/Merge"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 98 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "vel_case" ) , 0 , 0 , 0 , 0 , 1 } , { 99 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "Airspeed_in" ) , 0 , 0 , 0 , 0 , 1 } , { 100 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "SoS" ) , 0 , 0 , 0 , 0 , 1 } , { 101 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "Pstatic" ) , 0 , 0 , 0 , 0 , 1 } , { 102 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "gamma" ) , 0 , 0 , 0 , 0 , 1 } , { 103 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "P0_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 104 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS"
) , TARGET_STRING ( "a_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 105 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "vel_case" ) , 0 , 0 , 0 , 0 , 1 } , { 106 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "Airspeed_in" ) , 0 , 0 , 0 , 0 , 1 } , { 107 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "SoS" ) , 0 , 0 , 0 , 0 , 1 } , { 108 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "Pstatic" ) , 0 , 0 , 0 , 0 , 1 } , { 109 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "gamma" ) , 0 , 0 , 0 , 0 , 1 } , { 110 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "P0_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 111 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS"
) , TARGET_STRING ( "a_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 112 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "vel_case" ) , 0 , 0 , 0 , 0 , 1 } , { 113 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "Airspeed_in" ) , 0 , 0 , 0 , 0 , 1 } , { 114 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "SoS" ) , 0 , 0 , 0 , 0 , 1 } , { 115 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "Pstatic" ) , 0 , 0 , 0 , 0 , 1 } , { 116 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "gamma" ) , 0 , 0 , 0 , 0 , 1 } , { 117 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "P0_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 118 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/Merge1"
) , TARGET_STRING ( "a_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 119 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 120 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 121 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/Merge1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 122 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 123 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 124 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/Merge1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 125 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Calculate qdot/Product2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 126 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/calc alpha_dot/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 127 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Angular Velocity Conversion/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 128 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Abs1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 129 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Sign1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 130 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 131 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Abs"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 132 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 133 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 134 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 135 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Trigonometric Function1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 136 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Trigonometric Function2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 137 , 16 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion/Unit Conversion"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 138 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Compute Mach at Sea Level/Subsystem/Product"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 139 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Compute Mach at Sea Level/Subsystem1/Product"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 140 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "vel_case" ) , 0 , 0 , 0 , 0 , 1 } , { 141 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "SoS" ) , 0 , 0 , 0 , 0 , 1 } , { 142 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "Pstatic" ) , 0 , 0 , 0 , 0 , 1 } , { 143 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "gamma" ) , 0 , 0 , 0 , 0 , 1 } , { 144 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "P0_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 145 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "a_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 146 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/EAS2TAS/Product1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 147 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "vel_case" ) , 0 , 0 , 0 , 0 , 1 } , { 148 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "Airspeed_in" ) , 0 , 0 , 0 , 0 , 1 } , { 149 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "SoS" ) , 0 , 0 , 0 , 0 , 1 } , { 150 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "Pstatic" ) , 0 , 0 , 0 , 0 , 1 } , { 151 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "gamma" ) , 0 , 0 , 0 , 0 , 1 } , { 152 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "P0_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 153 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Convert Ain/TAS2TAS/BusConversion_InsertedFor_O_at_inport_0"
) , TARGET_STRING ( "a_sl" ) , 0 , 0 , 0 , 0 , 1 } , { 154 , 50 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M0"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 155 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 156 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 157 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 158 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M4"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 159 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/Merge"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 160 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M0"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 161 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 162 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 163 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 164 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M4"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 165 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/Merge"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 166 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Incidence, Sideslip, & Airspeed/dot/Product1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 167 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 2 } , { 168 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Stability Angular Rates/Unary Minus"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 169 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Sum"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 2 } , { 170 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant/Compare"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 171 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Abs"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 172 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 173 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant/Compare"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 174 , 26 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 175 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 176 , 31 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 177 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Switch"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 178 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M0/M0"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 179 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M1/M1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 180 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M2/M2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 181 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M3/M3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 182 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M4/M4"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 183 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M0/M0"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 184 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M1/M1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 185 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M2/M2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 186 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M3/M3"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 187 , 50 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M4/M4"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 188 , 0 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant/Compare"
) , TARGET_STRING ( "" ) , 0 , 1 , 0 , 0 , 1 } , { 189 , 26 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 190 , 31 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 191 , 26 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Divide"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 192 , 31 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Divide"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 5 } , { 0 , 0 , ( NULL ) , ( NULL
) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters
rtBlockParameters [ ] = { { 193 , TARGET_STRING (
"Simulink_Simulation/Environment/COESA Atmosphere Model" ) , TARGET_STRING (
"action" ) , 0 , 0 , 0 } , { 194 , TARGET_STRING (
"Simulink_Simulation/Environment/Longitude" ) , TARGET_STRING ( "Value" ) , 0
, 0 , 0 } , { 195 , TARGET_STRING (
"Simulink_Simulation/Environment/Gravity in Earth Axes" ) , TARGET_STRING (
"Gain" ) , 0 , 5 , 0 } , { 196 , TARGET_STRING (
"Simulink_Simulation/Pilot/throttle" ) , TARGET_STRING ( "Value" ) , 0 , 0 ,
0 } , { 197 , TARGET_STRING ( "Simulink_Simulation/Pilot/Rate Limiter" ) ,
TARGET_STRING ( "RisingSlewLimit" ) , 0 , 0 , 0 } , { 198 , TARGET_STRING (
"Simulink_Simulation/Pilot/Rate Limiter" ) , TARGET_STRING (
"FallingSlewLimit" ) , 0 , 0 , 0 } , { 199 , TARGET_STRING (
"Simulink_Simulation/Pilot/Step" ) , TARGET_STRING ( "Time" ) , 0 , 0 , 0 } ,
{ 200 , TARGET_STRING ( "Simulink_Simulation/Pilot/Step" ) , TARGET_STRING (
"Before" ) , 0 , 0 , 0 } , { 201 , TARGET_STRING (
"Simulink_Simulation/Pilot/Step" ) , TARGET_STRING ( "After" ) , 0 , 0 , 0 }
, { 202 , TARGET_STRING (
"Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation"
) , TARGET_STRING ( "DestinationPort" ) , 3 , 0 , 0 } , { 203 , TARGET_STRING
( "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "Gx" ) , 0 , 0 , 0 } , { 204 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "Gy" ) , 0 , 0 , 0 } , { 205 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "Gz" ) , 0 , 0 , 0 } , { 206 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "t_0" ) , 0 , 0 , 0 } , { 207 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "d_m" ) , 0 , 5 , 0 } , { 208 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model" ) ,
TARGET_STRING ( "v_m" ) , 0 , 5 , 0 } , { 209 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "W20" ) , 0 , 0 , 0 } , { 210 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "Wdeg" ) , 0 , 0 , 0 } , { 211 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "TurbProb" ) , 0 , 0 , 0 } , { 212 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "L_high" ) , 0 , 0 , 0 } , { 213 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "Wingspan" ) , 0 , 0 , 0 } , { 214 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "Seed" ) , 0 , 6 , 0 } , { 215 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))"
) , TARGET_STRING ( "T_on" ) , 0 , 0 , 0 } , { 216 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model" ) ,
TARGET_STRING ( "W_20" ) , 0 , 0 , 0 } , { 217 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model" ) ,
TARGET_STRING ( "Wdeg" ) , 0 , 0 , 0 } , { 218 , TARGET_STRING (
"Simulink_Simulation/Pilot/Transfer Fcn (with initial outputs)/State Space" )
, TARGET_STRING ( "A" ) , 0 , 0 , 0 } , { 219 , TARGET_STRING (
"Simulink_Simulation/Pilot/Transfer Fcn (with initial outputs)/State Space" )
, TARGET_STRING ( "B" ) , 0 , 0 , 0 } , { 220 , TARGET_STRING (
"Simulink_Simulation/Pilot/Transfer Fcn (with initial outputs)/State Space" )
, TARGET_STRING ( "C" ) , 0 , 0 , 0 } , { 221 , TARGET_STRING (
"Simulink_Simulation/Pilot/Transfer Fcn (with initial outputs)/State Space" )
, TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 222 , TARGET_STRING
( "Simulink_Simulation/Vehicle System Model/Vehicle/Constant" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 223 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/PacketSize"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 224 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P1" ) , 4 , 7 , 0 } , { 225 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P2" ) , 4 , 7 , 0 } , { 226 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P3" ) , 4 , 5 , 0 } , { 227 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P4" ) , 4 , 5 , 0 } , { 228 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P5" ) , 4 , 5 , 0 } , { 229 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P6" ) , 4 , 5 , 0 } , { 230 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P7" ) , 4 , 5 , 0 } , { 231 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Pack net_fdm Packet for FlightGear"
) , TARGET_STRING ( "P8" ) , 0 , 0 , 0 } , { 232 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Simulation Pace"
) , TARGET_STRING ( "P1" ) , 0 , 0 , 0 } , { 233 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Simulation Pace"
) , TARGET_STRING ( "P2" ) , 0 , 0 , 0 } , { 234 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Simulation Pace"
) , TARGET_STRING ( "P3" ) , 0 , 0 , 0 } , { 235 , TARGET_STRING (
 "Simulink_Simulation/Visualization/FlightGear Preconfigured  6DoF Animation/Simulation Pace"
) , TARGET_STRING ( "P4" ) , 0 , 0 , 0 } , { 236 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)"
) , TARGET_STRING ( "d_m" ) , 0 , 0 , 0 } , { 237 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)"
) , TARGET_STRING ( "d_m" ) , 0 , 0 , 0 } , { 238 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)"
) , TARGET_STRING ( "d_m" ) , 0 , 0 , 0 } , { 239 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/2" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 240 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise"
) , TARGET_STRING ( "pwr" ) , 0 , 6 , 0 } , { 241 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise"
) , TARGET_STRING ( "Ts" ) , 0 , 0 , 0 } , { 242 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model/Wdeg1" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 243 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model/ref_height//z0"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 244 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model/h//z0" ) ,
TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 245 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model/3ft-->inf" ) ,
TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , { 246 , TARGET_STRING (
"Simulink_Simulation/Environment/Wind Models/Wind Shear Model/3ft-->inf" ) ,
TARGET_STRING ( "LowerLimit" ) , 0 , 0 , 0 } , { 247 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Alt Controller"
) , TARGET_STRING ( "A" ) , 0 , 1 , 0 } , { 248 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Alt Controller"
) , TARGET_STRING ( "B" ) , 0 , 0 , 0 } , { 249 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Alt Controller"
) , TARGET_STRING ( "C" ) , 0 , 2 , 0 } , { 250 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Alt Controller"
) , TARGET_STRING ( "D" ) , 0 , 0 , 0 } , { 251 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Theta Controller"
) , TARGET_STRING ( "A" ) , 0 , 0 , 0 } , { 252 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Theta Controller"
) , TARGET_STRING ( "C" ) , 0 , 0 , 0 } , { 253 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Avionics/Autopilot/Theta Controller"
) , TARGET_STRING ( "D" ) , 0 , 0 , 0 } , { 254 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 255 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 256 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/gamma"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 257 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/seaLevelPstatic"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 258 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/seaLevelSOS"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 259 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Merge"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 260 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)"
) , TARGET_STRING ( "pos_ini" ) , 0 , 7 , 0 } , { 261 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)"
) , TARGET_STRING ( "Iyy" ) , 0 , 0 , 0 } , { 262 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)"
) , TARGET_STRING ( "g" ) , 0 , 0 , 0 } , { 263 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator"
) , TARGET_STRING ( "fin_maxrate" ) , 0 , 0 , 0 } , { 264 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator"
) , TARGET_STRING ( "fin_act_0" ) , 0 , 0 , 0 } , { 265 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator"
) , TARGET_STRING ( "fin_act_vel" ) , 0 , 0 , 0 } , { 266 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1"
) , TARGET_STRING ( "fin_maxrate" ) , 0 , 0 , 0 } , { 267 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1"
) , TARGET_STRING ( "fin_act_0" ) , 0 , 0 , 0 } , { 268 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator1"
) , TARGET_STRING ( "fin_act_vel" ) , 0 , 0 , 0 } , { 269 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2"
) , TARGET_STRING ( "fin_maxrate" ) , 0 , 0 , 0 } , { 270 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2"
) , TARGET_STRING ( "fin_act_0" ) , 0 , 0 , 0 } , { 271 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/AirframeActuators/Nonlinear Second-Order Actuator2"
) , TARGET_STRING ( "fin_act_vel" ) , 0 , 0 , 0 } , { 272 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/Thrust_LMN" ) ,
TARGET_STRING ( "Value" ) , 0 , 5 , 0 } , { 273 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/Thrust_YZ" ) ,
TARGET_STRING ( "Value" ) , 0 , 7 , 0 } , { 274 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/ThrustX" ) ,
TARGET_STRING ( "Table" ) , 0 , 8 , 0 } , { 275 , TARGET_STRING (
"Simulink_Simulation/Vehicle System Model/Vehicle/Propulsion/ThrustX" ) ,
TARGET_STRING ( "BreakpointsForDimension1" ) , 0 , 8 , 0 } , { 276 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)/x"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 277 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)/Distance into Gust (x) (Limited to gust length d)"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 278 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (x)/Distance into Gust (x) (Limited to gust length d)"
) , TARGET_STRING ( "LowerSaturationLimit" ) , 0 , 0 , 0 } , { 279 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)/x"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 280 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 281 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (y)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "LowerSaturationLimit" ) , 0 , 0 , 0 } , { 282 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)/x"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 283 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 284 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Discrete Wind Gust Model/Distance into gust (z)/Distance into Gust (x) (Limited to gust length d) "
) , TARGET_STRING ( "LowerSaturationLimit" ) , 0 , 0 , 0 } , { 285 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Lv"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 286 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Lw"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 287 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise/White Noise"
) , TARGET_STRING ( "Mean" ) , 0 , 0 , 0 } , { 288 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise/White Noise"
) , TARGET_STRING ( "StdDev" ) , 0 , 0 , 0 } , { 289 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/const"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 290 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Matrix Gain"
) , TARGET_STRING ( "Gain" ) , 0 , 9 , 0 } , { 291 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Theta"
) , TARGET_STRING ( "WrappedStateUpperValue" ) , 0 , 0 , 0 } , { 292 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Theta"
) , TARGET_STRING ( "WrappedStateLowerValue" ) , 0 , 0 , 0 } , { 293 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 294 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant10"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 295 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant12"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 296 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 297 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 298 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Constant6"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 299 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/zero1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 300 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Xcp"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 301 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Xcp"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 302 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/(Mach)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 6 , 0 } , { 303 , TARGET_STRING
(
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/(alpha)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 11 , 0 } , { 304 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/(altitude)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 12 , 0 } , { 305 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/(delta)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 13 , 0 } , { 306 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Forces and Moments /coefAdjust"
) , TARGET_STRING ( "Gain" ) , 0 , 5 , 0 } , { 307 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/(Mach)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 6 , 0 } , { 308 , TARGET_STRING
(
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/(alpha)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 11 , 0 } , { 309 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/(altitude)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 12 , 0 } , { 310 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 311 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 312 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 313 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 314 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/pgw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 315 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 316 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 317 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 318 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw/pgw_p"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 319 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw/qgw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 320 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw/pi//4"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 321 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw/qgw_p"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 322 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw/rgw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 323 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw/pi//3"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 324 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw/rgw_p"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 325 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)/ugw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 326 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)/(2//pi)"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 327 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)/ug_p"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 328 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/vgw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 329 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/(1//pi)"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 330 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/sqrt(3)"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 331 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/vg_p1"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 332 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)/vgw_p2"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 333 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/wgw"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 334 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 335 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/1//pi"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 336 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/wg_p1"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 337 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)/wg_p2"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 338 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/Medium//High Altitude Intensity"
) , TARGET_STRING ( "Table" ) , 0 , 14 , 0 } , { 339 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/Medium//High Altitude Intensity"
) , TARGET_STRING ( "maxIndex" ) , 2 , 2 , 0 } , { 340 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/PreLook-Up Index Search  (altitude)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 15 , 0 } , { 341 ,
TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity/PreLook-Up Index Search  (prob of exceed)"
) , TARGET_STRING ( "BreakpointsData" ) , 0 , 8 , 0 } , { 342 , TARGET_STRING
(
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/Low Altitude Intensity/sigma_wg "
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 343 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/Low Altitude Intensity/Limit Height h<1000ft"
) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , { 344 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/Low Altitude Intensity/Limit Height h<1000ft"
) , TARGET_STRING ( "LowerLimit" ) , 0 , 0 , 0 } , { 345 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates/max_height_low"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 346 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates/min_height_high"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 347 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Medium//High  altitude rates/Gain"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 348 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities/max_height_low"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 349 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities/min_height_high"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 350 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Medium//High  altitude velocities/Gain"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 351 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Low altitude scale length/Limit Function 10ft to 1000ft"
) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , { 352 , TARGET_STRING (
 "Simulink_Simulation/Environment/Wind Models/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Low altitude scale length/Limit Function 10ft to 1000ft"
) , TARGET_STRING ( "LowerLimit" ) , 0 , 0 , 0 } , { 353 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Relative Ratio/Po"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 354 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Relative Ratio/one"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 355 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Relative Ratio/one1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 356 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Calculate sqrtSigma/Relative Ratio/two"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 357 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "LUTM0" ) , 0 , 16 , 0 } , { 358 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "LUTM1" ) , 0 , 16 , 0 } , { 359 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "LUTM2" ) , 0 , 16 , 0 } , { 360 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "LUTM3" ) , 0 , 16 , 0 } , { 361 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "LUTM4" ) , 0 , 16 , 0 } , { 362 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "V_bp_M0" ) , 0 , 17 , 0 } , { 363 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "V_bp_M1" ) , 0 , 17 , 0 } , { 364 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "V_bp_M2" ) , 0 , 17 , 0 } , { 365 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "V_bp_M3" ) , 0 , 17 , 0 } , { 366 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "V_bp_M4" ) , 0 , 17 , 0 } , { 367 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "P_bp" ) , 0 , 18 , 0 } , { 368 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS"
) , TARGET_STRING ( "SOS_bp" ) , 0 , 19 , 0 } , { 369 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "LUTM0" ) , 0 , 16 , 0 } , { 370 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "LUTM1" ) , 0 , 16 , 0 } , { 371 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "LUTM2" ) , 0 , 16 , 0 } , { 372 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "LUTM3" ) , 0 , 16 , 0 } , { 373 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "LUTM4" ) , 0 , 16 , 0 } , { 374 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "V_bp_M0" ) , 0 , 17 , 0 } , { 375 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "V_bp_M1" ) , 0 , 17 , 0 } , { 376 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "V_bp_M2" ) , 0 , 17 , 0 } , { 377 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "V_bp_M3" ) , 0 , 17 , 0 } , { 378 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "V_bp_M4" ) , 0 , 17 , 0 } , { 379 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "P_bp" ) , 0 , 18 , 0 } , { 380 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS"
) , TARGET_STRING ( "SOS_bp" ) , 0 , 19 , 0 } , { 381 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Determine Force,  Mass & Inertia/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 382 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF (Body Axes)/Transform  to Earth Axes/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 383 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Aileron/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 384 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 385 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCDI"
) , TARGET_STRING ( "Table" ) , 0 , 20 , 0 } , { 386 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCDI"
) , TARGET_STRING ( "dimSizes" ) , 2 , 3 , 0 } , { 387 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCL"
) , TARGET_STRING ( "Table" ) , 0 , 21 , 0 } , { 388 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCL"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 389 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCm"
) , TARGET_STRING ( "Table" ) , 0 , 21 , 0 } , { 390 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Elevator/DCm"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 391 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Aerodynamic Coefficients/Rudder/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 392 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/DerivedConditions/Dynamic Pressure/1//2rhoV^2"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 393 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Body to Wind/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 4 , 0 } , { 394 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "S" ) , 0 , 0 , 0 } , { 395 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "b" ) , 0 , 0 , 0 } , { 396 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "cbar" ) , 0 , 0 , 0 } , { 397 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 398 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 399 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 400 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/zero"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 401 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/zero1"
) , TARGET_STRING ( "Value" ) , 0 , 5 , 0 } , { 402 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Gain1"
) , TARGET_STRING ( "Gain" ) , 0 , 22 , 0 } , { 403 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CLad"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 404 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CLad"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 405 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CLq"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 406 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CLq"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 407 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CYp"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 408 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/CYp"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 409 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Clp"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 410 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Clp"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 411 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Clr"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 412 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Clr"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 413 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cmad"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 414 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cmad"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 415 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cmq"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 416 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cmq"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 417 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cnp"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 418 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cnp"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 419 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cnr"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 420 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Cnr"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 421 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "S" ) , 0 , 0 , 0 } , { 422 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "b" ) , 0 , 0 , 0 } , { 423 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments"
) , TARGET_STRING ( "cbar" ) , 0 , 0 , 0 } , { 424 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/zero3"
) , TARGET_STRING ( "Value" ) , 0 , 5 , 0 } , { 425 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CD"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 426 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CD"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 427 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CL"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 428 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CL"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 429 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CYb"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 430 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/CYb"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 431 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Clb"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 432 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Clb"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 433 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Cm"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 434 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Cm"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 435 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Cnb"
) , TARGET_STRING ( "Table" ) , 0 , 10 , 0 } , { 436 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Cnb"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 437 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 438 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 439 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 440 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Gain"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 441 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 442 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 443 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 444 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 445 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 446 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 447 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 448 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Gain"
) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 449 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 450 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 451 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 452 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 453 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 454 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 455 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 456 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 457 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem1/Compare To Zero/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 458 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem2/Compare To Zero/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 459 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Validate Inputs/If Action Subsystem3/Compare To Zero/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 460 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/Direction Cosine Matrix Body to Wind/A32/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 461 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/coefAdjust"
) , TARGET_STRING ( "Gain" ) , 0 , 5 , 0 } , { 462 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/coefAdjust"
) , TARGET_STRING ( "Gain" ) , 0 , 5 , 0 } , { 463 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 464 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 465 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 466 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 467 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 468 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 469 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 470 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 471 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/denom/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 472 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 473 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e/f"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 474 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e^4/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 475 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M0/M0"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 476 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M0/M0"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 477 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M0/M0"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 478 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M1/M1"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 479 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M1/M1"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 480 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M1/M1"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 481 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M2/M2"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 482 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M2/M2"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 483 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M2/M2"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 484 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M3/M3"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 485 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M3/M3"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 486 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M3/M3"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 487 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M4/M4"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 488 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M4/M4"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 489 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/CAS2TAS/M4/M4"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 490 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M0/M0"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 491 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M0/M0"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 492 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M0/M0"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 493 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M1/M1"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 494 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M1/M1"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 495 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M1/M1"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 496 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M2/M2"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 497 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M2/M2"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 498 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M2/M2"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 499 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M3/M3"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 500 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M3/M3"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 501 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M3/M3"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 502 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M4/M4"
) , TARGET_STRING ( "maxIndex" ) , 2 , 1 , 0 } , { 503 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M4/M4"
) , TARGET_STRING ( "dimSizes" ) , 2 , 1 , 0 } , { 504 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Table Lookup/TAS2CAS/M4/M4"
) , TARGET_STRING ( "numYWorkElts" ) , 2 , 3 , 0 } , { 505 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/3DOF to 6DOF/3DOF to 6DOF/calc alpha_dot/Direction Cosine Matrix Body to Wind/A32/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 506 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/computeAS Subsonic/Bias4"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 507 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/computeAS Subsonic/Bias5"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 508 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/computeAS Subsonic/Bias6"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 509 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/computeAS Subsonic/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 510 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/Supersonic/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 511 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/Supersonic/Bias3"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 512 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/Supersonic/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 513 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/Supersonic/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 514 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/computeQc Subsonic/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 515 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/computeQc Subsonic/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 516 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/computeQc Subsonic/Bias2"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 517 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeQc/computeQc Subsonic/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 518 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/computeAS Subsonic/Bias4"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 519 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/computeAS Subsonic/Bias5"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 520 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/computeAS Subsonic/Bias6"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 521 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/computeAS Subsonic/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 522 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/Supersonic/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 523 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/Supersonic/Bias3"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 524 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/Supersonic/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 525 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/Supersonic/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 526 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/computeQc Subsonic/Bias"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 527 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/computeQc Subsonic/Bias1"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 528 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/computeQc Subsonic/Bias2"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 529 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeQc/computeQc Subsonic/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 530 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 531 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 532 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 533 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 534 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 535 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 536 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 537 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 538 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 539 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 540 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 541 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 542 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 543 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 544 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 545 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Dynamic Contribution/Stability Angular Rates/Direction Cosine Matrix Body to Wind/A32/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 546 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 547 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 548 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 549 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 550 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/CG-CP Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 551 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 552 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 553 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 554 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 555 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Force Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 556 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 557 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 558 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 559 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 560 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Vehicle/Aerodynamics/Digital DATCOM Forces and Moments/Static Contribution/Aerodynamic Forces and Moments/Moment Transformation/Create Transformation/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 561 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/asOut"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 562 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 563 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/Compare To Constant1"
) , TARGET_STRING ( "const" ) , 3 , 0 , 0 } , { 564 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/Memory"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 565 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/asOut"
) , TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 566 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/Compare To Constant"
) , TARGET_STRING ( "const" ) , 0 , 0 , 0 } , { 567 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/Compare To Constant1"
) , TARGET_STRING ( "const" ) , 3 , 0 , 0 } , { 568 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/Memory"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 569 ,
TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Bias4"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 570 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Bias5"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 571 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Bias6"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 572 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 573 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 574 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/CAS2TAS/computeAS/Supersonic/Supersonic/CalcAS/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 575 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Bias4"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 576 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Bias5"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 577 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Bias6"
) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 578 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Constant1"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 579 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Constant3"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 580 , TARGET_STRING (
 "Simulink_Simulation/Vehicle System Model/Flight Sensors/Ideal Airspeed Correction/Other/Equation/TAS2CAS/computeAS/Supersonic/Supersonic/CalcAS/Constant4"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 ,
0 , 0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { {
581 , TARGET_STRING ( "LatLong0" ) , 0 , 7 , 0 } , { 582 , TARGET_STRING (
"Sref" ) , 0 , 0 , 0 } , { 583 , TARGET_STRING ( "V0" ) , 0 , 0 , 0 } , { 584
, TARGET_STRING ( "alpha0" ) , 0 , 0 , 0 } , { 585 , TARGET_STRING ( "alt0" )
, 0 , 0 , 0 } , { 586 , TARGET_STRING ( "bref" ) , 0 , 0 , 0 } , { 587 ,
TARGET_STRING ( "cbar" ) , 0 , 0 , 0 } , { 588 , TARGET_STRING ( "heading0" )
, 0 , 0 , 0 } , { 589 , TARGET_STRING ( "mass" ) , 0 , 0 , 0 } , { 590 ,
TARGET_STRING ( "maxdef_aileron" ) , 0 , 0 , 0 } , { 591 , TARGET_STRING (
"maxdef_elevator" ) , 0 , 0 , 0 } , { 592 , TARGET_STRING ( "maxdef_rudder" )
, 0 , 0 , 0 } , { 593 , TARGET_STRING ( "mindef_aileron" ) , 0 , 0 , 0 } , {
594 , TARGET_STRING ( "mindef_elevator" ) , 0 , 0 , 0 } , { 595 ,
TARGET_STRING ( "mindef_rudder" ) , 0 , 0 , 0 } , { 596 , TARGET_STRING (
"theta0" ) , 0 , 0 , 0 } , { 597 , TARGET_STRING ( "wn_act" ) , 0 , 0 , 0 } ,
{ 598 , TARGET_STRING ( "wy0" ) , 0 , 0 , 0 } , { 599 , TARGET_STRING (
"z_act" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . cjcp2dooxg , & rtB . ivvuf1kwbw ,
& rtB . dv0mxcbo4y , & rtB . nts3iqoscx , & rtB . j2oyzznxjz , & rtB .
hkzk0mcpdz , & rtB . j2oyzznxjz , & rtB . e4lsqrzqjh , & rtB . cjcp2dooxg , &
rtB . ivvuf1kwbw , & rtB . dv0mxcbo4y , & rtB . nts3iqoscx , & rtB .
gsfqejjcfo , & rtB . dlbqs3jbq5 , & rtB . a2xdtn2hjy , & rtB . c2uwt5jnat , &
rtB . j1pkkc3ac5 , & rtB . ohglmemv1w , & rtB . mirhnoymfvo . cwtrsfu0ve , &
rtB . i5wp2ffbry . cwtrsfu0ve , & rtB . lvp0utv5zh , & rtB . drwselis2e , &
rtB . kulaji43pi , & rtB . pxhra0n40a , & rtB . fzxirwzcjx [ 0 ] , & rtB .
or3uazt4fw , & rtB . c2uwt5jnat , & rtB . hevlxlfvxw , & rtB . erkqk25euy , &
rtB . chm2baiq0o , & rtB . jeyqu0v1xd , & rtB . cbt1v4nio1 [ 0 ] , & rtB .
d2gfe2p42l , & rtB . nrzu1kxyjs [ 0 ] , & rtB . nw4xrcz5qc [ 0 ] , & rtB .
ohglmemv1w , & rtB . mirhnoymfvo . cwtrsfu0ve , & rtB . i5wp2ffbry .
cwtrsfu0ve , & rtB . pjsvxmsmf0 , & rtB . agqbvy2aau [ 0 ] , & rtB .
mv5ztdhgch [ 0 ] , & rtB . leujf5c51t [ 0 ] , & rtB . k3touvbkw2 [ 0 ] , &
rtB . ircgwytey3 [ 0 ] , & rtB . bs4j1k4omq , & rtB . bav3d1hobe , & rtB .
dp0qd0ktnr [ 0 ] , & rtB . nsbfkssthw [ 0 ] , & rtB . ewd50jnfc5 , & rtB .
iiwmf2tmrv , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , &
rtB . pkgvmqtajm [ 0 ] , & rtB . fdu3hdd1wq , & rtB . gectl3vjgg , & rtB .
gtmxqvxrzk [ 0 ] , & rtB . dlbqs3jbq5 , & rtB . a2xdtn2hjy , & rtB .
ddawuzzdzz [ 0 ] , & rtB . exldqi4iq0 , & rtB . jfhgj1iwfd , & rtB .
kjlhrll14x , & rtB . c12vlekqly , & rtB . n0vzjwkja5 , & rtB . nf1iua4byr , &
rtB . eyzfoa2rgd , & rtB . chmbb41gxy , & rtB . cchzsu55nk , & rtB .
pstvhythqv , & rtB . akzllltikr , & rtB . p1ud1zl0l5 , & rtB . h1os02q0lx , &
rtB . dojqnhucc1 , & rtB . kkwizmpnnu , & rtB . fffhvqsyn4 , & rtB .
f3yq1g0dcx [ 0 ] , & rtB . plmewtggpi , & rtB . agqbvy2aau [ 0 ] , & rtB .
mv5ztdhgch [ 0 ] , & rtB . fosv245qat [ 0 ] , & rtB . leujf5c51t [ 0 ] , &
rtB . ckuncjpeqo [ 0 ] , & rtB . gehuvldt5n [ 0 ] , & rtB . k3touvbkw2 [ 0 ]
, & rtB . ircgwytey3 [ 0 ] , & rtB . lxqebjchye [ 0 ] , & rtB . c5gx5xznq1 [
0 ] , & rtB . kxuhls1vu4 , & rtB . prdmace1ry , & rtB . pt0yyggmye , & rtB .
pk4ybsmrx1 , & rtB . mb51iyxxjy , & rtB . ntt1gar3xf , & rtB . kbkk0nabqt [ 0
] , & rtB . bz2agq32f5 , & rtB . bz2agq32f5 , & rtB . bz2agq32f5 , & rtB .
gakkrw1mku , & rtB . lldfszhlwc , & rtB . iqcro5zqc5 , & rtB . khr4lzxr01 , &
rtB . otbeb0smep , & rtB . mbiqcrghxm , & rtB . gxcomph2oi , & rtB .
gakkrw1mku , & rtB . lldfszhlwc , & rtB . iqcro5zqc5 , & rtB . khr4lzxr01 , &
rtB . otbeb0smep , & rtB . mbiqcrghxm , & rtB . gxcomph2oi , & rtB .
gakkrw1mku , & rtB . lldfszhlwc , & rtB . iqcro5zqc5 , & rtB . khr4lzxr01 , &
rtB . otbeb0smep , & rtB . mbiqcrghxm , & rtB . gxcomph2oi , & rtB .
pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , &
rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . bf5sj4v0nw , & rtB .
pxggbjiorr , & rtB . onh5wf0mbq , & rtB . pwdhurk11o , & rtB . lauuzxpsuv , &
rtB . kpfxhsrdzc , & rtB . hjs5mprgdw , & rtB . ckkqbbvet5 , & rtB .
knraoiqthj , & rtB . oxryesp4l3 , & rtB . ktgttcsket , & rtB . dtrbet0r1l , &
rtB . o1nidekcl5 , & rtB . bz2agq32f5 , & rtB . bz2agq32f5 , & rtB .
gakkrw1mku , & rtB . iqcro5zqc5 , & rtB . khr4lzxr01 , & rtB . otbeb0smep , &
rtB . mbiqcrghxm , & rtB . gxcomph2oi , & rtB . lldfszhlwc , & rtB .
gakkrw1mku , & rtB . lldfszhlwc , & rtB . iqcro5zqc5 , & rtB . khr4lzxr01 , &
rtB . otbeb0smep , & rtB . mbiqcrghxm , & rtB . gxcomph2oi , & rtB .
pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , &
rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB .
pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , &
rtB . pcxo4cuead , & rtB . ohifllm5xd , & rtB . ot3vk2b455 [ 0 ] , & rtB .
a4jldatvmm , & rtB . mzb2t2xbjn [ 0 ] , & rtB . beyrovclvb , & rtB .
chuqvl4sya , & rtB . hpaztoaur1 , & rtB . b5r5v125p5 , & rtB . euq0m2wkrk , &
rtB . pcxo4cuead , & rtB . lzfl1px3we , & rtB . pcxo4cuead , & rtB .
pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , &
rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB .
pcxo4cuead , & rtB . pcxo4cuead , & rtB . pcxo4cuead , & rtB . oov0isvnf1 , &
rtB . euq0m2wkrk , & rtB . lzfl1px3we , & rtB . euq0m2wkrk , & rtB .
lzfl1px3we , & rtP . COESAAtmosphereModel_action , & rtP . Longitude_Value ,
& rtP . GravityinEarthAxes_Gain [ 0 ] , & rtP . throttle_Value , & rtP .
RateLimiter_RisingLim , & rtP . RateLimiter_FallingLim , & rtP . Step_Time ,
& rtP . Step_Y0 , & rtP . Step_YFinal , & rtP .
FlightGearPreconfigured6DoFAnimation_DestinationPort , & rtP .
DiscreteWindGustModel_Gx , & rtP . DiscreteWindGustModel_Gy , & rtP .
DiscreteWindGustModel_Gz , & rtP . DiscreteWindGustModel_t_0 , & rtP .
DiscreteWindGustModel_d_m [ 0 ] , & rtP . DiscreteWindGustModel_v_m [ 0 ] , &
rtP . DrydenWindTurbulenceModelContinuousqr_W20 , & rtP .
DrydenWindTurbulenceModelContinuousqr_Wdeg , & rtP .
DrydenWindTurbulenceModelContinuousqr_TurbProb , & rtP .
DrydenWindTurbulenceModelContinuousqr_L_high , & rtP .
DrydenWindTurbulenceModelContinuousqr_Wingspan , & rtP .
DrydenWindTurbulenceModelContinuousqr_Seed [ 0 ] , & rtP .
DrydenWindTurbulenceModelContinuousqr_T_on , & rtP . WindShearModel_W_20 , &
rtP . WindShearModel_Wdeg , & rtP . StateSpace_A , & rtP . StateSpace_B , &
rtP . StateSpace_C , & rtP . StateSpace_InitialCondition , & rtP .
Constant_Value_klve0yes4x , & rtP . PacketSize_Value , & rtP .
Packnet_fdmPacketforFlightGear_P1 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P2 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P3 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P4 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P5 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P6 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P7 [ 0 ] , & rtP .
Packnet_fdmPacketforFlightGear_P8 , & rtP . SimulationPace_P1 , & rtP .
SimulationPace_P2 , & rtP . SimulationPace_P3 , & rtP . SimulationPace_P4 , &
rtP . Distanceintogustx_d_m , & rtP . Distanceintogusty_d_m , & rtP .
Distanceintogustz_d_m , & rtP . u_Value , & rtP . WhiteNoise_pwr [ 0 ] , &
rtP . WhiteNoise_Ts , & rtP . Wdeg1_Value , & rtP . ref_heightz0_Value , &
rtP . hz0_Gain , & rtP . uftinf_UpperSat , & rtP . uftinf_LowerSat , & rtP .
AltController_A [ 0 ] , & rtP . AltController_B , & rtP . AltController_C [ 0
] , & rtP . AltController_D , & rtP . ThetaController_A , & rtP .
ThetaController_C , & rtP . ThetaController_D , & rtP .
Constant_Value_iw00e14laz , & rtP . Constant2_Value_nwrewbsrgx , & rtP .
gamma_Value , & rtP . seaLevelPstatic_Value , & rtP . seaLevelSOS_Value , &
rtP . Merge_InitialOutput , & rtP . uDOFBodyAxes_pos_ini [ 0 ] , & rtP .
uDOFBodyAxes_Iyy , & rtP . uDOFBodyAxes_g , & rtP .
NonlinearSecondOrderActuator_fin_maxrate , & rtP .
NonlinearSecondOrderActuator_fin_act_0 , & rtP .
NonlinearSecondOrderActuator_fin_act_vel , & rtP .
NonlinearSecondOrderActuator1_fin_maxrate , & rtP .
NonlinearSecondOrderActuator1_fin_act_0 , & rtP .
NonlinearSecondOrderActuator1_fin_act_vel , & rtP .
NonlinearSecondOrderActuator2_fin_maxrate , & rtP .
NonlinearSecondOrderActuator2_fin_act_0 , & rtP .
NonlinearSecondOrderActuator2_fin_act_vel , & rtP . Thrust_LMN_Value [ 0 ] ,
& rtP . Thrust_YZ_Value [ 0 ] , & rtP . ThrustX_tableData [ 0 ] , & rtP .
ThrustX_bp01Data [ 0 ] , & rtP . x_Y0 , & rtP .
DistanceintoGustxLimitedtogustlengthd_IC , & rtP .
DistanceintoGustxLimitedtogustlengthd_LowerSat , & rtP . mirhnoymfvo . x_Y0 ,
& rtP . mirhnoymfvo . DistanceintoGustxLimitedtogustlengthd_IC , & rtP .
mirhnoymfvo . DistanceintoGustxLimitedtogustlengthd_LowerSat , & rtP .
i5wp2ffbry . x_Y0 , & rtP . i5wp2ffbry .
DistanceintoGustxLimitedtogustlengthd_IC , & rtP . i5wp2ffbry .
DistanceintoGustxLimitedtogustlengthd_LowerSat , & rtP . Lv_Gain , & rtP .
Lw_Gain , & rtP . WhiteNoise_Mean , & rtP . WhiteNoise_StdDev , & rtP .
const_Value , & rtP . MatrixGain_Gain [ 0 ] , & rtP .
Theta_WrappedStateUpperValue , & rtP . Theta_WrappedStateLowerValue , & rtP .
Constant1_Value_aypmrruauh , & rtP . Constant10_Value , & rtP .
Constant12_Value , & rtP . Constant2_Value_dp3hb0zdw4 , & rtP .
Constant3_Value_dde35bhip2 , & rtP . Constant6_Value , & rtP . zero1_Value ,
& rtP . Xcp_Table [ 0 ] , & rtP . Xcp_dimSize [ 0 ] , & rtP .
Mach_BreakpointsData_jg33uk3jci [ 0 ] , & rtP .
alpha_BreakpointsData_gcueg2tgd0 [ 0 ] , & rtP .
altitude_BreakpointsData_p0oyeupgfw [ 0 ] , & rtP . delta_BreakpointsData [ 0
] , & rtP . coefAdjust_Gain_oacgeai0xk [ 0 ] , & rtP . Mach_BreakpointsData [
0 ] , & rtP . alpha_BreakpointsData [ 0 ] , & rtP . altitude_BreakpointsData
[ 0 ] , & rtP . Constant_Value_gddg5y0bon , & rtP .
Constant1_Value_fdgj4x4yvk , & rtP . Constant_Value_iatzfmuax5 , & rtP .
Constant1_Value_jbkvuyvxyw , & rtP . pgw_Y0 , & rtP . Constant1_Value , & rtP
. Constant2_Value , & rtP . Constant3_Value , & rtP . pgw_p_IC , & rtP .
qgw_Y0 , & rtP . pi4_Gain , & rtP . qgw_p_IC , & rtP . rgw_Y0 , & rtP .
pi3_Gain , & rtP . rgw_p_IC , & rtP . ugw_Y0 , & rtP . upi_Gain , & rtP .
ug_p_IC , & rtP . vgw_Y0 , & rtP . upi_Gain_dmjomzxo4q , & rtP . sqrt3_Gain ,
& rtP . vg_p1_IC , & rtP . vgw_p2_IC , & rtP . wgw_Y0 , & rtP .
Constant_Value , & rtP . upi_Gain_ii5rsz2pui , & rtP . wg_p1_IC , & rtP .
wg_p2_IC , & rtP . MediumHighAltitudeIntensity_Table [ 0 ] , & rtP .
MediumHighAltitudeIntensity_maxIndex [ 0 ] , & rtP .
PreLookUpIndexSearchaltitude_BreakpointsData [ 0 ] , & rtP .
PreLookUpIndexSearchprobofexceed_BreakpointsData [ 0 ] , & rtP .
sigma_wg_Gain , & rtP . LimitHeighth1000ft_UpperSat , & rtP .
LimitHeighth1000ft_LowerSat , & rtP . max_height_low_Value , & rtP .
min_height_high_Value , & rtP . Gain_Gain , & rtP .
max_height_low_Value_bkbuta4yqd , & rtP . min_height_high_Value_okjndayw2f ,
& rtP . Gain_Gain_bzpcr2c2vd , & rtP . LimitFunction10ftto1000ft_UpperSat , &
rtP . LimitFunction10ftto1000ft_LowerSat , & rtP . Po_Value , & rtP .
one_Value , & rtP . one1_Value , & rtP . two_Value , & rtP . CAS2TAS_LUTM0 [
0 ] , & rtP . CAS2TAS_LUTM1 [ 0 ] , & rtP . CAS2TAS_LUTM2 [ 0 ] , & rtP .
CAS2TAS_LUTM3 [ 0 ] , & rtP . CAS2TAS_LUTM4 [ 0 ] , & rtP . CAS2TAS_V_bp_M0 [
0 ] , & rtP . CAS2TAS_V_bp_M1 [ 0 ] , & rtP . CAS2TAS_V_bp_M2 [ 0 ] , & rtP .
CAS2TAS_V_bp_M3 [ 0 ] , & rtP . CAS2TAS_V_bp_M4 [ 0 ] , & rtP . CAS2TAS_P_bp
[ 0 ] , & rtP . CAS2TAS_SOS_bp [ 0 ] , & rtP . TAS2CAS_LUTM0 [ 0 ] , & rtP .
TAS2CAS_LUTM1 [ 0 ] , & rtP . TAS2CAS_LUTM2 [ 0 ] , & rtP . TAS2CAS_LUTM3 [ 0
] , & rtP . TAS2CAS_LUTM4 [ 0 ] , & rtP . TAS2CAS_V_bp_M0 [ 0 ] , & rtP .
TAS2CAS_V_bp_M1 [ 0 ] , & rtP . TAS2CAS_V_bp_M2 [ 0 ] , & rtP .
TAS2CAS_V_bp_M3 [ 0 ] , & rtP . TAS2CAS_V_bp_M4 [ 0 ] , & rtP . TAS2CAS_P_bp
[ 0 ] , & rtP . TAS2CAS_SOS_bp [ 0 ] , & rtP . Constant2_Value_gr5f00mfnm , &
rtP . Constant_Value_n35gokrdfy , & rtP . Constant1_Value_n5qw4ehq0c , & rtP
. Constant1_Value_budxhsb1cr , & rtP . DCDI_Table [ 0 ] , & rtP .
DCDI_dimSize [ 0 ] , & rtP . DCL_Table [ 0 ] , & rtP . DCL_dimSize [ 0 ] , &
rtP . DCm_Table [ 0 ] , & rtP . DCm_dimSize [ 0 ] , & rtP .
Constant1_Value_nyr0ia51bl , & rtP . u2rhoV2_Gain , & rtP .
Constant_Value_olio5liftr [ 0 ] , & rtP .
AerodynamicForcesandMoments_S_miuqcjxiwz , & rtP .
AerodynamicForcesandMoments_b , & rtP . AerodynamicForcesandMoments_cbar , &
rtP . u_Value_pyce5cxhso , & rtP . u_Value_nkmodb5shf , & rtP .
u_Value_a3c51jcqno , & rtP . zero_Value , & rtP . zero1_Value_jquxqq4qvz [ 0
] , & rtP . Gain1_Gain [ 0 ] , & rtP . CLad_Table [ 0 ] , & rtP .
CLad_dimSize [ 0 ] , & rtP . CLq_Table [ 0 ] , & rtP . CLq_dimSize [ 0 ] , &
rtP . CYp_Table [ 0 ] , & rtP . CYp_dimSize [ 0 ] , & rtP . Clp_Table [ 0 ] ,
& rtP . Clp_dimSize [ 0 ] , & rtP . Clr_Table [ 0 ] , & rtP . Clr_dimSize [ 0
] , & rtP . Cmad_Table [ 0 ] , & rtP . Cmad_dimSize [ 0 ] , & rtP . Cmq_Table
[ 0 ] , & rtP . Cmq_dimSize [ 0 ] , & rtP . Cnp_Table [ 0 ] , & rtP .
Cnp_dimSize [ 0 ] , & rtP . Cnr_Table [ 0 ] , & rtP . Cnr_dimSize [ 0 ] , &
rtP . AerodynamicForcesandMoments_S , & rtP .
AerodynamicForcesandMoments_b_blpao4glfe , & rtP .
AerodynamicForcesandMoments_cbar_ect3p4btxl , & rtP . zero3_Value [ 0 ] , &
rtP . CD_Table [ 0 ] , & rtP . CD_dimSize [ 0 ] , & rtP . CL_Table [ 0 ] , &
rtP . CL_dimSize [ 0 ] , & rtP . CYb_Table [ 0 ] , & rtP . CYb_dimSize [ 0 ]
, & rtP . Clb_Table [ 0 ] , & rtP . Clb_dimSize [ 0 ] , & rtP . Cm_Table [ 0
] , & rtP . Cm_dimSize [ 0 ] , & rtP . Cnb_Table [ 0 ] , & rtP . Cnb_dimSize
[ 0 ] , & rtP . CompareToConstant_const_omj1fbfglm , & rtP .
Bias_Bias_k5qjjnbf1r , & rtP . Bias1_Bias_oluwwwumlb , & rtP .
Gain_Gain_opd2wvnqlj , & rtP . CompareToConstant_const_d4qgtkvaqb , & rtP .
Bias_Bias_du0tiht0fj , & rtP . Bias1_Bias_nbxpttwzhh , & rtP .
Constant2_Value_f3imtyqlu2 , & rtP . CompareToConstant_const_o4jsn2vuck , &
rtP . Bias_Bias_aujzrf41ca , & rtP . Bias1_Bias_jirwvifxga , & rtP .
Gain_Gain_fmc2fxjyzt , & rtP . CompareToConstant_const_n1tmrxoekt , & rtP .
Bias_Bias_gall5umyqi , & rtP . Bias1_Bias_g0mi2jjfta , & rtP .
Constant2_Value_pkv2f4fmve , & rtP . Constant_Value_bf4fbrpvxe , & rtP .
Constant1_Value_hjtl2lfjek , & rtP . Constant2_Value_dnwbrdkxjw , & rtP .
Constant3_Value_hlef1jp2hn , & rtP . Constant_Value_pmyhtxrqrr , & rtP .
Constant_Value_gun4ws4ai2 , & rtP . Constant_Value_mq1pkcil10 , & rtP .
Constant_Value_jdmivpanfl , & rtP . coefAdjust_Gain_m5bwkewsju [ 0 ] , & rtP
. coefAdjust_Gain [ 0 ] , & rtP . CompareToConstant_const_n1mct5qu1a , & rtP
. Bias_Bias_mahsvgbvzq , & rtP . Bias1_Bias_fiyyfd4u5t , & rtP .
Constant2_Value_g4cw5n3yku , & rtP . CompareToConstant_const_fkrbb1l4we , &
rtP . Bias_Bias_epi214vgpp , & rtP . Bias1_Bias_itbskgc1ah , & rtP .
Constant2_Value_oe5zwfuhbf , & rtP . Constant_Value_ltgnw1q5be , & rtP .
Constant_Value_ddl3nhb2x0 , & rtP . f_Value , & rtP .
Constant_Value_hdq4ys1wku , & rtP . aqq0sq0gty . M0_maxIndex [ 0 ] , & rtP .
aqq0sq0gty . M0_dimSizes [ 0 ] , & rtP . aqq0sq0gty . M0_numYWorkElts [ 0 ] ,
& rtP . aqq0sq0gty . M1_maxIndex [ 0 ] , & rtP . aqq0sq0gty . M1_dimSizes [ 0
] , & rtP . aqq0sq0gty . M1_numYWorkElts [ 0 ] , & rtP . aqq0sq0gty .
M2_maxIndex [ 0 ] , & rtP . aqq0sq0gty . M2_dimSizes [ 0 ] , & rtP .
aqq0sq0gty . M2_numYWorkElts [ 0 ] , & rtP . aqq0sq0gty . M3_maxIndex [ 0 ] ,
& rtP . aqq0sq0gty . M3_dimSizes [ 0 ] , & rtP . aqq0sq0gty . M3_numYWorkElts
[ 0 ] , & rtP . aqq0sq0gty . M4_maxIndex [ 0 ] , & rtP . aqq0sq0gty .
M4_dimSizes [ 0 ] , & rtP . aqq0sq0gty . M4_numYWorkElts [ 0 ] , & rtP .
cjigmwyptgo . M0_maxIndex [ 0 ] , & rtP . cjigmwyptgo . M0_dimSizes [ 0 ] , &
rtP . cjigmwyptgo . M0_numYWorkElts [ 0 ] , & rtP . cjigmwyptgo . M1_maxIndex
[ 0 ] , & rtP . cjigmwyptgo . M1_dimSizes [ 0 ] , & rtP . cjigmwyptgo .
M1_numYWorkElts [ 0 ] , & rtP . cjigmwyptgo . M2_maxIndex [ 0 ] , & rtP .
cjigmwyptgo . M2_dimSizes [ 0 ] , & rtP . cjigmwyptgo . M2_numYWorkElts [ 0 ]
, & rtP . cjigmwyptgo . M3_maxIndex [ 0 ] , & rtP . cjigmwyptgo . M3_dimSizes
[ 0 ] , & rtP . cjigmwyptgo . M3_numYWorkElts [ 0 ] , & rtP . cjigmwyptgo .
M4_maxIndex [ 0 ] , & rtP . cjigmwyptgo . M4_dimSizes [ 0 ] , & rtP .
cjigmwyptgo . M4_numYWorkElts [ 0 ] , & rtP . Constant_Value_fwsa41mimo , &
rtP . Bias4_Bias_odezqxp5ct , & rtP . Bias5_Bias_meya4vfioi , & rtP .
Bias6_Bias_jskr1b1mi4 , & rtP . Constant1_Value_ej01j13uct , & rtP .
Bias1_Bias_hfkjqppc2q , & rtP . Bias3_Bias_jd2w4riqmn , & rtP .
Constant_Value_lbccuktfh2 , & rtP . Constant2_Value_bopb4ki4xd , & rtP .
Bias_Bias_kruphzigt4 , & rtP . Bias1_Bias_ncd3jtrabd , & rtP .
Bias2_Bias_lotf5js1s3 , & rtP . Constant_Value_kueaisvxhv , & rtP .
Bias4_Bias_cy2cy0rkru , & rtP . Bias5_Bias_czj0v4rf55 , & rtP .
Bias6_Bias_cgt3io1in0 , & rtP . Constant1_Value_jb5j0tvdpj , & rtP .
Bias1_Bias , & rtP . Bias3_Bias , & rtP . Constant_Value_nv1uo0d3mg , & rtP .
Constant2_Value_pdjiwz3ftx , & rtP . Bias_Bias , & rtP .
Bias1_Bias_cjyekcjkqd , & rtP . Bias2_Bias , & rtP .
Constant_Value_bfoige4wh4 , & rtP . Constant_Value_lr2cknagpa , & rtP .
Constant1_Value_idys3ziitq , & rtP . Constant2_Value_axatlcv2rs , & rtP .
Constant3_Value_ockepn4d1g , & rtP . Constant4_Value_aprubsstdi , & rtP .
Constant_Value_mbry10wmmx , & rtP . Constant1_Value_ishhczx4h3 , & rtP .
Constant2_Value_bed4d3ljnh , & rtP . Constant3_Value_l1ogwvmgck , & rtP .
Constant4_Value_env11uw4lz , & rtP . Constant_Value_jkivyeh0ov , & rtP .
Constant1_Value_dttko5tnn2 , & rtP . Constant2_Value_d2teymf5hj , & rtP .
Constant3_Value_goletwl4mi , & rtP . Constant4_Value_iqinklay2r , & rtP .
Constant_Value_nm4nmql4xz , & rtP . Constant_Value_dk11ift0i1 , & rtP .
Constant1_Value_e0wp0jqtlj , & rtP . Constant2_Value_b5ilyky2la , & rtP .
Constant3_Value_d4mh0cif3h , & rtP . Constant4_Value_bqgwz3qflq , & rtP .
Constant_Value_pzjjbfwyj5 , & rtP . Constant1_Value_fvdtk4fzi5 , & rtP .
Constant2_Value_ms43idsvzx , & rtP . Constant3_Value_mgyvggblh0 , & rtP .
Constant4_Value_dv4oyrtdrl , & rtP . Constant_Value_aagndz5zvc , & rtP .
Constant1_Value_cviw4ezr5s , & rtP . Constant2_Value_faxwqrunex , & rtP .
Constant3_Value_hkckgaoniw , & rtP . Constant4_Value_ctnx5cdtqt , & rtP .
asOut_Y0_fruk2gwjsm , & rtP . CompareToConstant_const_j4cxad0q0g , & rtP .
CompareToConstant1_const_gvieufb20s , & rtP .
Memory_InitialCondition_lgdo1d41mj , & rtP . asOut_Y0 , & rtP .
CompareToConstant_const , & rtP . CompareToConstant1_const , & rtP .
Memory_InitialCondition , & rtP . Bias4_Bias_pl5eddn2p1 , & rtP .
Bias5_Bias_mcem25dje2 , & rtP . Bias6_Bias_hm1r0zlf2u , & rtP .
Constant1_Value_pb5yn2hc0w , & rtP . Constant3_Value_kvwuhyt22j , & rtP .
Constant4_Value_kox0vku1gy , & rtP . Bias4_Bias , & rtP . Bias5_Bias , & rtP
. Bias6_Bias , & rtP . Constant1_Value_iv5ptljryc , & rtP .
Constant3_Value_h0acrtutky , & rtP . Constant4_Value , & rtP . LatLong0 [ 0 ]
, & rtP . Sref , & rtP . V0 , & rtP . alpha0 , & rtP . alt0 , & rtP . bref ,
& rtP . cbar , & rtP . heading0 , & rtP . mass , & rtP . maxdef_aileron , &
rtP . maxdef_elevator , & rtP . maxdef_rudder , & rtP . mindef_aileron , &
rtP . mindef_elevator , & rtP . mindef_rudder , & rtP . theta0 , & rtP .
wn_act , & rtP . wy0 , & rtP . z_act , } ; static int32_T * rtVarDimsAddrMap
[ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 , 0 } , {
"unsigned char" , "boolean_T" , 0 , 0 , sizeof ( boolean_T ) , SS_BOOLEAN , 0
, 0 , 0 } , { "unsigned int" , "uint32_T" , 0 , 0 , sizeof ( uint32_T ) ,
SS_UINT32 , 0 , 0 , 0 } , { "int" , "int32_T" , 0 , 0 , sizeof ( int32_T ) ,
SS_INT32 , 0 , 0 , 0 } , { "unsigned short" , "uint16_T" , 0 , 0 , sizeof (
uint16_T ) , SS_UINT16 , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } , {
rtwCAPI_VECTOR , 12 , 2 , 0 } , { rtwCAPI_VECTOR , 14 , 2 , 0 } , {
rtwCAPI_VECTOR , 16 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 18 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR_ND , 20 , 3 , 0 } , { rtwCAPI_VECTOR , 23 , 2 , 0 }
, { rtwCAPI_VECTOR , 25 , 2 , 0 } , { rtwCAPI_VECTOR , 27 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 29 , 2 , 0 } , { rtwCAPI_VECTOR , 31 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR_ND , 33 , 3 , 0 } , { rtwCAPI_VECTOR , 36 , 2 , 0 }
, { rtwCAPI_VECTOR , 38 , 2 , 0 } , { rtwCAPI_VECTOR , 40 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR_ND , 42 , 4 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR_ND ,
46 , 3 , 0 } , { rtwCAPI_VECTOR , 49 , 2 , 0 } } ; static const uint_T
rtDimensionArray [ ] = { 1 , 1 , 3 , 1 , 2 , 1 , 4 , 1 , 3 , 3 , 1 , 3 , 1 ,
4 , 1 , 2 , 1 , 7 , 2 , 2 , 10 , 4 , 8 , 1 , 10 , 1 , 8 , 1 , 5 , 12 , 7 , 1
, 12 , 88 , 60 , 61 , 1 , 60 , 1 , 88 , 1 , 61 , 10 , 4 , 8 , 5 , 5 , 4 , 8 ,
1 , 6 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 ,
0.016666666666666666 , 0.1 } ; static const rtwCAPI_FixPtMap rtFixPtMap [ ] =
{ { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static
const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 0 ,
0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ] , ( const void * ) &
rtcapiStoredFloats [ 1 ] , 1 , 0 } , { ( NULL ) , ( NULL ) , 5 , 0 } , { (
const void * ) & rtcapiStoredFloats [ 2 ] , ( const void * ) &
rtcapiStoredFloats [ 0 ] , 2 , 0 } , { ( const void * ) & rtcapiStoredFloats
[ 3 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 4 , 0 } , { ( NULL ) ,
( NULL ) , - 1 , 0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
{ rtBlockSignals , 193 , ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters
, 388 , rtModelParameters , 19 } , { ( NULL ) , 0 } , { rtDataTypeMap ,
rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap ,
rtDimensionArray } , "float" , { 3787883694U , 3449449585U , 185154508U ,
4030028274U } , ( NULL ) , 0 , 0 } ; const rtwCAPI_ModelMappingStaticInfo *
Simulink_Simulation_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void Simulink_Simulation_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion
( ( * rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Simulink_Simulation_host_InitializeDataMapInfo (
Simulink_Simulation_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
