#ifndef _STATE_H_
#define _STATE_H_

#include <string.h>
#include "tools/std.h"
#include "tools/float_struct.h"


struct State
{
    /* Holds the status bits for all state representations. */
    uint16_t state_status;

    /* Position in Latitude, Longitude and Altitude. Units lat,lon: radians. Units alt: meters above reference ellipsoid */
    struct LlaCoor_f lla_pos_f;

    /* Position in EarthCenteredEarthFixed coordinates. Units: meters */
    struct EcefCoor_f ecef_pos_f;

    /* Altitude above ground level. Unit: meters */
    float alt_agl_f;

    /* Velocity in EarthCenteredEarthFixed coordinates. Units: m/s */
    struct EcefCoor_f ecef_speed_f;

    /* Speed in North East Down coordinates. Details Units: m/s */
    struct NedCoor_f ned_speed_f;

    /* Orientation as quaternion. Units: unit length quaternion */
    struct FloatQuat quat_f;

    /* Orientation in zyx euler angles. Units: rad */
    struct FloatEulers eulers_f;

    /* Orientation rotation matrix. Units: rad */
    struct FloatRMat   rmat_f;

    /* Angular rates in body frame. Units: rad/s */
    struct FloatRates  body_gyro_f;

    /* Acceleration in North East Down coordinates. Units: m/s^2 */
    struct NedCoor_f ned_accel_f;

    /* Horizontal windspeed. Units: m/s with x=north, y=east, z=down */
    union {
        struct FloatVect3 vect3;
        struct FloatVect2 vect2;
    } windspeed_f;


    /* Relative air speed. Unit: m/s */
    float airspeed_f;

    /* Angle of attack Unit: rad */
    float angle_of_attack_f;

    /* Sideslip angle. Unit: rad */
    float sideslip_f;

    /* Actuator Values. Unit: Degrees */
    struct Actuator actuator;

};

void stateInit();
void setStateStatus(uint16_t state_status);
void setStateQuaternion(FloatQuat *quaternion);
void setStateAccelNED(NedCoor_f *body_accel);
void setStateAngularRates(FloatRates *gyro_rates);
void setStateEulers(FloatEulers *euler_angles);


#endif // _STATE_H_