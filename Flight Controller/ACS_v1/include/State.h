#ifndef _STATE_H_
#define _STATE_H_

#include <string.h>
#include "tools/std.h"
#include "tools/float_struct.h"


class State
{
  public:
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
    union 
    {
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

    void stateInit();
    int stateSetPosLla(struct LlaCoor_f *lla_pos);
    void setStateStatus(uint16_t *current_state_status);
    void setStateQuaternion(FloatQuat *quaternion);
    void setStateAccelNED(NedCoor_f *body_accel);
    void setStateAngularRates(FloatRates *gyro_rates);
    void setStateEulers(FloatEulers *euler_angles);
    void setStateActuators(Actuator *actuator_state);
};

void State::stateInit(void)
{
  this->state_status = 0;
  this->alt_agl_f = 0;
  this->airspeed_f = 0;
  this->angle_of_attack_f = 0;
  this->sideslip_f = 0;
}

int State::stateSetPosLla(struct LlaCoor_f *lla_pos)
{
  LLA_COPY(this->lla_pos_f, *lla_pos);
  this->state_status = (1 << 5);
  return 0;
}

/**
 * @brief Set the State Status object
 * 
 * @param current_state_status 
 */
void State::setStateStatus(uint16_t *current_state_status){this->state_status = *current_state_status;}

/**
 * @brief Set the State Quaternion object
 * 
 * @param quaternion 
 */
void State::setStateQuaternion(FloatQuat *quaternion){QUAT_COPY(this->quat_f, *quaternion)};

/**
 * @brief Set the State Eulers object
 * 
 * @param euler_angles 
 */
void State::setStateEulers(FloatEulers *euler_angles){EULERS_COPY(this->eulers_f, *euler_angles)};

/**
 * @brief Set the State Angular Rates object
 * 
 * @param gyro_rates 
 */
void State::setStateAngularRates(FloatRates *gyro_rates){RATES_COPY(this->body_gyro_f, *gyro_rates)};

/**
 * @brief Set the State Accel N E D object
 * 
 * @param body_accel 
 */
void State::setStateAccelNED(NedCoor_f *body_accel){VECT3_COPY(this->ned_accel_f, *body_accel)};

/**
 * @brief Set the State Actuators object
 * 
 * @param actuator_state 
 */
void State::setStateActuators(Actuator *actuator_state){ACTUATOR_COPY(this->actuator, *actuator_state)};


#endif // _STATE_H_