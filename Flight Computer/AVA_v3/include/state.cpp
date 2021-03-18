#include <Arduino.h>
#include "state.h"

struct State state;

void stateInit(void)
{
  state.state_status = 0;
  state.alt_agl_f = 0;
  state.airspeed_f = 0;
  state.angle_of_attack_f = 0;
  state.sideslip_f = 0;
}

// Dummy Function Testing
int stateSetPosLla(struct LlaCoor_f *lla_pos)
{
  LLA_COPY(state.lla_pos_f, *lla_pos);
  state.state_status = (1 << 5);
  return 0;
}

/**
 * @brief Set the State Status object
 * 
 * @param current_state_status 
 */
void setStateStatus(uint16_t *current_state_status){state.state_status = *current_state_status;}

/**
 * @brief Set the State Quaternion object
 * 
 * @param quaternion 
 */
void setStateQuaternion(FloatQuat *quaternion){QUAT_COPY(state.quat_f, *quaternion)};

/**
 * @brief Set the State Eulers object
 * 
 * @param euler_angles 
 */
void setStateEulers(FloatEulers *euler_angles){EULERS_COPY(state.eulers_f, *euler_angles)};

/**
 * @brief Set the State Angular Rates object
 * 
 * @param gyro_rates 
 */
void setStateAngularRates(FloatRates *gyro_rates){RATES_COPY(state.body_gyro_f, *gyro_rates)};

/**
 * @brief Set the State Accel N E D object
 * 
 * @param body_accel 
 */
void setStateAccelNED(NedCoor_f *body_accel){VECT3_COPY(state.ned_accel_f, *body_accel)};

/**
 * @brief Set the State Actuators object
 * 
 * @param actuator_state 
 */
void setStateActuators(Actuator *actuator_state){ACTUATOR_COPY(state.actuator, *actuator_state)};
