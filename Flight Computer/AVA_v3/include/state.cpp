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
