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

void setStateStatus(uint16_t *current_state_status)
{
  state.state_status = *current_state_status;
}

// Dummy Function Testing
int stateSetPosLla(struct LlaCoor_f *lla_pos)
{
  LLA_COPY(state.lla_pos_f, *lla_pos);
  state.state_status = (1 << 5);
  return 0;
}