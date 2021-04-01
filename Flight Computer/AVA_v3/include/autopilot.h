#ifndef _AUTOPILOT_H_
#define _AUTOPILOT_H_

#include "tools/std.h"
#include "state.h"


struct Autopilot {
  uint8_t mode;             ///< current autopilot mode
  uint8_t mode_auto2;       ///< FIXME hide this in a private part ?
  uint16_t flight_time;     ///< flight time in seconds
  uint16_t throttle;          ///< throttle level as will be displayed in GCS
  uint8_t arming_status;    ///< arming status
  bool motors_on;           ///< motor status
  bool kill_throttle;       ///< allow autopilot to use throttle
  bool in_flight;           ///< in flight status
  bool launch;              ///< request launch
  bool use_rc;              ///< enable/disable RC input
  bool power_switch;        ///< enable/disable power from power switch (if any)
  bool ground_detected;     ///< automatic detection of landing
  bool detect_ground_once;  ///< enable automatic detection of ground (one shot)
};

#endif