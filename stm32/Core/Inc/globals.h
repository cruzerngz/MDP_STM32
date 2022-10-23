// Globals header
// Contains all configurable global files

#include "stdint.h"
#include "stdbool.h"

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

extern uint32_t _GLOBAL_MOVE_KP;
extern uint32_t _GLOBAL_MOVE_KI;
extern uint32_t _GLOBAL_MOVE_KD;

extern uint32_t _GLOBAL_MOVE_DEFAULT_SPEED_STRAIGHT_MM_S;
extern uint32_t _GLOBAL_MOVE_DEFAULT_SPEED_TURN_MM_S;

extern bool _GLOBAL_MOVE_SURFACE_LOW_GRIP;

#endif // INC_GLOBALS_H_
