/**
 * @file motion_primitive_avoider.c
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief avoider using motion primitive
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "modules/orange_avoider/motion_primitive_avoider.h"

#include <stdio.h>
#include <time.h>

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"

#define NAV_C  // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define MOTION_PRIMITIVE_AVOIDER_VERBOSE TRUE

#define PRINT(string, ...)                                                  \
  fprintf(stderr, "[motion_primitive_avoider->%s()] " string, __FUNCTION__, \
          ##__VA_ARGS__)
#if MOTION_PRIMITIVE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

void motion_primitive_avoider_init(void) {

}

void motion_primitive_avoider_periodic(void) {

  // evaluate state machine when flying
  if (!autopilot_in_flight()) {
    return;
  }

  switch (navigation_state)
  {
  case SAFE:
    break;
  
  default:
    break;
  }

}