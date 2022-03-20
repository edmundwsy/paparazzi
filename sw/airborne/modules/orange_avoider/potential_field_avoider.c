/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/potential_field_avoider.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at
 * the TU Delft. This module is used in combination with a color filter (cv_detect_color_object) and
 * the guided mode of the autopilot. The avoidance strategy is to simply count the total number of
 * orange pixels. When above a certain percentage threshold, (given by color_count_frac) we assume
 * that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple
 * filters simultaneously so you have to define which filter to use with the
 * POTENTIAL_FIELD_AVOIDER_VISUAL_DETECTION_ID setting. This module differs from the simpler
 * orange_avoider.xml in that this is flown in guided mode. This flight mode is less dependent on a
 * global positioning estimate as with the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with
 * the nets. For this we employ a simple color detector, similar to the orange poles but for green
 * to detect the floor. When the total amount of green drops below a given threshold (given by
 * floor_count_frac) we assume we are near the edge of the zoo and turn around. The color detection
 * is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to define
 * which filter to use.
 */

#include "modules/orange_avoider/potential_field_avoider.h"

#include <stdio.h>
#include <time.h>
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "modules/core/abi.h"
#include "state.h"
#define POTENTIAL_FIELD_AVOIDER_VERBOSE TRUE

#define PRINT(string, ...) \
  fprintf(stderr, "[potential_field_avoider->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if POTENTIAL_FIELD_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t           chooseRandomIncrementAvoidance(void);
float             computeDistance(float obs_x, float obs_y);
struct FloatVect2 potentialFieldPosUpdate(struct FloatVect2 goal, struct FloatVect2 curpt);
struct FloatVect2 potentialFieldVelUpdate(struct FloatVect2 goal, struct FloatVect2 curpt);

enum navigation_state_t { SAFE, PLANNING, WAIT_TARGET, EMERGENCY, OUT_OF_BOUNDS, REENTER_ARENA };

// define settings
float K_ATTRACTION        = 10;  // strength of attraction force
float K_REPULSION         = 10;   // strength of repulsion force
float PF_GOAL_THRES       = 0.2;  // threshold near the goal
float PF_MAX_ITER         = 10;   // max iteration of potential field iterations
float PF_STEP_SIZE        = 0.7;  // step size between current states and new goal
float PF_INFLUENCE_RADIUS = 1.0;  // distance where repulsion can take effect

// define and initialise global variables
enum navigation_state_t navigation_state = WAIT_TARGET;  // current state in state machine
int32_t color_count    = 0;  // orange color count from color filter for obstacle detection
int32_t floor_count    = 0;  // green color count from color filter for floor detection
int32_t floor_centroid = 0;  // floor detector centroid in y direction (along the horizon)
float   avoidance_heading_direction = 0.3;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence =
    0;  // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence =
    5;  // number of consecutive negative object detections to be sure we are obstacle free

// Define obstacle position
#define NUM_OBS 5
#define NUM_WPS 10
// array of obstacles
// TODO(@vision group): give these arguments
struct FloatVect2 _obs[NUM_OBS] = {
    // {0.6f, 0.7f}, {2.5f, 2.8f}, {1.5f, -2.5f}, {-3.4f, -1.8f}, {-1.8f, 0.5f}};
    {0.6f, 0.7f}, {2.5f, 2.8f}, {-2.5f, 1.5f}, { -1.8f, -3.4f}, {0.5f, -1.8f}};
// array of all waypoints
struct FloatVect2 _wps[NUM_WPS];
// array of all goals
int               _goal_flag = 0;
struct FloatVect2 _goals[4]  = {{2.0f, 2.0f}, {-2.0f, 2.0f}, {-2.0f, -2.0f}, {2.0f, -2.0f}};
struct FloatVect2 _goal;

// This call back will be used to receive the color count from the orange detector
// #ifndef POTENTIAL_FIELD_AVOIDER_VISUAL_DETECTION_ID
// #error This module requires two color filters, as such you have to define
// POTENTIAL_FIELD_AVOIDER_VISUAL_DETECTION_ID to the orange filter #error Please define
// POTENTIAL_FIELD_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or
// COLOR_OBJECT_DETECTION2_ID in your airframe #endif static abi_event color_detection_ev; static
// void      color_detection_cb(uint8_t __attribute__((unused)) sender_id,
//                                     int16_t __attribute__((unused)) pixel_x,
//                                     int16_t __attribute__((unused)) pixel_y,
//                                     int16_t __attribute__((unused)) pixel_width,
//                                     int16_t __attribute__((unused)) pixel_height, int32_t
//                                     quality, int16_t __attribute__((unused)) extra) {
//   color_count = quality;
// }

// #ifndef FLOOR_VISUAL_DETECTION_ID
// #error This module requires two color filters, as such you have to define
// FLOOR_VISUAL_DETECTION_ID to the orange filter #error Please define FLOOR_VISUAL_DETECTION_ID to
// be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe #endif static
// abi_event floor_detection_ev; static void      floor_detection_cb(uint8_t __attribute__((unused))
// sender_id,
//                                     int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
//                                     int16_t __attribute__((unused)) pixel_width,
//                                     int16_t __attribute__((unused)) pixel_height, int32_t
//                                     quality, int16_t __attribute__((unused)) extra) {
//   floor_count    = quality;
//   floor_centroid = pixel_y;
// }

// needed to receive output from a separate module running on a parallel process
int32_t x_flow=-1, y_flow=-1;
#ifndef FLOW_OPTICFLOW_CAM1_ID
#define FLOW_OPTICFLOW_CAM1_ID ABI_BROADCAST
#endif
static abi_event opticflow_ev;
static void opticflow_cb(uint8_t __attribute__((unused)) sender_id,
                         uint32_t __attribute__((unused)) stamp, 
                         int32_t flow_x, 
                         int32_t flow_y,
                         int32_t flow_der_x, 
                         int32_t flow_der_y,
                         float __attribute__((unused)) quality, 
                         float size_divergence) {
  x_flow = flow_x;
  y_flow = flow_y;
}

/*
 * Initialisation function
 */
void potential_field_avoider_init(void) {
  // Initialise random values
  srand(time(NULL));
  _goal      = _goals[0];
  _goal_flag = 0;
  VERBOSE_PRINT("[goal] Set goal at (%.2f, %.2f)\n", _goal.x, _goal.y);
  // bind our colorfilter callbacks to receive the color filter outputs
  // AbiBindMsgVISUAL_DETECTION(POTENTIAL_FIELD_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev,
  //                            color_detection_cb);
  // AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &opticflow_ev, opticflow_cb);
}

void potential_field_avoider_periodic(void) {
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SAFE;
    VERBOSE_PRINT("[GUIDE] guidance_h.mode is %i \n", guidance_h.mode);
    return;
  }
  switch (navigation_state) {
    case SAFE:
      VERBOSE_PRINT("======== SAFE ========\n");
      struct FloatVect2 state = {stateGetPositionNed_f()->x, stateGetPositionNed_f()->y};
      VERBOSE_PRINT("[state] (%.2f, %.2f)\n", state.x, state.y);
      struct FloatVect2 wpt = potentialFieldPosUpdate(_goal, state);

      /* distance to the goal */
      struct FloatVect2 diff;
      VECT2_DIFF(diff, _goal, wpt);
      float distance = VECT2_NORM2(diff);

      if (distance > PF_GOAL_THRES) {
        guidance_h_set_guided_pos(wpt.x, wpt.y);
        float agl = atan2f(wpt.y - state.y, wpt.x - state.x);
        guidance_h_set_guided_heading(agl);
        // guidance_h_set_guided_body_vel(0.5, 0);

        VERBOSE_PRINT("[state] current heading is %.3f \n", stateGetNedToBodyEulers_f()->psi);
        VERBOSE_PRINT("[state] current atan2f is %.3f \n", agl);
        // nav_set_heading_towards(wpt.x, wpt.y);
        // guidance_h_set_guided_vel(wpt.x, wpt.y);
      } else {
        navigation_state = PLANNING;
      }

      // TODO(@siyuan): understand how controller controls to set points
      // TODO(@siyuan): understand the execution time
      break;

    case PLANNING:
      VERBOSE_PRINT("======== PLANNING ========\n");
      _goal_flag++;
      if (_goal_flag == 4) {
        _goal_flag = 0;
      }
      _goal = _goals[_goal_flag];
      VERBOSE_PRINT("[goal] Set goal at (%.2f, %.2f)\n", _goal.x, _goal.y);
      navigation_state = SAFE;
      break;

    case EMERGENCY:
      VERBOSE_PRINT("FSM: ======== EMERGENCY ========\n");
      // step back if closed to obstacles
      guided_goto_body_relative(-0.5, 0, 0);
      navigation_state = SAFE;
      break;

    case WAIT_TARGET:
      VERBOSE_PRINT("FSM: ======== WAIT_TARGET ========\n");

      // turning slowly
      guidance_h_set_guided_heading_rate(RadOfDeg(5));

      break;

    case OUT_OF_BOUNDS:
      // TODO(@vision): detect out of bounds using bottom camera
      VERBOSE_PRINT("FSM: ======== OUT_OF_BOUNDS ========\n");
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;
      break;

    case REENTER_ARENA:
      VERBOSE_PRINT("FSM: ======== REENTER_ARENA ========\n");
      // force floor center to opposite side of turn to head back into arena
      // if (floor_count >= floor_count_threshold &&
      //     avoidance_heading_direction * floor_centroid_frac >= 0.f) {
      if (stateGetPositionEnu_f()->x > 2.5 || stateGetPositionEnu_f()->y > 2.5) {
        guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        guidance_h_set_guided_pos(0.5f, 0.0f);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
}

void guided_goto_ned(float x, float y, float heading) {
  guidance_h_set_guided_pos(x, y);
  guidance_h_set_guided_heading(heading);
}

void guided_goto_ned_relative(float dx, float dy, float dyaw) {
  float x       = stateGetPositionNed_f()->x + dx;
  float y       = stateGetPositionNed_f()->y + dy;
  float heading = stateGetNedToBodyEulers_f()->psi + dyaw;
  guided_goto_ned(x, y, heading);
}

void guided_goto_body_relative(float dx, float dy, float dyaw) {
  float psi     = stateGetNedToBodyEulers_f()->psi;
  float x       = stateGetPositionNed_f()->x + cosf(-psi) * dx + sinf(-psi) * dy;
  float y       = stateGetPositionNed_f()->y - sinf(-psi) * dx + cosf(-psi) * dy;
  float heading = psi + dyaw;
  guided_goto_ned(x, y, heading);
}

void guided_move_ned(float vx, float vy, float heading) {
  guidance_h_set_guided_vel(vx, vy);
  guidance_h_set_guided_heading(heading);
}

float computeDistance(float obs_x, float obs_y) {
  float x0 = stateGetPositionNed_f()->x;
  float y0 = stateGetPositionNed_f()->y;
  return sqrtf(SQUARE(x0 - obs_x) + SQUARE(y0 - obs_y));
}

struct FloatVect2 attractive(struct FloatVect2 goal, struct FloatVect2 current) {
  struct FloatVect2 att;
  VECT2_DIFF(att, goal, current);
  VECT2_SMUL(att, att, K_ATTRACTION);
  return att;
}

struct FloatVect2 repulsion(struct FloatVect2* obs, struct FloatVect2 current) {
  struct FloatVect2 rep, tmp, dir;
  VECT2_ASSIGN(rep, 0.0f, 0.0f);
  for (int i = 0; i < NUM_OBS; i++) {
    VECT2_DIFF(tmp, current, obs[i]);
    float distance = VECT2_NORM2(tmp);
    if (distance > PF_INFLUENCE_RADIUS) {
      continue;
    } else {
      VECT2_SDIV(dir, tmp, distance);
      VERBOSE_PRINT("[REP] distance is (%.2f)\n", distance);
      float u = K_REPULSION * (1.0f / distance - 1.0f / PF_INFLUENCE_RADIUS) / (SQUARE(distance));
      VERBOSE_PRINT("[REP] repulsion gain is (%.2f)\n", u);
      VECT2_SMUL(dir, dir, u);
      VECT2_ADD(rep, dir);
    }
  }
  VERBOSE_PRINT("[REP] computed repulsion direction is (%.2f, %.2f)\n", rep.x, rep.y);
  return rep;
}

struct FloatVect2 potentialFieldPosUpdate(struct FloatVect2 goal, struct FloatVect2 curpt) {
  struct FloatVect2 newpt;               // new position
  struct FloatVect2 force;               // potential force
  struct FloatVect2 diret;               // direction
  float             dis_to_goal = 0.0f;  // distance to goal
  int               iter        = 0;

  struct FloatVect2 att = attractive(goal, curpt);
  struct FloatVect2 rep = repulsion(&_obs, curpt);
  VECT2_SUM(force, att, rep);
  VECT2_SDIV(force, force, sqrtf(VECT2_NORM2(force)));
  VERBOSE_PRINT("[UPDATE] computed force as (%.2f, %.2f)\n", force.x, force.y);
  VECT2_SMUL(diret, force, PF_STEP_SIZE);
  VECT2_SUM(newpt, curpt, diret);
  VERBOSE_PRINT("[UPDATE] computed new waypoint as (%.2f, %.2f)\n", newpt.x, newpt.y);
  return newpt;
}

struct FloatVect2 potentialFieldVelUpdate(struct FloatVect2 goal, struct FloatVect2 curpt) {
  struct FloatVect2 newpt;               // new position
  struct FloatVect2 force;               // potential force
  struct FloatVect2 diret;               // direction
  float             dis_to_goal = 0.0f;  // distance to goal
  int               iter        = 0;

  struct FloatVect2 att = attractive(goal, curpt);
  struct FloatVect2 rep = repulsion(&_obs, curpt);
  VECT2_SUM(force, att, rep);
  VECT2_SDIV(force, force, sqrtf(VECT2_NORM2(force)));
  VERBOSE_PRINT("[UPDATE] computed force as (%.2f, %.2f)\n", force.x, force.y);
  VECT2_SMUL(diret, force, PF_STEP_SIZE);
  VERBOSE_PRINT("[UPDATE] computed new speed as (%.2f, %.2f)\n", diret.x, diret.y);
  return diret;
}

struct FloatVect2 fakeObservation(struct FloatVect2 pos_global) {}

bool potentialFieldPathPlan(struct FloatVect2 goal, struct FloatVect2 curpt) {
  // struct FloatVect2 curpt;               // current position
  struct FloatVect2 newpt;               // new position
  struct FloatVect2 force;               // potential force
  struct FloatVect2 diret;               // direction
  float             dis_to_goal = 0.0f;  // distance to goal
  int               iter        = 0;
  while ((iter < PF_MAX_ITER) && dis_to_goal > PF_GOAL_THRES) {
    struct FloatVect2 att = attractive(goal, curpt);
    struct FloatVect2 rep = repulsion(&_obs, curpt);
    VECT2_SUM(force, att, rep);
    VECT2_SDIV(force, force, VECT2_NORM2(force));
    VECT2_SMUL(diret, force, PF_STEP_SIZE);
    VECT2_SUM(newpt, curpt, diret);
    iter++;
    struct FloatVect2 diff;
    VECT2_DIFF(diff, goal, newpt);
    dis_to_goal = VECT2_NORM2(diff);
  }

  if (dis_to_goal <= PF_GOAL_THRES) {
    return true;
  }
  return false;
}