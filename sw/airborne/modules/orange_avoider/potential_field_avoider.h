/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/potential_field_avoider.h"
 * @author Siyuan Wu
 * Example on how to use the potential field to avoid orange pole in the cyberzoo
 */

#ifndef POTENTIAL_FIELD_AVOIDER_H
#define POTENTIAL_FIELD_AVOIDER_H

// settings
extern float K_ATTRACTION;         // strength of attraction force
extern float K_REPULSION;          // strength of repulsion force
extern float PF_GOAL_THRES;        // threshold near the goal
extern float PF_MAX_ITER;          // max iteration of potential field iterations
extern float PF_STEP_SIZE;         // step size between current states and new goal
extern float PF_INFLUENCE_RADIUS;  // distance where repulsion can take effect
extern float PF_MAX_VELOCITY;      // maximum velocity
extern float PF_FORWARD_WEIGHT;    // weight for moving forward
extern float PF_BOUND;             // boundary of the arena

// functions
extern void potential_field_avoider_init(void);
extern void potential_field_avoider_periodic(void);

/** Set position and heading setpoints in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @param heading Setpoint in radians.
 */
extern void guided_pos_ned(float x, float y, float heading);

/** Set position and heading setpoints wrt. current position in GUIDED mode.
 * @param dx Offset relative to current north position (local NED frame) in meters.
 * @param dy Offset relative to current east position (local NED frame) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 */
extern void guided_pos_ned_relative(float dx, float dy, float dyaw);

/** Set position and heading setpoints wrt. current position AND heading in GUIDED mode.
 * @param dx relative position (body frame, forward) in meters.
 * @param dy relative position (body frame, right) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 */
extern void guided_pos_body_relative(float dx, float dy, float dyaw);

/** Set velocity and heading setpoints in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @param heading Setpoint in radians.
 */
extern void guided_move_ned(float vx, float vy, float heading);

/**
 * @brief Set velocity and heading setpoints wrt. current position AND heading in GUIDED mode.
 *
 * @param vx forward velocity  (body frame)
 * @param vy move right velocity (body frame)
 * @param dyaw Offset relative to current heading setpoint in radians.
 */
extern void guided_vel_body_relative(float vx, float vy, float dyaw);

#endif
