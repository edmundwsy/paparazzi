/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/potential_field_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef POTENTIAL_FIELD_AVOIDER_H
#define POTENTIAL_FIELD_AVOIDER_H

// settings
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_floor_count_frac;  // floor detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]
// functions
extern void potential_field_avoider_init(void);
extern void potential_field_avoider_periodic(void);

/** Set position and heading setpoints in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern void guided_goto_ned(float x, float y, float heading);

/** Set position and heading setpoints wrt. current position in GUIDED mode.
 * @param dx Offset relative to current north position (local NED frame) in meters.
 * @param dy Offset relative to current east position (local NED frame) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern void guided_goto_ned_relative(float dx, float dy, float dyaw);

/** Set position and heading setpoints wrt. current position AND heading in GUIDED mode.
 * @param dx relative position (body frame, forward) in meters.
 * @param dy relative position (body frame, right) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern void guided_goto_body_relative(float dx, float dy, float dyaw);

/** Set velocity and heading setpoints in GUIDED mode.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern void guided_move_ned(float vx, float vy, float heading);

#endif

