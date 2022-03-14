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

#endif

