/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */



#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

int get_flow(char *prev, char *next,
        struct image_t *flow, double pyr_scale, int levels, int winsize,
        int iterations, int poly_n, double poly_sigma, int flags, double *of_diff, float *div, int w, int h);

#ifdef __cplusplus
}
#endif

#endif

