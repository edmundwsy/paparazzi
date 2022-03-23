/* This file was based on opencv_example.cpp
 *
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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "get_flow.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
using namespace cv;
#include "opencv_image_functions.h"


int get_flow(char *prev, char *next,
        struct image_t *flow, double pyr_scale, int levels, int winsize,
        int iterations, int poly_n, double poly_sigma, int flags, double *of_diff, float *div, int w, int h)
{
  // Create a new image, using the original bebop image.
  Mat M1(h, w, CV_8UC2, prev);
  Mat M2(h, w, CV_8UC2, next);
  Mat flowmat(h, w, CV_32FC2);

  Mat prevmat;
  Mat nextmat;

  cvtColor(M1, prevmat, CV_YUV2GRAY_Y422);
  cvtColor(M2, nextmat, CV_YUV2GRAY_Y422);

  float scale = 0.5;
  resize(prevmat, prevmat, Size(), scale, scale, CV_INTER_LINEAR);
  resize(nextmat, nextmat, Size(), scale, scale, CV_INTER_LINEAR);


  calcOpticalFlowFarneback(prevmat, nextmat,
                                 flowmat, pyr_scale, levels, winsize,
                                  iterations,  poly_n,  poly_sigma, flags);



  Mat leftflow = flowmat(Range(0, h/2 * scale), Range(0, w * scale));
  Mat rightflow = flowmat(Range(h/2 * scale, h * scale), Range(0, w * scale));

  Mat magnitudeleft, angleleft, magnituderight, angleright;
  Mat flow_parts_left[2], flow_parts_right[2];
  split(leftflow, flow_parts_left);
  split(rightflow, flow_parts_right);
  cartToPolar(flow_parts_left[0], flow_parts_left[1], magnitudeleft, angleleft, true);
  cartToPolar(flow_parts_right[0], flow_parts_right[1], magnituderight, angleright, true);

  *of_diff = sum(magnituderight)[0] - sum(magnitudeleft)[0];

  Mat flow_parts[2];
  split(flowmat, flow_parts);

  Mat div_ux;
  Mat div_vy;

  Sobel(flow_parts[0], div_ux, CV_8U, 1, 0, 3, 1, 0, BORDER_DEFAULT);
  Sobel(flow_parts[0], div_vy, CV_8U, 0, 1, 3, 1, 0, BORDER_DEFAULT);

  *div = sum(div_ux)[0] + sum(div_vy)[0];

  Mat magnitude, angle, magn_norm;
  cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
  normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
  angle *= ((1.f / 360.f) * (180.f / 255.f));
  //build hsv image
  Mat _hsv[3], hsv, hsv8, bgr;
  _hsv[0] = angle;
  _hsv[1] = Mat::ones(angle.size(), CV_32F);
  _hsv[2] = magn_norm;
  merge(_hsv, 3, hsv);
  hsv.convertTo(hsv8, CV_8U, 255.0);
  cvtColor(hsv8, bgr, COLOR_HSV2BGR);
  colorbgr_opencv_to_yuv422(bgr, prev, w, h);

  return 0;
}
