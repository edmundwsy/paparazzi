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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */
#include "opencv_example.h"

#include <chrono>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "modules/core/abi.h"
#include "opencv_image_functions.h"
#include "state.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

Mat    image, image_tmp, src, image2, image1, image3;
Scalar low    = Scalar(30, 80, 120);
Scalar high   = Scalar(110, 255, 180);
Scalar low2   = Scalar(0, 0, 0);
Scalar high2  = Scalar(150, 150, 150);
Scalar low3   = Scalar(180, 150, 50);
Scalar high3  = Scalar(200, 170, 70);
Mat    kernel = getStructuringElement(MORPH_RECT, Size(8, 8));
RNG    rng(42);
Mat    stats, centroids;
Mat    labels;
Mat    dst1;
int    valid_labels[20] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
Vec3b  colors[20];
int    obs_num_detected = 0;
Mat    camera_matrix_   = (cv::Mat_<float>(3, 3) << 282.9480391701469, -0.527071354834133,
                      28.9068266678759, 0, 282.665185424907, 275.109826551610, 0, 0, 1);
Mat distor_coeffs = (cv::Mat_<float>(1, 5) << -0.3122, 0.0860, -0.0004594689988183520, -0.0020, 0);
float depth_array[5]      = {0};
float x_position_array[5] = {0};
float y_position_array[5] = {0};
float f_y =
    camera_matrix_.at<float>(1, 1);  // intrinsic y aperature with image in original vertical
                                     // orientation, which means it is equal to the width
float f_x = camera_matrix_.at<float>(0, 0);
float c_y = camera_matrix_.at<float>(1, 2);
float c_x = camera_matrix_.at<float>(0, 2);
float y_0 = 330;  // 330 mm width of the pole in real life in original vertical orientation

int opencv_example(char *img, int width, int height) {
  obs_num_detected = 0;
  Mat M(height, width, CV_8UC2, img);
  Mat M1;

  cvtColor(M, image, COLOR_YUV2BGR_Y422);

  undistort(image, M1, camera_matrix_, distor_coeffs);

  blur(M1, image, Size(5, 5));
  // bilateralFilter(image, image_tmp, 15, 25, 25);
  cv::inRange(image, low, high, image1);
  cv::inRange(image, low2, high2, image2);
  cv::inRange(image, low3, high3, image3);
  image = 255 - (image2 | image1 | image3);
  for (int i = 0; i < M.rows; i++) {
    for (int j = 0; j < M.cols; j++) {
      if (j > M.cols / 2) {
        image.at<uchar>(i, j) = 0;
      }
    }
  }
  morphologyEx(image, src, MORPH_OPEN, kernel);
  labels         = Mat::zeros(src.size(), src.type());
  int num_labels = min(connectedComponentsWithStats(src, labels, stats, centroids, 4), 20);

  // Filtering and calculate depth
  int j = 0;
  for (int i = 1; i < num_labels; i++) {
    if (!(stats.at<int>(i, CC_STAT_WIDTH) < 30 || stats.at<int>(i, CC_STAT_HEIGHT) < 30 ||
          float(stats.at<int>(i, CC_STAT_WIDTH) / stats.at<int>(i, CC_STAT_HEIGHT)) > 5 ||
          stats.at<int>(i, CC_STAT_AREA) < 1000)) {
      valid_labels[i] = i;
      obs_num_detected++;

      /* Modified by siyuan, copied from Koen wrote below */
      depth_array[j]      = ((f_y * y_0) / stats.at<int>(i, CC_STAT_HEIGHT));
      y_position_array[j] = ((centroids.at<Vec2d>(i, 0)[0] - c_x) * depth_array[j]) /
                            f_x;  // y_position_array gives the y position of the obstacle with a
                                  // flipped frame, in original orientation this is the x axis
      x_position_array[j] = ((centroids.at<Vec2d>(i, 0)[1] - c_y) * depth_array[j]) /
                            f_y;  // x_position_array gives the x position of the obstacle with a
                                  // flipped frame, in original orientation this is the y axis

      depth_array[j]      = depth_array[j] / 1000;
      y_position_array[j] = x_position_array[j] / 1000;
      x_position_array[j] = y_position_array[j] / 1000;
      printf("obstacle depth (meters): %f\n", depth_array[j]);
      printf("position obstacle x: %f   y: %f\n", x_position_array[j], y_position_array[j]);
      j++;
    }
  }
  // generate background color => black
  colors[0] = Vec3b(0, 0, 0);
  // generate region color => random
  for (int i = 1; i < num_labels; i++) {
    colors[i] = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
  }
  // display components, drawing
  dst1 = Mat::zeros(src.size(), CV_8UC3);
  for (int row = 0; row < src.rows; row++) {
    for (int col = 0; col < src.cols; col++) {
      if (valid_labels[labels.at<int>(row, col)] == -1) {
        continue;
      }
      dst1.at<Vec3b>(row, col) = colors[labels.at<int>(row, col)];
    }
  }

  // static and drawing
  for (int i = 1; i < num_labels; i++) {
    if (valid_labels[i] == i) {
      circle(dst1, Point(centroids.at<Vec2d>(i, 0)[0], centroids.at<Vec2d>(i, 0)[1]), 2,
             Scalar(0, 0, 255), -1, 8, 0);  //中心点坐标
      rectangle(dst1,
                Rect(stats.at<int>(i, CC_STAT_LEFT), stats.at<int>(i, CC_STAT_TOP),
                     stats.at<int>(i, CC_STAT_WIDTH), stats.at<int>(i, CC_STAT_HEIGHT)),
                Scalar(255, 0, 255), 1, 8, 0);  //外接矩形
    }
  }
  printf("obstacle NUMBER: %i\n", obs_num_detected);
  // Calculate the depth (Koen Method)
  // https://mayavan95.medium.com/3d-position-estimation-of-a-known-object-using-a-single-camera-7a82b37b326b
  // assumptions:
  // int j = 0;
  // for (int i = 1; i < num_labels; i++) {
  //   if (valid_labels[i] == i) {
  // printf("obstacle height (pixels): %f\n",
  //        stats.at<int>(i, CC_STAT_HEIGHT));  // should be width when frame is rotated
  // printf("obstacle width (pixels): %f\n",
  //        stats.at<int>(i, CC_STAT_WIDTH));  // should be height when frame is rotated
  // printf("obstacle center x (pixels): %f\n", centroids.at<Vec2d>(i, 0)[0]);
  // printf("obstacle center y (pixels): %f\n", centroids.at<Vec2d>(i, 0)[1]);

  // depth_array1[j] = (camera_matrix_.at<float>(0,0) * -1200)/(stats.at<int>(i, CC_STAT_LEFT) -
  // camera_matrix_.at<float>(0,2) - camera_matrix_.at<float>(0,1) * (stats.at<int>(i,
  // CC_STAT_TOP)+stats.at<int>(i,
  // CC_STAT_HEIGHT)/2-camera_matrix_.at<float>(1,2))/camera_matrix_.at<float>(1,1))/1000;

  // }
  // printf("obstacle depth (meters): %f\n", depth_array[i]);
  // }

  // colorbgr_opencv_to_yuv422(dst1, img, width, height);

  AbiSendMsgOBSTACLE_ESTIMATION(1, obs_num_detected, x_position_array[0], y_position_array[0],
                                x_position_array[1], y_position_array[1], x_position_array[2],
                                y_position_array[2], x_position_array[3], y_position_array[3],
                                x_position_array[4], y_position_array[4]);
  return 0;
}

void opencv_example_init(void){};
// bind our colorfilter callbacks to receive the color filter outputs

void opencv_example_periodic(void){};