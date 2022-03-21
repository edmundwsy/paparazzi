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
using namespace std;
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"
using namespace std::chrono;
Mat image, image_tmp, src, image2, image1;
Scalar low = Scalar(30, 80, 120);
Scalar high = Scalar(110, 255, 180);
Scalar low2 = Scalar(0, 0, 0);
Scalar high2 = Scalar(150, 150, 150);
Mat kernel = getStructuringElement(MORPH_RECT, Size(8, 8));
RNG rng(42);
Mat stats, centroids;
Mat labels;
Mat dst1;
int valid_labels[20] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
Vec3b colors[20];

int opencv_example(char *img, int width, int height){

  Mat M(height, width, CV_8UC2, img);
  
  cvtColor(M, image, COLOR_YUV2BGR_Y422);

// TODO: undistort


  blur(image, image, Size(5, 5));
  // bilateralFilter(image, image_tmp, 15, 25, 25);
  cv::inRange(image, low, high, image1);
  cv::inRange(image, low2, high2, image2);
  image = 255 - (image2 | image1);
  for (int i=0;i<M.rows;i++)      
  {
    for (int j=0;j<M.cols;j++)
    {
      if(j > M.cols/2){
        image.at<uchar>(i,j)=0;
      }
    }
  }
  morphologyEx(image, src, MORPH_OPEN, kernel);
  labels     = Mat::zeros(src.size(), src.type());
  int num_labels = min(connectedComponentsWithStats(src, labels, stats, centroids, 4),20);

  // Filtering
  for (int i = 0; i < num_labels; i++) {
    if (!(stats.at<int>(i, CC_STAT_WIDTH) < 30 || stats.at<int>(i, CC_STAT_HEIGHT) < 30 || float(width / height) > 5)) {
      valid_labels[i] = i;
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
    if(stats.at<int>(i, CC_STAT_AREA) > 1000){
      circle(dst1, Point(centroids.at<Vec2d>(i, 0)[0], centroids.at<Vec2d>(i, 0)[1]), 2, Scalar(0, 0, 255), -1, 8, 0);         //中心点坐标
      rectangle(dst1, Rect(stats.at<int>(i, CC_STAT_LEFT), stats.at<int>(i, CC_STAT_TOP), stats.at<int>(i, CC_STAT_WIDTH), stats.at<int>(i, CC_STAT_HEIGHT)), Scalar(255, 0, 255), 1, 8, 0);  //外接矩形
    }
    
  }
  
  colorbgr_opencv_to_yuv422(dst1, img, width, height);



  // // Create a new image, using the original bebop image.
  // Mat M(height, width, CV_8UC2, img);
  // Mat image, image_tmp;

  // cvtColor(M, image, COLOR_YUV2BGR_Y422);
  // blur(image, image, Size(3, 3));
  // bilateralFilter(image, image_tmp, 15, 25, 25);
  // Scalar low = Scalar(0, 80, 50);
  // Scalar high = Scalar(20, 255, 100);
  // inRange(image_tmp, low, high, image);
  // // Convert back to YUV422, and put it in place of the original image
  // grayscale_opencv_to_yuv422(image, img, width, height);
  
  return 0;
}

void opencv_example_init(void){};
  // bind our colorfilter callbacks to receive the color filter outputs


void opencv_example_periodic(void){};