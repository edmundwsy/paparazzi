/**
 * @file depth_estimate.cpp
 * @author Chenghao Xu (xuchenghao10@hotmail.com)
 * @version 1.0
 * @date 2022-03-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "depth_estimate.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

cv::Mat depth_estimate(char *img, int width, int height)
{
  cv::Mat M(height, width, CV_8UC2, img);
  cv::Mat image, imgBlur, imgHSV, imgThresh, imgDepth, heatImg;

  // Tranpose the vertical image into horizontal image
  cv::transpose(M, image);
  cv::flip(image, image, 0);

  cv::Size dsize = cv::Size(260, 120);
  cv::resize(image, image, dsize, 0, 0, CV_INTER_LINEAR);
  cv::GaussianBlur(image, imgBlur, cv::Size(5, 5), 1);

  // Color Type Transformation
  cv::cvtColor(imgBlur, imgHSV, CV_YUV2BGR_Y422);
  cv::cvtColor(imgHSV, imgHSV, CV_BGR2HSV);

  cv::inRange(imgHSV, cv::Scalar(20, 43, 46), cv::Scalar(60, 255, 255), imgHSV);

  cv::distanceTransform(imgHSV, imgDepth, CV_DIST_L2, 3);

  /*
  cv::convertScaleAbs(imgDepth, imgDepth);
  cv::normalize(imgDepth, imgDepth, 1, 255, cv::NORM_MINMAX);
  cv::applyColorMap(imgDepth, heatImg, cv::COLORMAP_JET);
  */
  return imgDepth;
}
