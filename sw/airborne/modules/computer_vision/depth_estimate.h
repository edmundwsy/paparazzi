/**
 * @file depth_estimate.h
 * @author Chenghao Xu (xuchenghao10@hotmail.com)
 * @version 1.0
 * @date 2022-03-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef DEPTH_ESTIMATE_H
#define DEPTH_ESTIMATE_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

  cv::Mat depth_estimate(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif