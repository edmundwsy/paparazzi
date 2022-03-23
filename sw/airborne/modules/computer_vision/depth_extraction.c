/**
 * @file depth_extraction.c
 * @author Chenghao Xu (xuchenghao10@hotmail.com)
 * @version 1.0
 * @date 2022-03-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "depth_extraction.h"
#include "modules/computer_vision/cv.h"

#ifndef DEPTH_ESTIMATION_FPS
#define DEPTH_ESTIMATION_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

// Function
struct image_t *depth_img(struct image_t *img, uint8_t camera_id);
struct image_t *depth_img(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return depth_estimate((char *)img->buf, img->w, img->h);
}

void potential_vision_init(void)
{
  cv_add_to_device(&DEPTH_ESTIMATION_CAMERA, depth_img, DEPTH_ESTIMATION_FPS, 0);
}

void potential_vision_periodic(void)
{
}