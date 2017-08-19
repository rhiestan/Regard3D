#ifndef __OPENCV_FEATURES_2D_KAZE_UTILS_H__
#define __OPENCV_FEATURES_2D_KAZE_UTILS_H__

#include <opencv2/core/cvdef.h>
#include <cmath>

/* ************************************************************************* */
/**
 * @brief This function computes the angle from the vector given by (X Y). From 0 to 2*Pi
 */
inline float getAngleV2(float x, float y) {

    float theta = atan2f(y,x);

    if (theta >= 0)
	return theta;
    else
	return theta + static_cast<float>(2.0f*CV_PI);
}

/* ************************************************************************* */
/**
 * @brief This function computes the value of a 2D Gaussian function
 * @param x X Position
 * @param y Y Position
 * @param sig Standard Deviation
 */
inline float gaussianV2(float x, float y, float sigma) {
  return expf(-(x*x + y*y) / (2.0f*sigma*sigma));
}

/* ************************************************************************* */
/**
 * @brief This function checks descriptor limits
 * @param x X Position
 * @param y Y Position
 * @param width Image width
 * @param height Image height
 */
inline void checkDescriptorLimitsV2(int &x, int &y, int width, int height) {

  if (x < 0) {
    x = 0;
  }

  if (y < 0) {
    y = 0;
  }

  if (x > width - 1) {
    x = width - 1;
  }

  if (y > height - 1) {
    y = height - 1;
  }
}

/* ************************************************************************* */
/**
 * @brief This function rounds float to nearest integer
 * @param flt Input float
 * @return dst Nearest integer
 */
inline int fRoundV2(float flt) {
  return (int)(flt + 0.5f);
}

#endif
