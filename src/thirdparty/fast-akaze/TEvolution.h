/**
 * @file TEvolution.h
 * @brief Header file with the declaration of the TEvolution struct
 * @date Jun 02, 2014
 * @author Pablo F. Alcantarilla
 */

#ifndef __OPENCV_FEATURES_2D_TEVOLUTION_H__
#define __OPENCV_FEATURES_2D_TEVOLUTION_H__

#include <opencv2/core.hpp>

namespace cv
{

/* ************************************************************************* */
/// KAZE/A-KAZE nonlinear diffusion filtering evolution
struct TEvolutionV2
{
  TEvolutionV2() {
    etime = 0.0f;
    esigma = 0.0f;
    octave = 0;
    sublevel = 0;
    sigma_size = 0;
    octave_ratio = 1.0f;
  }

  Mat Lx, Ly;           ///< First order spatial derivatives
  Mat Lxx, Lxy, Lyy;    ///< Second order spatial derivatives
  Mat Lt;               ///< Evolution image
  Mat Lsmooth;          ///< Smoothed image
  Mat Ldet;             ///< Detector response

  Mat DxKx, DxKy;       ///< Derivative kernels (kx and ky) of xorder = 1
  Mat DyKx, DyKy;       ///< Derivative kernels (kx and ky) of yorder = 1

  float etime;              ///< Evolution time
  float esigma;             ///< Evolution sigma. For linear diffusion t = sigma^2 / 2
  int octave;               ///< Image octave
  int sublevel;             ///< Image sublevel in each octave
  int sigma_size;           ///< Scaling factor of esigma that is round(esigma * derivative_factor / power)
  int border;               ///< Width of border where descriptors cannot be computed
  float octave_ratio;       ///< Scaling ratio of this octave. ratio = 2^octave
};

}

#endif
