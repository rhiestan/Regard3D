/**
 * @file nldiffusion_functions.h
 * @brief Functions for non-linear diffusion applications:
 * 2D Gaussian Derivatives
 * Perona and Malik conductivity equations
 * Perona and Malik evolution
 * @date Dec 27, 2011
 * @author Pablo F. Alcantarilla
 */

#ifndef __OPENCV_FEATURES_2D_NLDIFFUSION_FUNCTIONS_H__
#define __OPENCV_FEATURES_2D_NLDIFFUSION_FUNCTIONS_H__

/* ************************************************************************* */
// Declaration of functions

#include <opencv2/core.hpp>

namespace cv
{

// Gaussian 2D convolution
void gaussian_2D_convolutionV2(const cv::Mat& src, cv::Mat& dst, int ksize_x, int ksize_y, float sigma);

// Diffusivity functions
void pm_g1V2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k);
void pm_g2V2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k);
void weickert_diffusivityV2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k);
void charbonnier_diffusivityV2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k);

float compute_k_percentileV2(const cv::Mat& Lx, const cv::Mat& Ly, float perc, cv::Mat& modgs, cv::Mat& hist);

// Image derivatives
void compute_scharr_derivative_kernelsV2(cv::OutputArray _kx, cv::OutputArray _ky, int dx, int dy, int scale);
void image_derivatives_scharrV2(const cv::Mat& src, cv::Mat& dst, int xorder, int yorder);

// Nonlinear diffusion filtering scalar step
void nld_step_scalarV2(const cv::Mat& Ld, const cv::Mat& c, cv::Mat& Lstep);

// For non-maxima suppresion
bool check_maximum_neighbourhoodV2(const cv::Mat& img, int dsize, float value, int row, int col, bool same_img);

// Image downsampling
void halfsample_imageV2(const cv::Mat& src, cv::Mat& dst);

}

#endif
