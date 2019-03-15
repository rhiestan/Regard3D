//=============================================================================
//
// nldiffusion_functions.cpp
// Author: Pablo F. Alcantarilla
// Institution: University d'Auvergne
// Address: Clermont Ferrand, France
// Date: 27/12/2011
// Email: pablofdezalc@gmail.com
//
// KAZE Features Copyright 2012, Pablo F. Alcantarilla
// All Rights Reserved
// See LICENSE for the license information
//=============================================================================

/**
 * @file nldiffusion_functions.cpp
 * @brief Functions for non-linear diffusion applications:
 * 2D Gaussian Derivatives
 * Perona and Malik conductivity equations
 * Perona and Malik evolution
 * @date Dec 27, 2011
 * @author Pablo F. Alcantarilla
 */

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "nldiffusion_functions.h"
#include <cstdint>
#include <cstring>
#include <iostream>

// Namespaces

/* ************************************************************************* */

namespace cv
{
using namespace std;

/* ************************************************************************* */
/**
 * @brief This function smoothes an image with a Gaussian kernel
 * @param src Input image
 * @param dst Output image
 * @param ksize_x Kernel size in X-direction (horizontal)
 * @param ksize_y Kernel size in Y-direction (vertical)
 * @param sigma Kernel standard deviation
 */
void gaussian_2D_convolutionV2(const cv::Mat& src, cv::Mat& dst, int ksize_x, int ksize_y, float sigma) {

    int ksize_x_ = 0, ksize_y_ = 0;

    // Compute an appropriate kernel size according to the specified sigma
    if (sigma > ksize_x || sigma > ksize_y || ksize_x == 0 || ksize_y == 0) {
        ksize_x_ = (int)ceil(2.0f*(1.0f + (sigma - 0.8f) / (0.3f)));
        ksize_y_ = ksize_x_;
    }

    // The kernel size must be and odd number
    if ((ksize_x_ % 2) == 0) {
        ksize_x_ += 1;
    }

    if ((ksize_y_ % 2) == 0) {
        ksize_y_ += 1;
    }

    // Perform the Gaussian Smoothing with border replication
    GaussianBlur(src, dst, Size(ksize_x_, ksize_y_), sigma, sigma, BORDER_REPLICATE);
}

/* ************************************************************************* */
/**
 * @brief This function computes image derivatives with Scharr kernel
 * @param src Input image
 * @param dst Output image
 * @param xorder Derivative order in X-direction (horizontal)
 * @param yorder Derivative order in Y-direction (vertical)
 * @note Scharr operator approximates better rotation invariance than
 * other stencils such as Sobel. See Weickert and Scharr,
 * A Scheme for Coherence-Enhancing Diffusion Filtering with Optimized Rotation Invariance,
 * Journal of Visual Communication and Image Representation 2002
 */
void image_derivatives_scharrV2(const cv::Mat& src, cv::Mat& dst, int xorder, int yorder) {
    Scharr(src, dst, CV_32F, xorder, yorder, 1.0, 0, BORDER_DEFAULT);
}

/* ************************************************************************* */
/**
 * @brief This function computes the Perona and Malik conductivity coefficient g1
 * g1 = exp(-|dL|^2/k^2)
 * @param Lx First order image derivative in X-direction (horizontal)
 * @param Ly First order image derivative in Y-direction (vertical)
 * @param dst Output image
 * @param k Contrast factor parameter
 */
void pm_g1V2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k) {

  // Compute: dst = exp((Lx.mul(Lx) + Ly.mul(Ly)) / (-k * k))

  const float neg_inv_k2 = -1.0f / (k*k);

  const int total = Lx.rows * Lx.cols;
  const float* lx = Lx.ptr<float>(0);
  const float* ly = Ly.ptr<float>(0);
  float* d = dst.ptr<float>(0);

  for (int i = 0; i < total; i++)
    d[i] = neg_inv_k2 * (lx[i]*lx[i] + ly[i]*ly[i]);

  exp(dst, dst);
}

/* ************************************************************************* */
/**
 * @brief This function computes the Perona and Malik conductivity coefficient g2
 * g2 = 1 / (1 + dL^2 / k^2)
 * @param Lx First order image derivative in X-direction (horizontal)
 * @param Ly First order image derivative in Y-direction (vertical)
 * @param dst Output image
 * @param k Contrast factor parameter
 */
void pm_g2V2(const cv::Mat &Lx, const cv::Mat& Ly, cv::Mat& dst, float k) {

  // Compute: dst = 1.0f / (1.0f + ((Lx.mul(Lx) + Ly.mul(Ly)) / (k * k)) );

  const float inv_k2 = 1.0f / (k * k);

  const int total = Lx.rows * Lx.cols;
  const float* lx = Lx.ptr<float>(0);
  const float* ly = Ly.ptr<float>(0);
  float* d = dst.ptr<float>(0);

  for (int i = 0; i < total; i++)
    d[i] = 1.0f / (1.0f + ((lx[i] * lx[i] + ly[i] * ly[i]) * inv_k2));
}

/* ************************************************************************* */
/**
 * @brief This function computes Weickert conductivity coefficient gw
 * @param Lx First order image derivative in X-direction (horizontal)
 * @param Ly First order image derivative in Y-direction (vertical)
 * @param dst Output image
 * @param k Contrast factor parameter
 * @note For more information check the following paper: J. Weickert
 * Applications of nonlinear diffusion in image processing and computer vision,
 * Proceedings of Algorithmy 2000
 */
void weickert_diffusivityV2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k) {

  // Compute: dst = 1.0f - exp(-3.315f / ((Lx.mul(Lx) + Ly.mul(Ly)) / (k * k))^4)

  const float inv_k2 = 1.0f / (k * k);

  const int total = Lx.rows * Lx.cols;
  const float* lx = Lx.ptr<float>(0);
  const float* ly = Ly.ptr<float>(0);
  float* d = dst.ptr<float>(0);

  for (int i = 0; i < total; i++) {
    float dL = inv_k2 * (lx[i] * lx[i] + ly[i] * ly[i]);
    d[i] = -3.315f / (dL*dL*dL*dL);
  }

  exp(dst, dst);

  for (int i = 0; i < total; i++)
    d[i] = 1.0f - d[i];
}


/* ************************************************************************* */
/**
* @brief This function computes Charbonnier conductivity coefficient gc
* gc = 1 / sqrt(1 + dL^2 / k^2)
* @param Lx First order image derivative in X-direction (horizontal)
* @param Ly First order image derivative in Y-direction (vertical)
* @param dst Output image
* @param k Contrast factor parameter
* @note For more information check the following paper: J. Weickert
* Applications of nonlinear diffusion in image processing and computer vision,
* Proceedings of Algorithmy 2000
*/
void charbonnier_diffusivityV2(const cv::Mat& Lx, const cv::Mat& Ly, cv::Mat& dst, float k) {

  // Compute: dst = 1.0f / sqrt(1.0f + (Lx.mul(Lx) + Ly.mul(Ly)) / (k * k))

  const float inv_k2 = 1.0f / (k * k);

  const int total = Lx.rows * Lx.cols;
  const float* lx = Lx.ptr<float>(0);
  const float* ly = Ly.ptr<float>(0);
  float* d = dst.ptr<float>(0);

  for (int i = 0; i < total; i++)
    d[i] = 1.0f / sqrtf(1.0f + inv_k2 * (lx[i]*lx[i] + ly[i]*ly[i]));
}


/* ************************************************************************* */
/**
 * @brief This function computes a good empirical value for the k contrast factor
 * given two gradient images, the percentile (0-1), the temporal storage to hold
 * gradient norms and the histogram bins
 * @param Lx Horizontal gradient of the input image
 * @param Ly Vertical gradient of the input image
 * @param perc Percentile of the image gradient histogram (0-1)
 * @param modgs Temporal vector to hold the gradient norms
 * @param histogram Temporal vector to hold the gradient histogram
 * @return k contrast factor
 */
float compute_k_percentileV2(const cv::Mat& Lx, const cv::Mat& Ly, float perc, cv::Mat& modgs, cv::Mat& histogram) {

    const int total = modgs.cols;
    const int nbins = histogram.cols;

    CV_DbgAssert(total == (Lx.rows - 2)*(Lx.cols - 2));
    CV_DbgAssert(nbins > 2);

    float *modg = modgs.ptr<float>(0);
    int32_t *hist = histogram.ptr<int32_t>(0);

    for (int i = 1; i < Lx.rows - 1; i++) {
        const float *lx = Lx.ptr<float>(i) + 1;
        const float *ly = Ly.ptr<float>(i) + 1;
        const int cols = Lx.cols - 2;

        for (int j = 0; j < cols; j++)
            *modg++ = sqrtf(lx[j] * lx[j] + ly[j] * ly[j]);
    }
    modg = modgs.ptr<float>(0);

    // Get the maximum
    float hmax = 0.0f;
    for (int i = 0; i < total; i++)
        if (hmax < modg[i])
            hmax = modg[i];

    if (hmax == 0.0f)
        return 0.03f;  // e.g. a blank image

    // Compute the bin numbers: the value range [0, hmax] -> [0, nbins-1]
    for (int i = 0; i < total; i++)
        modg[i] *= (nbins - 1) / hmax;

    // Count up
    std::memset(hist, 0, sizeof(int32_t)*nbins);
    for (int i = 0; i < total; i++)
        hist[(int)modg[i]]++;

    // Now find the perc of the histogram percentile
    const int nthreshold = (int)((total - hist[0]) * perc);  // Exclude hist[0] as background
    int nelements = 0;
    for (int k = 1; k < nbins; k++) {
        if (nelements >= nthreshold)
            return (float)hmax * k / nbins;

        nelements = nelements + hist[k];
    }

    return 0.03f;
}


/* ************************************************************************* */
/**
 * @brief Compute Scharr derivative kernels for sizes different than 3
 * @param _kx Horizontal kernel ues
 * @param _ky Vertical kernel values
 * @param dx Derivative order in X-direction (horizontal)
 * @param dy Derivative order in Y-direction (vertical)
 * @param scale_ Scale factor or derivative size
 */
void compute_scharr_derivative_kernelsV2(cv::OutputArray _kx, cv::OutputArray _ky, int dx, int dy, int scale) {

    int ksize = 3 + 2 * (scale - 1);

    // The standard Scharr kernel
    if (scale == 1) {
        getDerivKernels(_kx, _ky, dx, dy, 0, true, CV_32F);
        return;
    }

    _kx.create(ksize, 1, CV_32F, -1, true);
    _ky.create(ksize, 1, CV_32F, -1, true);
    Mat kx = _kx.getMat();
    Mat ky = _ky.getMat();

    float w = 10.0f / 3.0f;
    float norm = 1.0f / (2.0f*(w + 2.0f));

    std::vector<float> kerI(ksize, 0.0f);

    if (dx == 0) {
        kerI[0] = norm, kerI[ksize / 2] = w*norm, kerI[ksize - 1] = norm;
    }
    else if (dx == 1) {
        kerI[0] = -1, kerI[ksize / 2] = 0, kerI[ksize - 1] = 1;
    }
    Mat(kx.rows, kx.cols, CV_32F, &kerI[0]).copyTo(kx);

    kerI.assign(ksize, 0.0f);

    if (dy == 0) {
        kerI[0] = norm, kerI[ksize / 2] = w*norm, kerI[ksize - 1] = norm;
    }
    else if (dy == 1) {
        kerI[0] = -1, kerI[ksize / 2] = 0, kerI[ksize - 1] = 1;
    }
    Mat(ky.rows, ky.cols, CV_32F, &kerI[0]).copyTo(ky);
}



inline
void nld_step_scalar_one_lane(const cv::Mat& Lt, const cv::Mat& Lf, cv::Mat& Lstep, int idx, int skip)
{
    /* The labeling scheme for this five star stencil:
         [    a    ]
         [ -1 c +1 ]
         [    b    ]
     */

    const int cols = Lt.cols - 2;
    int row = idx;

    const float *lt_a, *lt_c, *lt_b;
    const float *lf_a, *lf_c, *lf_b;
    float *dst;

    // Process the top row
    if (row == 0) {
        lt_c = Lt.ptr<float>(0) + 1;  /* Skip the left-most column by +1 */
        lf_c = Lf.ptr<float>(0) + 1;
        lt_b = Lt.ptr<float>(1) + 1;
        lf_b = Lf.ptr<float>(1) + 1;
        dst = Lstep.ptr<float>(0) + 1;

        for (int j = 0; j < cols; j++) {
            dst[j] = (lf_c[j] + lf_c[j + 1])*(lt_c[j + 1] - lt_c[j]) +
                     (lf_c[j] + lf_c[j - 1])*(lt_c[j - 1] - lt_c[j]) +
                     (lf_c[j] + lf_b[j    ])*(lt_b[j    ] - lt_c[j]);
        }
        row += skip;
    }

    // Process the middle rows
    for (; row < Lt.rows - 1; row += skip)
    {
        lt_a = Lt.ptr<float>(row - 1);
        lf_a = Lf.ptr<float>(row - 1);
        lt_c = Lt.ptr<float>(row    );
        lf_c = Lf.ptr<float>(row    );
        lt_b = Lt.ptr<float>(row + 1);
        lf_b = Lf.ptr<float>(row + 1);
        dst = Lstep.ptr<float>(row);

        // The left-most column
        dst[0] = (lf_c[0] + lf_c[1])*(lt_c[1] - lt_c[0]) +
                 (lf_c[0] + lf_b[0])*(lt_b[0] - lt_c[0]) +
                 (lf_c[0] + lf_a[0])*(lt_a[0] - lt_c[0]);

        lt_a++; lt_c++; lt_b++;
        lf_a++; lf_c++; lf_b++;
        dst++;

        // The middle columns
        for (int j = 0; j < cols; j++)
        {
            dst[j] = (lf_c[j] + lf_c[j + 1])*(lt_c[j + 1] - lt_c[j]) +
                     (lf_c[j] + lf_c[j - 1])*(lt_c[j - 1] - lt_c[j]) +
                     (lf_c[j] + lf_b[j    ])*(lt_b[j    ] - lt_c[j]) +
                     (lf_c[j] + lf_a[j    ])*(lt_a[j    ] - lt_c[j]);
        }

        // The right-most column
        dst[cols] = (lf_c[cols] + lf_c[cols - 1])*(lt_c[cols - 1] - lt_c[cols]) +
                    (lf_c[cols] + lf_b[cols    ])*(lt_b[cols    ] - lt_c[cols]) +
                    (lf_c[cols] + lf_a[cols    ])*(lt_a[cols    ] - lt_c[cols]);
    }

    // Process the bottom row
    if (row == Lt.rows - 1) {
        lt_a = Lt.ptr<float>(row - 1) + 1;  /* Skip the left-most column by +1 */
        lf_a = Lf.ptr<float>(row - 1) + 1;
        lt_c = Lt.ptr<float>(row    ) + 1;
        lf_c = Lf.ptr<float>(row    ) + 1;
        dst = Lstep.ptr<float>(row) +1;

        for (int j = 0; j < cols; j++) {
            dst[j] = (lf_c[j] + lf_c[j + 1])*(lt_c[j + 1] - lt_c[j]) +
                     (lf_c[j] + lf_c[j - 1])*(lt_c[j - 1] - lt_c[j]) +
                     (lf_c[j] + lf_a[j    ])*(lt_a[j    ] - lt_c[j]);
        }
    }
}

/* ************************************************************************* */
/**
* @brief This function computes a scalar non-linear diffusion step
* @param Ld Base image in the evolution
* @param c Conductivity image
* @param Lstep Output image that gives the difference between the current
* Ld and the next Ld being evolved
* @note Forward Euler Scheme 3x3 stencil
* The function c is a scalar value that depends on the gradient norm
* dL_by_ds = d(c dL_by_dx)_by_dx + d(c dL_by_dy)_by_dy
*/
void nld_step_scalarV2(const cv::Mat& Ld, const cv::Mat& c, cv::Mat& Lstep)
{
    nld_step_scalar_one_lane(Ld, c, Lstep, 0, 1);
}



/* ************************************************************************* */
/**
* @brief This function downsamples the input image using OpenCV resize
* @param src Input image to be downsampled
* @param dst Output image with half of the resolution of the input image
*/
void halfsample_imageV2(const cv::Mat& src, cv::Mat& dst) {

    // Make sure the destination image is of the right size
    CV_Assert(src.cols / 2 == dst.cols);
    CV_Assert(src.rows / 2 == dst.rows);
    resize(src, dst, dst.size(), 0, 0, cv::INTER_AREA);
}

/* ************************************************************************* */
/**
 * @brief This function checks if a given pixel is a maximum in a local neighbourhood
 * @param img Input image where we will perform the maximum search
 * @param dsize Half size of the neighbourhood
 * @param value Response value at (x,y) position
 * @param row Image row coordinate
 * @param col Image column coordinate
 * @param same_img Flag to indicate if the image value at (x,y) is in the input image
 * @return 1->is maximum, 0->otherwise
 */
bool check_maximum_neighbourhoodV2(const cv::Mat& img, int dsize, float value, int row, int col, bool same_img) {

    bool response = true;

    for (int i = row - dsize; i <= row + dsize; i++) {
        for (int j = col - dsize; j <= col + dsize; j++) {
            if (i >= 0 && i < img.rows && j >= 0 && j < img.cols) {
                if (same_img == true) {
                    if (i != row || j != col) {
                        if ((*(img.ptr<float>(i)+j)) > value) {
                            response = false;
                            return response;
                        }
                    }
                }
                else {
                    if ((*(img.ptr<float>(i)+j)) > value) {
                        response = false;
                        return response;
                    }
                }
            }
        }
    }

    return response;
}

}
