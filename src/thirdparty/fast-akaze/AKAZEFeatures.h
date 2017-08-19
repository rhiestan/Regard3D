/**
 * @file AKAZE.h
 * @brief Main class for detecting and computing binary descriptors in an
 * accelerated nonlinear scale space
 * @date Mar 27, 2013
 * @author Pablo F. Alcantarilla, Jesus Nuevo
 */

#ifndef __OPENCV_FEATURES_2D_AKAZE_FEATURES_H__
#define __OPENCV_FEATURES_2D_AKAZE_FEATURES_H__

/* ************************************************************************* */
// Includes
#include <vector>

#define AKAZE_USE_CPP11_THREADING

#ifdef AKAZE_USE_CPP11_THREADING
#include <future>
#include <atomic>
#endif

#include <opencv2/core.hpp>

#include "AKAZEConfig.h"
#include "TEvolution.h"

namespace cv
{

/* ************************************************************************* */
// AKAZE Class Declaration
class AKAZEFeaturesV2 {

private:

  AKAZEOptionsV2 options_;                ///< Configuration options for AKAZE
  std::vector<TEvolutionV2> evolution_;        ///< Vector of nonlinear diffusion evolution

  /// FED parameters
  bool reordering_;              ///< Flag for reordering time steps
  std::vector<std::vector<float> > tsteps_;  ///< Vector of FED dynamic time steps

  /// Matrices for the M-LDB descriptor computation
  cv::Mat descriptorSamples_;  // List of positions in the grids to sample LDB bits from.
  cv::Mat descriptorBits_;
  cv::Mat bitMask_;

  /// Preallocated temporary variables
  cv::Mat gray_, lx_, ly_;
  cv::Mat lflow_, lstep_;
  cv::Mat histgram_, modgs_;
  std::vector<std::vector<cv::KeyPoint>> kpts_aux_;

#ifdef AKAZE_USE_CPP11_THREADING
  using task = std::future<void>;
  std::vector<std::vector<task>> tasklist_;
  std::vector<std::atomic_int> taskdeps_;
  std::future<float> kcontrast_;
#endif

public:

  /// Constructor with input arguments
  AKAZEFeaturesV2(const AKAZEOptionsV2& options);

  /// Getters and Setters
  void setThreshold(double threshold_) { options_.dthreshold = std::max((float)threshold_, options_.min_dthreshold); };
  double getThreshold() const { return options_.dthreshold; }
  void setDiffusivity(int diff_) { options_.diffusivity = diff_; }
  int getDiffusivity() const { return options_.diffusivity; }

  /// Scale Space methods
  void Allocate_Memory_Evolution();
  int Create_Nonlinear_Scale_Space(const cv::Mat& img);
  float Compute_Base_Evolution_Level(const cv::Mat& img);
  void Feature_Detection(std::vector<cv::KeyPoint>& kpts);
  void Compute_Determinant_Hessian_Response(const int level);
  void Compute_Determinant_Hessian_Response_Single(const int level);
  void Find_Scale_Space_Extrema(std::vector<std::vector<cv::KeyPoint>>& kpts_aux);
  void Find_Scale_Space_Extrema_Single(std::vector<std::vector<cv::KeyPoint>>& kpts_aux);
  void Do_Subpixel_Refinement(std::vector<std::vector<cv::KeyPoint>>& kpts_aux, std::vector<cv::KeyPoint>& kpts);

  /// Feature description methods
  void Compute_Descriptors(std::vector<cv::KeyPoint>& kpts, cv::Mat& desc);
};

}

#endif
