#ifndef __OPENCV_FEATURES_2D_AKAZE2_HPP__
#define __OPENCV_FEATURES_2D_AKAZE2_HPP__

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

/*
  This file is the excerpt from opencv/feature2d.hpp to provide the local AKAZE class definition.
  In addition, the class name is changed from AKAZE to AKAZE2 to avoid possible confusion between
  this local variant and OpenCV's original AKAZE class.
*/

namespace cv
{

//! @addtogroup features2d
//! @{

//! @addtogroup features2d_main
//! @{

/** @brief Class implementing the AKAZE keypoint detector and descriptor extractor, described in @cite ANB13 . :

@note AKAZE descriptors can only be used with KAZE or AKAZE keypoints. Try to avoid using *extract*
and *detect* instead of *operator()* due to performance reasons. .. [ANB13] Fast Explicit Diffusion
for Accelerated Features in Nonlinear Scale Spaces. Pablo F. Alcantarilla, Jesús Nuevo and Adrien
Bartoli. In British Machine Vision Conference (BMVC), Bristol, UK, September 2013.
 */
class CV_EXPORTS_W AKAZE2 : public Feature2D
{
public:
    // AKAZE descriptor type
    enum
    {
        DESCRIPTOR_KAZE_UPRIGHT = 2, ///< Upright descriptors, not invariant to rotation
        DESCRIPTOR_KAZE = 3,
        DESCRIPTOR_MLDB_UPRIGHT = 4, ///< Upright descriptors, not invariant to rotation
        DESCRIPTOR_MLDB = 5
    };

    /** @brief The AKAZE constructor

    @param descriptor_type Type of the extracted descriptor: DESCRIPTOR_KAZE,
    DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
    @param descriptor_size Size of the descriptor in bits. 0 -\> Full size
    @param descriptor_channels Number of channels in the descriptor (1, 2, 3)
    @param threshold Detector response threshold to accept point
    @param nOctaves Maximum octave evolution of the image
    @param nOctaveLayers Default number of sublevels per scale level
    @param diffusivity Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or
    DIFF_CHARBONNIER
     */
    CV_WRAP static Ptr<AKAZE2> create(int descriptor_type=AKAZE::DESCRIPTOR_MLDB,
                                     int descriptor_size = 0, int descriptor_channels = 3,
                                     float threshold = 0.001f, int nOctaves = 4,
                                     int nOctaveLayers = 4, int diffusivity = KAZE::DIFF_PM_G2);

    CV_WRAP virtual void setDescriptorType(int dtype) = 0;
    CV_WRAP virtual int getDescriptorType() const = 0;

    CV_WRAP virtual void setDescriptorSize(int dsize) = 0;
    CV_WRAP virtual int getDescriptorSize() const = 0;

    CV_WRAP virtual void setDescriptorChannels(int dch) = 0;
    CV_WRAP virtual int getDescriptorChannels() const = 0;

    CV_WRAP virtual void setThreshold(double threshold) = 0;
    CV_WRAP virtual double getThreshold() const = 0;

    CV_WRAP virtual void setNOctaves(int octaves) = 0;
    CV_WRAP virtual int getNOctaves() const = 0;

    CV_WRAP virtual void setNOctaveLayers(int octaveLayers) = 0;
    CV_WRAP virtual int getNOctaveLayers() const = 0;

    CV_WRAP virtual void setDiffusivity(int diff) = 0;
    CV_WRAP virtual int getDiffusivity() const = 0;
};

//! @} features2d_main

//! @} features2d

} /* namespace cv */

#endif
