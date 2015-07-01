/**
 * Copyright (C) 2015 Roman Hiestand
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef REGARD3DFEATURES_H
#define REGARD3DFEATURES_H

#include <vector>

// OpenMVG
#include "openMVG/image/image_container.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/keypointSet.hpp"
#if !defined(R3D_USE_OPENMVG_PRE08)
#	include "openMVG/features/regions.hpp"
#endif

// OpenCV
#include "opencv2/features2d/features2d.hpp"

class Regard3DFeatures
{
public:

	typedef openMVG::features::SIOPointFeature FeatureR3D;
	typedef std::vector<FeatureR3D> FeatsR3D;
	typedef openMVG::features::Descriptor<float, 144> DescriptorR3D;	// LIOP: 144, Daisy: 200
	typedef std::vector<DescriptorR3D> DescsR3D;
	typedef openMVG::features::KeypointSet<FeatsR3D, DescsR3D > KeypointSetR3D;
#if !defined(R3D_USE_OPENMVG_PRE08)
	typedef openMVG::features::Scalar_Regions<FeatureR3D, float, 144> R3D_AKAZE_LIOP_Regions;
#endif

	// Class for holding feature extraction parameters
	struct R3DFParams
	{
		R3DFParams();
		R3DFParams(const R3DFParams &o);
		virtual ~R3DFParams();
		R3DFParams &copy(const R3DFParams &o);
		R3DFParams & operator=(const R3DFParams & o);

		std::vector<std::string> keypointDetectorList_;
		float threshold_;	// AKAZE, DoG, BRISK (int)
		int nFeatures_;		// GFTT, HARRIS, ORB

		float distRatio_;	// Nearest Neighbor distance ratio

		bool computeHomographyMatrix_;
		bool computeFundalmentalMatrix_;
		bool computeEssentialMatrix_;
	};

	static bool initAKAZESemaphore(int count = 1);
	static void uninitializeAKAZESemaphore();

	/**
	 * Return a list of supported keypoint detectors.
	 */
	static std::vector<std::string> getKeypointDetectors();

	/**
	 * Return a list of supported feature extractors.
	 */
	static std::vector<std::string> getFeatureExtractors();

	/**
	 * Detect keypoints and extract feature descriptors.
	 */
	static void detectAndExtract(const openMVG::image::Image<float> &img,
		FeatsR3D &feats, DescsR3D &descs, const R3DFParams &params);

	// The same, but only used for optimizing kpSizeFactor
	static void detectAndExtract_NLOPT(const openMVG::image::Image<unsigned char> &img,
		FeatsR3D &feats, DescsR3D &descs, double kpSizeFactorIn = 0.5);

	// Old method, not used anymore
	static void detectAndExtractVLFEAT_CoV_Daisy(const openMVG::image::Image<unsigned char> &img,
		FeatsR3D &feats, DescsR3D &descs);

	// Old method, not used anymore
	static void detectAndExtractVLFEAT_MSER_LIOP(const openMVG::image::Image<unsigned char> &img,
		FeatsR3D &feats, DescsR3D &descs);

	// Old method, not used anymore
	static void detectAndExtractVLFEAT_CoV_LIOP(const openMVG::image::Image<unsigned char> &img,
		FeatsR3D &feats, DescsR3D &descs);

	
private:
	static void detectKeypoints(const openMVG::image::Image<float> &img,
		std::vector< cv::KeyPoint > &vec_keypoints, const std::string &fdname,
		const R3DFParams &params);

	static void extractDaisyFeatures(const openMVG::image::Image<unsigned char> &img,
		std::vector< cv::KeyPoint > &vec_keypoints, float kpSizeFactor,
		FeatsR3D &feats, DescsR3D &descs);

	static void extractLIOPFeatures(const openMVG::image::Image<float> &img,
		std::vector< cv::KeyPoint > &vec_keypoints, float kpSizeFactor,
		FeatsR3D &feats, DescsR3D &descs);

	static float getKpSizeFactor(const std::string &fdname);

	Regard3DFeatures();
	virtual ~Regard3DFeatures();
};

#endif
