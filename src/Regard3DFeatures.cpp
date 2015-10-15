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

#include "CommonIncludes.h"
#include "Regard3DFeatures.h"
#include <memory>
#include <boost/locale.hpp>

#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#if defined(R3D_HAVE_TBB)	// && !defined(R3D_HAVE_OPENMP)

#define R3D_USE_TBB_THREADING 1
#	include <tbb/tbb.h>
#endif

#undef R3D_USE_TBB_THREADING
#undef R3D_HAVE_TBB
#undef R3D_HAVE_OPENMP

#undef R3D_HAVE_VLFEAT

#if defined(R3D_HAVE_VLFEAT)
// VLFEAT
extern "C" {
#include "vlfeat/covdet.h"
#include "vlfeat/mser.h"
}
#endif

// LIOP (copied from VLFEAT)
extern "C" {
#include "vl_liop.h"
}

// OpenCV Includes
#include "opencv2/core/eigen.hpp" //To Convert Eigen matrix to cv matrix
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"

// Daisy
#include "daisy/daisy.h"

// AKAZE
#include "AKAZE.h"


// Helper class for locking the AKAZE semaphore (based on wxMutexLocker)
class AKAZESemaLocker
{
public:
	// lock the mutex in the ctor
	AKAZESemaLocker()
		: isOK_(false)
	{
		if(pAKAZESemaphore_ != NULL)
			isOK_ = (pAKAZESemaphore_->Wait() == wxSEMA_NO_ERROR);
	}

	// returns true if mutex was successfully locked in ctor
	bool IsOk() const
	{
		return isOK_;
	}

	// unlock the mutex in dtor
	~AKAZESemaLocker()
	{
		if(IsOk() && pAKAZESemaphore_ != NULL)
			pAKAZESemaphore_->Post();
	}

	// Initializes the semaphore with the amount of allowed simultaneous threads
	static bool initialize(int count)
	{
		if(pAKAZESemaphore_ != NULL)
			delete pAKAZESemaphore_;
		pAKAZESemaphore_ = new wxSemaphore(count, 0);

		if(pAKAZESemaphore_->IsOk())
			return true;

		delete pAKAZESemaphore_;
		return false;
	}

	static void uninitialize()
	{
		if(pAKAZESemaphore_ != NULL)
			delete pAKAZESemaphore_;
		pAKAZESemaphore_ = NULL;
	}

private:
	// no assignment operator nor copy ctor
	AKAZESemaLocker(const AKAZESemaLocker&);
	AKAZESemaLocker& operator=(const AKAZESemaLocker&);

	bool     isOK_;
	static wxSemaphore *pAKAZESemaphore_;
};

wxSemaphore *AKAZESemaLocker::pAKAZESemaphore_ = NULL;


Regard3DFeatures::R3DFParams::R3DFParams()
	: threshold_(0.001), nFeatures_(20000), distRatio_(0.6),
	computeHomographyMatrix_(true),
	computeFundalmentalMatrix_(true), computeEssentialMatrix_(true)
{
	keypointDetectorList_.push_back( std::string("AKAZE") );
}

Regard3DFeatures::R3DFParams::R3DFParams(const Regard3DFeatures::R3DFParams &o)
{
	copy(o);
}

Regard3DFeatures::R3DFParams::~R3DFParams()
{
}

Regard3DFeatures::R3DFParams &Regard3DFeatures::R3DFParams::copy(const Regard3DFeatures::R3DFParams &o)
{
	keypointDetectorList_ = o.keypointDetectorList_;
	threshold_ = o.threshold_;
	nFeatures_ = o.nFeatures_;
	distRatio_ = o.distRatio_;
	computeHomographyMatrix_ = o.computeHomographyMatrix_;
	computeFundalmentalMatrix_ = o.computeFundalmentalMatrix_;
	computeEssentialMatrix_ = o.computeEssentialMatrix_;

	return *this;
}

Regard3DFeatures::R3DFParams & Regard3DFeatures::R3DFParams::operator=(const Regard3DFeatures::R3DFParams &o)
{
	return copy(o);
}

Regard3DFeatures::Regard3DFeatures()
{
}

Regard3DFeatures::~Regard3DFeatures()
{
}

bool Regard3DFeatures::initAKAZESemaphore(int count)
{
	return AKAZESemaLocker::initialize(count);
}

void Regard3DFeatures::uninitializeAKAZESemaphore()
{
	AKAZESemaLocker::uninitialize();
}

std::vector<std::string> Regard3DFeatures::getKeypointDetectors()
{
	std::vector<std::string> ret;
	ret.push_back( std::string( "AKAZE" ) );		// Own
	ret.push_back( std::string( "MSER" ) );			// OpenCV
	ret.push_back( std::string( "ORB" ) );			// OpenCV
	ret.push_back( std::string( "BRISK" ) );		// OpenCV
	ret.push_back( std::string( "GFTT" ) );			// OpenCV
#if defined(R3D_HAVE_VLFEAT)
	ret.push_back( std::string( "DOG" ) );			// VLFEAT
#endif

	return ret;
}

std::vector<std::string> Regard3DFeatures::getFeatureExtractors()
{
	std::vector<std::string> ret;
	// DAISY is not enabled, because Daisy descriptors have a different size
//	ret.push_back( std::string( "DAISY" ) );
	ret.push_back( std::string( "LIOP" ) );

	return ret;
}

void Regard3DFeatures::detectAndExtract(const openMVG::image::Image<float> &img,
	Regard3DFeatures::FeatsR3D &feats, Regard3DFeatures::DescsR3D &descs,
	const Regard3DFeatures::R3DFParams &params)
{
	std::vector< cv::KeyPoint > vec_keypoints;

	// Iterate over all keypoint detectors
	std::vector<std::string>::const_iterator iter = params.keypointDetectorList_.begin();
	for(;iter != params.keypointDetectorList_.end(); iter++)
	{
		std::string keypointDetector = *iter;
		vec_keypoints.clear();
		detectKeypoints(img, vec_keypoints, keypointDetector, params);

		float kpSizeFactor = getKpSizeFactor(keypointDetector);
		extractLIOPFeatures(img, vec_keypoints, kpSizeFactor, feats, descs);
	}
}

void Regard3DFeatures::detectAndExtract_NLOPT(const openMVG::image::Image<unsigned char> &img,
	Regard3DFeatures::FeatsR3D &feats, Regard3DFeatures::DescsR3D &descs, double kpSizeFactorIn)
{
	std::vector< cv::KeyPoint > vec_keypoints;
	Regard3DFeatures::R3DFParams params;
	params.nFeatures_ = 10000;
	params.threshold_ = 0.0007;

	std::string keypointDetector("AKAZE");
	vec_keypoints.clear();
//	detectKeypoints(img, vec_keypoints, keypointDetector, params);

//	extractLIOPFeatures(img, vec_keypoints, kpSizeFactorIn, feats, descs);
}

/**
 * Detect keypoints using VLFEAT covariant feature detectors, create daisy descriptors.
 */
void Regard3DFeatures::detectAndExtractVLFEAT_CoV_Daisy(const openMVG::image::Image<unsigned char> &img,
	Regard3DFeatures::FeatsR3D &feats, Regard3DFeatures::DescsR3D &descs)
{
#if defined(R3D_HAVE_VLFEAT)
	// Convert image float
	openMVG::image::Image<float> imgFloat( img.GetMat().cast<float>() );
	int w = img.Width(), h = img.Height();

	int octaveResolution = 2;
	double peakThreshold = 0.01;	// Default values: for DoG 0.01, Harris 0.000002, Hessian 0.003
	double edgeThreshold = 10.0;	// Default values: for all variants 10
	double boundaryMargin = 2.0;
	int patchResolution = 20;
	double patchRelativeExtent = 4.0;
	double patchRelativeSmoothing = 1.2;	//0.5;
	bool doubleImage = false;
	double smoothBeforeDetection = 0;	//1.0;

	// VL_COVDET_METHOD_DOG
	// VL_COVDET_METHOD_MULTISCALE_HARRIS
	// VL_COVDET_METHOD_MULTISCALE_HESSIAN (didn't find a good value for peakThreshold, always returns lots of features)
	VlCovDet * covdet = vl_covdet_new(VL_COVDET_METHOD_DOG);

	vl_covdet_set_first_octave(covdet, doubleImage ? -1 : 0);
	vl_covdet_set_octave_resolution(covdet, octaveResolution);
	vl_covdet_set_peak_threshold(covdet, peakThreshold);
	vl_covdet_set_edge_threshold(covdet, edgeThreshold);

	// Smooth image
	if(smoothBeforeDetection > 0)
		vl_imsmooth_f(imgFloat.data(), w, imgFloat.data(), w, h, w,
			smoothBeforeDetection, smoothBeforeDetection);

	// process the image and run the detector
	vl_covdet_put_image(covdet, imgFloat.data(), w, h);
	vl_covdet_detect(covdet);

	// drop features on the margin
	vl_covdet_drop_features_outside(covdet, boundaryMargin);

	// compute the affine shape of the features
//	vl_covdet_extract_affine_shape(covdet);

	// compute the orientation of the features
	vl_covdet_extract_orientations(covdet);

	// get feature frames back
	vl_size numFeatures = vl_covdet_get_num_features(covdet) ;
	VlCovDetFeature const *feature = reinterpret_cast<VlCovDetFeature const *>(vl_covdet_get_features(covdet));


	// get normalized feature appearance patches
	vl_size patchSize = 2*patchResolution + 1;
	std::vector<float> patch(patchSize * patchSize);

	// Prepare Daisy descriptor
	daisy* pDaisyDescr = new daisy();
	pDaisyDescr->verbose( 0 );
	double rad = 15.0;
	int radq = 3, thq = 8, histq = 8;
	pDaisyDescr->set_parameters(rad, radq, thq, histq); // we use 15,3,8,8 for wide baseline stereo.
//	pDaisyDescr->initialize_single_descriptor_mode();

	int dimension = pDaisyDescr->descriptor_size();
	std::vector<float> desc(dimension);
	DescriptorR3D descriptor;

	for (int i = 0 ; i < numFeatures ; i++)
	{
		// This is very slow, as for each keypoint the patch needs to be extracted and
		// the daisy descriptor calculated.
		daisy daisyDescr;
		daisyDescr.verbose( 0 );
		daisyDescr.set_parameters(rad, radq, thq, histq);
		vl_covdet_extract_patch_for_frame(covdet,
			&(patch[0]),
			patchResolution,
			patchRelativeExtent,
			patchRelativeSmoothing,
			feature[i].frame);

		daisyDescr.set_image(&(patch[0]), patchSize, patchSize);
		daisyDescr.initialize_single_descriptor_mode();
		int dimension2 = daisyDescr.descriptor_size();

		// Calculate Daisy descriptor
		double x = static_cast<double>(patchSize)/2.0;	//feature[i].frame.x;
		double y = static_cast<double>(patchSize)/2.0;	//feature[i].frame.y;
		int orientation = 0;
		double scale = 1.0;
		daisyDescr.get_descriptor(y, x, orientation,  &(desc[0]));

		// Convert to OpenMVG keypoint and descriptor
		openMVG::features::SIOPointFeature fp;
		fp.x() = feature[i].frame.x;
		fp.y() = feature[i].frame.y;
		fp.scale() = static_cast<float>(scale);
		fp.orientation() = 0.0f;		// TODO

		for(int j = 0; j < dimension; j++)
			descriptor[j] = desc[j];
		descs.push_back(descriptor);
		feats.push_back(fp);

	}

	// Clean up
	delete pDaisyDescr;
	vl_covdet_delete(covdet);
#endif
}

/**
 * Detect keypoints using MSER from OpenCV and create LIOP descriptors.
 *
 * Make sure DescriptorR3D is set to 144 floats.
 */
void Regard3DFeatures::detectAndExtractVLFEAT_MSER_LIOP(const openMVG::image::Image<unsigned char> &img,
	Regard3DFeatures::FeatsR3D &feats, Regard3DFeatures::DescsR3D &descs)
{
	// Convert image to OpenCV data
	cv::Mat cvimg;
	cv::eigen2cv(img.GetMat(), cvimg);

	std::vector< cv::KeyPoint > vec_keypoints;
	cv::Mat m_desc;

//	cv::Ptr<cv::FeatureDetector> fd(cv::FeatureDetector::create(std::string("MSER")));

	int nrPixels = img.Width() * img.Height();
	int maxArea = static_cast<int>(nrPixels * 0.75);
/*
	fd->setInt("delta", 5);
	fd->setInt("minArea", 3);
	fd->setInt("maxArea", maxArea);
	fd->setDouble("maxVariation", 0.25);
	fd->setDouble("minDiversity", 0.2);

	fd->detect(cvimg, vec_keypoints);
*/
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Rect> bboxes;
	cv::Ptr<cv::MSER> mser = cv::MSER::create();	//(5, 3, maxArea, 0.25, 0.2);
	mser->detectRegions(cvimg, contours, bboxes);

	double expandPatchFactor = 2.0;		// This expands the extracted patch around the found MSER. Seems to help...
	int patchResolution = 20;
	vl_size patchSize = 2*patchResolution + 1;
	std::vector<float> patch(patchSize * patchSize);

	// Prepare LIOP descriptor
	VlLiopDesc * liop = r3d_vl_liopdesc_new_basic((vl_size)patchSize);
	vl_size dimension = r3d_vl_liopdesc_get_dimension(liop);

	assert(dimension == DescriptorR3D::static_size);	// Make sure descriptor sizes match

	std::vector<float> desc(dimension);
	DescriptorR3D descriptor;

	for(size_t i = 0; i < contours.size(); i++)
	{
		// Find enclosing rectangle for all points in the contour
		const std::vector<cv::Point> &curContour = contours[i];
		int minx = img.Width() - 1, maxx = 0;
		int miny = img.Height() - 1, maxy = 0;
		for(size_t j = 0; j < curContour.size(); j++)
		{
			int curX = curContour[j].x;
			int curY = curContour[j].y;
			minx = std::min(minx, curX);
			maxx = std::max(maxx, curX);
			miny = std::min(miny, curY);
			maxy = std::max(maxy, curY);
		}

		if(expandPatchFactor > 0)
		{
			int minxnew = static_cast<int>( ((1.0 - expandPatchFactor)*maxx + (1.0 + expandPatchFactor)*minx ) / 2.0 );
			int maxxnew = static_cast<int>( ((1.0 + expandPatchFactor)*maxx + (1.0 - expandPatchFactor)*minx ) / 2.0 );
			int minynew = static_cast<int>( ((1.0 - expandPatchFactor)*maxy + (1.0 + expandPatchFactor)*miny ) / 2.0 );
			int maxynew = static_cast<int>( ((1.0 + expandPatchFactor)*maxy + (1.0 - expandPatchFactor)*miny ) / 2.0 );
			minx = minxnew;
			maxx = maxxnew;
			miny = minynew;
			maxy = maxynew;
		}

		// Extract patch from image
		int x = minx, y = miny;
		int width = maxx - minx + 1, height = maxy - miny + 1;
		if(x >= 0
			&& y >= 0
			&& (x + width) < cvimg.cols
			&& (y + height) < cvimg.rows
			&& width > 2
			&& height > 2)
		{
			cv::Rect keyPointRect(x, y, width, height);
			cv::Mat patch(cvimg, keyPointRect);
			int matType = patch.type();

			cv::Mat patchF_tmp, patchF;
			patch.convertTo(patchF_tmp, CV_32F);
			cv::resize(patchF_tmp, patchF, cv::Size(patchSize, patchSize));

			float *patchPtr = NULL;
			std::vector<float> patchVec;
			if(patch.isContinuous())
			{
				patchPtr = patchF.ptr<float>(0);
			}
			else
			{
				patchVec.resize(patchSize*patchSize);
				for(int i = 0; i < patchSize; i++)
				{
					float *rowPtr = patchF.ptr<float>(i);
					for(int j = 0; j < patchSize; j++)
						patchVec[i * patchSize + j] = *(rowPtr++);
				}
				patchPtr = &(patchVec[0]);
			}
			r3d_vl_liopdesc_process(liop, &(desc[0]), patchPtr);

			// Convert to OpenMVG keypoint and descriptor
			openMVG::features::SIOPointFeature fp;
			fp.x() = minx + (width/2);
			fp.y() = miny + (height/2);
			fp.scale() = std::sqrt(static_cast<float>(width*width + height*height)) / 2.0f;
			fp.orientation() = 0;

			for(int j = 0; j < dimension; j++)
				descriptor[j] = desc[j];
			descs.push_back(descriptor);
			feats.push_back(fp);
		}
	}

/*
	for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
		i_keypoint != vec_keypoints.end(); ++i_keypoint)
	{
		const cv::KeyPoint &kp = *(i_keypoint);
//		int x = static_cast<int>(kp.pt.x + 0.5f) - patchResolution;
//		int y = static_cast<int>(kp.pt.y + 0.5f) - patchResolution;
		int keypointSize = static_cast<int>(kp.size + 0.5f);	// Although kp.size is diameter, use it as radius for extracting the patch for LIOP
		int x = static_cast<int>(kp.pt.x + 0.5f) - keypointSize;
		int y = static_cast<int>(kp.pt.y + 0.5f) - keypointSize;
		//int width = patchSize, height = patchSize;
		int width = 2*keypointSize + 1, height = 2*keypointSize+1;

		if(x >= 0
			&& y >= 0
			&& (x + width) < cvimg.cols
			&& (y + height) < cvimg.rows
			&& keypointSize > 2)
		{
			cv::Rect keyPointRect(x, y, width, height);
			cv::Mat patch(cvimg, keyPointRect);
			int matType = patch.type();

			cv::Mat patchF_tmp, patchF;
			patch.convertTo(patchF_tmp, CV_32F);
			cv::resize(patchF_tmp, patchF, cv::Size(patchSize, patchSize));

			float *patchPtr = NULL;
			std::vector<float> patchVec;
			if(patch.isContinuous())
			{
				patchPtr = patchF.ptr<float>(0);
			}
			else
			{
				patchVec.resize(patchSize*patchSize);
				for(int i = 0; i < patchSize; i++)
				{
					float *rowPtr = patchF.ptr<float>(i);
					for(int j = 0; j < patchSize; j++)
						patchVec[i * patchSize + j] = *(rowPtr++);
				}
				patchPtr = &(patchVec[0]);
			}
			vl_liopdesc_process(liop, &(desc[0]), patchPtr);

			// Convert to OpenMVG keypoint and descriptor
			openMVG::features::SIOPointFeature fp;
			fp.x() = kp.pt.x;
			fp.y() = kp.pt.y;
			fp.scale() = kp.size/2.0f;		// kp.size is diameter, convert to radius
			fp.orientation() = kp.angle;

			for(int j = 0; j < dimension; j++)
				descriptor[j] = desc[j];
			descs.push_back(descriptor);
			feats.push_back(fp);
		}

	}
*/
}


/**
 * Detect keypoints using covariant feature detectors from VLFEAT and create LIOP descriptors.
 *
 * Make sure DescriptorR3D is set to 144 floats.
 */
void Regard3DFeatures::detectAndExtractVLFEAT_CoV_LIOP(const openMVG::image::Image<unsigned char> &img,
	Regard3DFeatures::FeatsR3D &feats, Regard3DFeatures::DescsR3D &descs)
{
#if defined(R3D_HAVE_VLFEAT)
	// Convert image float
	openMVG::image::Image<float> imgFloat( img.GetMat().cast<float>() );
	int w = img.Width(), h = img.Height();

	int octaveResolution = 2;
	double peakThreshold = 0.01;
	double edgeThreshold = 10.0;
	double boundaryMargin = 2.0;
	int patchResolution = 20;
	double patchRelativeExtent = 4.0;
	double patchRelativeSmoothing = 1.2;	//0.5;
	bool doubleImage = false;
	double smoothBeforeDetection = 0;	//1.0;

	// VL_COVDET_METHOD_DOG
	// VL_COVDET_METHOD_MULTISCALE_HARRIS
	VlCovDet * covdet = vl_covdet_new(VL_COVDET_METHOD_DOG);

	vl_covdet_set_first_octave(covdet, doubleImage ? -1 : 0);
	vl_covdet_set_octave_resolution(covdet, octaveResolution);
	vl_covdet_set_peak_threshold(covdet, peakThreshold);
	vl_covdet_set_edge_threshold(covdet, edgeThreshold);

	// Smooth image
	if(smoothBeforeDetection > 0)
		vl_imsmooth_f(imgFloat.data(), w, imgFloat.data(), w, h, w,
		smoothBeforeDetection, smoothBeforeDetection);

	// process the image and run the detector
	vl_covdet_put_image(covdet, imgFloat.data(), w, h);
	vl_covdet_detect(covdet);

	// drop features on the margin
	vl_covdet_drop_features_outside(covdet, boundaryMargin);

	// compute the affine shape of the features
	vl_covdet_extract_affine_shape(covdet);

	// compute the orientation of the features
//	vl_covdet_extract_orientations(covdet);

	// get feature frames back
	vl_size numFeatures = vl_covdet_get_num_features(covdet) ;
	VlCovDetFeature const *feature = reinterpret_cast<VlCovDetFeature const *>(vl_covdet_get_features(covdet));

	// get normalized feature appearance patches
	vl_size patchSize = 2*patchResolution + 1;
	std::vector<float> patch(patchSize * patchSize);

	// Prepare LIOP descriptor
	VlLiopDesc * liop = r3d_vl_liopdesc_new_basic((vl_size)patchSize);

	vl_size dimension = r3d_vl_liopdesc_get_dimension(liop);

	assert(dimension == DescriptorR3D::static_size);	// Make sure descriptor sizes match

	std::vector<float> desc(dimension);
	DescriptorR3D descriptor;

	for (int i = 0 ; i < numFeatures ; i++)
	{
		vl_covdet_extract_patch_for_frame(covdet,
			&(patch[0]),
			patchResolution,
			patchRelativeExtent,
			patchRelativeSmoothing,
			feature[i].frame);
/*
		vl_size numOrientations = 0;
		VlCovDetFeatureOrientation *featOrient = vl_covdet_extract_orientations_for_frame(covdet,
			&numOrientations, feature[i].frame);

		double angle1 = featOrient->angle;
//		double angle2 = std::atan2(feature[i].frame.a11, feature[i].frame.a12);
//		double angle3 = std::atan2(feature[i].frame.a21, feature[i].frame.a22);
		double angle2 = std::atan2(feature[i].frame.a11, feature[i].frame.a21);
		double angle3 = std::atan2(feature[i].frame.a12, feature[i].frame.a22);
**/
		double scale = 0;
		{
			double a11 = feature[i].frame.a11;
			double a12 = feature[i].frame.a12;
			double a21 = feature[i].frame.a21;
			double a22 = feature[i].frame.a22;
			double a = std::sqrt(a11*a11 + a21*a21);
			double b = std::sqrt(a21*a21 + a22*a22);
			scale = std::sqrt(a*a + b*b);
		}

		// Calculate LIOP descriptor
		r3d_vl_liopdesc_process(liop, &(desc[0]), &(patch[0]));

		// Convert to OpenMVG keypoint and descriptor
		openMVG::features::SIOPointFeature fp;
		fp.x() = feature[i].frame.x;
		fp.y() = feature[i].frame.y;
		fp.scale() = static_cast<float>(scale);
		fp.orientation() = 0.0f;		// TODO

		//siftDescToFloat(descr, descriptor, bRootSift);
		for(int j = 0; j < dimension; j++)
			descriptor[j] = desc[j];
		descs.push_back(descriptor);
		feats.push_back(fp);

	}

	// Clean up
	r3d_vl_liopdesc_delete(liop);
	vl_covdet_delete(covdet);
#endif
}


void Regard3DFeatures::detectKeypoints(const openMVG::image::Image<float> &img,
	std::vector< cv::KeyPoint > &vec_keypoints, const std::string &fdname,
	const Regard3DFeatures::R3DFParams &params)
{
	if(fdname == std::string("AKAZE"))
	{
		AKAZESemaLocker akazeLocker;	// Only allow one thread in this area

		// Convert image to OpenCV data
		cv::Mat cvimg;
		cv::eigen2cv(img.GetMat(), cvimg);

//		cv::Mat img_32;
//		cvimg.convertTo(img_32, CV_32F, 1.0/255.0, 0); // Convert to float, value range 0..1

		AKAZEOptions options;
		options.omin = 0;
		options.verbosity = false;

		options.dthreshold = params.threshold_;	// default: 0.001
		options.img_width = img.Width();
		options.img_height = img.Height();

		// Extract AKAZE features
		libAKAZE::AKAZE evolution(options);
		evolution.Create_Nonlinear_Scale_Space(cvimg);			//(img_32);
		evolution.Feature_Detection(vec_keypoints);

#if defined(R3D_USE_TBB_THREADING)
		tbb::parallel_for(tbb::blocked_range<size_t>(0, vec_keypoints.size()),
			[=, &vec_keypoints](const tbb::blocked_range<size_t>& r)				// Use lambda notation
		{
			for(size_t i = r.begin(); i != r.end(); ++i)
#else
#ifdef R3D_HAVE_OPENMP
		//omp_set_num_threads(OMP_MAX_THREADS);
#pragma omp parallel for
#endif
		for(int i = 0; i < static_cast<int>(vec_keypoints.size()); i++)
#endif
		{
			evolution.Compute_Main_Orientation(vec_keypoints[i]);
			// Convert from radians to degrees
			(vec_keypoints[i].angle) *= 180.0 / CV_PI;
			vec_keypoints[i].angle += 90.0f;
			while(vec_keypoints[i].angle < 0)
				vec_keypoints[i].angle += 360.0f;
			while(vec_keypoints[i].angle > 360.0f)
				vec_keypoints[i].angle -= 360.0f;
		}
#if defined(R3D_USE_TBB_THREADING)
	} );
#endif
	}
	else if(fdname == std::string("DOG"))		// VLFEAT
	{
	}
	else	// OpenCV
	{
		// Convert image to OpenCV data
		cv::Mat cvimg;
		cv::eigen2cv(img.GetMat(), cvimg);

		// Convert string to C locale
//		std::string fdname_cloc = boost::locale::conv::from_utf(fdname, "C");
		//cv::Ptr<cv::FeatureDetector> fd(cv::FeatureDetector::create(fdname));
		cv::Ptr<cv::FeatureDetector> fd;

		//assert(!fd.empty());

		if(fdname == std::string( "MSER" ))
		{
			fd = cv::MSER::create();
			/*int nrPixels = img.Width() * img.Height();
			int maxArea = static_cast<int>(nrPixels * 0.75);
			fd->setInt("delta", 5);
			fd->setInt("minArea", 3);
			fd->setInt("maxArea", maxArea);
			fd->setDouble("maxVariation", 0.25);
			fd->setDouble("minDiversity", 0.2);*/
		}
		else if(fdname == std::string( "ORB" ))
		{
//			fd->setInt("nFeatures", params.nFeatures_);
			fd = cv::ORB::create(params.nFeatures_);
		}
		else if(fdname == std::string( "BRISK" ))
		{
//			fd->setInt("thres", static_cast<int>(params.threshold_));
			fd = cv::BRISK::create();
		}
		else if(fdname == std::string( "GFTT" ))
		{
			//fd->setInt("nfeatures", params.nFeatures_);
			fd = cv::GFTTDetector::create(params.nFeatures_);
		}
//	fd->setDouble("edgeThreshold", 0.01);	// for SIFT

		fd->detect(cvimg, vec_keypoints);
	}
}

/**
 * Return the factor from keypoint size to feature size.
 *
 * Those factors have been determined with nlopt.
 */
float Regard3DFeatures::getKpSizeFactor(const std::string &fdname)
{
	float retVal = 1.0f;

	if(fdname == std::string("AKAZE"))
		retVal = 8.0f;
	else if(fdname == std::string("DOG"))
		retVal = 0.25f;
	else if(fdname == std::string( "MSER" ))
		retVal = 0.08;
	else if(fdname == std::string( "ORB" ))
		retVal = 0.025f;
	else if(fdname == std::string( "BRISK" ))
		retVal = 0.15f;
	else if(fdname == std::string( "GFTT" ))
		retVal = 0.13f;
	else if(fdname == std::string( "HARRIS" ))
		retVal = 0.25f;
	else if(fdname == std::string( "SimpleBlob" ))
		retVal = 1.0f;	// No working parameters found

	return retVal;
}

void Regard3DFeatures::extractDaisyFeatures(const openMVG::image::Image<unsigned char> &img, 
	std::vector< cv::KeyPoint > &vec_keypoints, float kpSizeFactor,
	FeatsR3D &feats, DescsR3D &descs)
{
	// Prepare Daisy descriptor
	std::auto_ptr<daisy> pDaisyDescr(new daisy());
	pDaisyDescr->verbose( 0 );
	pDaisyDescr->set_image(img.data(), img.Height(), img.Width());
	double rad = 15.0;
	int radq = 3, thq = 8, histq = 8;
	pDaisyDescr->set_parameters(rad, radq, thq, histq); // Quote Daisy doc: "We use 15,3,8,8 for wide baseline stereo"
	pDaisyDescr->initialize_single_descriptor_mode();

	int dimension = pDaisyDescr->descriptor_size();
	assert(dimension == DescriptorR3D::static_size);	// Must be equal to the size of DescriptorR3D
/*
	{
		cv::Mat cvimg, outimg;
		cv::eigen2cv(img.GetMat(), cvimg);

		std::ostringstream ostr;
		ostr << "F:/Projects/libs/openMVG/run/matches/kp_" << wxThread::GetCurrentId() << ".png";
		cv::drawKeypoints(cvimg, vec_keypoints, outimg, cv::Scalar(1.0, 0, 0, 1.0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imwrite(ostr.str(), outimg);
	}
*/
	std::vector<float> desc(dimension);
	std::vector<double> H(9);
	DescriptorR3D descriptor;

	for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
		i_keypoint != vec_keypoints.end(); ++i_keypoint)
	{
		const cv::KeyPoint &kp = *(i_keypoint);
		double x = kp.pt.x;
		double y = kp.pt.y;
		int orientation = static_cast<int>(kp.angle + 0.5f);	//180.0 * kp.angle / CV_PI + 0.5f);//static_cast<int>(kpSizeFactor*kp.angle + 0.5f);
		if(orientation < 0)	// Value -1 means "not valid"
			orientation = 0;
		if(orientation > 359)
			orientation = 0;
/*
		char buf[1024];
		sprintf_s(buf, 1024, "%g", kp.angle);
		OutputDebugStringA(buf);
*/
		double keypointSize = kp.size * 0.25;	//kpSizeFactor;
		for(int i = 0; i < 9; i++)
			H[i] = 0;
		H[0] = keypointSize;
		H[4] = keypointSize;
		H[8] = 1.0;
		H[2] = (1.0 - keypointSize) * x;
		H[5] = (1.0 - keypointSize) * y;
		bool extractOK = pDaisyDescr->get_descriptor(y, x, orientation, &(H[0]), &(desc[0]));

		if(extractOK)
		{
			// Convert to OpenMVG keypoint and descriptor
			openMVG::features::SIOPointFeature fp;
			fp.x() = kp.pt.x;
			fp.y() = kp.pt.y;
			fp.scale() = kp.size/2.0f;		// kp.size is diameter, convert to radius
			fp.orientation() = kp.angle;

			for(int j = 0; j < dimension; j++)
				descriptor[j] = desc[j];
			descs.push_back(descriptor);
			feats.push_back(fp);
		}
	}
}

void Regard3DFeatures::extractLIOPFeatures(const openMVG::image::Image<float> &img, 
	std::vector< cv::KeyPoint > &vec_keypoints, float kpSizeFactor,
	FeatsR3D &feats, DescsR3D &descs)
{
	// Convert image to OpenCV data
	cv::Mat cvimg;
	cv::eigen2cv(img.GetMat(), cvimg);

	const int patchResolution = 20;
	const double patchRelativeExtent = 4.0;
	const double patchRelativeSmoothing = 1.2;

	vl_size patchSize = 2*patchResolution + 1;

#if defined(R3D_USE_TBB_THREADING)
	tbb::mutex critSectionMutex;
	tbb::parallel_for(tbb::blocked_range<size_t>(0, vec_keypoints.size()),
		[=, &critSectionMutex, &vec_keypoints, &descs, &feats](const tbb::blocked_range<size_t>& r)	// Use lambda notation
#else
#if defined(R3D_HAVE_OPENMP)
#pragma omp parallel
#endif
#endif
	{
		// Initialisation of (in OpenMP or TBB case: per-thread) variables/structures
		std::vector<float> patchvec(patchSize * patchSize);
		// Prepare LIOP descriptor
		VlLiopDesc * liop = r3d_vl_liopdesc_new_basic((vl_size)patchSize);
		vl_size dimension = r3d_vl_liopdesc_get_dimension(liop);
		assert(dimension == DescriptorR3D::static_size);	// Must be equal to the size of DescriptorR3D

		std::vector<float> desc(dimension);
		DescriptorR3D descriptor;
		cv::Mat M(2, 3, CV_32F);	// 2x3 matrix

		// Prepare patch
		cv::Mat patchcv;
		patchcv.create(patchSize, patchSize, CV_32F);

#if defined(R3D_USE_TBB_THREADING)
		for(size_t i = r.begin(); i != r.end(); ++i)
#else
#if defined(R3D_HAVE_OPENMP)
#pragma omp for schedule(static)
#endif
		for (int i = 0 ; i < static_cast<int>(vec_keypoints.size()); i++)
#endif
		{
			const cv::KeyPoint &kp = vec_keypoints[i];
			float x = kp.pt.x;
			float y = kp.pt.y;

			// LIOP is rotationally invariant, so angle is not improving much
			float angle = -90.0f-kp.angle;	// angle in degrees
			float kpsize = kp.size;		// diameter
			float scale = kpsize / static_cast<float>(patchSize) * kpSizeFactor;

			// Extract patch TODO: Detect corner case
/*			cv::Mat M_rot, M, M_trans;

			M_trans = cv::Mat::eye(3, 3, CV_64F);	// Identity matrix: rows, columns, type
			M_trans.at<double>(0, 2) = x - static_cast<double>(patchResolution);
			M_trans.at<double>(1, 2) = y - static_cast<double>(patchResolution);
				
			M_rot = cv::getRotationMatrix2D(cv::Point2f(x, y), angle, scale);
			M_rot.resize(3, cv::Scalar(0));		// Set number of rows to 3 (-> 3x3 matrix)
			M_rot.at<double>(2, 2) = 1.0;

			M = M_rot * M_trans;
			M.resize(2);		// Reduce one row to 2x3 matrix
*/
			float alpha = scale * std::cos( angle * CV_PI / 180.0f );
			float beta = scale * std::sin( angle * CV_PI / 180.0f );
			float trans_x = x - static_cast<float>(patchResolution);
			float trans_y = y - static_cast<float>(patchResolution);
			M.at<float>(0, 0) = alpha;
			M.at<float>(0, 1) = beta;
			M.at<float>(0, 2) = beta*trans_y + alpha*trans_x - beta*y + (1.0f - alpha)*x;
			M.at<float>(1, 0) = -beta;
			M.at<float>(1, 1) = alpha;
			M.at<float>(1, 2) = alpha*trans_y - beta*trans_x + beta*x + (1.0f - alpha)*y;

			// Extract patch, rotate and resize it to patchSize * patchSize
			cv::warpAffine(cvimg, patchcv, M, cv::Size(patchSize, patchSize),
				cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

			// Gauss filter for smoothing (suggested by LIOP paper)
			if(patchRelativeSmoothing > 1.0)
				cv::GaussianBlur(patchcv, patchcv, cv::Size(0, 0), patchRelativeSmoothing);

			float *patchPtr = NULL;
			if(patchcv.isContinuous())
			{
				patchPtr = patchcv.ptr<float>(0);
			}
			else
			{
				patchvec.resize(patchSize*patchSize);
				for(int i = 0; i < patchSize; i++)
				{
					float *rowPtr = patchcv.ptr<float>(i);
					for(int j = 0; j < patchSize; j++)
						patchvec[i * patchSize + j] = *(rowPtr++);
				}
				patchPtr = &(patchvec[0]);
			}

			// Calculate LIOP descriptor
			r3d_vl_liopdesc_process(liop, &(desc[0]), patchPtr);

			{
				// Convert to OpenMVG keypoint and descriptor
				openMVG::features::SIOPointFeature fp;
				fp.x() = kp.pt.x;
				fp.y() = kp.pt.y;
				fp.scale() = kp.size/2.0f;		// kp.size is diameter, convert to radius
				fp.orientation() = kp.angle;

				for(int j = 0; j < dimension; j++)
					descriptor[j] = desc[j];

#if defined(R3D_USE_TBB_THREADING)
#elif defined(USE_OPENMP)
#pragma omp critical
#endif
				{
#if defined(R3D_USE_TBB_THREADING)
					tbb::mutex::scoped_lock lock(critSectionMutex);
#endif
					descs.push_back(descriptor);
					feats.push_back(fp);
				}
			}

		}
		r3d_vl_liopdesc_delete(liop);
	}
#if defined(R3D_USE_TBB_THREADING)
	);
#endif
	// Clean up
	//r3d_vl_liopdesc_delete(liop);
}

