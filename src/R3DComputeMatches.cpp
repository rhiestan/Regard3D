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
#include "R3DComputeMatches.h"
#include "OpenMVGHelper.h"
#include "Regard3DFeatures.h"
#include "R3DFeaturesThread.h"
#include "R3DProject.h"
#include "Regard3DMainFrame.h"

#include "minilog/minilog.h"



#if defined(R3D_HAVE_OPENMP)
#	include <omp.h>
#endif

#if defined(R3D_HAVE_TBB) && !defined(R3D_HAVE_OPENMP)

#define R3D_USE_TBB_THREADING 1
#	include <tbb/tbb.h>
#	include <tbb/task_scheduler_init.h>
#endif

R3DComputeMatches::R3DComputeMatches()
	: pMainFrame_(NULL)
{
}

R3DComputeMatches::~R3DComputeMatches()
{
}

void R3DComputeMatches::addImages(const ImageInfoVector &iiv)
{
	imageInfoVector_ = iiv;
}


// The following code is heavily based on main_computeMatches_OpenCV.cpp by Pierre Moulon (part of OpenMVG).
// Original copyright follows:

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/image/image_container.hpp"
#include "openMVG/image/pixel_types.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/feature.hpp"

/// Generic Image Collection image matching
#if defined(R3D_USE_OPENMVG_PRE08)
#	include "openMVG/matching_image_collection/Matcher_AllInMemory.hpp"
#else
//#	include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
//#include "software/SfM/io_regions_type.hpp"
#endif
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching/indMatch_utils.hpp"

#include "software/SfM/SfMIOHelper.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/matching/matcher_kdtree_flann.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/metric_hamming.hpp"

#include "third_party/cmdLine/cmdLine.h"

// OpenCV Includes
#include "opencv2/core/eigen.hpp" //To Convert Eigen matrix to cv matrix
// Legacy free features
#include "opencv2/features2d/features2d.hpp"
// Patent protected features
//#include "opencv2/nonfree/features2d.hpp"

#if defined(R3D_HAVE_NLOPT)
// NLOPT
#	include "nlopt.hpp"
#endif

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;
using namespace std;

enum eGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX = 1,
  HOMOGRAPHY_MATRIX = 2
};


// Equality functor to count the number of similar K matrices in the essential matrix case.
bool testIntrinsicsEquality(
  SfMIO::IntrinsicCameraInfo const &ci1,
  SfMIO::IntrinsicCameraInfo const &ci2)
{
  return ci1.m_K == ci2.m_K;
}

/// Extract OpenCV features and convert them to openMVG features/descriptor data
template <class DescriptorT, class cvFeature2DInterfaceT>
static bool ComputeCVFeatAndDesc(const openMVG::image::Image<unsigned char>& I,
  std::vector<openMVG::features::SIOPointFeature>& feats,
  std::vector<DescriptorT >& descs,
  cvFeature2DInterfaceT &detectAndDescribeClass)
{
  //Convert image to OpenCV data
  cv::Mat img;
  cv::eigen2cv(I.GetMat(), img);

  std::vector< cv::KeyPoint > vec_keypoints;
  cv::Mat m_desc;

  detectAndDescribeClass(img, cv::Mat(), vec_keypoints, m_desc);

  if (!vec_keypoints.empty())
  {
    feats.reserve(vec_keypoints.size());
    descs.reserve(vec_keypoints.size());

    DescriptorT descriptor;
    int cpt = 0;
    for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
      i_keypoint != vec_keypoints.end(); ++i_keypoint, ++cpt){

      openMVG::features::SIOPointFeature feat((*i_keypoint).pt.x, (*i_keypoint).pt.y, (*i_keypoint).size, (*i_keypoint).angle);
      feats.push_back(feat);

      memcpy(descriptor.getData(),
             m_desc.ptr<typename DescriptorT::bin_type>(cpt),
             DescriptorT::static_size*sizeof(typename DescriptorT::bin_type));
      descs.push_back(descriptor);
    }
    return true;
  }
  return false;
}

void extractFeaturesAndDescriptors_r3df_nlopt(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  double factor = 0.5)
{
  openMVG::image::Image<openMVG::image::RGBColor> imageRGB;
  openMVG::image::Image<unsigned char> imageGray;

  for(size_t i=0; i < vec_fileNames.size(); ++i) 
  {
    Regard3DFeatures::KeypointSetR3D kpSet;

    std::string sFeat = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "feat");
    std::string sDesc = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "desc");

    //Test if descriptor and feature was already computed
    if (stlplus::file_exists(sFeat) && stlplus::file_exists(sDesc)) {

    }
    else  { //Not already computed, so compute and save

      if (!ReadImage(vec_fileNames[i].c_str(), &imageGray))
        continue;

      // Compute features and descriptors and export them to file
	  Regard3DFeatures::detectAndExtract_NLOPT(imageGray,  kpSet.features(),
		  kpSet.descriptors(), factor);
      kpSet.saveToBinFile(sFeat, sDesc);
    }
  }
}


/// Extract features and descriptor and save them to files
void extractFeaturesAndDescriptors_SingleFile_r3df(
  const std::string &filename, // input filename
  const std::string & sOutDir, // Output directory where features and descriptor will be saved
  const Regard3DFeatures::R3DFParams &params)
{
  openMVG::image::Image<openMVG::image::RGBColor> imageRGB;
  openMVG::image::Image<unsigned char> imageGray;

  Regard3DFeatures::KeypointSetR3D kpSet;

  std::string sFeat = stlplus::create_filespec(sOutDir,
    stlplus::basename_part(filename), "feat");
  std::string sDesc = stlplus::create_filespec(sOutDir,
    stlplus::basename_part(filename), "desc");

  //Test if descriptor and feature was already computed
  if (stlplus::file_exists(sFeat) && stlplus::file_exists(sDesc)) {

  }
  else  { //Not already computed, so compute and save

    if (!ReadImage(filename.c_str(), &imageGray))
      return;

    // Compute features and descriptors and export them to file
//    Regard3DFeatures::detectAndExtract(imageGray,  kpSet.features(),
 //     kpSet.descriptors(), params);
    kpSet.saveToBinFile(sFeat, sDesc);
  }
}

void extractFeaturesAndDescriptors_Parallel_r3df(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  const Regard3DFeatures::R3DFParams &params)
{
#if defined(R3D_HAVE_OPENMP)
  int i = 0;
#	pragma omp parallel shared(vec_fileNames, sOutDir, params) private(i)
  {
#	pragma omp for schedule(dynamic, 1)
    for(i = 0; i < static_cast<int>(vec_fileNames.size()); ++i)
    {
      extractFeaturesAndDescriptors_SingleFile_r3df(vec_fileNames[i], sOutDir, params);
    }
  }
#else
  for(size_t i=0; i < vec_fileNames.size(); ++i)
  {
    extractFeaturesAndDescriptors_SingleFile_r3df(vec_fileNames[i], sOutDir, params);
  }
#endif
}



typedef openMVG::features::SIOPointFeature FeatureT_Test;
typedef std::vector<FeatureT_Test> FeatsT_Test;

typedef openMVG::features::Descriptor<unsigned char, 64> DescriptorT_BRISK_Test;
typedef vector<DescriptorT_BRISK_Test> DescsT_BRISK_Test;
typedef openMVG::features::KeypointSet<FeatsT_Test, DescsT_BRISK_Test > KeypointSetT_BRISK_Test;


static bool ComputeCVFeatAndDesc_Test(const openMVG::image::Image<unsigned char>& I,
  std::vector<openMVG::features::SIOPointFeature>& feats,
  std::vector<DescriptorT_BRISK_Test >& descs)
{
  // Convert image to OpenCV data
  cv::Mat img;
  cv::eigen2cv(I.GetMat(), img);

  std::vector< cv::KeyPoint > vec_keypoints;
  cv::Mat m_desc;

  cv::Ptr<cv::FeatureDetector> fd(cv::ORB::create());
  fd->detect(img, vec_keypoints);
/*
  {
    std::vector<std::string> parameters;
	fd->getParams(parameters);

    for (int i = 0; i < (int) parameters.size(); i++)
	{
        std::string param = parameters[i];
        int type = fd->paramType(param);
        std::string helpText = fd->paramHelp(param);
        std::string typeText;

        switch (type) {
        case cv::Param::BOOLEAN:
            typeText = "bool";
            break;
        case cv::Param::INT:
            typeText = "int";
            break;
        case cv::Param::REAL:
            typeText = "real (double)";
            break;
        case cv::Param::STRING:
            typeText = "string";
            break;
        case cv::Param::MAT:
            typeText = "Mat";
            break;
        case cv::Param::ALGORITHM:
            typeText = "Algorithm";
            break;
        case cv::Param::MAT_VECTOR:
            typeText = "Mat vector";
            break;
        }
	}
  }*/





  //detectAndDescribeClass(img, cv::Mat(), vec_keypoints, m_desc);
  cv::Ptr<cv::DescriptorExtractor> de(cv::BRISK::create());
  int descrSize = de->descriptorSize();
  int descrType = de->descriptorType();


  /*{
    std::vector<std::string> parameters;
	de->getParams(parameters);

    for (int i = 0; i < (int) parameters.size(); i++)
	{
        std::string param = parameters[i];
        int type = de->paramType(param);
        std::string helpText = de->paramHelp(param);
        std::string typeText;

        switch (type) {
        case cv::Param::BOOLEAN:
            typeText = "bool";
            break;
        case cv::Param::INT:
            typeText = "int";
            break;
        case cv::Param::REAL:
            typeText = "real (double)";
            break;
        case cv::Param::STRING:
            typeText = "string";
            break;
        case cv::Param::MAT:
            typeText = "Mat";
            break;
        case cv::Param::ALGORITHM:
            typeText = "Algorithm";
            break;
        case cv::Param::MAT_VECTOR:
            typeText = "Mat vector";
            break;
        }
	}
  }*/




  de->compute(img, vec_keypoints, m_desc);

  if (!vec_keypoints.empty())
  {
    feats.reserve(vec_keypoints.size());
    descs.reserve(vec_keypoints.size());

    DescriptorT_BRISK_Test descriptor;
    int cpt = 0;
    for(std::vector< cv::KeyPoint >::const_iterator i_keypoint = vec_keypoints.begin();
      i_keypoint != vec_keypoints.end(); ++i_keypoint, ++cpt){

      openMVG::features::SIOPointFeature feat((*i_keypoint).pt.x, (*i_keypoint).pt.y, (*i_keypoint).size, (*i_keypoint).angle);
      feats.push_back(feat);

      memcpy(descriptor.data(),
             m_desc.ptr<DescriptorT_BRISK_Test::bin_type>(cpt),
             DescriptorT_BRISK_Test::static_size*sizeof(DescriptorT_BRISK_Test::bin_type));
      descs.push_back(descriptor);
    }
    return true;
  }
  return false;
}

void extractFeaturesAndDescriptors_Test(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir)  // Output directory where features and descriptor will be saved
{
  typedef openMVG::features::SIOPointFeature FeatureT;
  typedef std::vector<FeatureT> FeatsT;
  openMVG::image::Image<openMVG::image::RGBColor> imageRGB;
  openMVG::image::Image<unsigned char> imageGray;

  for(size_t i=0; i < vec_fileNames.size(); ++i) 
  {
    KeypointSetT_BRISK_Test kpSet;

    std::string sFeat = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "feat");
    std::string sDesc = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "desc");

    //Test if descriptor and feature was already computed
    if (stlplus::file_exists(sFeat) && stlplus::file_exists(sDesc)) {

    }
    else  { //Not already computed, so compute and save

      if (!ReadImage(vec_fileNames[i].c_str(), &imageGray))
        continue;

      // Compute features and descriptors and export them to file
      ComputeCVFeatAndDesc_Test(imageGray,  kpSet.features(),
		  kpSet.descriptors());
      kpSet.saveToBinFile(sFeat, sDesc);
    }
  }
}

/// Extract features and descriptor and save them to files
template<class KeypointSetT, class DescriptorT, class cvFeature2DInterfaceT>
void extractFeaturesAndDescriptors_SingleFile(
  const std::string &filename, // input filename
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  cvFeature2DInterfaceT &detectAndDescribeClass)
{
  openMVG::image::Image<openMVG::image::RGBColor> imageRGB;
  openMVG::image::Image<unsigned char> imageGray;

  KeypointSetT kpSet;

  std::string sFeat = stlplus::create_filespec(sOutDir,
    stlplus::basename_part(filename), "feat");
  std::string sDesc = stlplus::create_filespec(sOutDir,
    stlplus::basename_part(filename), "desc");

  //Test if descriptor and feature was already computed
  if (stlplus::file_exists(sFeat) && stlplus::file_exists(sDesc)) {

  }
  else  { //Not already computed, so compute and save

    if (!ReadImage(filename.c_str(), &imageGray))
      return;

    // Compute features and descriptors and export them to file
    ComputeCVFeatAndDesc<DescriptorT, cvFeature2DInterfaceT>(imageGray,  kpSet.features(),
	  kpSet.descriptors(), detectAndDescribeClass);
    kpSet.saveToBinFile(sFeat, sDesc);
  }
}

template<class KeypointSetT, class DescriptorT, class cvFeature2DInterfaceT>
void extractFeaturesAndDescriptors_Parallel(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  cvFeature2DInterfaceT &detectAndDescribeClass)
{
#if defined(R3D_HAVE_OPENMP)
  int i = 0;
  cvFeature2DInterfaceT detectAndDescribeClassCopy;
#	pragma omp parallel shared(detectAndDescribeClass, vec_fileNames, sOutDir) private(i, detectAndDescribeClassCopy)
  {
#	pragma omp for schedule(dynamic, 1)
    for(i = 0; i < static_cast<int>(vec_fileNames.size()); ++i)
    {
      detectAndDescribeClassCopy = detectAndDescribeClass;
      extractFeaturesAndDescriptors_SingleFile<KeypointSetT, DescriptorT, cvFeature2DInterfaceT>
       (vec_fileNames[i], sOutDir, detectAndDescribeClassCopy);
    }
  }
#else
  for(size_t i=0; i < vec_fileNames.size(); ++i)
  {
    cvFeature2DInterfaceT detectAndDescribeClassCopy(detectAndDescribeClass);
    extractFeaturesAndDescriptors_SingleFile<KeypointSetT, DescriptorT, cvFeature2DInterfaceT>
     (vec_fileNames[i], sOutDir, detectAndDescribeClassCopy);
  }
#endif
}

template<class KeypointSetT, class DescriptorT, class cvFeature2DInterfaceT>
void extractFeaturesAndDescriptors(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  std::vector<std::pair<size_t, size_t> > & vec_imagesSize, // input image size (w,h)
  cvFeature2DInterfaceT &detectAndDescribeClass)
{
  vec_imagesSize.resize(vec_fileNames.size());
  openMVG::image::Image<openMVG::image::RGBColor> imageRGB;
  openMVG::image::Image<unsigned char> imageGray;

  C_Progress_display my_progress_bar( vec_fileNames.size() );
  for(size_t i=0; i < vec_fileNames.size(); ++i)  {
    KeypointSetT kpSet;

    std::string sFeat = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "feat");
    std::string sDesc = stlplus::create_filespec(sOutDir,
      stlplus::basename_part(vec_fileNames[i]), "desc");

    //Test if descriptor and feature was already computed
    if (stlplus::file_exists(sFeat) && stlplus::file_exists(sDesc)) {

      if (ReadImage(vec_fileNames[i].c_str(), &imageRGB)) {
        vec_imagesSize[i] = make_pair(imageRGB.Width(), imageRGB.Height());
      }
      else {
        ReadImage(vec_fileNames[i].c_str(), &imageGray);
        vec_imagesSize[i] = make_pair(imageGray.Width(), imageGray.Height());
      }
    }
    else  { //Not already computed, so compute and save

      if (!ReadImage(vec_fileNames[i].c_str(), &imageGray))
        continue;

      // Compute features and descriptors and export them to file
      ComputeCVFeatAndDesc<DescriptorT, cvFeature2DInterfaceT>(imageGray,  kpSet.features(),
		  kpSet.descriptors(), detectAndDescribeClass);
      kpSet.saveToBinFile(sFeat, sDesc);
      vec_imagesSize[i] = make_pair(imageGray.Width(), imageGray.Height());
    }
    ++my_progress_bar;
  }
}

template<class DescriptorT, class KeypointSetT, class MatcherT>
void computePutativeDescriptorMatches(
  const std::vector<std::string> & vec_fileNames, // input filenames
  const std::string & sOutDir,  // Output directory where features and descriptor will be saved
  PairWiseMatches &map_PutativesMatches,
  float fDistRatio
)
{
#if defined(R3D_USE_OPENMVG_PRE08)
  //---------------------------------------
  // c. Compute putative descriptor matches
  //    - L2 descriptor matching
  //    - Keep correspondences only if NearestNeighbor ratio is ok
  //---------------------------------------
  //PairWiseMatches map_PutativesMatches;
  // Define the matcher and the used metric (Squared L2)
  // ANN matcher could be defined as follow:
  //typedef flann::L2<DescriptorT::bin_type> MetricT;
  //typedef ArrayMatcher_Kdtree_Flann<DescriptorT::bin_type, MetricT> MatcherT;
  // Brute force matcher can be defined as following:
  //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
  //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;

  // If the matches already exists, reload them
  if (stlplus::file_exists(sOutDir + "/matches.putative.txt"))
  {
    PairedIndMatchImport(sOutDir + "/matches.putative.txt", map_PutativesMatches);
    MLOG << std::endl << "PUTATIVE MATCHES -- PREVIOUS RESULTS LOADED" << std::endl;
  }
  else // Compute the putatives matches
  {
    Matcher_AllInMemory<KeypointSetT, MatcherT> collectionMatcher(fDistRatio);
    if (collectionMatcher.loadData(vec_fileNames, sOutDir))
    {
      MLOG  << std::endl << "PUTATIVE MATCHES" << std::endl;
      collectionMatcher.Match(vec_fileNames, map_PutativesMatches);
      //---------------------------------------
      //-- Export putative matches
      //---------------------------------------
      std::ofstream file (std::string(sOutDir + "/matches.putative.txt").c_str());
      if (file.is_open())
        PairedIndMatchToStream(map_PutativesMatches, file);
      file.close();
    }
  }
  //-- export putative matches Adjacency matrix
  PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
    map_PutativesMatches,
    stlplus::create_filespec(sOutDir, "PutativeAdjacencyMatrix", "svg"));
#endif
}

void R3DComputeMatches::setMainFrame(Regard3DMainFrame *pMainFrame)
{
	pMainFrame_ = pMainFrame;
}

bool R3DComputeMatches::computeMatches(Regard3DFeatures::R3DFParams &params, bool svgOutput, const R3DProjectPaths &paths)
{
  R3DProject *pProject = R3DProject::getInstance();
  assert(pProject != NULL);
  std::string sImaDirectory(paths.relativeImagePath_);		//pProject->getRelativeImagePath());
  std::string sOutDir(paths.relativeMatchesPath_);		//pProject->getRelativeMatchesPath());
  R3DFeatureExtractor fe = R3DFER3DF;

#if defined(R3D_USE_OPENMVG_PRE08)
  // -----------------------------
  // a. List images
  // b. Compute features and descriptors
  // c. Compute putative descriptor matches
  // d. Geometric filtering of putative matches
  // e. Export some statistics
  // -----------------------------

  // Create output dir
  if(!stlplus::folder_exists(sOutDir))
	  stlplus::folder_create(sOutDir);

  // Create lists.txt
  pProject->writeImageListTXT(paths);

  //---------------------------------------
  // a. List images
  //---------------------------------------
  std::string sListsFile = stlplus::create_filespec(sOutDir, "lists.txt" );
  if (!stlplus::is_file(sListsFile)) {
 //   std::cerr << std::endl
 //     << "The input file \""<< sListsFile << "\" is missing" << std::endl;
    return false;
  }

  std::vector<openMVG::SfMIO::CameraInfo> vec_camImageName;
  std::vector<openMVG::SfMIO::IntrinsicCameraInfo> vec_focalGroup;
  if (!openMVG::SfMIO::loadImageList( vec_camImageName,
                                      vec_focalGroup,
                                      sListsFile) )
  {
    std::cerr << "\nEmpty image list." << std::endl;
    return false;
  }

  //  if (eGeometricModelToCompute == ESSENTIAL_MATRIX)
  if(params.computeEssentialMatrix_)
  {
	  //-- In the case of the essential matrix we check if only one K matrix is present.
	  //-- Due to the fact that the generic framework allows only one K matrix for the
	  // robust essential matrix estimation in image collection.
	  std::vector<openMVG::SfMIO::IntrinsicCameraInfo>::iterator iterF =
		  std::unique(vec_focalGroup.begin(), vec_focalGroup.end(), testIntrinsicsEquality);
	  vec_focalGroup.resize(std::distance(vec_focalGroup.begin(), iterF));
	  if(vec_focalGroup.size() == 1) {
		  // Set all the intrinsic ID to 0
		  for(size_t i = 0; i < vec_camImageName.size(); ++i)
			  vec_camImageName[i].m_intrinsicId = 0;
	  }
	  else  {
		  //      std::cerr << "There is more than one focal group in the lists.txt file." << std::endl
		  //        << "Only one focal group is supported for the image collection robust essential matrix estimation." << std::endl;
		  //      return false;

		  params.computeEssentialMatrix_ = false;
	  }
  }

  //-- Two alias to ease access to image filenames and image sizes
  std::vector<std::string> vec_fileNames;
  std::vector<std::pair<size_t, size_t> > vec_imagesSize;
  for(std::vector<openMVG::SfMIO::CameraInfo>::const_iterator
	  iter_camInfo = vec_camImageName.begin();
	  iter_camInfo != vec_camImageName.end();
  iter_camInfo++)
  {
	  vec_imagesSize.push_back(std::make_pair(vec_focalGroup[iter_camInfo->m_intrinsicId].m_w,
		  vec_focalGroup[iter_camInfo->m_intrinsicId].m_h));
	  vec_fileNames.push_back(stlplus::create_filespec(sImaDirectory, iter_camInfo->m_sImageName));
  }
#else
  std::string sSfM_Data_Filename(paths.matchesSfmDataFilename_);
  pProject->writeSfmData(paths);

  //---------------------------------------
  // Read SfM Scene (image view & intrinsics data)
  //---------------------------------------
  openMVG::sfm::SfM_Data sfm_data;
  if(!Load(sfm_data, sSfM_Data_Filename, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS | openMVG::sfm::INTRINSICS))) {
	  MLOG << std::endl
		  << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
	  return false;	// EXIT_FAILURE;
  }

  // Build some alias from SfM_Data Views data:
  // - List views as a vector of filenames & image sizes
  std::vector<std::string> vec_fileNames;
  std::vector<std::pair<size_t, size_t> > vec_imagesSize;
  {
    vec_fileNames.reserve(sfm_data.GetViews().size());
    vec_imagesSize.reserve(sfm_data.GetViews().size());
    for(openMVG::sfm::Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end();
      ++iter)
    {
      const openMVG::sfm::View * v = iter->second.get();
      vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,
        v->s_Img_path));
      vec_imagesSize.push_back(std::make_pair(v->ui_width, v->ui_height));
    }
  }

  std::unique_ptr<openMVG::features::Regions> regions_type(new Regard3DFeatures::R3D_AKAZE_LIOP_Regions);
  std::string sMatchesDirectory(paths.relativeMatchesPath_);
  std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
#endif


  //---------------------------------------
  // b. Compute features and descriptor
  //    - extract sift features and descriptor
  //    - if keypoints already computed, re-load them
  //    - else save features and descriptors on disk
  //---------------------------------------

  //-- Make your choice about the Feature Detector your want to use
  //--  Note that the openCV + openMVG interface is working only for
  //     floating point descriptor.

  //-- Surf opencv => default 64 floating point values
  //typedef cv::SURF cvFeature2DInterfaceT;
  //typedef Descriptor<float, 64> DescriptorT;
  //MLOG << "\nUse the opencv SURF interface" << std::endl;
  
  //-- Sift opencv => default 128 floating point values
  //typedef cv::SIFT cvFeature2DInterfaceT;
  //typedef Descriptor<float, 128> DescriptorT;
  //MLOG << "\nUse the opencv SIFT interface" << std::endl;

  //-- ORB opencv => default 128 floating point values
/*  typedef cv::ORB cvFeature2DInterfaceT;
  typedef Descriptor<unsigned char, 32> DescriptorT;
  MLOG << "\nUse the opencv ORB interface" << std::endl;
  */
/*  {
    cv::SURF surfT;
	int descrSize = surfT.descriptorSize();		// 128 oder 64 (Standard 64)
	int descrType = surfT.descriptorType();		// float
	int jj = 27;
  }
  {
    cv::SIFT siftT;
	int descrSize = siftT.descriptorSize();		// 128
	int descrType = siftT.descriptorType();		// float
	int jj = 27;
  }
  {
    cv::ORB orbT;
	int descrSize = orbT.descriptorSize();		// 32
	int descrType = orbT.descriptorType();		// bytes
	int jj = 27;
  }
  {
    cv::BRISK briskT;
	int descrSize = briskT.descriptorSize();		// 64
	int descrType = briskT.descriptorType();		// bytes
	int jj = 27;
  }*/
/*
  typedef SIOPointFeature FeatureT;
  typedef std::vector<FeatureT> FeatsT;
  typedef vector<DescriptorT > DescsT;
  typedef KeypointSet<FeatsT, DescsT > KeypointSetT;
*/

  //MLOG << "\n\nEXTRACT FEATURES" << std::endl;

#if defined(R3D_USE_TBB_THREADING)
  tbb::task_scheduler_init tbb_taskscheduler(tbb::task_scheduler_init::automatic);	// Let TBB determine number of threads
#endif
  Regard3DFeatures::initAKAZESemaphore(1);

  typedef openMVG::features::SIOPointFeature FeatureT;
  typedef std::vector<FeatureT> FeatsT;

  PairWiseMatches map_PutativesMatches;
  float fDistRatio = params.distRatio_;	// Nearest Neighbor distance ratio, default value is set to 0.6
							// 0.6 is restrictive, -r 0.8 is less restrictive
							// contrastThreshold = 0.01, fDistRatio = 0.8 helps having more matches


  if(fe == R3DFESIFT)
  {
#if defined(R3D_USE_PATENTED_ALGORITHMS)
    //-- Sift opencv => default 128 floating point values
    typedef cv::SIFT cvFeature2DInterfaceT_SIFT;
    typedef Descriptor<float, 128> DescriptorT_SIFT;
    typedef vector<DescriptorT_SIFT> DescsT_SIFT;
    typedef KeypointSet<FeatsT, DescsT_SIFT > KeypointSetT_SIFT;

	int nFeatures = 0, nOctaveLayers = 3;
	// defaults: contrastThreshold = 0.04, edgeThreshold = 10.0, sigma = 1.6;
	double contrastThreshold = 0.01, edgeThreshold = 10.0, sigma = 1.6;
	cv::SIFT siftInstance(nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
	extractFeaturesAndDescriptors_Parallel<KeypointSetT_SIFT, DescriptorT_SIFT, cv::SIFT>(
	  vec_fileNames, // input filenames
	  sOutDir,  // Output directory where features and descriptor will be saved
	  siftInstance);

	OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);

    typedef flann::L2<DescriptorT_SIFT::bin_type> MetricT_SIFT;
    typedef ArrayMatcher_Kdtree_Flann<DescriptorT_SIFT::bin_type, MetricT_SIFT> MatcherT_SIFT;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT_SIFT::bin_type> MetricT_SIFT;
    //typedef ArrayMatcherBruteForce<DescriptorT_SIFT::bin_type, MetricT_SIFT> MatcherT_SIFT;
    computePutativeDescriptorMatches<DescriptorT_SIFT, KeypointSetT_SIFT, MatcherT_SIFT>
		(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);
#endif
  }
  else if(fe == R3DFESURF)
  {
#if defined(R3D_USE_PATENTED_ALGORITHMS)
    //-- Surf opencv => 64 floating point values
    typedef cv::SURF cvFeature2DInterfaceT_SURF;
    typedef Descriptor<float, 64> DescriptorT_SURF;
    typedef vector<DescriptorT_SURF> DescsT_SURF;
    typedef KeypointSet<FeatsT, DescsT_SURF > KeypointSetT_SURF;

    double hessianThreshold = 20.0;
    int nOctaves = 4, nOctaveLayers = 2;
    bool extended = false, upright = false;
    cv::SURF surfInstance(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
    extractFeaturesAndDescriptors_Parallel<KeypointSetT_SURF, DescriptorT_SURF, cv::SURF>(
      vec_fileNames, // input filenames
      sOutDir,  // Output directory where features and descriptor will be saved
      surfInstance);

	OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);

    typedef flann::L2<DescriptorT_SURF::bin_type> MetricT_SURF;
    typedef ArrayMatcher_Kdtree_Flann<DescriptorT_SURF::bin_type, MetricT_SURF> MatcherT_SURF;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
    //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
    computePutativeDescriptorMatches<DescriptorT_SURF, KeypointSetT_SURF, MatcherT_SURF>
		(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);
#endif
  }
  else if(fe == R3DFEORB)
  {
#if 0			// TODO
    // Doesn't work properly, as OpenMVG only works with float descriptors
    typedef cv::ORB cvFeature2DInterfaceT_ORB;
    typedef Descriptor<unsigned char, 32> DescriptorT_ORB;
    typedef vector<DescriptorT_ORB> DescsT_ORB;
    typedef KeypointSet<FeatsT, DescsT_ORB > KeypointSetT_ORB;

	int nFeatures = 1000;
	double scaleFactor = 1.2f;
	cv::ORB orbInstance(nFeatures, scaleFactor);
	extractFeaturesAndDescriptors_Parallel<KeypointSetT_ORB, DescriptorT_ORB, cv::ORB>(
	  vec_fileNames, // input filenames
	  sOutDir,  // Output directory where features and descriptor will be saved
	  orbInstance);

	OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);
	
	typedef flann::L2<DescriptorT_ORB::bin_type> MetricT_ORB;
    typedef ArrayMatcher_Kdtree_Flann<DescriptorT_ORB::bin_type, MetricT_ORB> MatcherT_ORB;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
    //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
    computePutativeDescriptorMatches<DescriptorT_ORB, KeypointSetT_ORB, MatcherT_ORB>
		(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);
  }
  else if(fe == R3DFEBRISK)
  {
    // Doesn't work properly, as OpenMVG only works with float descriptors
    typedef cv::BRISK cvFeature2DInterfaceT_BRISK;
    typedef Descriptor<unsigned char, 64> DescriptorT_BRISK;
    typedef vector<DescriptorT_BRISK> DescsT_BRISK;
    typedef KeypointSet<FeatsT, DescsT_BRISK > KeypointSetT_BRISK;

	int threshold = 10, octaves = 3;
	cv::BRISK briskInstance(threshold, octaves);
	extractFeaturesAndDescriptors_Parallel<KeypointSetT_BRISK, DescriptorT_BRISK, cv::BRISK>(
	  vec_fileNames, // input filenames
	  sOutDir,  // Output directory where features and descriptor will be saved
	  briskInstance);

	OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);

    typedef flann::L2<DescriptorT_BRISK::bin_type> MetricT_BRISK;
    typedef ArrayMatcher_Kdtree_Flann<DescriptorT_BRISK::bin_type, MetricT_BRISK> MatcherT_BRISK;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
    //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
    computePutativeDescriptorMatches<DescriptorT_BRISK, KeypointSetT_BRISK, MatcherT_BRISK>
		(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);
  }
  else if(fe == R3DFETEST)
  {
    //-- Sift opencv => default 128 floating point values
/*    typedef cv::BRISK cvFeature2DInterfaceT_BRISK;
    typedef Descriptor<unsigned char, 64> DescriptorT_BRISK;
    typedef vector<DescriptorT_BRISK> DescsT_BRISK;
    typedef KeypointSet<FeatsT, DescsT_BRISK > KeypointSetT_BRISK;

	int threshold = 10, octaves = 3;
	cv::BRISK briskInstance(threshold, octaves);*/
	extractFeaturesAndDescriptors_Test(
	  vec_fileNames, // input filenames
	  sOutDir  // Output directory where features and descriptor will be saved
	  );

	OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);
/*
    typedef flann::L2<DescriptorT_BRISK::bin_type> MetricT_BRISK;
    typedef ArrayMatcher_Kdtree_Flann<DescriptorT_BRISK::bin_type, MetricT_BRISK> MatcherT_BRISK;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
    //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
    computePutativeDescriptorMatches<DescriptorT_BRISK, KeypointSetT_BRISK, MatcherT_BRISK>
		(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);*/
#endif
  }
  else if(fe == R3DFER3DF)
  {
    R3DFeaturesThread::extractFeaturesAndDescriptors(vec_fileNames,
	  sOutDir, params);
	statistics_.numberOfKeypoints_ = R3DFeaturesThread::getNumberOfKeypoints();

	updateProgress(0.7, wxT("Find putative matches"));

//    if(svgOutput)
//     OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);
    typedef flann::L2<Regard3DFeatures::DescriptorR3D::bin_type> MetricR3D;
    typedef ArrayMatcher_Kdtree_Flann<Regard3DFeatures::DescriptorR3D::bin_type, MetricR3D> MatcherR3D;
    // Brute force matcher can be defined as following:
    //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
    //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
#if defined(R3D_USE_OPENMVG_PRE08)
    computePutativeDescriptorMatches<Regard3DFeatures::DescriptorR3D, Regard3DFeatures::KeypointSetR3D, MatcherR3D>
      (vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);
#else
    std::unique_ptr<Matcher_Regions> collectionMatcher(new Matcher_Regions(fDistRatio, ANN_L2));
	if(regions_provider->load(sfm_data, sMatchesDirectory, regions_type))
    //if(collectionMatcher->loadData(regions_type, vec_fileNames, sMatchesDirectory))
    {
      Pair_Set pairs = exhaustivePairs(sfm_data.GetViews().size());
	  // Photometric matching of putative pairs
	  //collectionMatcher->Match(vec_fileNames, pairs, map_PutativesMatches);
	  collectionMatcher->Match(sfm_data, regions_provider, pairs, map_PutativesMatches);
	  //-- Export putative matches
      if (!Save(map_PutativesMatches, std::string(sMatchesDirectory + "/matches.putative.txt")))
      {
        MLOG
          << "Cannot save computed matches in: "
          << std::string(sMatchesDirectory + "/matches.putative.txt");
        return EXIT_FAILURE;
      }
    }
	//-- export putative matches Adjacency matrix
	PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
		map_PutativesMatches,
		stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg"));
#endif
  }

  statistics_.putativeMatches_ = map_PutativesMatches;

  //---------------------------------------
  // d. Geometric filtering of putative matches
  //    - AContrario Estimation of the desired geometric model
  //    - Use an upper bound for the a contrario estimated threshold
  //---------------------------------------
  PairWiseMatches map_GeometricMatches;

  const double maxResidualError = 4.0;	// Orig: 4 (higher is more relaxed, e.g. 5.5)
#if defined(R3D_USE_OPENMVG_PRE08)
  ImageCollectionGeometricFilter<FeatureT> collectionGeomFilter;
  if(collectionGeomFilter.loadData(vec_fileNames, sOutDir))
#else
  // Load the features
  std::shared_ptr<openMVG::sfm::Features_Provider> feats_provider = std::make_shared<openMVG::sfm::Features_Provider>();
  if(!feats_provider->load(sfm_data, sMatchesDirectory, regions_type)) {
    MLOG << std::endl << "Invalid features." << std::endl;
	return false;	// EXIT_FAILURE;
  }
  ImageCollectionGeometricFilter collectionGeomFilter(&sfm_data, regions_provider);
  int imax_iteration = 2048;
  bool bGuided_matching = false;
#endif
  {
    if(params.computeFundalmentalMatrix_)
    {
      map_GeometricMatches.clear();
      updateProgress(0.8, wxT("Calculate fundamental matrix"));
//      collectionGeomFilter.Filter(
//        GeometricFilter_FMatrix_AC(maxResidualError),
//        map_PutativesMatches,
//        map_GeometricMatches,
//        vec_imagesSize);
	  collectionGeomFilter.Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
		  map_PutativesMatches, bGuided_matching);
	  map_GeometricMatches = collectionGeomFilter.Get_geometric_matches();

      //---------------------------------------
      //-- Export geometric filtered matches
      //---------------------------------------
      if(!Save(map_GeometricMatches,
        std::string(paths.matchesFFilename_.c_str())))
      {
        MLOG
            << "Cannot save computed matches in: "
            << std::string(paths.matchesFFilename_.c_str());
        return EXIT_FAILURE;
      }
      statistics_.fundamentalMatches_ = map_GeometricMatches;
    }
    if(params.computeEssentialMatrix_)
    {
      map_GeometricMatches.clear();
      updateProgress(0.9, wxT("Calculate essential matrix"));
#if defined(R3D_USE_OPENMVG_PRE08)
      collectionGeomFilter.Filter(
        GeometricFilter_EMatrix_AC(vec_focalGroup[0].m_K, maxResidualError),
        map_PutativesMatches,
        map_GeometricMatches,
        vec_imagesSize);
#else
/*	  // Build the intrinsic parameter map for each view
	  std::map<IndexT, Mat3> map_K;
	  size_t cpt = 0;
	  for(openMVG::sfm::Views::const_iterator iter = sfm_data.GetViews().begin();
		  iter != sfm_data.GetViews().end();
		  ++iter, ++cpt)
	  {
		  const openMVG::sfm::View * v = iter->second.get();
		  if(sfm_data.GetIntrinsics().count(v->id_intrinsic))
		  {
			  const IntrinsicBase * ptrIntrinsic = sfm_data.GetIntrinsics().find(v->id_intrinsic)->second.get();
			  switch(ptrIntrinsic->getType())
			  {
			  case PINHOLE_CAMERA:
			  case PINHOLE_CAMERA_RADIAL1:
			  case PINHOLE_CAMERA_RADIAL3:
				  const Pinhole_Intrinsic * ptrPinhole = (const Pinhole_Intrinsic*)(ptrIntrinsic);
				  map_K[cpt] = ptrPinhole->K();
				  break;
			  }
		  }
	  }

	  collectionGeomFilter.Filter(
		  GeometricFilter_EMatrix_AC(map_K, maxResidualError),
		  map_PutativesMatches,
		  map_GeometricMatches,
		  vec_imagesSize);*/
	  collectionGeomFilter.Robust_model_estimation(GeometricFilter_EMatrix_AC(4.0, imax_iteration),
		  map_PutativesMatches, bGuided_matching);
	  map_GeometricMatches = collectionGeomFilter.Get_geometric_matches();
#endif
	  //-- Perform an additional check to remove pairs with poor overlap
	  std::vector<PairWiseMatches::key_type> vec_toRemove;
	  for(PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
		  iterMap != map_GeometricMatches.end(); ++iterMap)
	  {
		  const size_t putativePhotometricCount = map_PutativesMatches.find(iterMap->first)->second.size();
		  const size_t putativeGeometricCount = iterMap->second.size();
		  const float ratio = putativeGeometricCount / (float)putativePhotometricCount;
		  if(putativeGeometricCount < 50 || ratio < .3f)  {
			  // the pair will be removed
			  vec_toRemove.push_back(iterMap->first);
		  }
	  }
	  //-- remove discarded pairs
	  for(std::vector<PairWiseMatches::key_type>::const_iterator
		  iter = vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
	  {
		  map_GeometricMatches.erase(*iter);
	  }

      //---------------------------------------
      //-- Export geometric filtered matches
      //---------------------------------------
      if(!Save(map_GeometricMatches,
        std::string(paths.matchesEFilename_.c_str())))
      {
        MLOG
            << "Cannot save computed matches in: "
            << std::string(paths.matchesEFilename_.c_str());
        return EXIT_FAILURE;
      }
      statistics_.essentialMatches_ = map_GeometricMatches;
    }
    if(params.computeHomographyMatrix_)
    {
      map_GeometricMatches.clear();
      updateProgress(0.95, wxT("Calculate homography matrix"));
/*      collectionGeomFilter.Filter(
        GeometricFilter_HMatrix_AC(maxResidualError),
        map_PutativesMatches,
        map_GeometricMatches,
        vec_imagesSize);*/
	  const bool bGeometric_only_guided_matching = true;
	  collectionGeomFilter.Robust_model_estimation(GeometricFilter_HMatrix_AC(4.0, imax_iteration),
		  map_PutativesMatches, bGuided_matching,
		  bGeometric_only_guided_matching ? -1.0 : 0.6);
	  map_GeometricMatches = collectionGeomFilter.Get_geometric_matches();

      //---------------------------------------
      //-- Export geometric filtered matches
      //---------------------------------------
      if(!Save(map_GeometricMatches,
        std::string(paths.matchesHFilename_.c_str())))
      {
        MLOG
            << "Cannot save computed matches in: "
            << std::string(paths.matchesEFilename_.c_str());
        return EXIT_FAILURE;
      }
      statistics_.homographyMatches_ = map_GeometricMatches;
    }

    //-- export Adjacency matrix
//    MLOG << "\n Export Adjacency Matrix of the pairwise's geometric matches"
//      << std::endl;
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
      map_GeometricMatches,
      stlplus::create_filespec(sOutDir, "GeometricAdjacencyMatrix", "svg"));
/*
    if(svgOutput)
    {
      std::string sGeometricMatchesFilename;
      if(params.computeFundalmentalMatrix_)
        sGeometricMatchesFilename =	paths.matchesFFilename_;		//pProject->getMatchesFFilename();
      else if(params.computeEssentialMatrix_)
        sGeometricMatchesFilename =	paths.matchesEFilename_;			//pProject->getMatchesEFilename();
      OpenMVGHelper::exportMatches(vec_camImageName, vec_focalGroup,
        sImaDirectory, sOutDir, sOutDir, sGeometricMatchesFilename);
    }*/
  }
  Regard3DFeatures::uninitializeAKAZESemaphore();

  return true;
}













#if 0
#include "boost/filesystem.hpp"

int R3DComputeMatches::computeMatchesOpenCV_NLOPT_step(double factor)
{
  std::string sImaDirectory("F:/Projects/libs/openMVG/run/in");
  std::string sOutDir("F:/Projects/libs/openMVG/run/matches");
  std::string sGeometricModel = "f";	// f for incremental, e for global


  // Empty matches directory
  {
	boost::filesystem::path matchesPath("F:/Projects/libs/openMVG/run/matches");
	boost::system::error_code ec;
	//boost::filesystem::remove_all(matchesPath, ec);
	//boost::filesystem::create_directory(matchesPath, ec);
	boost::filesystem::directory_iterator dir_iter(matchesPath, ec);
	while(dir_iter != boost::filesystem::directory_iterator())
	{
		const boost::filesystem::directory_entry &dirEntry = *(dir_iter);
		if(boost::filesystem::is_regular_file(dirEntry.status()))
			boost::filesystem::remove(dir_iter->path());

		dir_iter++;
	}
  }


  eGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
  std::string sGeometricMatchesFilename = "";
  switch(sGeometricModel[0])
  {
    case 'f': case 'F':
      eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
      sGeometricMatchesFilename = "matches.f.txt";
    break;
    case 'e': case 'E':
      eGeometricModelToCompute = ESSENTIAL_MATRIX;
      sGeometricMatchesFilename = "matches.e.txt";
    break;
    case 'h': case 'H':
      eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
      sGeometricMatchesFilename = "matches.h.txt";
    break;
    default:
      //std::cerr << "Unknown geometric model" << std::endl;
      return false;
  }

  // -----------------------------
  // a. List images
  // b. Compute features and descriptors
  // c. Compute putative descriptor matches
  // d. Geometric filtering of putative matches
  // e. Export some statistics
  // -----------------------------

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  //---------------------------------------
  // a. List images
  //---------------------------------------
  std::vector<openMVG::SfMIO::CameraInfo> vec_camImageName;
  std::vector<openMVG::SfMIO::IntrinsicCameraInfo> vec_focalGroup;

  // Inserted by RH
  bool firstItem = true;
  ImageInfoVectorIterator iter = imageInfoVector_.begin();
  while(iter != imageInfoVector_.end())
  {
    ImageInfo &ii = (*iter);

    openMVG::SfMIO::CameraInfo camInfo;
    camInfo.m_sImageName = std::string(ii.importedFilename_.mb_str());	// Use imported name (can safely be converted to C-string)    camInfo.m_intrinsicId = 0;
    vec_camImageName.push_back(camInfo);

	if(firstItem)
	{
		firstItem = false;
		openMVG::SfMIO::IntrinsicCameraInfo intrinsicCamInfo;
		intrinsicCamInfo.m_bKnownIntrinsic = true;
		double focal = std::max ( ii.imageWidth_, ii.imageHeight_ ) * ii.focalLength_ / ii.sensorWidth_;
		intrinsicCamInfo.m_focal = focal;
		intrinsicCamInfo.m_sCameraMaker = std::string(ii.cameraMaker_.mb_str());
		intrinsicCamInfo.m_sCameraModel = std::string(ii.cameraModel_.mb_str());
		intrinsicCamInfo.m_w = ii.imageWidth_;
		intrinsicCamInfo.m_h = ii.imageHeight_;
        Mat3 K;
        K << focal, 0, double(ii.imageWidth_) / 2.,
             0, focal, double(ii.imageHeight_) / 2.,
             0, 0, 1.;
        intrinsicCamInfo.m_K = K;

		vec_focalGroup.push_back(intrinsicCamInfo);
	}
    iter++;
  }

  // Create lists.txt
  std::ofstream listTXT( stlplus::create_filespec( sOutDir,
                                                   "lists.txt" ).c_str() );
  if ( listTXT )
  {
    iter = imageInfoVector_.begin();
	while(iter != imageInfoVector_.end())
	{
      ImageInfo &ii = (*iter);
	  double focal = ii.focalLength_;
	  int width = ii.imageWidth_;
	  int height = ii.imageHeight_;
	  std::string sCamName = std::string(ii.cameraMaker_.mb_str());
	  std::string sCamModel = std::string(ii.cameraModel_.mb_str());
	  std::string imgFilenameBase = std::string(ii.importedFilename_.mb_str());	// Use imported name (can safely be converted to C-string)

      // The camera model was found in the database so we can compute its approximated focal length
	  double ccdw = ii.sensorWidth_;
      focal = std::max ( width, height ) * focal / ccdw;
      listTXT << imgFilenameBase << ";" << width << ";" << height << ";" << focal << ";" << sCamName << ";" << sCamModel << std::endl;
	  iter++;
    }
  }
  listTXT.close();
  // End inserted by RH

  if (eGeometricModelToCompute == ESSENTIAL_MATRIX)
  {
    //-- In the case of the essential matrix we check if only one K matrix is present.
    //-- Due to the fact that the generic framework allows only one K matrix for the
    // robust essential matrix estimation in image collection.
    std::vector<openMVG::SfMIO::IntrinsicCameraInfo>::iterator iterF =
    std::unique(vec_focalGroup.begin(), vec_focalGroup.end(), testIntrinsicsEquality);
    vec_focalGroup.resize( std::distance(vec_focalGroup.begin(), iterF) );
    if (vec_focalGroup.size() == 1) {
      // Set all the intrinsic ID to 0
      for (size_t i = 0; i < vec_camImageName.size(); ++i)
        vec_camImageName[i].m_intrinsicId = 0;
    }
    else  {
//      std::cerr << "There is more than one focal group in the lists.txt file." << std::endl
//        << "Only one focal group is supported for the image collection robust essential matrix estimation." << std::endl;
      return false;
    }
  }

  //-- Two alias to ease access to image filenames and image sizes
  std::vector<std::string> vec_fileNames;
  std::vector<std::pair<size_t, size_t> > vec_imagesSize;
  for ( std::vector<openMVG::SfMIO::CameraInfo>::const_iterator
    iter_camInfo = vec_camImageName.begin();
    iter_camInfo != vec_camImageName.end();
    iter_camInfo++ )
  {
    vec_imagesSize.push_back( std::make_pair( vec_focalGroup[iter_camInfo->m_intrinsicId].m_w,
                                              vec_focalGroup[iter_camInfo->m_intrinsicId].m_h ) );
    vec_fileNames.push_back( stlplus::create_filespec( sImaDirectory, iter_camInfo->m_sImageName) );
  }

  //---------------------------------------
  // b. Compute features and descriptor
  //    - extract sift features and descriptor
  //    - if keypoints already computed, re-load them
  //    - else save features and descriptors on disk
  //---------------------------------------

  typedef SIOPointFeature FeatureT;
  typedef std::vector<FeatureT> FeatsT;

  PairWiseMatches map_PutativesMatches;
  float fDistRatio = .6f;	// Nearest Neighbor distance ratio, default value is set to 0.6
							// 0.6 is restrictive, -r 0.8 is less restrictive
							// contrastThreshold = 0.01, fDistRatio = 0.8 helps having more matches


  extractFeaturesAndDescriptors_r3df_nlopt(
    vec_fileNames, // input filenames
    sOutDir,  // Output directory where features and descriptor will be saved
	factor
    );

  OpenMVGHelper::exportKeypoints(vec_camImageName, vec_focalGroup, sImaDirectory, sOutDir);
  typedef flann::L2<Regard3DFeatures::DescriptorR3D::bin_type> MetricR3D;
  typedef ArrayMatcher_Kdtree_Flann<Regard3DFeatures::DescriptorR3D::bin_type, MetricR3D> MatcherR3D;
  // Brute force matcher can be defined as following:
  //typedef L2_Vectorized<DescriptorT::bin_type> MetricT;
  //typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;
  computePutativeDescriptorMatches<Regard3DFeatures::DescriptorR3D, Regard3DFeatures::KeypointSetR3D, MatcherR3D>
  	(vec_fileNames, sOutDir, map_PutativesMatches, fDistRatio);

  //---------------------------------------
  // d. Geometric filtering of putative matches
  //    - AContrario Estimation of the desired geometric model
  //    - Use an upper bound for the a contrario estimated threshold
  //---------------------------------------
  PairWiseMatches map_GeometricMatches;

  const double maxResidualError = 4;	// Orig: 4 (higher is more relaxed, e.g. 5.5)
#if defined(R3D_USE_OPENMVG_PRE08)
  ImageCollectionGeometricFilter<FeatureT> collectionGeomFilter;
  if(collectionGeomFilter.loadData(vec_fileNames, sOutDir))
#else
  // Load the features
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if(!feats_provider->load(sfm_data, sMatchesDirectory, regions_type)) {
	  std::cerr << std::endl << "Invalid features." << std::endl;
	  return false;	// EXIT_FAILURE;
  }
  ImageCollectionGeometricFilter collectionGeomFilter;
#endif
  {
    MLOG << std::endl << " - GEOMETRIC FILTERING - " << std::endl;
    switch (eGeometricModelToCompute)
    {
      case FUNDAMENTAL_MATRIX:
      {
       collectionGeomFilter.Filter(
          GeometricFilter_FMatrix_AC(maxResidualError),
          map_PutativesMatches,
          map_GeometricMatches,
          vec_imagesSize);
      }
      break;
      case ESSENTIAL_MATRIX:
      {
        collectionGeomFilter.Filter(
          GeometricFilter_EMatrix_AC(vec_focalGroup[0].m_K, maxResidualError),
          map_PutativesMatches,
          map_GeometricMatches,
          vec_imagesSize);

        //-- Perform an additional check to remove pairs with poor overlap
        std::vector<PairWiseMatches::key_type> vec_toRemove;
        for (PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
          iterMap != map_GeometricMatches.end(); ++iterMap)
        {
          size_t putativePhotometricCount = map_PutativesMatches.find(iterMap->first)->second.size();
          size_t putativeGeometricCount = iterMap->second.size();
          float ratio = putativeGeometricCount / (float)putativePhotometricCount;
          if (putativeGeometricCount < 50 || ratio < .3f)  {
            // the pair will be removed
            vec_toRemove.push_back(iterMap->first);
          }
        }
        //-- remove discarded pairs
        for (std::vector<PairWiseMatches::key_type>::const_iterator
          iter =  vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
        {
          map_GeometricMatches.erase(*iter);
        }
      }
      break;
      case HOMOGRAPHY_MATRIX:
      {

        collectionGeomFilter.Filter(
          GeometricFilter_HMatrix_AC(maxResidualError),
          map_PutativesMatches,
          map_GeometricMatches,
          vec_imagesSize);
      }
      break;
    }

    //---------------------------------------
    //-- Export geometric filtered matches
    //---------------------------------------
    std::ofstream file (string(sOutDir + "/" + sGeometricMatchesFilename).c_str());
    if (file.is_open())
      PairedIndMatchToStream(map_GeometricMatches, file);
    file.close();

    //-- export Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
      map_GeometricMatches,
      stlplus::create_filespec(sOutDir, "GeometricAdjacencyMatrix", "svg"));

	OpenMVGHelper::exportMatches(vec_camImageName, vec_focalGroup,
		sImaDirectory, sOutDir, sOutDir, string(sOutDir + "/" + sGeometricMatchesFilename));//"matches.putative.txt"));//

	// Determine score
	int sumOfMatches = 0;
    for (PairWiseMatches::const_iterator iter = map_GeometricMatches.begin();
      iter != map_GeometricMatches.end();
      ++iter)
    {
      const size_t I = iter->first.first;
      const size_t J = iter->first.second;

      const vector<IndMatch> & vec_FilteredMatches = iter->second;
	  sumOfMatches += static_cast<int>(vec_FilteredMatches.size());
	}

	return sumOfMatches;
  }
  return 0;
}


double R3DComputeMatches::myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
	int res = 0;
	if(my_func_data != NULL)
	{
		R3DComputeMatches *pCM = reinterpret_cast<R3DComputeMatches*>(my_func_data);
		res = pCM->computeMatchesOpenCV_NLOPT_step(x[0]);
	}

	char buf[1024];
#if defined(_MSC_VER)
	sprintf_s(buf, 1024, "factor: %g, result: %d\n", x[0], res);
#else
	sprintf(buf, "factor: %g, result: %d\n", x[0], res);
#endif
#if defined(R3D_WIN32)
	OutputDebugStringA(buf);
#endif
	return static_cast<double>(res);
}

bool R3DComputeMatches::computeMatchesOpenCV_NLOPT()
{
/*	for(int i = 2; i < 30; i++)
	{
		double factor = 4.0 / static_cast<double>(i);
		int res = computeMatchesOpenCV_NLOPT_step(factor);

		char buf[1024];
		sprintf_s(buf, 1024, "factor: %g i: %d, result: %d\n", factor, i, res);
		OutputDebugStringA(buf);
	}
*/
/*	{
		char buf[1024];
		double factor = M_PI/180.0;
		int res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
		factor = -M_PI/180.0;
		res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
		factor = 180.0/M_PI;
		res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
		factor = -180.0/M_PI;
		res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
		factor = 1.0;
		res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
		factor = -1.0;
		res = computeMatchesOpenCV_NLOPT_step(factor);
		sprintf_s(buf, 1024, "factor: %g result: %d\n", factor, res);
		OutputDebugStringA(buf);
	}*/
#if defined(R3D_HAVE_NLOPT)
	int count = 0;
	nlopt::opt opt(nlopt::GN_DIRECT_L, 1);			//LN_COBYLA, 1);
	std::vector<double> lb(1), ub(1);
	//lb[0] = 0.000000001; ub[0] = 0.5;
	lb[0] = 1.0; ub[0] = 20.0;
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_xtol_rel(1e-4);

	opt.set_max_objective(myfunc, this);

	std::vector<double> x(1);
	x[0] = 8.0;

	double maxf;
	nlopt::result result = opt.optimize(x, maxf);

	char buf[1024];
#	if defined(_MSC_VER)
	sprintf_s(buf, 1024, "Final result: %g\n", x[0]);
#	else
	sprintf(buf, "Final result: %g\n", x[0]);
#	endif

#	if defined(R3D_WIN32)
	OutputDebugStringA(buf);
#	endif

#endif
	return true;
}
#endif

void R3DComputeMatches::updateProgress(float progress, const wxString &msg)
{
	if(pMainFrame_ != NULL)
	{
		pMainFrame_->sendUpdateProgressBarEvent(progress, msg);
	}
}
