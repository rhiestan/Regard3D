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
#include "OpenMVGHelper.h"
#include "OpenCVHelper.h"
#include "OpenMVGExportToMVS.h"

#include <memory>
#include <sstream>
#include <cmath>
#include <iterator>
#include <limits>
#include <float.h>
#include <set>
#include <cmath>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

using namespace std;

#include <wx/ffile.h>

// OpenMVG
#include "openMVG/multiview/projection.hpp"
#include "openMVG/image/image_container.hpp"
#include "openMVG/image/pixel_types.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/feature.hpp"
#include "third_party/vectorGraphics/svgDrawer.hpp"
using namespace svg;
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#if defined(R3D_USE_OPENMVG_PRE08)
#include "openMVG/matching/indexed_sort.hpp"
#else
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/stl/indexed_sort.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#endif
#include "third_party/progress/progress.hpp"

// stlpus
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::image;

/**
 * Export keypoints to SVG.
 *
 * Functionality from main_ExportKeypoints.cpp
 */
void OpenMVGHelper::exportKeypoints(const std::vector<openMVG::SfMIO::CameraInfo> &vec_camImageName,
	const std::vector<openMVG::SfMIO::IntrinsicCameraInfo> &vec_focalGroup,
	const std::string &sImaDirectory,
	const std::string &sOutDir)
{
	for (std::vector<SfMIO::CameraInfo>::const_iterator iterFilename = vec_camImageName.begin();
		iterFilename != vec_camImageName.end(); ++iterFilename)
	{
		const size_t I = std::distance(
			(std::vector<SfMIO::CameraInfo>::const_iterator)vec_camImageName.begin(),
			iterFilename);

		const std::pair<size_t, size_t>
			dimImage = std::make_pair(vec_focalGroup[iterFilename->m_intrinsicId].m_w,
			vec_focalGroup[iterFilename->m_intrinsicId].m_h);

		svgDrawer svgStream( dimImage.first, dimImage.second);
		svgStream.drawImage(stlplus::create_filespec(sImaDirectory, iterFilename->m_sImageName),
			dimImage.first,
			dimImage.second);

		// Load the features from the feature file
		std::vector<openMVG::features::SIOPointFeature> vec_feat;
		loadFeatsFromFile(
			stlplus::create_filespec(sOutDir, stlplus::basename_part(iterFilename->m_sImageName), ".feat"),
			vec_feat);

		//-- Draw features
		for (size_t i=0; i< vec_feat.size(); ++i)  {
			const openMVG::features::SIOPointFeature & feature = vec_feat[i];
			svgStream.drawCircle(feature.x(), feature.y(), feature.scale(),
				svgStyle().stroke("yellow", 2.0));
		}

		// Write the SVG file
		std::ostringstream os;
		os << stlplus::folder_append_separator(sOutDir)
			<< stlplus::basename_part(iterFilename->m_sImageName)
			<< "_" << vec_feat.size() << "_.svg";
		ofstream svgFile( os.str().c_str() );
		svgFile << svgStream.closeSvgFile().str();
		svgFile.close();
	}
}

// Convert HUE color to RGB
inline float hue2rgb(float p, float q, float t) {
	if(t < 0) t += 1;
	if(t > 1) t -= 1;
	if(t < 1.f / 6.f) return p + (q - p) * 6.f * t;
	if(t < 1.f / 2.f) return q;
	if(t < 2.f / 3.f) return p + (q - p) * (2.f / 3.f - t) * 6.f;
	return p;
}

//
// Converts an HSL color value to RGB. Conversion formula
// adapted from http://en.wikipedia.org/wiki/HSL_color_space.
// Assumes h, s, and l are contained in the set [0, 1] and
// returns r, g, and b in the set [0, 255].
void hslToRgb(
	float h, float s, float l,
	unsigned char & r, unsigned char & g, unsigned char & b)
{
	if(s == 0) {
		r = g = b = static_cast<unsigned char>(l * 255.f); // achromatic
	}
	else {
		const float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
		const float p = 2.f * l - q;
		r = static_cast<unsigned char>(hue2rgb(p, q, h + 1.f / 3.f) * 255.f);
		g = static_cast<unsigned char>(hue2rgb(p, q, h) * 255.f);
		b = static_cast<unsigned char>(hue2rgb(p, q, h - 1.f / 3.f) * 255.f);
	}
}

/**
 * Export matches to SVG.
 *
 * Functionality from main_exportMatches.cpp
 */
void OpenMVGHelper::exportMatches(const std::vector<openMVG::SfMIO::CameraInfo> &vec_camImageName,
	const std::vector<openMVG::SfMIO::IntrinsicCameraInfo> &vec_focalGroup,
	const std::string &sImaDirectory,
	const std::string &sMatchesDir,
	const std::string &sOutDir,
	const std::string &sMatchFile)
{
#if 0
	//---------------------------------------
	// Load SfM Scene regions
	//---------------------------------------
	// Init the regions_type from the image describer file (used for image regions extraction)
	const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
	std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
	if(!regions_type)
	{
		std::cerr << "Invalid: "
			<< sImage_describer << " regions type file." << std::endl;
		return;
	}

	// Read the features
	std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
	if(!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
		std::cerr << std::endl
			<< "Invalid features." << std::endl;
		return;
	}
	std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
	if(!matches_provider->load(sfm_data, sMatchFile)) {
		std::cerr << "\nInvalid matches file." << std::endl;
		return;
	}

	// ------------
	// For each pair, export the matches
	// ------------

	stlplus::folder_create(sOutDir);
	std::cout << "\n Export pairwise matches" << std::endl;
	const Pair_Set pairs = matches_provider->getPairs();
	C_Progress_display my_progress_bar(pairs.size());
	for(Pair_Set::const_iterator iter = pairs.begin();
		iter != pairs.end();
		++iter, ++my_progress_bar)
	{
		const size_t I = iter->first;
		const size_t J = iter->second;

		const View * view_I = sfm_data.GetViews().at(I).get();
		const std::string sView_I = stlplus::create_filespec(sfm_data.s_root_path,
			view_I->s_Img_path);
		const View * view_J = sfm_data.GetViews().at(J).get();
		const std::string sView_J = stlplus::create_filespec(sfm_data.s_root_path,
			view_J->s_Img_path);

		const std::pair<size_t, size_t>
			dimImage_I = std::make_pair(view_I->ui_width, view_I->ui_height),
			dimImage_J = std::make_pair(view_J->ui_width, view_J->ui_height);

		svgDrawer svgStream(dimImage_I.first + dimImage_J.first, max(dimImage_I.second, dimImage_J.second));
		svgStream.drawImage(sView_I,
			dimImage_I.first,
			dimImage_I.second);
		svgStream.drawImage(sView_J,
			dimImage_J.first,
			dimImage_J.second, dimImage_I.first);

		const vector<IndMatch> & vec_FilteredMatches = matches_provider->pairWise_matches_.at(*iter);

		if(!vec_FilteredMatches.empty()) {

			const PointFeatures & vec_feat_I = feats_provider->getFeatures(view_I->id_view);
			const PointFeatures & vec_feat_J = feats_provider->getFeatures(view_J->id_view);

			//-- Draw link between features :
			for(size_t i = 0; i< vec_FilteredMatches.size(); ++i) {
				const PointFeature & imaA = vec_feat_I[vec_FilteredMatches[i].i_];
				const PointFeature & imaB = vec_feat_J[vec_FilteredMatches[i].j_];
				// Compute a flashy colour for the correspondence
				unsigned char r, g, b;
				hslToRgb((rand() % 360) / 360., 1.0, .5, r, g, b);
				std::ostringstream osCol;
				osCol << "rgb(" << (int)r << ',' << (int)g << ',' << (int)b << ")";
				svgStream.drawLine(imaA.x(), imaA.y(),
					imaB.x() + dimImage_I.first, imaB.y(), svgStyle().stroke(osCol.str(), 2.0));
			}

			//-- Draw features (in two loop, in order to have the features upper the link, svg layer order):
			for(size_t i = 0; i< vec_FilteredMatches.size(); ++i) {
				const PointFeature & imaA = vec_feat_I[vec_FilteredMatches[i].i_];
				const PointFeature & imaB = vec_feat_J[vec_FilteredMatches[i].j_];
				svgStream.drawCircle(imaA.x(), imaA.y(), 3.0,
					svgStyle().stroke("yellow", 2.0));
				svgStream.drawCircle(imaB.x() + dimImage_I.first, imaB.y(), 3.0,
					svgStyle().stroke("yellow", 2.0));
			}
		}
		std::ostringstream os;
		os << stlplus::folder_append_separator(sOutDir)
			<< iter->first << "_" << iter->second
			<< "_" << vec_FilteredMatches.size() << "_.svg";
		ofstream svgFile(os.str().c_str());
		svgFile << svgStream.closeSvgFile().str();
		svgFile.close();
  }
#endif
}

/**
 * Determine the best F matrix validated image pairs.
 *
 * Functionality copied from IncrementalReconstructionEngine::InitialPairChoice and
 * IncrementalReconstructionEngine::ReadInputData.
 */
OpenMVGHelper::ImagePairList OpenMVGHelper::getBestValidatedPairs(const R3DProjectPaths &paths, const std::string &matchesfilename, int pairCount)
{
  OpenMVGHelper::ImagePairList imgPairList;
#if defined(R3D_USE_OPENMVG_PRE08)
  std::vector<openMVG::SfMIO::CameraInfo> _vec_camImageNames;
  std::vector<openMVG::SfMIO::IntrinsicCameraInfo> _vec_intrinsicGroups;
  openMVG::matching::PairWiseMatches _map_Matches_F; // pairwise matches for Fundamental model
  std::string sMatchesDir(paths.relativeMatchesPath_);		//pProject->getRelativeMatchesPath());

  std::string sListsFile = paths.listsTxtFilename_;		//stlplus::create_filespec(sMatchesDir,"lists","txt");
  std::string sComputedMatchesFile_F = matchesfilename;	//paths.matchesFFilename_;		//stlplus::create_filespec(sMatchesDir,"matches.f","txt");
  if (!stlplus::is_file(sListsFile)||
    !stlplus::is_file(sComputedMatchesFile_F) )
  {
//    std::cerr << std::endl
//      << "One of the input required file is not a present (lists.txt, matches.f.txt)" << std::endl;
//    return false;
	  return imgPairList;
  }

  // a. Read images names
  if (!openMVG::SfMIO::loadImageList( _vec_camImageNames,
                                      _vec_intrinsicGroups,
                                      sListsFile) )
  {
    //std::cerr << "\nEmpty image list." << std::endl;
    //return false;
    return imgPairList;
  }

  // b. Read matches (Fundamental)
  if (!matching::PairedIndMatchImport(sComputedMatchesFile_F, _map_Matches_F)) {
//    std::cerr<< "Unable to read the Fundamental matrix matches" << std::endl;
//    return false;
	  return imgPairList;
  }

    // Display to the user the 10 (pairCount) top Fundamental matches pair
    std::vector< size_t > vec_NbMatchesPerPair;
    for (openMVG::matching::PairWiseMatches::const_iterator
      iter = _map_Matches_F.begin();
      iter != _map_Matches_F.end(); ++iter)
    {
      vec_NbMatchesPerPair.push_back(iter->second.size());
    }

    if(pairCount <= 0)
      pairCount = _map_Matches_F.size();

    // sort in descending order
    using namespace indexed_sort;
    std::vector< sort_index_packet_descend< size_t, size_t> > packet_vec(vec_NbMatchesPerPair.size());
    sort_index_helper(packet_vec, &vec_NbMatchesPerPair[0], std::min((size_t)pairCount, _map_Matches_F.size()));

    for (size_t i = 0; i < std::min((size_t)pairCount, _map_Matches_F.size()); ++i) {
      size_t index = packet_vec[i].index;
      openMVG::matching::PairWiseMatches::const_iterator iter = _map_Matches_F.begin();
      std::advance(iter, index);
//      std::cout << "(" << iter->first.first << "," << iter->first.second <<")\t\t"
//        << iter->second.size() << " matches" << std::endl;

      if(iter->second.size() > 0)
	  {
        ImagePair imgPair;
        imgPair.indexA_ = iter->first.first;
        imgPair.indexB_ = iter->first.second;
        imgPair.matches_ = iter->second.size();
        imgPair.imageFilenameA_ = _vec_camImageNames[iter->first.first].m_sImageName;
        imgPair.imageFilenameB_ = _vec_camImageNames[iter->first.second].m_sImageName;
        imgPairList.push_back(imgPair);
      }
    }

    return imgPairList;
#else
	std::string sSfM_Data_Filename(paths.matchesSfmDataFilename_);

	// Load input SfM_Data scene
	openMVG::sfm::SfM_Data _sfm_data;
	if(!Load(_sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS)))
		return imgPairList;
	std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
	if(!matches_provider->load(_sfm_data, matchesfilename))
		return imgPairList;

	// Functionality copied from SequentialSfMReconstructionEngine::ChooseInitialPair

    // List Views that support valid intrinsic
    std::set<IndexT> valid_views;
    for (Views::const_iterator it = _sfm_data.GetViews().begin();
      it != _sfm_data.GetViews().end(); ++it)
    {
      const View * v = it->second.get();
      if( _sfm_data.GetIntrinsics().find(v->id_intrinsic) != _sfm_data.GetIntrinsics().end())
        valid_views.insert(v->id_view);
    }

	// Try to list the 10 (pairCount) top pairs that have:
	//  - valid intrinsics,
	//  - valid estimated Fundamental matrix.
    std::vector< size_t > vec_NbMatchesPerPair;
    std::vector<openMVG::matching::PairWiseMatches::const_iterator> vec_MatchesIterator;
    const openMVG::matching::PairWiseMatches & map_Matches = matches_provider->pairWise_matches_;
    for (openMVG::matching::PairWiseMatches::const_iterator
      iter = map_Matches.begin();
      iter != map_Matches.end(); ++iter)
    {
      const Pair current_pair = iter->first;
      if (valid_views.count(current_pair.first) &&
        valid_views.count(current_pair.second) )
      {
        vec_NbMatchesPerPair.push_back(iter->second.size());
        vec_MatchesIterator.push_back(iter);
      }
    }

	if(vec_NbMatchesPerPair.empty())
		return imgPairList;

	if(pairCount <= 0)
      pairCount = vec_NbMatchesPerPair.size();

	// sort the Pairs in descending order according their correspondences count
    using namespace stl::indexed_sort;
    std::vector< sort_index_packet_descend< size_t, size_t> > packet_vec(vec_NbMatchesPerPair.size());
    sort_index_helper(packet_vec, &vec_NbMatchesPerPair[0], std::min((size_t)pairCount, vec_NbMatchesPerPair.size()));

    for (size_t i = 0; i < std::min((size_t)pairCount, vec_NbMatchesPerPair.size()); ++i) {
      const size_t index = packet_vec[i].index;
      openMVG::matching::PairWiseMatches::const_iterator iter = vec_MatchesIterator[index];
		//std::cout << "(" << iter->first.first << "," << iter->first.second << ")\t\t"
		//	<< iter->second.size() << " matches" << std::endl;
		if(iter->second.size() > 0)
		{
			ImagePair imgPair;
			imgPair.indexA_ = iter->first.first;
			imgPair.indexB_ = iter->first.second;
			imgPair.matches_ = iter->second.size();
			imgPair.imageFilenameA_ = _sfm_data.GetViews().at(imgPair.indexA_)->s_Img_path;	// _vec_camImageNames[iter->first.first].m_sImageName;
			imgPair.imageFilenameB_ = _sfm_data.GetViews().at(imgPair.indexB_)->s_Img_path;
			imgPairList.push_back(imgPair);
		}
	}

	return imgPairList;
#endif
}

// Equality functor to count the number of similar K matrices in the essential matrix case.
static inline bool testIntrinsicsEquality(
  SfMIO::IntrinsicCameraInfo const &ci1,
  SfMIO::IntrinsicCameraInfo const &ci2)
{
  return ci1.m_K == ci2.m_K;
}

bool OpenMVGHelper::isGlobalSfmAvailable(const R3DProjectPaths &paths, R3DProject::PictureSet *pPictureSet)
{
#if defined(R3D_USE_OPENMVG_PRE08)
  //std::string sMatchesDir("F:/Projects/libs/openMVG/run/matches");
  std::string sListsFile = paths.listsTxtFilename_;		//stlplus::create_filespec(sMatchesDir,"lists","txt");
  std::vector<openMVG::SfMIO::CameraInfo> vec_camImageName;
  std::vector<openMVG::SfMIO::IntrinsicCameraInfo> vec_focalGroup;
  if (!openMVG::SfMIO::loadImageList( vec_camImageName,
                                      vec_focalGroup,
                                      sListsFile) )
  {
    return false;
  }
  std::vector<openMVG::SfMIO::IntrinsicCameraInfo>::iterator iterF =
    std::unique(vec_focalGroup.begin(), vec_focalGroup.end(), testIntrinsicsEquality);
  vec_focalGroup.resize( std::distance(vec_focalGroup.begin(), iterF) );
  if (vec_focalGroup.size() == 1) {
    return true;
  }

  return false;
#else
  // Check for essential matches file
  wxFileName matchesEFN = wxFileName(wxString(paths.matchesEFilename_.c_str(), wxConvLibc));
  if(!matchesEFN.FileExists())
    return false;

  // Check for at least 3 images with known focal length
  int count = 0;
  const ImageInfoVector &iiv = pPictureSet->getImageInfoVector();
  for(size_t i = 0; i < iiv.size(); i++)
  {
    const ImageInfo &ii = iiv[i];
	if(ii.focalLength_ > 0 && ii.sensorWidth_ > 0)
      count++;
  }
  return (count > 2);
#endif
}

bool OpenMVGHelper::hasMatchesSfM_DataFile(const R3DProjectPaths &paths)
{
	wxFileName sfmDataFN(wxString(paths.matchesSfmDataFilename_.c_str(), wxConvLibc));
	return sfmDataFN.FileExists();
}

bool OpenMVGHelper::hasTriSfM_DataFile(const R3DProjectPaths &paths)
{
	wxFileName sfmDataFN(wxString(paths.relativeTriSfmDataFilename_.c_str(), wxConvLibc));
	return sfmDataFN.FileExists();
}

// The following code is from main_openMVG2PMVS.cpp, slightly adapted

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

bool OpenMVGHelper::exportToPMVSFormat(
  const Document & doc,
  const std::string & sOutDirectory,  //Output PMVS files directory
  const std::string & sImagePath,  // The images path
  R3DProject::Densification *pDensification
  )
{
  bool bOk = true;
  if (!stlplus::is_folder(sOutDirectory))
  {
    stlplus::folder_create(sOutDirectory);
    bOk = stlplus::is_folder(sOutDirectory);
  }

  // Create basis directory structure
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "models");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "txt");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "visualize");

  if (bOk &&
      stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "models") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "txt") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "visualize")
      )
  {
    bOk = true;
  }
  else  {
    std::cerr << "Cannot access to one of the desired output directory" << std::endl;
  }

  if (bOk)
  {
    C_Progress_display my_progress_bar( doc._map_camera.size()*2 );
    // Export data :
    //Camera

    size_t count = 0;
    for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
      iter != doc._map_camera.end(); ++iter, ++count, ++my_progress_bar)
    {
      const Mat34 & PMat = iter->second._P;
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << count;
      std::ofstream file(
        stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "txt",
        os.str() ,"txt").c_str());
      file << "CONTOUR" << os.widen('\n')
        << PMat.row(0) <<"\n"<< PMat.row(1) <<"\n"<< PMat.row(2) << os.widen('\n');
      file.close();
    }

    // Image
    count = 0;
    openMVG::image::Image<openMVG::image::RGBColor> image;
    for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
      iter != doc._map_camera.end();  ++iter, ++count, ++my_progress_bar)
    {
      const size_t imageIndex = iter->first;
      const std::string & sImageName = doc._vec_imageNames[imageIndex];
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << count;
      std::string srcImage = stlplus::create_filespec( sImagePath, sImageName);
      std::string dstImage = stlplus::create_filespec(
        stlplus::folder_append_separator(sOutDirectory) + "visualize", os.str(),"jpg");
      if (stlplus::extension_part(srcImage) == "JPG" ||
          stlplus::extension_part(srcImage) == "jpg")
      {
        stlplus::file_copy(srcImage, dstImage);
      }
      else
      {
        ReadImage( srcImage.c_str(), &image );
        WriteImage( dstImage.c_str(), image);
      }
    }

    //pmvs_options.txt
    std::ostringstream os;
	os << "level " << pDensification->pmvsLevel_ << os.widen('\n')
     << "csize " << pDensification->pmvsCSize_ << os.widen('\n')
     << "threshold " << pDensification->pmvsThreshold_ << os.widen('\n')
     << "wsize " << pDensification->pmvsWSize_ << os.widen('\n')
     << "minImageNum " << pDensification->pmvsMinImageNum_ << os.widen('\n')
	 << "CPU " << pDensification->pmvsNumThreads_ << os.widen('\n')
     << "setEdge 0" << os.widen('\n')
     << "useBound 0" << os.widen('\n')
     << "useVisData 0" << os.widen('\n')
     << "sequence -1" << os.widen('\n')
     << "maxAngle 10" << os.widen('\n')
     << "quad 2.0" << os.widen('\n')
     << "timages -1 0 " << doc._map_camera.size() << os.widen('\n')
     << "oimages 0" << os.widen('\n'); // ?

    std::ofstream file(stlplus::create_filespec(sOutDirectory, "pmvs_options", "txt").c_str());
    file << os.str();
    file.close();
  }
  return bOk;
}


bool OpenMVGHelper::exportToBundlerFormat(
  const Document & doc,
  const std::string & sOutFile, //Output Bundle.rd.out file
  const std::string & sOutListFile)  //Output Bundler list.txt file
{
  std::ofstream os(sOutFile.c_str()	);
  std::ofstream osList(sOutListFile.c_str()	);
  if (! os.is_open() || ! osList.is_open())
  {
    return false;
  }
  else
  {
    os << "# Bundle file v0.3" << os.widen('\n')
      << doc._map_camera.size()
      << " " << doc._tracks.size() << os.widen('\n');

    for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
      iter != doc._map_camera.end(); ++iter)
    {
      const PinholeCamera & PMat = iter->second;

      Mat3 D;
      D.fill(0.0);
      D .diagonal() = Vec3(1., -1., -1.); // mapping between our pinhole and Bundler convention
      Mat3 R = D * PMat._R;
      Vec3 t = D * PMat._t;
      double focal = PMat._K(0,0);
      double k1 = 0.0, k2 = 0.0; // distortion already removed

      os << focal << " " << k1 << " " << k2 << os.widen('\n') //f k1 k2
        << R(0,0) << " " << R(0, 1) << " " << R(0, 2) << os.widen('\n')   //R[0]
        << R(1,0) << " " << R(1, 1) << " " << R(1, 2) << os.widen('\n')   //R[1]
        << R(2,0) << " " << R(2, 1) << " " << R(2, 2) << os.widen('\n')  //R[2]
        << t(0)  << " " << t(1)   << " " << t(2)   << os.widen('\n');     //t

      osList << doc._vec_imageNames[iter->first] << " 0 " << focal << os.widen('\n');
    }

    size_t trackIndex = 0;

    for (std::map< uint32_t, tracks::submapTrack >::const_iterator
      iterTracks = doc._tracks.begin();
      iterTracks != doc._tracks.end();
      ++iterTracks,++trackIndex)
    {
      const tracks::submapTrack & map_track = iterTracks->second;

      const float * ptr3D = & doc._vec_points[trackIndex*3];
      os << ptr3D[0] << " " << ptr3D[1] << " " << ptr3D[2] << os.widen('\n');
      os <<  "255 255 255" << os.widen('\n');
      os << map_track.size() << " ";
      const Vec3 vec(ptr3D[0],ptr3D[1],ptr3D[2]);
      for (tracks::submapTrack::const_iterator iterTrack = map_track.begin();
        iterTrack != map_track.end();
        ++iterTrack)
      {
        const PinholeCamera & PMat = doc._map_camera.find(iterTrack->first)->second;
        Vec2 pt = PMat.Project(vec);
        os << iterTrack->first << " " << iterTrack->second << " " << pt(0) << " " << pt(1) << " ";
      }
      os << os.widen('\n');

    }
    os.close();
    osList.close();
  }
  return true;
}

#if !defined(R3D_USE_OPENMVG_PRE08)
// Methods copied from main_openMVG2PMVS.cpp, adjusted
bool OpenMVGHelper::exportToPMVSFormat(
  const openMVG::sfm::SfM_Data & sfm_data,
  const std::string & sOutDirectory,  //Output PMVS files directory
  R3DProject::Densification *pDensification)
{
  bool bOk = true;
  if (!stlplus::is_folder(sOutDirectory))
  {
    stlplus::folder_create(sOutDirectory);
    bOk = stlplus::is_folder(sOutDirectory);
  }

  // Create basis directory structure
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "models");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "txt");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "visualize");

  if (bOk &&
      stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "models") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "txt") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "visualize")
      )
  {
    bOk = true;
  }
  else  {
    std::cerr << "Cannot access to one of the desired output directory" << std::endl;
  }

  if (bOk)
  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size()*2 );

    // Export valid views as Projective Cameras:
    size_t count = 0;
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

      if (iterPose == sfm_data.GetPoses().end() ||
        iterIntrinsic == sfm_data.GetIntrinsics().end())
      continue;

      // We have a valid view with a corresponding camera & pose
      const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(iterPose->second);
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << count;
      std::ofstream file(
        stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "txt",
        os.str() ,"txt").c_str());
      file << "CONTOUR" << os.widen('\n')
        << P.row(0) <<"\n"<< P.row(1) <<"\n"<< P.row(2) << os.widen('\n');
      file.close();
      ++count;
    }

    // Export (calibrated) views as undistorted images
    count = 0;
    openMVG::image::Image<openMVG::image::RGBColor> image, image_ud;
    Hash_Map<IndexT, IndexT> map_viewIdToContiguous;
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

      if (iterPose == sfm_data.GetPoses().end() ||
        iterIntrinsic == sfm_data.GetIntrinsics().end())
      continue;

      map_viewIdToContiguous[view->id_view] = count;
      // We have a valid view with a corresponding camera & pose
      const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << count;
      const std::string dstImage = stlplus::create_filespec(
        stlplus::folder_append_separator(sOutDirectory) + "visualize", os.str(),"jpg");

      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if (cam->have_disto())
      {
        // undistort the image and save it
        ReadImage( srcImage.c_str(), &image);
        UndistortImage(image, cam, image_ud, openMVG::image::BLACK);
        WriteImage(dstImage.c_str(), image_ud);
      }
      else // (no distortion)
      {
        // copy the image if extension match
        if (stlplus::extension_part(srcImage) == "JPG" ||
          stlplus::extension_part(srcImage) == "jpg")
        {
          stlplus::file_copy(srcImage, dstImage);
        }
        else
        {
          ReadImage( srcImage.c_str(), &image);
          WriteImage( dstImage.c_str(), image);
        }
      }
      ++count;
    }

    bool b_VisData = pDensification->useCMVS_;

    //pmvs_options.txt
    std::ostringstream os;
	os << "level " << pDensification->pmvsLevel_ << os.widen('\n')
     << "csize " << pDensification->pmvsCSize_ << os.widen('\n')
     << "threshold " << pDensification->pmvsThreshold_ << os.widen('\n')
     << "wsize " << pDensification->pmvsWSize_ << os.widen('\n')
     << "minImageNum " << pDensification->pmvsMinImageNum_ << os.widen('\n')
	 << "CPU " << pDensification->pmvsNumThreads_ << os.widen('\n')
     << "setEdge 0" << os.widen('\n')
     << "useBound 0" << os.widen('\n')
     << "useVisData " << (int) b_VisData << os.widen('\n')
     << "sequence -1" << os.widen('\n')
     << "maxAngle 10" << os.widen('\n')
     << "quad 2.0" << os.widen('\n')
     << "timages -1 0 " << count << os.widen('\n')
     << "oimages 0" << os.widen('\n');

    if (b_VisData)
    {
      std::map< IndexT, std::set<IndexT> > view_shared;
      // From the structure observations, list the putatives pairs (symmetric)
      for (Landmarks::const_iterator itL = sfm_data.GetLandmarks().begin();
        itL != sfm_data.GetLandmarks().end(); ++itL)
      {
        const Landmark & landmark = itL->second;
        const Observations & obs = landmark.obs;
        for (Observations::const_iterator itOb = obs.begin();
          itOb != obs.end(); ++itOb)
        {
          const IndexT viewId = itOb->first;
          Observations::const_iterator itOb2 = itOb;
          ++itOb2;
          for (itOb2; itOb2 != obs.end(); ++itOb2)
          {
            const IndexT viewId2 = itOb2->first;
            view_shared[map_viewIdToContiguous[viewId]].insert(map_viewIdToContiguous[viewId2]);
            view_shared[map_viewIdToContiguous[viewId2]].insert(map_viewIdToContiguous[viewId]);
          }
        }
      }
      // Export the vis.dat file
      std::ostringstream osVisData;
      osVisData
        << "VISDATA" << os.widen('\n')
        << view_shared.size() << os.widen('\n'); // #images
      // Export view shared visibility
      for (std::map< IndexT, std::set<IndexT> >::const_iterator it = view_shared.begin();
        it != view_shared.end(); ++it)
      {
        const std::set<IndexT> & setView = it->second;
        osVisData << it->first << ' ' << setView.size();
        for (std::set<IndexT>::const_iterator itV = setView.begin();
          itV != setView.end(); ++itV)
        {
          osVisData << ' ' << *itV;
        }
        osVisData << os.widen('\n');
      }
      std::ofstream file(stlplus::create_filespec(sOutDirectory, "vis", "dat").c_str());
      file << osVisData.str();
      file.close();
    }

    std::ofstream file(stlplus::create_filespec(sOutDirectory, "pmvs_options", "txt").c_str());
    file << os.str();
    file.close();
  }
  return bOk;
}

bool OpenMVGHelper::exportToBundlerFormat(
  const openMVG::sfm::SfM_Data & sfm_data,
  const std::string & sOutFile, //Output Bundle.rd.out file
  const std::string & sOutListFile)  //Output Bundler list.txt file
{
  std::ofstream os(sOutFile.c_str()	);
  std::ofstream osList(sOutListFile.c_str()	);
  if (! os.is_open() || ! osList.is_open())
  {
    return false;
  }
  else
  {
    // Count the number of valid cameras
    IndexT validCameraCount = 0;
	std::map<size_t, size_t> map_cameratoIndex;
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

      if (iterPose == sfm_data.GetPoses().end() ||
        iterIntrinsic == sfm_data.GetIntrinsics().end())
      continue;

	  map_cameratoIndex[view->id_view] = validCameraCount;
      ++validCameraCount;
    }

    // Fill the "Bundle file"
    os << "# Bundle file v0.3" << os.widen('\n')
      << validCameraCount  << " " << sfm_data.GetLandmarks().size() << os.widen('\n');

    // Export camera properties & image filenames
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

      if (iterPose == sfm_data.GetPoses().end() ||
        iterIntrinsic == sfm_data.GetIntrinsics().end())
      continue;

      // Must export focal, k1, k2, R, t

      Mat3 D;
      D.fill(0.0);
      D .diagonal() = Vec3(1., -1., -1.); // mapping between our pinhole and Bundler convention
      double k1 = 0.0, k2 = 0.0; // distortion already removed

      switch(iterIntrinsic->second.get()->getType())
      {
      case PINHOLE_CAMERA:
      case PINHOLE_CAMERA_RADIAL1:
      case PINHOLE_CAMERA_RADIAL3:
      {
        const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsic->second.get());
        const double focal = cam->focal();
        const Mat3 R = D * iterPose->second.rotation();
        const Vec3 t = D * iterPose->second.translation();

        os << focal << " " << k1 << " " << k2 << os.widen('\n') //f k1 k2
          << R(0,0) << " " << R(0, 1) << " " << R(0, 2) << os.widen('\n')  //R.row(0)
          << R(1,0) << " " << R(1, 1) << " " << R(1, 2) << os.widen('\n')  //R.row(1)
          << R(2,0) << " " << R(2, 1) << " " << R(2, 2) << os.widen('\n')  //R.row(2)
          << t(0)   << " " << t(1)    << " " << t(2)    << os.widen('\n'); //t

        osList << stlplus::basename_part(view->s_Img_path) + "." + stlplus::extension_part(view->s_Img_path)
          << " 0 " << focal << os.widen('\n');
      }
      break;
      default:
        std::cerr << "Unsupported camera model for Bundler export." << std::endl;
        return false;
      }
    }
    // Export structure and visibility
    IndexT count = 0;
    for (Landmarks::const_iterator iter = sfm_data.GetLandmarks().begin();
      iter != sfm_data.GetLandmarks().end(); ++iter, ++count)
    {
      const Landmark & landmark = iter->second;
      const Observations & obs = landmark.obs;
      const Vec3 & X = landmark.X;
      // X, color, obsCount
      os << X[0] << " " << X[1] << " " << X[2] << os.widen('\n')
         <<  "255 255 255" << os.widen('\n')
         << obs.size() << " ";
      for(Observations::const_iterator iterObs = obs.begin();
        iterObs != obs.end(); ++iterObs)
      {
        const Observation & ob = iterObs->second;

		size_t cameraIndex = map_cameratoIndex[iterObs->first];

        // ViewId, FeatId, x, y
		os << cameraIndex << " " << ob.id_feat << " " << ob.x(0) << " " << ob.x(1) << " ";
      }
      os << os.widen('\n');
    }
    os.close();
    osList.close();
  }
  return true;
}
#endif

bool OpenMVGHelper::exportToPMVS(R3DProject::Densification *pDensification)
{
  R3DProject *pProject = R3DProject::getInstance();
  R3DProjectPaths paths;
  pProject->getProjectPathsDns(paths, pDensification);

  std::string sSfMDir = paths.relativeSfmOutPath_;
  std::string sOutDir = paths.relativePMVSOutPath_;

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  // Check whether we have SfM_Data
  if(hasTriSfM_DataFile(paths))
  {
    openMVG::sfm::SfM_Data sfm_data;
    if(Load(sfm_data, paths.relativeTriSfmDataFilename_, ESfM_Data(ALL)))
    {
      exportToPMVSFormat(sfm_data,
        sOutDir,
        pDensification);

      exportToBundlerFormat(sfm_data,
        stlplus::folder_append_separator(sOutDir) + "bundle.rd.out",
        stlplus::folder_append_separator(sOutDir) + "list.txt"
      );
      return true;
    }
  }
  else
  {
    Document m_doc;
    if (m_doc.load(sSfMDir))
    {
      exportToPMVSFormat(m_doc,
        sOutDir,
        stlplus::folder_append_separator(sSfMDir) + "images",
        pDensification );

      exportToBundlerFormat(m_doc,
        stlplus::folder_append_separator(sOutDir) + "bundle.rd.out",
        stlplus::folder_append_separator(sOutDir) + "list.txt"
        );

      return true;
    }
  }

  return false;
}

// The following code is from main_openMVG2MESHLAB.cpp, adapted
bool OpenMVGHelper::exportToMeshLab(R3DProject::Densification *pDensification, const wxString &pathname)
{
	// Make sure output directory exists
	wxFileName outDir(pathname, wxT(""));
	if(!outDir.DirExists())
	{
#if wxCHECK_VERSION(2, 9, 0)
		if(!outDir.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
		if(!outDir.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
			return false;
	}

	R3DProject *pProject = R3DProject::getInstance();
	R3DProjectPaths paths;
	pProject->getProjectPathsDns(paths, pDensification);
	bool hasSfmData = OpenMVGHelper::hasMatchesSfM_DataFile(paths);

	// Load OpenMVG document/Sfm_Data file
	std::string sSfMDir = paths.relativeSfmOutPath_;
	Document doc;
	openMVG::sfm::SfM_Data sfm_data;
	size_t numberOfCalibratedViews = 0;
	if(hasSfmData)
	{
		if(!Load(sfm_data, paths.relativeTriSfmDataFilename_, ESfM_Data(VIEWS | INTRINSICS | EXTRINSICS)))
			return false;
		numberOfCalibratedViews = sfm_data.GetViews().size();
	}
	else
	{
		if(!doc.load(sSfMDir))
			return false;
		numberOfCalibratedViews = doc._map_camera.size();
	}

	// Copy model
	wxFileName modelFN(outDir);
	modelFN.AppendDir(wxT("model"));
	if(!modelFN.Mkdir())
		return false;
	modelFN.SetFullName(wxT("dense_model.ply"));
	if(!wxCopyFile(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc), modelFN.GetFullPath(), false))
		return false;

	// Write to MeshLab project file (first in memory)
	std::string sPly = "model/dense_model.ply";
	std::ostringstream outfile;

	//Init mlp file  
	outfile << "<!DOCTYPE MeshLabDocument>" << std::endl 
		<< "<MeshLabProject>" << std::endl 
		<< " <MeshGroup>" << std::endl 
		<< "  <MLMesh label=\"" << sPly << "\" filename=\"" << sPly << "\">" << std::endl 
		<< "   <MLMatrix44>" << std::endl 
		<< "1 0 0 0 " << std::endl 
		<< "0 1 0 0 " << std::endl 
		<< "0 0 1 0 " << std::endl 
		<< "0 0 0 1 " << std::endl 
		<< "</MLMatrix44>" << std::endl 
		<< "  </MLMesh>" << std::endl 
		<< " </MeshGroup>" << std::endl; 

	outfile <<  " <RasterGroup>" << std::endl;

	if(hasSfmData)
	{
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)
		{
			const View * view = iter->second.get();
			Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			if(iterPose == sfm_data.GetPoses().end() ||
				iterIntrinsic == sfm_data.GetIntrinsics().end())
				continue;

			// We have a valid view with a corresponding camera & pose
//			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			const std::string srcImage = stlplus::filename_part(view->s_Img_path);
			const IntrinsicBase * cam = iterIntrinsic->second.get();
			Mat34 P = cam->get_projective_equivalent(iterPose->second);

			for(int i = 1; i < 3; ++i)
				for(int j = 0; j < 4; ++j)
					P(i, j) *= -1.;

			Mat3 R, K;
			Vec3 t;
			KRt_From_P(P, &K, &R, &t);

			const Vec3 optical_center = R.transpose() * t;

			outfile
//				<< "  <MLRaster label=\"" << stlplus::filename_part(view->s_Img_path) << "\">" << std::endl
				<< "  <MLRaster label=\"" << srcImage << "\">" << std::endl
				<< "   <VCGCamera TranslationVector=\""
				<< optical_center[0] << " "
				<< optical_center[1] << " "
				<< optical_center[2] << " "
				<< " 1 \""
				<< " LensDistortion=\"0 0\""
				<< " ViewportPx=\"" << cam->w() << " " << cam->h() << "\""
				<< " PixelSizeMm=\"" << 1 << " " << 1 << "\""
				<< " CenterPx=\"" << cam->w() / 2.0 << " " << cam->h() / 2.0 << "\""
				<< " FocalMm=\"" << (double)K(0, 0) << "\""
				<< " RotationMatrix=\""
				<< R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " 0 "
				<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " 0 "
				<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " 0 "
				<< "0 0 0 1 \"/>" << std::endl;

			// Link the image plane
			outfile << "   <Plane semantic=\"\" fileName=\"images/" << srcImage << "\"/> " << std::endl;
			outfile << "  </MLRaster>" << std::endl;
		}
	}
	else
	{
		std::map<size_t, PinholeCamera >::const_iterator iterCamera = doc._map_camera.begin();
		std::map<size_t, std::pair<size_t, size_t> >::const_iterator iterSize = doc._map_imageSize.begin();
		std::vector<std::string>::const_iterator iterName = doc._vec_imageNames.begin();
		for ( ;
			iterCamera != doc._map_camera.end();
			iterCamera++,
			iterSize++,
			iterName++ )
		{
			PinholeCamera camera = iterCamera->second;
			Mat34 P = camera._P;
			for ( int i = 1; i < 3 ; i++)
				for ( int j = 0; j < 4; j++)
					P(i, j) *= -1;

			Mat3 R, K;
			Vec3 t;
			KRt_From_P( P, &K, &R, &t);

			Vec3 optical_center = R.transpose() * t;

			outfile << "  <MLRaster label=\"" << *iterName << "\">" << std::endl
				<< "   <VCGCamera TranslationVector=\""
				<< optical_center[0] << " " 
				<< optical_center[1] << " " 
				<< optical_center[2] << " " 
				<< " 1 \"" 
				<< " LensDistortion=\"0 0\""
				<< " ViewportPx=\"" << iterSize->second.first << " " << iterSize->second.second << "\"" 
				<< " PixelSizeMm=\"" << 1  << " " << 1 << "\""  
				<< " CenterPx=\"" << iterSize->second.first / 2.0 << " " << iterSize->second.second / 2.0 << "\""
				<< " FocalMm=\"" << (double)K(0, 0 )  << "\"" 
				<< " RotationMatrix=\""
				<< R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " 0 "
				<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " 0 "
				<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " 0 " 
				<< "0 0 0 1 \"/>"  << std::endl;
/*			std::string soffsetImagePath =  stlplus::create_filespec( sSfM_OutputPath, "imagesOffset" );
			if ( stlplus::folder_exists( soffsetImagePath ) )
				outfile << "   <Plane semantic=\"\" fileName=\"" << stlplus::create_filespec( soffsetImagePath, 
					stlplus::basename_part( *iterName ) + "_of", 
					stlplus::extension_part( *iterName ) ) << "\"/> "<< std::endl;
			else
				outfile << "   <Plane semantic=\"\" fileName=\"" << stlplus::create_filespec( stlplus::create_filespec( sSfM_OutputPath, "images" ), 
					*iterName ) << "\"/> "<< std::endl;*/
			outfile << "   <Plane semantic=\"\" fileName=\"images/" << *iterName << "\"/>"<< std::endl;
			outfile << "  </MLRaster>" << std::endl;
		}
	}

	outfile << "   </RasterGroup>" << std::endl
		<< "</MeshLabProject>" << std::endl;

	// Write MeshLab project file
	wxFileName projectFN(outDir);
	projectFN.SetFullName(wxT("sceneMeshlab.mlp"));
	wxFile projectFile(projectFN.GetFullPath(), wxFile::write);
	if(!projectFile.IsOpened())
		return false;

	std::string outfilestr = outfile.str();
	projectFile.Write( outfilestr.c_str(), outfilestr.length() );
	projectFile.Close();

	// Copy undistorted image files
	wxFileName undistImageFN(wxString(paths.relativeSfmOutPath_.c_str(), wxConvLibc), wxT(""));
	undistImageFN.AppendDir(wxT("images"));
	wxFileName destImageFN(outDir);
	destImageFN.AppendDir(wxT("images"));
	if(!destImageFN.Mkdir())
		return false;

	if(hasSfmData)
	{
		openMVG::image::Image<openMVG::image::RGBColor> image, image_ud;
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)
		{
			const View * view = iter->second.get();
			Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			if(iterPose == sfm_data.GetPoses().end() ||
				iterIntrinsic == sfm_data.GetIntrinsics().end())
				continue;

			// We have a valid view with a corresponding camera & pose
			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			destImageFN.SetName(wxString(stlplus::filename_part(view->s_Img_path).c_str(), wxConvLibc));
			destImageFN.SetExt(wxT("jpg"));

			const IntrinsicBase * cam = iterIntrinsic->second.get();
			if(cam->have_disto())
			{
				// undistort the image and save it
				ReadImage(srcImage.c_str(), &image);
				UndistortImage(image, cam, image_ud, openMVG::image::BLACK);
				wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
				if(outImageFile.IsOpened())
				{
					const unsigned char * ptr = (unsigned char*)(image_ud.GetMat().data());
					int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
					std::vector<unsigned char> array(ptr, ptr + image_ud.Width()*image_ud.Height()*depth);
					int w = image_ud.Width(), h = image_ud.Height();
					openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
				}
			}
			else // (no distortion)
			{
				// copy the image if extension match
				if(stlplus::extension_part(srcImage) == "JPG" ||
					stlplus::extension_part(srcImage) == "jpg")
				{
					//stlplus::file_copy(srcImage, dstImage);
					if(!wxCopyFile(wxString(srcImage.c_str(), wxConvLibc), destImageFN.GetFullPath(), false))
						return false;
				}
				else
				{
					ReadImage(srcImage.c_str(), &image);
					//					WriteImage(dstImage.c_str(), image);
					wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
					if(outImageFile.IsOpened())
					{
						const unsigned char * ptr = (unsigned char*)(image.GetMat().data());
						int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
						std::vector<unsigned char> array(ptr, ptr + image.Width()*image.Height()*depth);
						int w = image.Width(), h = image.Height();
						openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
					}
				}
			}
		}
	}
	else
	{
		std::vector<std::string>::const_iterator iterName = doc._vec_imageNames.begin();
		for ( ;
			iterName != doc._vec_imageNames.end();
			iterName++ )
		{
			wxString curName( iterName->c_str(), wxConvLibc );
			undistImageFN.SetFullName(curName);
			destImageFN.SetFullName(curName);

			if(!wxCopyFile(undistImageFN.GetFullPath(), destImageFN.GetFullPath(), false))
				return false;
		}
	}
	return true;
}

// Code copied from main_openMVG2NVM.cpp, adjusted

/**
* @brief Create a N-View file and export images
* @param sfm_data Structure from Motion file
* @param sOutDirectory Output directory
* @param filename Name of the file to create
*/
bool CreateNVMFile( const SfM_Data & sfm_data ,
                    const boost::filesystem::path &sOutDirectory ,
                    const boost::filesystem::path &filename )
{
  boost::system::error_code ec;
  const boost::filesystem::path sOutViewsDirectory = sOutDirectory / "views";
  if ( !boost::filesystem::exists( sOutViewsDirectory, ec ) )
  {
    //std::cout << "\033[1;31mCreating directory:  " << sOutViewsDirectory << "\033[0m\n";

    boost::filesystem::create_directories( sOutViewsDirectory, ec );
  }
  if(ec)
	  return false;

  // Header
  boost::filesystem::ofstream file( filename );

  if ( ! file )
  {
    //std::cerr << "Cannot write file" << filename << std::endl;
    return false;
  }
  file << "NVM_V3" << std::endl;

  // we reindex the poses to ensure a contiguous pose list.
  Hash_Map<IndexT, IndexT> map_viewIdToContiguous;
  int nb_cam = 0;
  for (Views::const_iterator iter = sfm_data.GetViews().begin();
       iter != sfm_data.GetViews().end(); ++iter )
  {
    const View * view = iter->second.get();
    if ( !sfm_data.IsPoseAndIntrinsicDefined( view ) )
    {
      continue;
    }

    nb_cam ++;
    map_viewIdToContiguous.insert( std::make_pair( view->id_view, map_viewIdToContiguous.size() ) );
  }

  // Number of cameras
  // For each camera : File_name Focal Qw Qx Qy Qz Cx Cy Cz D0 0

  file << nb_cam << std::endl;

  // Export undistorted images
  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(), std::cout, "\n- EXPORT UNDISTORTED IMAGES -\n" );
    Image<openMVG::image::RGBColor> image, image_ud;
  #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for schedule(dynamic) private(image, image_ud)
  #endif
    for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
    {
      Views::const_iterator iterViews = sfm_data.views.begin();
      std::advance(iterViews, i);
      const View * view = iterViews->second.get();

      if ( !sfm_data.IsPoseAndIntrinsicDefined( view ) )
      {
        continue;
      }

      std::ostringstream padding;
      padding << std::setw( 4 ) << std::setfill( '0' ) << view->id_view;
      const boost::filesystem::path sAbsoluteOutputDir = "view_" + padding.str();
      const boost::filesystem::path sFullOutputDir =
        sOutViewsDirectory / sAbsoluteOutputDir;

      const boost::filesystem::path dstImage = sFullOutputDir / "undistorted.png";
      const boost::filesystem::path srcImage = boost::filesystem::path(sfm_data.s_root_path) / view->s_Img_path;

      #ifdef OPENMVG_USE_OPENMP
        #pragma omp critical
      #endif
      // Create output dir if not present
      if ( !boost::filesystem::exists( sFullOutputDir ) )
      {
        boost::filesystem::create_directories( sFullOutputDir );
      }

      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find( view->id_intrinsic );
      const IntrinsicBase * cam = iterIntrinsic->second.get();

      // Remove distortion
      if ( cam->have_disto() )
      {
        // Undistort and save the image
        ReadImage( srcImage.string().c_str(), &image );
        UndistortImage( image, cam, image_ud, BLACK );
        WriteImage( dstImage.string().c_str(), image_ud );
      }
      else // (no distortion)
      {
        // If extensions match, copy the PNG image
        if ( boost::filesystem::extension( srcImage ) == "PNG" ||
             boost::filesystem::extension( srcImage ) == "png" )
        {
          boost::filesystem::copy_file( srcImage, dstImage );
        }
        else
        {
          ReadImage( srcImage.string().c_str(), &image );
          WriteImage( dstImage.string().c_str(), image );
        }
      }
      ++my_progress_bar;
    }
  }

  // Export camera parameters
  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(), std::cout, "\n- EXPORT CAMERA PARAMETERS -\n" );
    for (Views::const_iterator iter = sfm_data.GetViews().begin();
         iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();

      if ( !sfm_data.IsPoseAndIntrinsicDefined( view ) )
      {
        continue;
      }

      std::ostringstream padding;
      padding << std::setw( 4 ) << std::setfill( '0' ) << view->id_view;
      const std::string sAbsoluteOutputDir = stlplus::folder_append_separator("views") + "view_" + padding.str();
      const std::string dstImage = stlplus::create_filespec( sAbsoluteOutputDir, "undistorted", "png");

      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find( view->id_intrinsic );
      const IntrinsicBase * cam = iterIntrinsic->second.get();
      const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>( cam );
      const double flen = pinhole_cam->focal();
      const Pose3 pose = sfm_data.GetPoseOrDie( view );
      const Mat3 rotation = pose.rotation();
      const Vec3 center = pose.center();

      const double Cx = center[0];
      const double Cy = center[1];
      const double Cz = center[2];
      Eigen::Quaterniond q( rotation );
      const double Qx = q.x();
      const double Qy = q.y();
      const double Qz = q.z();
      const double Qw = q.w();
      const double d0 = 0.0;

      file << dstImage << " "
         << flen << " "

         << Qw << " "
         << Qx << " "
         << Qy << " "
         << Qz << " "

         << Cx << " "
         << Cy << " "
         << Cz << " "
         << d0 << " "
         << 0 << std::endl;
    }
  }

  // Now exports points
  // Number of points
  // For each points : X Y Z R G B Nm [ measurements ]
  // mesurements : Img_idx Feat_idx X Y
  const Landmarks & landmarks = sfm_data.GetLandmarks();
  const size_t featureCount = landmarks.size();
  file << featureCount << std::endl;
  C_Progress_display my_progress_bar( featureCount, std::cout, "\n- EXPORT LANDMARKS DATA -\n" );
  for ( Landmarks::const_iterator iterLandmarks = landmarks.begin();
        iterLandmarks != landmarks.end(); ++iterLandmarks, ++my_progress_bar )
  {
    const Vec3 exportPoint = iterLandmarks->second.X;
    file << exportPoint.x() << " " << exportPoint.y() << " " << exportPoint.z() << " ";
    file << 250 << " " << 100 << " " << 150 << " ";  // Write arbitrary RGB color

    // Tally set of feature observations
    const Observations & obs = iterLandmarks->second.obs;
    const size_t featureCount = std::distance( obs.begin(), obs.end() );
    file << featureCount;

    for ( Observations::const_iterator itObs = obs.begin(); itObs != obs.end(); ++itObs )
    {
      const IndexT viewId = map_viewIdToContiguous.at(itObs->first);
      const IndexT featId = itObs->second.id_feat;
      const Observation & ob = itObs->second;

      file << " " << viewId << " " << featId << " " << ob.x( 0 ) << " " << ob.x( 1 ) << " ";
    }
    file << "\n";
  }
  // EOF indicator
  file << "0";

  return true;
}

// Code based on main_openMVG2CMPMVS.cpp
bool OpenMVGHelper::exportToExternalMVS(R3DProject::Triangulation *pTriangulation, const wxString &pathname)
{
	// Make sure output directory exists
	wxFileName outDir(pathname, wxT(""));
	if(!outDir.DirExists())
	{
#if wxCHECK_VERSION(2, 9, 0)
		if(!outDir.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
		if(!outDir.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
			return false;
	}

	R3DProject *pProject = R3DProject::getInstance();
	R3DProjectPaths paths;
	pProject->getProjectPathsTri(paths, pTriangulation);
	bool hasSfmData = hasTriSfM_DataFile(paths);

	// Find parent ComputeMatches
	R3DProject::ComputeMatches *pComputeMatches = NULL;
	R3DProject::Object *pObject = R3DProject::getInstance()->getObjectByTypeAndID(R3DProject::R3DTreeItem::TypeComputeMatches, pTriangulation->parentId_);
	if(pObject != NULL)
		pComputeMatches = dynamic_cast<R3DProject::ComputeMatches *>(pObject);
	if(pComputeMatches == NULL)
		return false;
	// Find parent PictureSet
	R3DProject::PictureSet *pPictureSet = NULL;
	R3DProject::Object *pObject2 = R3DProject::getInstance()->getObjectByTypeAndID(R3DProject::R3DTreeItem::TypePictureSet, pComputeMatches->parentId_);
	if(pObject2 != NULL)
		pPictureSet = dynamic_cast<R3DProject::PictureSet *>(pObject2);
	if(pPictureSet == NULL)
		return false;

	// Load OpenMVG document/Sfm_Data file
	std::string sSfMDir = paths.relativeSfmOutPath_;
	Document doc;
	openMVG::sfm::SfM_Data sfm_data;
	size_t numberOfCalibratedViews = 0;
	if(hasSfmData)
	{
		if(!Load(sfm_data, paths.relativeTriSfmDataFilename_, ESfM_Data(ALL)))
			return false;
		numberOfCalibratedViews = sfm_data.GetPoses().size();
	}
	else
	{
		if(!doc.load(sSfMDir))
			return false;
		numberOfCalibratedViews = doc._map_camera.size();
	}

	wxFileName cmpmvsOutDir(outDir);
	cmpmvsOutDir.AppendDir(wxT("CMPMVS"));
	if(!cmpmvsOutDir.DirExists())
	{
		if(!cmpmvsOutDir.Mkdir())
			return false;
	}

	size_t count = 1;
	std::map<size_t, size_t> map_cameratoIndex;
	size_t indexCount = 0;
	// Create file for MeshRecon, see http://www-scf.usc.edu/~zkang/software.html
	std::ostringstream sfm_out;
	sfm_out << numberOfCalibratedViews << std::endl << std::endl;

	if(hasSfmData)
	{

		// Export valid views as Projective Cameras:
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)
		{
			const View * view = iter->second.get();
			Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			if(iterPose == sfm_data.GetPoses().end() ||
				iterIntrinsic == sfm_data.GetIntrinsics().end())
				continue;

			map_cameratoIndex[view->id_view] = indexCount++;

			// We have a valid view with a corresponding camera & pose
			const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(iterPose->second);
			std::ostringstream os;
			os << std::setw(5) << std::setfill('0') << count << "_P";
			//std::ofstream file(
			//	stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
			//	os.str(), "txt").c_str());
			std::ostringstream file;
			file << "CONTOUR" << os.widen('\n')
				<< P.row(0) << "\n" << P.row(1) << "\n" << P.row(2) << os.widen('\n');
			//file.close();

			wxFileName cameraFN(cmpmvsOutDir);
			cameraFN.SetName(wxString(os.str().c_str(), wxConvLibc));
			cameraFN.SetExt(wxT("txt"));
			wxFile camFile(cameraFN.GetFullPath(), wxFile::write);
			if(!camFile.IsOpened())
				return false;
			std::string camfileStr = file.str();
			camFile.Write(camfileStr.c_str(), camfileStr.length());
			camFile.Close();

			// MeshRecon export
			Mat3 R, K;
			Vec3 t;
			KRt_From_P(P, &K, &R, &t);
//			Mat3 R = iter->second._R;
//			Vec3 t = iter->second._t;
			double focal = K(0, 0);	// iter->second._K(0, 0);
			double k1 = 0.0, k2 = 0.0; // distortion already removed

			std::ostringstream os2;
			os2 << std::setw(5) << std::setfill('0') << count;

			sfm_out << "../CMPMVS/" << os2.str() << ".jpg "
				<< R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
				<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
				<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " "
				<< t[0] << " "
				<< t[1] << " "
				<< t[2] << " "
				<< focal << " " << focal << " "
				<< static_cast<float>(view->ui_width) / 2.0f << " " << static_cast<float>(view->ui_height) / 2.0f
				<< std::endl;

			++count;
		}

	}
	else
	{
		std::map<size_t, std::pair<size_t, size_t> >::const_iterator iterSize = doc._map_imageSize.begin();
		std::vector<std::string>::const_iterator iterName = doc._vec_imageNames.begin();

		// Export cameras
		for(std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
			//		iter != doc._map_camera.end(); ++iter, ++count)
			iter != doc._map_camera.end(); ++iter, ++count, ++iterSize, ++iterName)
		{
			const Mat34 & PMat = iter->second._P;
			std::ostringstream os;
			os << std::setw(5) << std::setfill('0') << count << "_P";
			//		std::ofstream file(
			//			stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
			//			os.str() ,"txt").c_str());
			std::ostringstream file;
			file << "CONTOUR\n"
				<< PMat.row(0) << "\n" << PMat.row(1) << "\n" << PMat.row(2) << std::endl;
			//		file.close();

			wxFileName cameraFN(cmpmvsOutDir);
			cameraFN.SetName(wxString(os.str().c_str(), wxConvLibc));
			cameraFN.SetExt(wxT("txt"));
			wxFile camFile(cameraFN.GetFullPath(), wxFile::write);
			if(!camFile.IsOpened())
				return false;
			std::string camfileStr = file.str();
			camFile.Write(camfileStr.c_str(), camfileStr.length());
			camFile.Close();


			// MeshRecon export
			Mat3 R = iter->second._R;
			Vec3 t = iter->second._t;
			double focal = iter->second._K(0, 0);
			double k1 = 0.0, k2 = 0.0; // distortion already removed

			std::ostringstream os2;
			os2 << std::setw(5) << std::setfill('0') << count;

			sfm_out << "../CMPMVS/" << os2.str() << ".jpg "
				<< R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
				<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
				<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " "
				<< t[0] << " "
				<< t[1] << " "
				<< t[2] << " "
				<< focal << " " << focal << " "
				<< iterSize->second.first / 2.0 << " " << iterSize->second.second / 2.0
				<< std::endl;
		}
	}

	// MeshRecon: Bounding box, image neighbours
	sfm_out << std::endl;

	std::vector< std::set<size_t> > image_neighbours;
	float minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX, minZ = FLT_MAX, maxZ = -FLT_MAX;

	if(hasSfmData)
	{
		image_neighbours.resize(numberOfCalibratedViews);
		for(Landmarks::const_iterator itL = sfm_data.GetLandmarks().begin();
			itL != sfm_data.GetLandmarks().end(); ++itL)
		{
			const Landmark & landmark = itL->second;
			const Vec3 & X = landmark.X;

			minX = std::min(minX, static_cast<float>(X.x()));
			maxX = std::max(maxX, static_cast<float>(X.x()));
			minY = std::min(minY, static_cast<float>(X.y()));
			maxY = std::max(maxY, static_cast<float>(X.y()));
			minZ = std::min(minZ, static_cast<float>(X.z()));
			maxZ = std::max(maxZ, static_cast<float>(X.z()));

			std::vector<IndexT> imageIndexes;
			for(Observations::const_iterator iterO = landmark.obs.begin();
				iterO != landmark.obs.end(); ++iterO)
			{
				const IndexT id_view = iterO->first;
				const Observation & ob = iterO->second;

				if(map_cameratoIndex.find(id_view) != map_cameratoIndex.end())
					imageIndexes.push_back(map_cameratoIndex[id_view]);
			}
			for(size_t i = 0; i < imageIndexes.size(); i++)
			{
				for(size_t j = 0; j < imageIndexes.size(); j++)
				{
					if(i != j)
					{
						image_neighbours[i].insert(j);
					}
				}
			}
		}
	}
	else
	{
		image_neighbours.resize(doc._map_camera.size());

		int trackIndex = 0;
		for (std::map< uint32_t, tracks::submapTrack >::const_iterator
			iterTracks = doc._tracks.begin();
			iterTracks != doc._tracks.end();
			++iterTracks,++trackIndex)
		{
			const tracks::submapTrack & map_track = iterTracks->second;

			const float * ptr3D = & doc._vec_points[trackIndex*3];

			minX = std::min(minX, ptr3D[0]);
			maxX = std::max(maxX, ptr3D[0]);
			minY = std::min(minY, ptr3D[1]);
			maxY = std::max(maxY, ptr3D[1]);
			minZ = std::min(minZ, ptr3D[2]);
			maxZ = std::max(maxZ, ptr3D[2]);

			std::vector<size_t> imageIndexes;
			for (tracks::submapTrack::const_iterator iterTrack = map_track.begin();
				iterTrack != map_track.end();
				++iterTrack)
			{
				imageIndexes.push_back(iterTrack->first);
			}
			for(size_t i = 0; i < imageIndexes.size(); i++)
			{
				for(size_t j = 0; j < imageIndexes.size(); j++)
				{
					if(i != j)
					{
						image_neighbours[i].insert(j);
					}
				}
			}

		}
	}
	sfm_out << minX << " " << maxX << " " << minY << " " << maxY << " " << minZ << " " << maxZ << std::endl << std::endl;

	for(size_t i = 0; i < image_neighbours.size(); i++)
	{
		std::set<size_t>::const_iterator iter = image_neighbours[i].begin();
		sfm_out << i << " " << image_neighbours[i].size();
		while(iter != image_neighbours[i].end())
		{
			sfm_out << " " << (*iter);
			iter++;
		}
		sfm_out << std::endl;
	}
	sfm_out << std::endl;

	wxFileName sfm_outFN(outDir);
	sfm_outFN.AppendDir(wxT("meshrecon"));
	if(!sfm_outFN.DirExists())
		sfm_outFN.Mkdir();
	sfm_outFN.SetFullName(wxT("output.sfm"));
	wxFile sfm_outFile(sfm_outFN.GetFullPath(), wxFile::write);
	if(!sfm_outFile.IsOpened())
		return false;
	std::string sfm_outStr = sfm_out.str();
	sfm_outFile.Write(sfm_outStr.c_str(), sfm_outStr.length());
	sfm_outFile.Close();
	// MeshRecon finished



	// Export undistorted images
	wxFileName destImageFN(cmpmvsOutDir);
	count = 1;

	if(hasSfmData)
	{
		openMVG::image::Image<openMVG::image::RGBColor> image, image_ud;
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)
		{
			const View * view = iter->second.get();
			Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			if(iterPose == sfm_data.GetPoses().end() ||
				iterIntrinsic == sfm_data.GetIntrinsics().end())
				continue;

			// We have a valid view with a corresponding camera & pose
			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			std::ostringstream os;
			os << std::setw(5) << std::setfill('0') << count;
			destImageFN.SetName(wxString(os.str().c_str(), wxConvLibc));
			destImageFN.SetExt(wxT("jpg"));

			const IntrinsicBase * cam = iterIntrinsic->second.get();
			if(cam->have_disto())
			{
				// undistort the image and save it
				openMVG::image::ReadImage(srcImage.c_str(), &image);
				UndistortImage(image, cam, image_ud, openMVG::image::BLACK);
				wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
				if(outImageFile.IsOpened())
				{
					const unsigned char * ptr = (unsigned char*)(image_ud.GetMat().data());
					int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
					std::vector<unsigned char> array(ptr, ptr + image_ud.Width()*image_ud.Height()*depth);
					int w = image_ud.Width(), h = image_ud.Height();
					openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
				}
			}
			else // (no distortion)
			{
				// copy the image if extension match
				if(stlplus::extension_part(srcImage) == "JPG" ||
					stlplus::extension_part(srcImage) == "jpg")
				{
					//stlplus::file_copy(srcImage, dstImage);
					if(!wxCopyFile(wxString(srcImage.c_str(), wxConvLibc), destImageFN.GetFullPath(), false))
						return false;
				}
				else
				{
					openMVG::image::ReadImage(srcImage.c_str(), &image);
//					WriteImage(dstImage.c_str(), image);
					wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
					if(outImageFile.IsOpened())
					{
						const unsigned char * ptr = (unsigned char*)(image.GetMat().data());
						int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
						std::vector<unsigned char> array(ptr, ptr + image.Width()*image.Height()*depth);
						int w = image.Width(), h = image.Height();
						openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
					}
				}
			}

			++count;
		}
	}
	else
	{
		wxFileName undistImageFN(wxString(paths.relativeSfmOutPath_.c_str(), wxConvLibc), wxT(""));
		undistImageFN.AppendDir(wxT("images"));
		//Image<openMVG::image::RGBColor> image;
		for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
			iter != doc._map_camera.end();  ++iter, ++count)
		{
			size_t imageIndex = iter->first;
			const std::string & sImageName = doc._vec_imageNames[imageIndex];
			std::ostringstream os;
			os << std::setw(5) << std::setfill('0') << count;
/*			ReadImage( stlplus::create_filespec( sImagePath, sImageName).c_str(), &image );
			w = image.Width();
			h = image.Height();
			std::string sCompleteImageName = stlplus::create_filespec(
				stlplus::folder_append_separator(sOutDirectory), os.str(),"jpg");
			WriteImage( sCompleteImageName.c_str(), image);*/

			wxString curName( sImageName.c_str(), wxConvLibc );
			undistImageFN.SetFullName(curName);
			destImageFN.SetName( wxString( os.str().c_str(), wxConvLibc ) );
			destImageFN.SetExt( wxT("jpg") );

			if(!wxCopyFile(undistImageFN.GetFullPath(), destImageFN.GetFullPath(), false))
				return false;
		}
	}
	// Determine width and height of the largest image. CMPMVS scales the smaller images accordingly.

	const ImageInfoVector &iiv = pPictureSet->getImageInfoVector();
	int w = 0, h = 0;
	for(size_t i = 0; i < iiv.size(); i++)
	{
		const ImageInfo &iv = iiv[i];
		w = std::max(w, iv.imageWidth_);
		h = std::max(h, iv.imageHeight_);
	}

	{
		// Write the mvs_firstRun script
		std::ostringstream os;
		os << "[global]" << std::endl
			<< "dirName=\"CMPMVS\\\"" << std::endl
			<< "prefix=\"\"" << std::endl
			<< "imgExt=\"jpg\"" << std::endl
			<< "ncams=" << numberOfCalibratedViews << std::endl
			<< "width=" << w << std::endl
			<< "height=" << h << std::endl
			<< "scale=2" << std::endl
			<< "workDirName=\"_tmp\"" << std::endl
			<< "doPrepareData=TRUE" << std::endl
			<< "doPrematchSifts=TRUE" << std::endl
			<< "doPlaneSweepingSGM=TRUE"  << std::endl
			<< "doFuse=TRUE" << std::endl
			<< "nTimesSimplify=10" << std::endl
			<< std::endl
			<< "[prematching]" << std::endl
			<< "minAngle=3.0" << std::endl
			<< std::endl
			<< "[grow]" << std::endl
			<< "minNumOfConsistentCams=6" << std::endl
			<< std::endl
			<< "[filter]" << std::endl
			<< "minNumOfConsistentCams=2" << std::endl
			<< std::endl
			<< "[semiGlobalMatching]" << std::endl
			<< "wsh=4" << std::endl
			<< std::endl
			<< "[refineRc]" << std::endl
			<< "wsh=4" << std::endl
			<< std::endl
			<< "#do not erase empy lines after this comment otherwise it will crash ... bug" << std::endl
			<< std::endl
			<< std::endl;

		wxFileName scriptFN(cmpmvsOutDir);
		scriptFN.SetFullName( wxT("01_mvs_firstRun.ini") );
		wxFile scriptFile(scriptFN.GetFullPath(), wxFile::write);
		if(!scriptFile.IsOpened())
			return false;
		std::string scriptStr = os.str();
		scriptFile.Write(scriptStr.c_str(), scriptStr.length());
		scriptFile.Close();
	}

	{
		// Write the limitedScale script
		std::ostringstream os;
		os << "[global]" << std::endl
			<< "dirName=\"CMPMVS\\\"" << std::endl
			<< "prefix=\"\"" << std::endl
			<< "imgExt=\"jpg\"" << std::endl
			<< "ncams=" << numberOfCalibratedViews << std::endl
			<< "width=" << w << std::endl
			<< "height=" << h << std::endl
			<< "scale=2" << std::endl
			<< "workDirName=\"_tmp\"" << std::endl
			<< "doPrepareData=FALSE" << std::endl
			<< "doPrematchSifts=FALSE" << std::endl
			<< "doPlaneSweepingSGM=FALSE"  << std::endl
			<< "doFuse=FALSE" << std::endl
			<< std::endl
			<< "[uvatlas]" << std::endl
			<< "texSide=4096" << std::endl
			<< "scale=1" << std::endl
			<< std::endl
			<< "[delanuaycut]" << std::endl
			<< "saveMeshTextured=FALSE" << std::endl
			<< std::endl
			<< "[hallucinationsFiltering]" << std::endl
			<< "useSkyPrior=FALSE" << std::endl
			<< "doLeaveLargestFullSegmentOnly=FALSE" << std::endl
			<< "doRemoveHugeTriangles=TRUE" << std::endl
			<< std::endl
			<< "[largeScale]" << std::endl
			<< "doGenerateAndReconstructSpaceMaxPts=TRUE" << std::endl
			<< "doGenerateSpace=TRUE" << std::endl
			<< "planMaxPts=3000000" << std::endl
			<< "doComputeDEMandOrtoPhoto=FALSE" << std::endl
			<< "doGenerateVideoFrames=FALSE" << std::endl
			<< std::endl
			<< "[meshEnergyOpt]" << std::endl
			<< "doOptimizeOrSmoothMesh=FALSE" << std::endl
			<< std::endl
			<< std::endl
			<< "#EOF" << std::endl
			<< std::endl
			<< std::endl;

		wxFileName scriptFN(cmpmvsOutDir);
		scriptFN.SetFullName( wxT("02_mvs_limitedScale.ini") );
		wxFile scriptFile(scriptFN.GetFullPath(), wxFile::write);
		if(!scriptFile.IsOpened())
			return false;
		std::string scriptStr = os.str();
		scriptFile.Write(scriptStr.c_str(), scriptStr.length());
		scriptFile.Close();
	}

	// Export to OpenMVS
	wxFileName openMVSOutDir(outDir);
	openMVSOutDir.AppendDir(wxT("OpenMVS"));
	if(!openMVSOutDir.DirExists())
	{
		if(!openMVSOutDir.Mkdir())
			return false;
	}

	OpenMVGExportToMVS::exportToOpenMVS(pTriangulation, openMVSOutDir.GetPath());

	// Export to NVM
#if defined(_WIN32)
	boost::filesystem::path nvmOutDir = boost::filesystem::path(outDir.GetPath().wc_str()) / "nvm";
#else
	boost::filesystem::path nvmOutDir = boost::filesystem::path(outDir.GetPath().ToUTF8()) / "nvm";
#endif
	boost::filesystem::path nvmOutFile = nvmOutDir / "project.nvm";
	bool isOk = CreateNVMFile( sfm_data, nvmOutDir, nvmOutFile );


	// Export to SURE
	wxFileName sureOutDir(outDir);
	sureOutDir.AppendDir(wxT("SURE"));
	if(!sureOutDir.DirExists())
	{
		if(!sureOutDir.Mkdir())
			return false;
	}

	exportToSure( sfm_data, sureOutDir.GetFullPath() );

	return isOk;
}

bool OpenMVGHelper::exportToSure(const openMVG::sfm::SfM_Data &sfm_data, const wxString &pathname)
{
	wxFileName sureOutDir(pathname);

	// Create img and ori directories
	wxFileName imgOutDir(sureOutDir);
	imgOutDir.AppendDir(wxT("img"));
	if(!imgOutDir.DirExists())
		if(!imgOutDir.Mkdir())
			return false;
	wxFileName oriOutDir(sureOutDir);
	oriOutDir.AppendDir(wxT("ori"));
	if(!oriOutDir.DirExists())
		if(!oriOutDir.Mkdir())
			return false;

	// Export undistorted images
	wxFileName destImageFN(imgOutDir);
	int count = 1;
	{
		openMVG::image::Image<openMVG::image::RGBColor> image, image_ud;
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)
		{
			const View * view = iter->second.get();
			Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			if(iterPose == sfm_data.GetPoses().end() ||
				iterIntrinsic == sfm_data.GetIntrinsics().end())
				continue;

			// We have a valid view with a corresponding camera & pose
			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			std::ostringstream os;
			os << std::setw(5) << std::setfill('0') << count;
			destImageFN.SetName(wxString(os.str().c_str(), wxConvLibc));
			destImageFN.SetExt(wxT("jpg"));
			std::string baseimagefilename = os.str();

			const IntrinsicBase * cam = iterIntrinsic->second.get();
			const Pinhole_Intrinsic_Brown_T2 *pBrownCamera = dynamic_cast<const Pinhole_Intrinsic_Brown_T2 *>(cam);
			const Pinhole_Intrinsic_Radial_K3 *pPinholeRadialK3Camera = dynamic_cast<const Pinhole_Intrinsic_Radial_K3 *>(cam);
			bool isSuitableCamera = (pBrownCamera != nullptr || pPinholeRadialK3Camera != nullptr);
			if(!isSuitableCamera && cam->have_disto())
			{
				// undistort the image and save it
				openMVG::image::ReadImage(srcImage.c_str(), &image);
				UndistortImage(image, cam, image_ud, openMVG::image::BLACK);
				wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
				if(outImageFile.IsOpened())
				{
					const unsigned char * ptr = (unsigned char*)(image_ud.GetMat().data());
					int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
					std::vector<unsigned char> array(ptr, ptr + image_ud.Width()*image_ud.Height()*depth);
					int w = image_ud.Width(), h = image_ud.Height();
					openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
				}
			}
			else // (no distortion)
			{
				// copy the image if extension match
				if(stlplus::extension_part(srcImage) == "JPG" ||
					stlplus::extension_part(srcImage) == "jpg")
				{
					//stlplus::file_copy(srcImage, dstImage);
					if(!wxCopyFile(wxString(srcImage.c_str(), wxConvLibc), destImageFN.GetFullPath(), false))
						return false;
				}
				else
				{
					openMVG::image::ReadImage(srcImage.c_str(), &image);
//					WriteImage(dstImage.c_str(), image);
					wxFFile outImageFile(destImageFN.GetFullPath(), wxT("wb"));
					if(outImageFile.IsOpened())
					{
						const unsigned char * ptr = (unsigned char*)(image.GetMat().data());
						int depth = sizeof(openMVG::image::RGBColor) / sizeof(unsigned char);
						std::vector<unsigned char> array(ptr, ptr + image.Width()*image.Height()*depth);
						int w = image.Width(), h = image.Height();
						openMVG::image::WriteJpgStream(outImageFile.fp(), array, w, h, depth);
					}
				}
			}

			// Create SURE ori file
			wxFileName oriFileName(oriOutDir);
			oriFileName.SetName(wxString(os.str().c_str(), wxConvLibc));
			oriFileName.SetExt(wxT("ori"));

			wxFFile oriFile(oriFileName.GetFullPath(), wxT("wb"));
			if(oriFile.IsOpened())
			{
				const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>( cam );
				const Pinhole_Intrinsic_Brown_T2 *pBrownCamera = dynamic_cast<const Pinhole_Intrinsic_Brown_T2 *>(pinhole_cam);
				const Pinhole_Intrinsic_Radial_K3 *pPinholeRadialK3Camera = dynamic_cast<const Pinhole_Intrinsic_Radial_K3 *>(pinhole_cam);
				const double flen = pinhole_cam->focal();
				const Pose3 pose = sfm_data.GetPoseOrDie( view );
				const Mat3 rotation = pose.rotation();
				const Vec3 center = pose.center();
				Vec3 transl = pose.translation();

				const double Cx = center[0];
				const double Cy = center[1];
				const double Cz = center[2];
				Eigen::Quaterniond q( rotation );
				const double Qx = q.x();
				const double Qy = q.y();
				const double Qz = q.z();
				const double Qw = q.w();
				const double d0 = 0.0;

				const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(iterPose->second);

				Mat3 R, K;
				Vec3 t2;
				KRt_From_P(P, &K, &R, &t2);
				double k1 = 0.0, k2 = 0.0; // distortion already removed

				Vec3 t3 = - (R.transpose() * t2);		// in world coordinates
				const auto &t = t3;
				double focal = K(0, 0);

				std::ostringstream os;
				os << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
				os << "$ImageID___________________________________________________(ORI_Ver_1.0)" << std::endl
					<< "	    " << baseimagefilename << std::endl
					<< "$IntOri_FocalLength_________________________________________________[mm]" << std::endl
					<< "	      " << flen << std::endl
					<< "$IntOri_PixelSize______(x|y)________________________________________[mm]" << std::endl
					<< 	"        0.001000	        0.001000" << std::endl					// TODO
					<< "$IntOri_SensorSize_____(x|y)_____________________________________[pixel]" << std::endl
					<< "	            " << view->ui_width << "	            " << view->ui_height << std::endl
					<< "$IntOri_PrincipalPoint_(x|y)_____________________________________[pixel]" << std::endl
					<< "	   " << K(0, 2) << "	   " << K(1, 2) << std::endl
					<< "$IntOri_CameraMatrix_____________________________(ImageCoordinateSystem)" << std::endl
					<< "	   " << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << " " << std::endl
					<< "	   " << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << " " << std::endl
					<< "	   " << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << " " << std::endl
					<< "$ExtOri_RotationMatrix____________________(World->ImageCoordinateSystem)" << std::endl
					<< "	   " << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << std::endl
					<< "	   " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << std::endl
					<< "	   " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << std::endl
					<< "$ExtOri_TranslationVector________________________(WorldCoordinateSystem)" << std::endl
					<< "	   " << t[0] << " " << t[1] << " " << t[2] << std::endl
					<< "$IntOri_Distortion_____(Model|ParameterCount|(Parameters))______________" << std::endl;

				if(pBrownCamera != nullptr)
				{
					auto params = pBrownCamera->getParams();	// K1, K2, K3, T1, T2
					os << "	    Brown5	  5" << std::endl
						<< "	 " << params[3] << "	  " << params[4] << "	  " << params[5] << std::endl
						<< "	  " << params[6] << "	  " << params[7] << std::endl;
				}
				else if(pPinholeRadialK3Camera != nullptr)
				{
					auto params = pPinholeRadialK3Camera->getParams();	// K1, K2, K3
					os << "	    Brown5	  5" << std::endl
						<< "	 " << params[3] << "	  " << params[4] << "	  " << params[5] << std::endl
						<< "	  " << 0.0 << "	  " << 0.0 << std::endl;
				}
				else
				{
					os << "	    NONE	  0" << std::endl;
				}
				std::string scriptStr = os.str();
				oriFile.Write(scriptStr.c_str(), scriptStr.length());
				oriFile.Close();

			}

			++count;
		}
	}

	return true;
}


#if !defined(R3D_USE_OPENMVG_PRE08)

// The following code is copied from main_ComputeSfm_DataColor.cpp, slightly adjusted.
// Original license follows:

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

void OpenMVGHelper::ColorizeTracks(const openMVG::sfm::SfM_Data &sfm_data, std::vector<Vec3> &vec_3dPoints, std::vector<Vec3> &vec_tracksColor)
{
	// Colorize each track
	//  Start with the most representative image
	//    and iterate to provide a color to each 3D point

	{
		C_Progress_display my_progress_bar(sfm_data.GetLandmarks().size(),
			std::cout,
			"\nCompute scene structure color\n");

		vec_tracksColor.resize(sfm_data.GetLandmarks().size());
		vec_3dPoints.resize(sfm_data.GetLandmarks().size());

		//Build a list of contiguous index for the trackIds
		std::map<IndexT, IndexT> trackIds_to_contiguousIndexes;
		IndexT cpt = 0;
		for(Landmarks::const_iterator it = sfm_data.GetLandmarks().begin();
			it != sfm_data.GetLandmarks().end(); ++it, ++cpt)
		{
			trackIds_to_contiguousIndexes[it->first] = cpt;
			vec_3dPoints[cpt] = it->second.X;
		}

		// The track list that will be colored (point removed during the process)
		std::set<IndexT> remainingTrackToColor;
		std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
			std::inserter(remainingTrackToColor, remainingTrackToColor.begin()),
			stl::RetrieveKey());

		while(!remainingTrackToColor.empty())
		{
			// Find the most representative image (for the remaining 3D points)
			//  a. Count the number of observation per view for each 3Dpoint Index
			//  b. Sort to find the most representative view index

			std::map<IndexT, IndexT> map_IndexCardinal; // ViewId, Cardinal
			for(std::set<IndexT>::const_iterator
				iterT = remainingTrackToColor.begin();
				iterT != remainingTrackToColor.end();
			++iterT)
			{
				const size_t trackId = *iterT;
				const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;
				for(Observations::const_iterator iterObs = obs.begin();
					iterObs != obs.end(); ++iterObs)
				{
					const size_t viewId = iterObs->first;
					if(map_IndexCardinal.find(viewId) == map_IndexCardinal.end())
						map_IndexCardinal[viewId] = 1;
					else
						++map_IndexCardinal[viewId];
				}
			}

			// Find the View index that is the most represented
			std::vector<IndexT> vec_cardinal;
			std::transform(map_IndexCardinal.begin(),
				map_IndexCardinal.end(),
				std::back_inserter(vec_cardinal),
				stl::RetrieveValue());
			using namespace stl::indexed_sort;
			std::vector< sort_index_packet_descend< IndexT, IndexT> > packet_vec(vec_cardinal.size());
			sort_index_helper(packet_vec, &vec_cardinal[0], 1);

			// First image index with the most of occurence
			std::map<IndexT, IndexT>::const_iterator iterTT = map_IndexCardinal.begin();
			std::advance(iterTT, packet_vec[0].index);
			const size_t view_index = iterTT->first;
			const View * view = sfm_data.GetViews().at(view_index).get();
			const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,
				view->s_Img_path);
			openMVG::image::Image<openMVG::image::RGBColor> image;
			openMVG::image::ReadImage(sView_filename.c_str(), &image);

			// Iterate through the remaining track to color
			// - look if the current view is present to color the track
			std::set<IndexT> set_toRemove;
			for(std::set<IndexT>::const_iterator
				iterT = remainingTrackToColor.begin();
				iterT != remainingTrackToColor.end();
			++iterT)
			{
				const size_t trackId = *iterT;
				const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;
				Observations::const_iterator it = obs.find(view_index);

				if(it != obs.end())
				{
					// Color the track
					const Vec2 & pt = it->second.x;
					const openMVG::image::RGBColor &color = image(pt.y(), pt.x());

					vec_tracksColor[trackIds_to_contiguousIndexes[trackId]] = Vec3(color.r(), color.g(), color.b());

					set_toRemove.insert(trackId);
					++my_progress_bar;
				}
			}
			// Remove colored track
			for(std::set<IndexT>::const_iterator iter = set_toRemove.begin();
				iter != set_toRemove.end(); ++iter)
			{
				remainingTrackToColor.erase(*iter);
			}
		}
	}
}

void OpenMVGHelper::GetCameraPositions(const openMVG::sfm::SfM_Data &sfm_data, std::vector<Vec3> &vec_camPosition)
{
	const Poses & poses = sfm_data.GetPoses();
	for(Poses::const_iterator iterPose = poses.begin();
		iterPose != poses.end(); ++iterPose)
	{
		vec_camPosition.push_back(iterPose->second.center());
	}
}

void OpenMVGHelper::calculateResiduals(const openMVG::sfm::SfM_Data &sfm_data, std::vector<float> &residuals)
{
	// Calculate residuals from SfM_Data
	// Code copied from SequentialSfMReconstructionEngine::ComputeResidualsHistogram
	residuals.reserve(sfm_data.structure.size());
	for (const auto & landmark_entry : sfm_data.GetLandmarks())
	{
		const Observations & obs = landmark_entry.second.obs;
		for (const auto & observation : obs)
		{
			const View * view = sfm_data.GetViews().find(observation.first)->second.get();
			const Pose3 pose = sfm_data.GetPoseOrDie(view);
			const auto intrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic)->second;
			const Vec2 residual = intrinsic->residual(pose(landmark_entry.second.X), observation.second.x);
			residuals.emplace_back( std::abs(residual(0)) );
			residuals.emplace_back( std::abs(residual(1)) );
		}
	}
}

void OpenMVGHelper::exportOldSfM_output(const R3DProjectPaths &paths)
{
	const std::string sComment = std::string("Generated by the OpenMVG library");

	openMVG::sfm::SfM_Data sfm_data;
	if(Load(sfm_data, paths.relativeTriSfmDataFilename_, ESfM_Data(ALL)))
	{
		// The following code is taken mostly from SfMReconstructionData.hpp from openMVG 0.7
		// (reconstructorHelper::ExportToOpenMVGFormat)

		string sOutDirectory = paths.relativeSfmOutPath_;
		bool bOk = true;
		if(!stlplus::is_folder(sOutDirectory))
		{
			stlplus::folder_create(sOutDirectory);
			bOk = stlplus::is_folder(sOutDirectory);
		}

		// Create basis directory structure
		stlplus::folder_create(stlplus::folder_append_separator(sOutDirectory) + "cameras");
		stlplus::folder_create(stlplus::folder_append_separator(sOutDirectory) + "cameras_disto");
		stlplus::folder_create(stlplus::folder_append_separator(sOutDirectory) + "clouds");
		stlplus::folder_create(stlplus::folder_append_separator(sOutDirectory) + "images");

		if(bOk &&
			stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "cameras") &&
			stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "cameras_disto") &&
			stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "clouds") &&
			stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "images")
			)
		{
			bOk = true;
		}

		if(bOk)
		{
			//Export Camera as binary files
			std::map<size_t, size_t> map_cameratoIndex;
			size_t count = 0;

			for(Views::const_iterator iter = sfm_data.GetViews().begin();
				iter != sfm_data.GetViews().end(); ++iter)
			{
				const View * view = iter->second.get();
				Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
				Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

				if(iterPose == sfm_data.GetPoses().end() ||
					iterIntrinsic == sfm_data.GetIntrinsics().end())
					continue;

				map_cameratoIndex[iter->first] = count++;

				// We have a valid view with a corresponding camera & pose
				const Mat34 PMat = iterIntrinsic->second.get()->get_projective_equivalent(iterPose->second);

				std::ofstream file(
					stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "cameras",
					stlplus::basename_part(iter->second->s_Img_path)
					, "bin").c_str(), std::ios::out | std::ios::binary);
				file.write((const char*)PMat.data(), static_cast<std::streamsize>(3 * 4)*sizeof(double));

				bOk &= (!file.fail());
				file.close();
			}


			//-- Export the camera with disto
			for(Views::const_iterator iter = sfm_data.GetViews().begin();
				iter != sfm_data.GetViews().end(); ++iter)
			{
				const View * view = iter->second.get();
				Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
				Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

				if(iterPose == sfm_data.GetPoses().end() ||
					iterIntrinsic == sfm_data.GetIntrinsics().end())
					continue;

				std::ofstream file(
					stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "cameras_disto",
					stlplus::basename_part(iter->second->s_Img_path)
					, "txt").c_str(), std::ios::out);


				const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(iterPose->second);
				Mat3 R, K;
				Vec3 t;
//				KRt_From_P(P, &K, &R, &t);
				R = iterPose->second.rotation();
				t = iterPose->second.translation();
				double focal = 0;	//K(0, 0);
				double k1 = 0.0, k2 = 0.0, k3 = 0.0;
				double ppx = static_cast<double>(view->ui_width) / 2.0;
				double ppy = static_cast<double>(view->ui_height) / 2.0;

				const IntrinsicBase * cam = iterIntrinsic->second.get();
				std::vector<double> cameraParameters = cam->getParams();
				if(cameraParameters.size() > 0)
					focal = cameraParameters[0];
				if(cameraParameters.size() > 2)
				{
					ppx = cameraParameters[1];
					ppy = cameraParameters[2];
				}
				if(cameraParameters.size() > 3)
					k1 = cameraParameters[3];
				if(cameraParameters.size() > 4)
					k2 = cameraParameters[4];
				if(cameraParameters.size() > 5)
					k3 = cameraParameters[5];
				// Save intrinsic data:
				file << focal << " "
					<< ppx << " "
					<< ppy << " "
					<< k1 << " "
					<< k2 << " "
					<< k3 << "\n";
				// Save extrinsic data
				file << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
					<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
					<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n";
				file << t(0) << " " << t(1) << " " << t(2) << "\n";
				bOk &= (!file.fail());
				file.close();
			}


			//Export 3D point and tracks

			size_t nc = sfm_data.GetViews().size();
			size_t nt = sfm_data.GetLandmarks().size();

			// Clipping planes (near and far Z depth per view)
			std::vector<double> znear(nc, std::numeric_limits<double>::max()), zfar(nc, 0);
			// Cloud
			std::ofstream f_cloud(
				stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "clouds",
				"calib", "ply").c_str());
			std::ofstream f_visibility(
				stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "clouds",
				"visibility", "txt").c_str());

			if(!f_cloud.is_open()) {
//				std::cerr << "cannot save cloud" << std::endl;
				return;
			}
			if(!f_visibility.is_open()) {
//				std::cerr << "cannot save cloud desc" << std::endl;
				return;
			}
			f_cloud << "ply\nformat ascii 1.0\n"
				<< "comment " << sComment << "\n"
				<< "element vertex " << nt << "\n"
				<< "property float x\nproperty float y\nproperty float z" << "\n"
				<< "property uchar red\nproperty uchar green\nproperty uchar blue" << "\n"
				<< "property float confidence\nproperty list uchar int visibility" << "\n"
				<< "element face 0\nproperty list uchar int vertex_index" << "\n"
				<< "end_header" << "\n";

			for(Landmarks::const_iterator itL = sfm_data.GetLandmarks().begin();
				itL != sfm_data.GetLandmarks().end(); ++itL)
			{
				const Landmark & landmark = itL->second;
				const Vec3 & X = landmark.X;

				f_cloud << X.transpose() << " 255 255 255 " << 3.14;

				std::ostringstream s_visibility;

				std::set< size_t > set_imageIndex;
				for(Observations::const_iterator iterO = landmark.obs.begin();
					iterO != landmark.obs.end(); ++iterO)
				{
					const IndexT id_view = iterO->first;
					const Observation & ob = iterO->second;

					// Determine camera extrinsics
					Views::const_iterator iterView = sfm_data.GetViews().find(id_view);
					if(iterView == sfm_data.GetViews().end())
						continue;
					Poses::const_iterator iterPose = sfm_data.GetPoses().find(iterView->second->id_pose);
					if(iterPose == sfm_data.GetPoses().end())
						continue;

					if(map_cameratoIndex.find(id_view) != map_cameratoIndex.end())
					{
						size_t cameraIndex = map_cameratoIndex[id_view];
						set_imageIndex.insert(cameraIndex);

						double z = Depth(iterPose->second.rotation(), iterPose->second.translation(), X);
						znear[id_view] = std::min(znear[id_view], z);
						zfar[id_view] = std::max(zfar[id_view], z);

						s_visibility << cameraIndex << " " << ob.id_feat << " ";
					}
				}

				//export images indexes
				f_cloud << " " << set_imageIndex.size() << " ";
				copy(set_imageIndex.begin(), set_imageIndex.end(), std::ostream_iterator<size_t>(f_cloud, " "));
				f_cloud << std::endl;

				f_visibility << X.transpose() << " " << set_imageIndex.size() << " ";
				f_visibility << s_visibility.str() << "\n";
			}
			f_cloud.close();
			f_visibility.close();

			// Views
			f_cloud.open(stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
				"views", "txt").c_str());
			if(!f_cloud.is_open()) {
//				std::cerr << "Cannot write views" << endl;
				return;
			}
			f_cloud << "images\ncameras\n" << nc << "\n";

			count = 0;
			for(Views::const_iterator iter = sfm_data.GetViews().begin();
				iter != sfm_data.GetViews().end(); ++iter, ++count)
			{
				const View * view = iter->second.get();
				Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
				Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

				// Only export views with valid pose and intrinsic
				if(iterPose == sfm_data.GetPoses().end() ||
					iterIntrinsic == sfm_data.GetIntrinsics().end())
					continue;

				f_cloud << stlplus::filename_part(view->s_Img_path)
					<< ' ' << view->ui_width
					<< ' ' << view->ui_height
					<< ' ' << stlplus::basename_part(view->s_Img_path) << ".bin"
					<< ' ' << znear[count] / 2
					<< ' ' << zfar[count] * 2
					<< "\n";
			}
			f_cloud.close();

			// Export (calibrated) views as undistorted images
			count = 0;
			openMVG::image::Image<openMVG::image::RGBColor> image, image_ud;
			for(Views::const_iterator iter = sfm_data.GetViews().begin();
				iter != sfm_data.GetViews().end(); ++iter)
			{
				const View * view = iter->second.get();
				Poses::const_iterator iterPose = sfm_data.GetPoses().find(view->id_pose);
				Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

				if(iterPose == sfm_data.GetPoses().end() ||
					iterIntrinsic == sfm_data.GetIntrinsics().end())
					continue;

				// We have a valid view with a corresponding camera & pose
				const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
				std::string sImageName = view->s_Img_path;
				std::string dstImage =
					stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "images",
					stlplus::basename_part(sImageName),
					stlplus::extension_part(sImageName));

				const IntrinsicBase * cam = iterIntrinsic->second.get();
				if(cam->have_disto())
				{
					// undistort the image and save it
					ReadImage(srcImage.c_str(), &image);
					UndistortImage(image, cam, image_ud, openMVG::image::BLACK);
					WriteImage(dstImage.c_str(), image_ud);
				}
				else // (no distortion)
				{
					// copy the image if extension match
					if(stlplus::extension_part(srcImage) == "JPG" ||
						stlplus::extension_part(srcImage) == "jpg")
					{
						stlplus::file_copy(srcImage, dstImage);
					}
					else
					{
						ReadImage(srcImage.c_str(), &image);
						WriteImage(dstImage.c_str(), image);
					}
				}
				++count;
			}

/*

			// Test
			R3DProject::Densification tempDens;
			tempDens.densificationType_ = R3DProject::DTCMVSPMVS;
			tempDens.useCMVS_ = false;

			exportToPMVSFormat(sfm_data, "pmvs_compare_sfmdata", &tempDens);
			exportToBundlerFormat(sfm_data,
				stlplus::folder_append_separator("pmvs_compare_sfmdata") + "bundle.rd.out",
				stlplus::folder_append_separator("pmvs_compare_sfmdata") + "list.txt"
				);

			std::string sSfMDir = paths.relativeSfmOutPath_;
			Document doc;
			doc.load(sSfMDir);
			exportToPMVSFormat(doc, "pmvs_compare_doc", stlplus::folder_append_separator(sSfMDir) + "images", &tempDens);
			exportToBundlerFormat(doc,
				stlplus::folder_append_separator("pmvs_compare_doc") + "bundle.rd.out",
				stlplus::folder_append_separator("pmvs_compare_doc") + "list.txt"
			);

*/
		}

	}
}

// The following method is copied mostly from OpenMVG's main_openMVG2MVE2.cpp and slightly adjusted.
// Original copyright follows.

/* v.0.16 25 August 2015
* Kevin CAIN, www.insightdigital.org
* Adapted from the openMVG libraries,
* Copyright (c) 2012-2015 Pierre MOULON.
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;

bool OpenMVGHelper::exportToMVE2Format(
	const SfM_Data & sfm_data,
	const wxString& sOutDirectory // Output MVE2 files directory
	)
{
	bool bOk = true;
	// Create basis directory structure
	wxFileName outDir(sOutDirectory, "");
	if(!outDir.DirExists())
	{
#if wxCHECK_VERSION(2, 9, 0)
		if(!outDir.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
		if(!outDir.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
			return false;
	}

	// Export the SfM_Data scene to the MVE2 format
	{
		// Create 'views' subdirectory
		wxFileName outViewsDir(outDir);
		outViewsDir.AppendDir(wxT("views"));
		if(!outViewsDir.DirExists())
		{
#if wxCHECK_VERSION(2, 9, 0)
			if(!outViewsDir.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
			if(!outViewsDir.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
				return false;
		}
		wxFileName outFN(outDir);
		outFN.SetFullName(wxT("synth_0.out"));
		std::string sOutFNStr(outFN.GetFullPath().mb_str());

		// Prepare to write bundle file
		// Get cameras and features from OpenMVG
		const size_t cameraCount = std::distance(sfm_data.GetViews().begin(), sfm_data.GetViews().end());
		// Tally global set of feature landmarks
		const Landmarks & landmarks = sfm_data.GetLandmarks();
		const size_t featureCount = std::distance(landmarks.begin(), landmarks.end());
//		const std::string filename = "synth_0.out";
//		std::cout << "Writing bundle (" << cameraCount << " cameras, "
//			<< featureCount << " features): to " << filename << "...\n";
		std::ofstream out(sOutFNStr.c_str());	// stlplus::folder_append_separator(sOutDirectory) + filename);
		out << "drews 1.0\n";  // MVE expects this header
		out << cameraCount << " " << featureCount << "\n";

		// Export (calibrated) views as undistorted images
		//C_Progress_display my_progress_bar(sfm_data.GetViews().size());
		std::pair<int, int> w_h_image_size;
		Image<openMVG::image::RGBColor> image, image_ud, thumbnail;
		//std::string sOutViewIteratorDirectory;
		for(Views::const_iterator iter = sfm_data.GetViews().begin();
			iter != sfm_data.GetViews().end(); ++iter)//, ++my_progress_bar)
		{
			const View * view = iter->second.get();

			// Create current view subdirectory 'view_xxxx.mve'
			std::ostringstream padding;
			padding << std::setw(4) << std::setfill('0') << view->id_view;

			wxFileName viewFN(outViewsDir);
			viewFN.AppendDir(wxString(wxT("view_")) + wxString(padding.str().c_str(), *wxConvCurrent) + wxString(wxT(".mve")));
			//sOutViewIteratorDirectory = std::string(viewFN.GetFullPath().mb_str());	//stlplus::folder_append_separator(sOutViewsDirectory) + "view_" + padding.str() + ".mve";

			Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);

			// We have a valid view with a corresponding camera & pose
			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
			wxFileName dstImageFN(viewFN);
			dstImageFN.SetFullName(wxT("undistorted.jpg"));	//png"));
			const std::string dstImage = std::string(dstImageFN.GetFullPath().mb_str());
				//stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory), "undistorted", "png");

			if(sfm_data.IsPoseAndIntrinsicDefined(view))
			{
				//			if(!stlplus::folder_exists(sOutViewIteratorDirectory))
				//			{
				//				stlplus::folder_create(sOutViewIteratorDirectory);
				//			}
				if(!viewFN.DirExists())
				{
#if wxCHECK_VERSION(2, 9, 0)
					if(!viewFN.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
					if(!viewFN.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
						return false;
				}

				Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
				const IntrinsicBase * cam = iterIntrinsic->second.get();
				if(cam->have_disto())
				{
					// Undistort and save the image
					ReadImage(srcImage.c_str(), &image);
					UndistortImage(image, cam, image_ud, BLACK);
					WriteImage(dstImage.c_str(), image_ud);
				}
				else // (no distortion)
				{
					// If extensions match, copy the PNG image
					if(stlplus::extension_part(srcImage) == "PNG" ||
						stlplus::extension_part(srcImage) == "png")
					{
						stlplus::file_copy(srcImage, dstImage);
					}
					else
					{
						ReadImage(srcImage.c_str(), &image);
						WriteImage(dstImage.c_str(), image);
					}
				}

				// Prepare to write an MVE 'meta.ini' file for the current view
				const Pose3 pose = sfm_data.GetPoseOrDie(view);
				const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam);

				const Mat3 rotation = pose.rotation();
				const Vec3 translation = pose.translation();
				// Pixel aspect: assuming square pixels
				const float pixelAspect = 1.f;
				// Focal length and principal point must be normalized (0..1)
				const float flen = pinhole_cam->focal() / static_cast<double>(std::max(cam->w(), cam->h()));
				const float ppX = std::abs(pinhole_cam->principal_point()(0) / cam->w());
				const float ppY = std::abs(pinhole_cam->principal_point()(1) / cam->h());

				std::ostringstream fileOut;
				fileOut << "# MVE view meta data is stored in INI-file syntax." << fileOut.widen('\n')
					<< "# This file is generated, formatting will get lost." << fileOut.widen('\n')
					<< fileOut.widen('\n')
					<< "[camera]" << fileOut.widen('\n')
					<< "focal_length = " << flen << fileOut.widen('\n')
					<< "pixel_aspect = " << pixelAspect << fileOut.widen('\n')
					<< "principal_point = " << ppX << " " << ppY << fileOut.widen('\n')
					<< "rotation = " << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " "
					<< rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " "
					<< rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << fileOut.widen('\n')
					<< "translation = " << translation[0] << " " << translation[1] << " "
					<< translation[2] << " " << fileOut.widen('\n')
					<< fileOut.widen('\n')
					<< "[view]" << fileOut.widen('\n')
					<< "id = " << view->id_view << fileOut.widen('\n')
					<< "name = " << stlplus::filename_part(srcImage.c_str()) << fileOut.widen('\n');

				// To do:  trim any extra separator(s) from openMVG name we receive, e.g.:
				// '/home/insight/openMVG_KevinCain/openMVG_Build/software/SfM/ImageDataset_SceauxCastle/images//100_7100.JPG'
				wxFileName metaIniFN(viewFN);
				metaIniFN.SetFullName(wxT("meta.ini"));

				std::ofstream file( metaIniFN.GetFullPath().mb_str() );
	//				stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory),
	//				"meta", "ini").c_str());
				file << fileOut.str();
				file.close();

				out
					<< flen << " " << "0" << " " << "0" << "\n"  // Write '0' distortion values for pre-corrected images
					<< rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << "\n"
					<< rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << "\n"
					<< rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << "\n"
					<< translation[0] << " " << translation[1] << " " << translation[2] << "\n";
			}
			else
			{
				// export a camera without pose & intrinsic info (export {0})
				// see: https://github.com/simonfuhrmann/mve/blob/952a80b0be48e820b8c72de1d3df06efc3953bd3/libs/mve/bundle_io.cc#L448
				for(int i = 0; i < 5 * 3; ++i)
					out << "0" << (i % 3 == 2 ? "\n" : " ");
				continue;
			}
			// Save a thumbnail image "thumbnail.png", 50x50 pixels
//			thumbnail = create_thumbnail(image, 50, 50);

			// Use OpenCV to create thumbnail image

			wxFileName thumbNailFN(viewFN);
			thumbNailFN.SetFullName(wxT("thumbnail.png"));

			thumbnail = OpenCVHelper::createThumbnail(image, 50, 50);
			const std::string dstThumbnailImage(thumbNailFN.GetFullPath().mb_str());
//				stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory), "thumbnail", "png");
			WriteImage(dstThumbnailImage.c_str(), thumbnail);
		}

	  // For each feature, write to bundle:  position XYZ[0-3], color RGB[0-2], all ref.view_id & ref.feature_id
	  // The following method is adapted from Simon Fuhrmann's MVE project:
	  // https://github.com/simonfuhrmann/mve/blob/e3db7bc60ce93fe51702ba77ef480e151f927c23/libs/mve/bundle_io.cc

	  for(Landmarks::const_iterator iterLandmarks = landmarks.begin(); iterLandmarks != landmarks.end(); ++iterLandmarks)
	  {
		  const Vec3 exportPoint = iterLandmarks->second.X;
		  out << exportPoint.x() << " " << exportPoint.y() << " " << exportPoint.z() << "\n";
		  out << 250 << " " << 100 << " " << 150 << "\n";  // Write arbitrary RGB color, see above note

		  // Tally set of feature observations
		  const Observations & obs = iterLandmarks->second.obs;
		  const size_t featureCount = std::distance(obs.begin(), obs.end());
		  out << featureCount;

		  for(Observations::const_iterator itObs = obs.begin(); itObs != obs.end(); ++itObs)
		  {
			  const IndexT viewId = itObs->first;
			  const IndexT featId = itObs->second.id_feat;
			  out << " " << viewId << " " << featId << " 0";
		  }
		  out << "\n";
	  }
	  out.close();
  }
  return bOk;
}

// The following code is copied mostly from openMVG's main_openMVG2MVSTEXTURING.cpp and slightly adjusted.
// Original copyright follows.

// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

bool OpenMVGHelper::exportToMVSTexturing(const openMVG::sfm::SfM_Data & sfm_data, const wxString &sOutDirectory)
{
	wxFileName outDir(sOutDirectory, "");
	if(!outDir.DirExists())
	{
#if wxCHECK_VERSION(2, 9, 0)
		if(!outDir.Mkdir(wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
#else
		if(!outDir.Mkdir(0777, wxPATH_MKDIR_FULL))
#endif
			return false;
	}
	std::string sOutDir = std::string(sOutDirectory.mb_str());

	for(Views::const_iterator iter = sfm_data.GetViews().begin();
		iter != sfm_data.GetViews().end(); ++iter)
	{
		const View * view = iter->second.get();
		if(!sfm_data.IsPoseAndIntrinsicDefined(view))
			continue;

		// Valid view, we can ask a pose & intrinsic data
		const Pose3 pose = sfm_data.GetPoseOrDie(view);
		Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
		const IntrinsicBase * cam = iterIntrinsic->second.get();

		if(!cameras::isPinhole(cam->getType()))
			continue;
		const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam);

		// Extrinsic
		const Vec3 t = pose.translation();
		const Mat3 R = pose.rotation();
		// Intrinsic
		const double f = pinhole_cam->focal();
		const Vec2 pp = pinhole_cam->principal_point();

		// Image size in px
		const int w = pinhole_cam->w();
		const int h = pinhole_cam->h();

		// We can now create the .cam file for the View in the output dir 
		std::ofstream outfile(stlplus::create_filespec(
			sOutDir, stlplus::basename_part(view->s_Img_path), "cam").c_str());
/*		wxFileName imgFN(wxString(view->s_Img_path.c_str(), wxConvLibc));
		wxFileName outfileFN(outDir);
		outfileFN.SetName(imgFN.GetName());
		outfileFN.SetExt(wxT("cam"));

		std::ostringstream outfile;*/
		// See https://github.com/nmoehrle/mvs-texturing/blob/master/Arguments.cpp
		// for full specs
		const int largerDim = w > h ? w : h;
		outfile << t(0) << " " << t(1) << " " << t(2) << " "
			<< R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
			<< R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
			<< R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n"
			<< f / largerDim << " 0 0 1 " << pp(0) / w << " " << pp(1) / h;
		outfile.close();

//		if(cam->have_disto())
//			bOneHaveDisto = true;
	}

	return true;
}

#endif
