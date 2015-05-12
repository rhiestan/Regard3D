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

#include <memory>
#include <sstream>
#include <cmath>
#include <iterator>
#include <limits>
#include <float.h>
#include <set>

using namespace std;

// OpenMVG
#include "openMVG/multiview/projection.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "third_party/vectorGraphics/svgDrawer.hpp"
using namespace svg;
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/indexed_sort.hpp"
#include "third_party/progress/progress.hpp"

// stlpus
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;


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
		std::vector<SIOPointFeature> vec_feat;
		loadFeatsFromFile(
			stlplus::create_filespec(sOutDir, stlplus::basename_part(iterFilename->m_sImageName), ".feat"),
			vec_feat);

		//-- Draw features
		for (size_t i=0; i< vec_feat.size(); ++i)  {
			const SIOPointFeature & feature = vec_feat[i];
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
  //---------------------------------------
  // Read matches
  //---------------------------------------

  openMVG::matching::PairWiseMatches map_Matches;
  openMVG::matching::PairedIndMatchImport(sMatchFile, map_Matches);

  // ------------
  // For each pair, export the matches
  // ------------

  stlplus::folder_create(sOutDir);
  for (openMVG::matching::PairWiseMatches::const_iterator iter = map_Matches.begin();
    iter != map_Matches.end();
    ++iter)
  {
    const size_t I = iter->first.first;
    const size_t J = iter->first.second;

    std::vector<SfMIO::CameraInfo>::const_iterator camInfoI = vec_camImageName.begin() + I;
    std::vector<SfMIO::CameraInfo>::const_iterator camInfoJ = vec_camImageName.begin() + J;

    const std::pair<size_t, size_t>
      dimImage0 = std::make_pair(vec_focalGroup[camInfoI->m_intrinsicId].m_w, vec_focalGroup[camInfoI->m_intrinsicId].m_h),
      dimImage1 = std::make_pair(vec_focalGroup[camInfoJ->m_intrinsicId].m_w, vec_focalGroup[camInfoJ->m_intrinsicId].m_h);

    svgDrawer svgStream( dimImage0.first + dimImage1.first, max(dimImage0.second, dimImage1.second));
    svgStream.drawImage(stlplus::create_filespec(sImaDirectory,vec_camImageName[I].m_sImageName),
      dimImage0.first,
      dimImage0.second);
    svgStream.drawImage(stlplus::create_filespec(sImaDirectory,vec_camImageName[J].m_sImageName),
      dimImage1.first,
      dimImage1.second, dimImage0.first);

    const vector<openMVG::matching::IndMatch> & vec_FilteredMatches = iter->second;

    if (!vec_FilteredMatches.empty()) {
      // Load the features from the features files
      std::vector<SIOPointFeature> vec_featI, vec_featJ;
      loadFeatsFromFile(
        stlplus::create_filespec(sMatchesDir, stlplus::basename_part(vec_camImageName[I].m_sImageName), ".feat"),
        vec_featI);
      loadFeatsFromFile(
        stlplus::create_filespec(sMatchesDir, stlplus::basename_part(vec_camImageName[J].m_sImageName), ".feat"),
        vec_featJ);

      //-- Draw link between features :
      for (size_t i=0; i< vec_FilteredMatches.size(); ++i)  {
        const SIOPointFeature & imaA = vec_featI[vec_FilteredMatches[i]._i];
        const SIOPointFeature & imaB = vec_featJ[vec_FilteredMatches[i]._j];
        svgStream.drawLine(imaA.x(), imaA.y(),
          imaB.x()+dimImage0.first, imaB.y(), svgStyle().stroke("green", 2.0));
      }

      //-- Draw features (in two loop, in order to have the features upper the link, svg layer order):
      for (size_t i=0; i< vec_FilteredMatches.size(); ++i)  {
        const SIOPointFeature & imaA = vec_featI[vec_FilteredMatches[i]._i];
        const SIOPointFeature & imaB = vec_featJ[vec_FilteredMatches[i]._j];
        svgStream.drawCircle(imaA.x(), imaA.y(), imaA.scale(),
          svgStyle().stroke("yellow", 2.0));
        svgStream.drawCircle(imaB.x() + dimImage0.first, imaB.y(), imaB.scale(),
          svgStyle().stroke("yellow", 2.0));
      }
    }
    std::ostringstream os;
    os << stlplus::folder_append_separator(sOutDir)
      << iter->first.first << "_" << iter->first.second
      << "_" << iter->second.size() << "_.svg";
    ofstream svgFile( os.str().c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }
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
}

// Equality functor to count the number of similar K matrices in the essential matrix case.
static inline bool testIntrinsicsEquality(
  SfMIO::IntrinsicCameraInfo const &ci1,
  SfMIO::IntrinsicCameraInfo const &ci2)
{
  return ci1.m_K == ci2.m_K;
}

bool OpenMVGHelper::isGlobalSfmAvailable(const R3DProjectPaths &paths)
{
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
    Image<openMVG::RGBColor> image;
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

    for (std::map< size_t, tracks::submapTrack >::const_iterator
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

  Document m_doc;
  if (m_doc.load(sSfMDir))
  {
/*    exportToPMVSFormat(m_doc,
      stlplus::folder_append_separator(sOutDir) + "PMVS",
      stlplus::folder_append_separator(sSfMDir) + "images",
      pDensification );

    exportToBundlerFormat(m_doc,
      stlplus::folder_append_separator(sOutDir) +
      stlplus::folder_append_separator("PMVS") + "bundle.rd.out",
      stlplus::folder_append_separator(sOutDir) +
      stlplus::folder_append_separator("PMVS") + "list.txt"
      );*/
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

	// Load OpenMVG document
	std::string sSfMDir = paths.relativeSfmOutPath_;
	Document m_doc;
	if(!m_doc.load(sSfMDir))
		return false;

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

	std::map<size_t, PinholeCamera >::const_iterator iterCamera = m_doc._map_camera.begin();
	std::map<size_t, std::pair<size_t,size_t> >::const_iterator iterSize = m_doc._map_imageSize.begin();
	std::vector<std::string>::const_iterator iterName = m_doc._vec_imageNames.begin();
	for ( ;
		iterCamera != m_doc._map_camera.end();
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
/*		std::string soffsetImagePath =  stlplus::create_filespec( sSfM_OutputPath, "imagesOffset" );
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

	iterName = m_doc._vec_imageNames.begin();
	for ( ;
		iterName != m_doc._vec_imageNames.end();
		iterName++ )
	{
		wxString curName( iterName->c_str(), wxConvLibc );
		undistImageFN.SetFullName(curName);
		destImageFN.SetFullName(curName);

		if(!wxCopyFile(undistImageFN.GetFullPath(), destImageFN.GetFullPath(), false))
			return false;
	}

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

	// Load OpenMVG document
	std::string sSfMDir = paths.relativeSfmOutPath_;
	Document doc;
	if(!doc.load(sSfMDir))
		return false;

	wxFileName cmpmvsOutDir(outDir);
	cmpmvsOutDir.AppendDir(wxT("CMPMVS"));
	if(!cmpmvsOutDir.DirExists())
	{
		if(!cmpmvsOutDir.Mkdir())
			return false;
	}

	// Create file for MeshRecon, see http://www-scf.usc.edu/~zkang/software.html
	std::ostringstream sfm_out;
	sfm_out << doc._map_camera.size() << std::endl << std::endl;
	std::map<size_t, std::pair<size_t,size_t> >::const_iterator iterSize = doc._map_imageSize.begin();
	std::vector<std::string>::const_iterator iterName = doc._vec_imageNames.begin();

	// Export cameras
	size_t count = 1;
	for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
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
			<< PMat.row(0) <<"\n"<< PMat.row(1) <<"\n"<< PMat.row(2) << std::endl;
//		file.close();

		wxFileName cameraFN(cmpmvsOutDir);
		cameraFN.SetName( wxString( os.str().c_str(), wxConvLibc ) );
		cameraFN.SetExt( wxT("txt") );
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


	// MeshRecon: Bounding box, image neighbours
	sfm_out << std::endl;

	std::vector< std::set<size_t> > image_neighbours;
	image_neighbours.resize(doc._map_camera.size());

	float minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX, minZ = FLT_MAX, maxZ = -FLT_MAX;
	int trackIndex = 0;
	for (std::map< size_t, tracks::submapTrack >::const_iterator
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
	sfm_out << minX << " " << maxX << " " << minY << " " << maxY << " " << minZ << " " << maxZ << std::endl << std::endl;
/*	
	size_t numCams = doc._map_camera.size();
	for(size_t i = 0; i < numCams; i++)
	{
		sfm_out << i << " " << numCams - 1;
		for(size_t j = 0; j < numCams; j++)
			if(i != j)
				sfm_out << " " << j;
		sfm_out << std::endl;
	}*/
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
	wxFileName undistImageFN(wxString(paths.relativeSfmOutPath_.c_str(), wxConvLibc), wxT(""));
	undistImageFN.AppendDir(wxT("images"));
	wxFileName destImageFN(cmpmvsOutDir);

	count = 1;
	//Image<RGBColor> image;
	for (std::map<size_t, PinholeCamera>::const_iterator iter = doc._map_camera.begin();
		iter != doc._map_camera.end();  ++iter, ++count)
	{
		size_t imageIndex = iter->first;
		const std::string & sImageName = doc._vec_imageNames[imageIndex];
		std::ostringstream os;
		os << std::setw(5) << std::setfill('0') << count;
/*		ReadImage( stlplus::create_filespec( sImagePath, sImageName).c_str(), &image );
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

	// Determine width and height of the largest image. CMPMVS scales the smaller images accordingly.
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
			<< "ncams=" << doc._map_camera.size() << std::endl
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
			<< "ncams=" << doc._map_camera.size() << std::endl
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

	return true;
}
