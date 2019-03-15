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
#ifndef OPENMVGHELPER_H
#define OPENMVGHELPER_H

#include <string>
#include <vector>

#include "R3DProject.h"

// OpenMVG
#include "software/SfM/SfMIOHelper.hpp"
#include "software/SfMViewer/document.h"
#if !defined(R3D_USE_OPENMVG_PRE08)
#include "openMVG/sfm/sfm_data.hpp"
#endif

class OpenMVGHelper
{
public:
	static void exportKeypoints(const std::vector<openMVG::SfMIO::CameraInfo> &vec_camImageName,
		const std::vector<openMVG::SfMIO::IntrinsicCameraInfo> &vec_focalGroup,
		const std::string &sImaDirectory,
		const std::string &sOutDir);
	static void exportMatches(const std::vector<openMVG::SfMIO::CameraInfo> &vec_camImageName,
		const std::vector<openMVG::SfMIO::IntrinsicCameraInfo> &vec_focalGroup,
		const std::string &sImaDirectory,
		const std::string &sMatchesDir,
		const std::string &sOutDir,
		const std::string &sMatchFile);


	struct ImagePair
	{
		size_t indexA_, indexB_;
		std::string imageFilenameA_, imageFilenameB_;
		size_t matches_;
	};
	typedef std::vector<ImagePair> ImagePairList;

	static ImagePairList getBestValidatedPairs(const R3DProjectPaths &paths, const std::string &matchesfilename, int pairCount = 10);

	static bool isGlobalSfmAvailable(const R3DProjectPaths &paths, R3DProject::PictureSet *pPictureSet);

	static bool hasMatchesSfM_DataFile(const R3DProjectPaths &paths);
	static bool hasTriSfM_DataFile(const R3DProjectPaths &paths);

	static bool exportToPMVS(R3DProject::Densification *pDensification);

	static bool exportToMeshLab(R3DProject::Densification *pDensification, const wxString &pathname);

	static bool exportToExternalMVS(R3DProject::Triangulation *pTriangulation, const wxString &pathname);

	static bool exportToSure(const openMVG::sfm::SfM_Data &sfm_data, const wxString &pathname);

	static bool exportToMVMPR(const openMVG::sfm::SfM_Data &sfm_data, const wxString &pathname);

#if !defined(R3D_USE_OPENMVG_PRE08)
	static void ColorizeTracks(const openMVG::sfm::SfM_Data &sfm_data, std::vector<Vec3> &vec_3dPoints, std::vector<Vec3> &vec_tracksColor);
	static void GetCameraPositions(const openMVG::sfm::SfM_Data &sfm_data, std::vector<Vec3> &vec_camPosition);

	static void calculateResiduals(const openMVG::sfm::SfM_Data &sfm_data, std::vector<float> &residuals);

	static void exportOldSfM_output(const R3DProjectPaths &paths);

	static bool exportToMVE2Format(const openMVG::sfm::SfM_Data & sfm_data, const wxString &sOutDirectory);

	static bool exportToMVSTexturing(const openMVG::sfm::SfM_Data & sfm_data, const wxString &sOutDirectory);

#endif

private:
	OpenMVGHelper() { }
	~OpenMVGHelper() { }

	// For triangulations created with openMVG 0.7
	static bool exportToPMVSFormat(const Document & doc,
		const std::string & sOutDirectory,
		const std::string & sImagePath,
		R3DProject::Densification *pDensification);

	static bool exportToBundlerFormat(const Document & doc,
		const std::string & sOutFile,
		const std::string & sOutListFile);

	// For triangulations created with openMVG 0.8
#if !defined(R3D_USE_OPENMVG_PRE08)
	static bool exportToPMVSFormat(
		const openMVG::sfm::SfM_Data & sfm_data,
		const std::string & sOutDirectory,
		R3DProject::Densification *pDensification);

	static bool exportToBundlerFormat(
		const openMVG::sfm::SfM_Data & sfm_data,
		const std::string & sOutFile,
		const std::string & sOutListFile);
#endif
};

#endif
