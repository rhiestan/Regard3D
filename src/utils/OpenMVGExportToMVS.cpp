/**
 * Copyright (C) 2017 Roman Hiestand
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
#include "OpenMVGExportToMVS.h"
#include "OpenMVGHelper.h"

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/image/image_container.hpp"
#include "openMVG/image/pixel_types.hpp"
#include "openMVG/image/image_io.hpp"

//#include "opencv2/core/matx.hpp"

#define _USE_EIGEN
//#define _USE_OPENCV
#include "openMVG/software/SfM/InterfaceMVS.h"

//#include <opencv2/core/eigen.hpp>

#include "third_party/progress/progress.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

#include <wx/ffile.h>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;

// Code based on main_openMVG2openMVS.cpp
// This method is in a separate file because it does not compile when OpenCV header files are included.
bool OpenMVGExportToMVS::exportToOpenMVS(R3DProject::Triangulation *pTriangulation, const wxString &pathname)
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

	std::string sOutDir(outDir.GetFullPath().mbc_str());
	wxFileName outFileFN(outDir);
	outFileFN.SetFullName(wxT("scene.mvs"));
	std::string sOutFile(outFileFN.GetFullPath().mbc_str());

	R3DProject *pProject = R3DProject::getInstance();
	R3DProjectPaths paths;
	pProject->getProjectPathsTri(paths, pTriangulation);
	bool hasSfmData = OpenMVGHelper::hasMatchesSfM_DataFile(paths);

	// Load OpenMVG document/Sfm_Data file
	std::string sSfMDir = paths.relativeSfmOutPath_;
	Document doc;
	openMVG::sfm::SfM_Data sfm_data;
	size_t numberOfCalibratedViews = 0;
	if(hasSfmData)
	{
		if(!Load(sfm_data, paths.relativeTriSfmDataFilename_, ESfM_Data(ALL)))
			return false;
		numberOfCalibratedViews = sfm_data.GetViews().size();
	}
	else
	{
		if(!doc.load(sSfMDir))
			return false;
		numberOfCalibratedViews = doc._map_camera.size();
	}
  // Export data :
  MVS::Interface scene;
  size_t nPoses(0);
  const uint32_t nViews((uint32_t)sfm_data.GetViews().size());

  C_Progress_display my_progress_bar(nViews);

  // OpenMVG can have not contiguous index, use a map to create the required OpenMVS contiguous ID index
  std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

  // define a platform with all the intrinsic group
  for (const auto& intrinsic: sfm_data.GetIntrinsics())
  {
    if (isPinhole(intrinsic.second.get()->getType()))
    {
      const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());
      if (map_intrinsic.count(intrinsic.first) == 0)
        map_intrinsic.insert(std::make_pair(intrinsic.first, scene.platforms.size()));
      MVS::Interface::Platform platform;
      // add the camera
      MVS::Interface::Platform::Camera camera;
      camera.K = cam->K();
	  //cv::eigen2cv(cam->K(), camera.K);
      // sub-pose
      camera.R = Mat3::Identity();
	  //cv::eigen2cv(Mat3::Identity(), camera.R);
      camera.C = Vec3::Zero();
	  //cv::eigen2cv(Vec3::Zero(), camera.C);
      platform.cameras.push_back(camera);
      scene.platforms.push_back(platform);
    }
  }

  // define images & poses
  scene.images.reserve(nViews);
  for (const auto& view : sfm_data.GetViews())
  {
    map_view[view.first] = scene.images.size();
    MVS::Interface::Image image;
    const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view.second->s_Img_path);
    image.name = stlplus::create_filespec(sOutDir, view.second->s_Img_path);
    image.platformID = map_intrinsic.at(view.second->id_intrinsic);
    MVS::Interface::Platform& platform = scene.platforms[image.platformID];
    image.cameraID = 0;
    if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()) && stlplus::is_file(srcImage))
    {
      MVS::Interface::Platform::Pose pose;
      image.poseID = platform.poses.size();
      const openMVG::geometry::Pose3 poseMVG(sfm_data.GetPoseOrDie(view.second.get()));
      pose.R = poseMVG.rotation();
	  //cv::eigen2cv(poseMVG.rotation(), pose.R);
      pose.C = poseMVG.center();
	  //cv::eigen2cv(poseMVG.center(), pose.C);
      // export undistorted images
      const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view.second->id_intrinsic).get();
      if (cam->have_disto())
      {
        // undistort image and save it
        Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
        ReadImage(srcImage.c_str(), &imageRGB);
        UndistortImage(imageRGB, cam, imageRGB_ud, BLACK);
        WriteImage(image.name.c_str(), imageRGB_ud);
      }
      else
      {
        // just copy image
        stlplus::file_copy(srcImage, image.name);
      }
      platform.poses.push_back(pose);
      ++nPoses;
    }
    else
    {
      // image have not valid pose, so set an undefined pose
      image.poseID = NO_ID;
      // just copy the image
      stlplus::file_copy(srcImage, image.name);
    }
    scene.images.push_back(image);
    ++my_progress_bar;
  }

  // define structure
  scene.vertices.reserve(sfm_data.GetLandmarks().size());
  for (const auto& vertex: sfm_data.GetLandmarks())
  {
    const Landmark & landmark = vertex.second;
    MVS::Interface::Vertex vert;
    MVS::Interface::Vertex::ViewArr& views = vert.views;
    for (const auto& observation: landmark.obs)
    {
      const auto it(map_view.find(observation.first));
      if (it != map_view.end()) {
        MVS::Interface::Vertex::View view;
        view.imageID = it->second;
        view.confidence = 0;
        views.push_back(view);
      }
    }
    if (views.size() < 2)
      continue;
    std::sort(
      views.begin(), views.end(),
      [] (const MVS::Interface::Vertex::View& view0, const MVS::Interface::Vertex::View& view1)
      {
        return view0.imageID < view1.imageID;
      }
    );
    vert.X = landmark.X.cast<float>();
	//cv::eigen2cv(landmark.X.cast<float>(), vert.X);
    scene.vertices.push_back(vert);
  }

  // normalize camera intrinsics
  for (size_t p=0; p<scene.platforms.size(); ++p)
  {
    MVS::Interface::Platform& platform = scene.platforms[p];
    for (size_t c=0; c<platform.cameras.size(); ++c) {
      MVS::Interface::Platform::Camera& camera = platform.cameras[c];
      // find one image using this camera
      MVS::Interface::Image* pImage(NULL);
      for (MVS::Interface::Image& image: scene.images)
      {
	      if (image.platformID == p && image.cameraID == c && image.poseID != NO_ID)
	      {
		      pImage = &image;
		      break;
	      }
      }
      if (pImage == NULL)
      {
	      std::cerr << "error: no image using camera " << c << " of platform " << p << std::endl;
	      continue;
      }
      // read image meta-data
      ImageHeader imageHeader;
      ReadImageHeader(pImage->name.c_str(), &imageHeader);
      const double fScale(1.0/std::max(imageHeader.width, imageHeader.height));
      camera.K(0, 0) *= fScale;
      camera.K(1, 1) *= fScale;
      camera.K(0, 2) *= fScale;
      camera.K(1, 2) *= fScale;
    }
  }

  // write OpenMVS data
  if (!MVS::ARCHIVE::SerializeSave(scene, sOutFile))
  {
	  MLOG << "Could not write scene.mvs" << std::endl;
    return false;
  }

  MLOG
    << "Scene saved to OpenMVS interface format:\n"
    << "\t" << scene.images.size() << " images (" << nPoses << " calibrated)\n"
    << "\t" << scene.vertices.size() << " Landmarks\n";
  return true;
}
