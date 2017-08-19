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
#include "R3DModelOperations.h"

#if !wxCHECK_VERSION(2, 9, 0)
// wxWidgets before 2.9.0 does not support recursive delete
// Use boost::filesystem instead
#include <boost/filesystem.hpp>
#endif

// PointCloudLibrary
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
#include <pcl/io/vtk_lib_io.h>
#endif

#include <assimp/postprocess.h>
#include <assimp/version.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/Exporter.hpp>

void R3DModelOperations::combineDenseModels(R3DProject::Densification *pDensification, int numModels)
{
	wxString combinedModelName(wxT("pmvs_combined.ply"));
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsDns(paths, pDensification);

	wxFileName modelFN(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc));
	pcl::PointCloud<pcl::PointXYZRGBNormal> combinedCloud;
	for(int i = 0; i < numModels; i++)
	{
		modelFN.SetFullName(wxString::Format(wxT("option-%04d.ply"), i));
		if(modelFN.FileExists())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal> tempCloud;
			if(pcl::io::loadPLYFile(std::string(modelFN.GetFullPath().mb_str()), tempCloud) != -1)
			{
				combinedCloud += tempCloud;		// Concatenate new point cloud
			}
		}
	}

	// Save combined point cloud
	wxFileName combinedModelFN(modelFN);
	combinedModelFN.SetFullName(combinedModelName);
	pcl::io::savePLYFileASCII(std::string(combinedModelFN.GetFullPath().mb_str()), combinedCloud);
	pDensification->finalDenseModelName_ = combinedModelName;
}

void R3DModelOperations::colorizeSurface(R3DProject::Surface *pSurface)
{
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsSrf(paths, pSurface);

	// Load surface model
	wxString surfaceFilename(pSurface->finalSurfaceFilename_);
	wxFileName surfaceFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), surfaceFilename);
	if(!surfaceFN.FileExists())
		return;

	pcl::PolygonMesh surfaceModel;
#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	if(pcl::io::loadPolygonFilePLY(std::string(surfaceFN.GetFullPath().mb_str()), surfaceModel) == -1)
#else
	if(pcl::io::loadPLYFile(std::string(surfaceFN.GetFullPath().mb_str()), surfaceModel) == -1)
#endif
		return;
	if(surfaceModel.polygons.empty())
	{
		// TODO: Issue warning
		// Empty surface (will crash in pcl::fromPCLPointCloud2 if not caught here)
		return;
	}

	// Load dense point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(paths.relativeDenseModelName_, *(cloud.get())) == -1)
		return;

	// Put point cloud into search structure
	//pcl::search::BruteForce<pcl::PointXYZRGB> search;
	pcl::search::KdTree<pcl::PointXYZRGB> search;
	search.setInputCloud(cloud);

	// Convert points from surface to pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZRGB> surfacePoints;
	pcl::fromPCLPointCloud2(surfaceModel.cloud, surfacePoints);

	const int n_neighbours = pSurface->colVertNumNeighbours_;
	std::vector<int> k_indices;
	std::vector<float> k_distances, k_weights;
	for(size_t i = 0; i < surfacePoints.points.size(); i++)
	{
		k_indices.clear();
		k_distances.clear();
		k_weights.clear();
		pcl::PointXYZRGB &point = surfacePoints.points[i];

		int retVal = search.nearestKSearch(point, n_neighbours, k_indices, k_distances);
		
		// Copy color
		if(!k_indices.empty())
		{
			k_weights.resize(k_distances.size());
			if(k_distances[0] == 0.0f)
			{
				// Avoid division by zero
				k_weights[0] = 1.0f;
				for(size_t j = 1; j < k_distances.size(); j++)
					k_weights[j] = 0;
			}
			else
			{
				float totalweight = 0;
				for(size_t j = 0; j < k_distances.size(); j++)
					totalweight += (1.0f / k_distances[j]);
				for(size_t j = 0; j < k_distances.size(); j++)
					k_weights[j] = (1.0f / k_distances[j]) / totalweight;
			}

			float redf = 0, greenf = 0, bluef = 0;
			for(size_t j = 0; j < k_indices.size(); j++)
			{
				const pcl::PointXYZRGB &nearestPoint = (cloud->points)[k_indices[j]];
				redf += k_weights[j] * static_cast<float>(nearestPoint.r);
				greenf += k_weights[j] * static_cast<float>(nearestPoint.g);
				bluef += k_weights[j] * static_cast<float>(nearestPoint.b);
			}
			point.r = static_cast<uint8_t>(std::min(255, static_cast<int>(redf)));
			point.g = static_cast<uint8_t>(std::min(255, static_cast<int>(greenf)));
			point.b = static_cast<uint8_t>(std::min(255, static_cast<int>(bluef)));
			point.a = 0;
		}
	}

	// Convert points back into surface model
	pcl::toPCLPointCloud2(surfacePoints, surfaceModel.cloud);

	// Save colorized model
	wxString colorizedModelName(wxT("model_surface_col.ply"));
	pSurface->finalSurfaceFilename_ = colorizedModelName;
	wxFileName surfaceOutFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), colorizedModelName);
	pcl::io::savePLYFile(std::string(surfaceOutFN.GetFullPath().mb_str()), surfaceModel);
}

bool R3DModelOperations::exportToPointCloud(R3DProject::Densification *pDensification, const wxString &filename)
{
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsDns(paths, pDensification);
	bool retVal = true;

	// Determine export type
	wxFileName exportFN(filename);
	if(exportFN.GetExt().IsSameAs(wxT("pcd"), false))
	{
		// Convert to Point Cloud Data
		pcl::PCLPointCloud2 cloud;
		if(pcl::io::loadPLYFile(paths.relativeDenseModelName_, cloud) == -1)
			return false;

		// Make temporary directory (directly under the project dir)
		wxFileName tmpDir(wxT("tmp"), wxEmptyString);
		if(tmpDir.DirExists())
#if wxCHECK_VERSION(2, 9, 0)
			tmpDir.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
		{
			boost::system::error_code ec;
			boost::filesystem::remove_all(boost::filesystem::path(tmpDir.GetPath().c_str()), ec);
		}
#endif
		if(!tmpDir.Mkdir())
			return false;

		std::string tmpfn("tmp/temp_surface.pcd");
		if(pcl::io::savePCDFile(tmpfn, cloud) != 0)
			retVal = false;

		wxCopyFile(wxString(tmpfn.c_str(), wxConvLibc), filename, true);

#if wxCHECK_VERSION(2, 9, 0)
		tmpDir.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
		boost::system::error_code ec;
		boost::filesystem::remove_all(boost::filesystem::path(tmpDir.GetPath().c_str()), ec);
#endif
	}
	else
	{
		// Simple copy
		return wxCopyFile(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc), filename, true);
	}

	return retVal;
}

bool R3DModelOperations::exportSurface(R3DProject::Surface *pSurface, const wxString &filename)
{
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsSrf(paths, pSurface);

	wxFileName surfaceModelFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), pSurface->finalSurfaceFilename_);
	std::string surfaceModelFNStr(surfaceModelFN.GetFullPath().mb_str());

	// Make temporary directory (directly under the project dir)
	wxFileName tmpDir(wxT("tmp"), wxEmptyString);
	if(tmpDir.DirExists())
#if wxCHECK_VERSION(2, 9, 0)
		tmpDir.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	{
		boost::system::error_code ec;
		boost::filesystem::remove_all(boost::filesystem::path(tmpDir.GetPath().c_str()), ec);
	}
#endif
	if(!tmpDir.Mkdir())
		return false;

	bool retVal = true;

	Assimp::Importer imp;
	Assimp::Exporter exp;

	// Determine original surface file type
	wxFileName exportFN(filename);
	if(surfaceModelFN.GetExt().IsSameAs(wxT("ply"), false))
	{
		if(exportFN.GetExt().IsSameAs(wxT("obj")))
		{
			// Convert to Alias Wavefront Object
			if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
			{
				pcl::PolygonMesh mesh;
#if PCL_VERSION_COMPARE(<, 1, 7, 2)
				if(pcl::io::loadPolygonFilePLY(surfaceModelFNStr, mesh) == -1)
#else
				if(pcl::io::loadPLYFile(surfaceModelFNStr, mesh) == -1)
#endif
					retVal = false;

//				const aiScene* pScene = imp.ReadFile(surfaceModelFNStr.c_str(), aiProcess_JoinIdenticalVertices);
//				if(pScene == NULL)
//					retVal = false;
//				else
				{
					std::string tmpfn("tmp/temp_surface.obj");
					retVal &= (pcl::io::saveOBJFile(tmpfn, mesh, 7) == 0);
					//const aiReturn res = exp.Export(pScene, "obj", tmpfn.c_str());
					retVal &= wxCopyFile(wxString(tmpfn.c_str(), wxConvLibc), filename, true);
				}
			}
			else
			{
				// PointCloudLibrary/AssImp does not support textures in PLY files
				retVal = false;
			}
		}
		else
		{
			// Copy file
			retVal = wxCopyFile(surfaceModelFN.GetFullPath(), filename, true);
		}
	}
	else if(surfaceModelFN.GetExt().IsSameAs(wxT("obj"), false))
	{
		if(exportFN.GetExt().IsSameAs(wxT("ply")))
		{
			if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
			{
/*				pcl::PolygonMesh mesh;
#if PCL_VERSION_COMPARE(<, 1, 7, 2)
				if(pcl::io::loadPolygonFileOBJ(surfaceModelFNStr, mesh) == -1)
#else
				if(pcl::io::loadOBJFile(surfaceModelFNStr, mesh) == -1)
#endif
					retVal = false;*/

				const aiScene* pScene = imp.ReadFile(surfaceModelFNStr.c_str(), aiProcess_JoinIdenticalVertices);
				if(pScene == NULL)
					retVal = false;
				else
				{
					std::string fn(filename.mb_str());
					//retVal = (pcl::io::savePLYFile(fn, mesh) == 0);
					const aiReturn res = exp.Export(pScene, "ply", fn.c_str());
				}
			}
			else
			{
				// PointCloudLibrary does not support textures in PLY files
				retVal = false;
			}
		}
		else
		{
			// Load and save OBJ
/*			pcl::TextureMesh mesh;
#if PCL_VERSION_COMPARE(<, 1, 7, 2)
			if(pcl::io::loadPolygonFileOBJ(surfaceModelFNStr, mesh) == -1)
#else
			if(pcl::io::loadOBJFile(surfaceModelFNStr, mesh) == -1)
#endif
				retVal = false;*/
			const aiScene* pScene = imp.ReadFile(surfaceModelFNStr.c_str(), aiProcess_JoinIdenticalVertices);
			if(pScene == NULL)
				retVal = false;
			else
			{
				std::string exportNameC(exportFN.GetName().ToAscii());		// Converted to ASCII
				wxString exportNameASCII(wxString::FromAscii(exportNameC.c_str()));

				wxFileName tempFN(wxT("tmp"), exportNameASCII, wxT("obj"));
				std::string tmpfn(std::string( tempFN.GetFullPath(wxPATH_UNIX).ToAscii() ));
				const aiReturn res = exp.Export(pScene, "obj", tmpfn.c_str());
				retVal &= wxCopyFile(tempFN.GetFullPath(), filename, true);

				// Problem: We would like to eventually have 3 files:
				// - Model file with name filename
				// - Material file with same or similar name, but .mtl extension
				// - Texture file with same or similar name (PNG format)
				// This is a problem on Windows, since the pcl::io methods/AssImp only accept
				// C-Strings (instead of Unicode) and the name of the material file is stored
				// in the OBJ file.
				// It is also unclear how the textual representation of the material file name
				// and the texture file name is interpreted, since the OBJ and the material files
				// are text files.
				// Workaround: map name to ASCII as good as possble, use it to store
				// OBJ file and material file. Rename OBJ file to user requested filename.
				wxFileName materialInFN(tempFN), materialOutFN(filename);
				materialInFN.SetExt(wxT("mtl"));
				materialOutFN.SetName( exportNameASCII );
				materialOutFN.SetExt(wxT("mtl"));
				retVal &= wxCopyFile(materialInFN.GetFullPath(), materialOutFN.GetFullPath(), true);

				bool hasTexture = false;
				for(int i = 0; i < static_cast<int>(pScene->mNumMeshes); i++)
				{
					aiMesh *pMesh = pScene->mMeshes[i];
					if(pMesh != NULL)
					{
						if(pMesh->GetNumUVChannels() > 0
							&& pMesh->HasTextureCoords(0))
							hasTexture = true;

						if(hasTexture)
						{
							unsigned int usedMaterial = pMesh->mMaterialIndex;
							if(usedMaterial < pScene->mNumMaterials)
							{
								const aiMaterial *pMaterial = pScene->mMaterials[usedMaterial];
								unsigned int textureCount = pMaterial->GetTextureCount(aiTextureType_DIFFUSE);
								for(unsigned int j = 0; j < textureCount; j++)
								{
									aiString path;
									aiTextureMapping mapping;
									unsigned int uvindex;
									float blend;
									aiTextureOp op;
									aiTextureMapMode mapmode[3];
									pMaterial->GetTexture(aiTextureType_DIFFUSE, j, &path, &mapping, &uvindex, &blend, &op, mapmode);

									wxFileName textureFromFN(surfaceModelFN), textureToFN(filename);
									textureFromFN.SetFullName( wxString( path.C_Str(), wxConvUTF8 ) );
									textureToFN.SetFullName( wxString( path.C_Str(), wxConvUTF8 ) );
									retVal &= wxCopyFile(textureFromFN.GetFullPath(), textureToFN.GetFullPath(), true);
								}
							}
						}
					}
				}
			}

/*			std::string exportNameC(exportFN.GetName().ToAscii());		// Converted to ASCII
			wxString exportNameASCII(wxString::FromAscii(exportNameC.c_str()));

			// Copy texture files
			for(size_t i = 0; i < mesh.tex_materials.size(); i++)
			{
				pcl::TexMaterial &mat = mesh.tex_materials[i];
				std::string textureFilename = mat.tex_file;

				wxFileName textureInFN(wxString(textureFilename.c_str(), wxConvLibc));
				wxFileName textureOutFN(exportFN);
				textureOutFN.SetName( exportNameASCII + wxString::Format( wxT("_%d"), i ) );
				textureOutFN.SetExt( textureInFN.GetExt() );
				if(!wxCopyFile(textureInFN.GetFullPath(), textureOutFN.GetFullPath(), true))
					retVal = false;

				// Remove path from filename
				mat.tex_file = std::string(textureOutFN.GetFullName().mb_str());
			}

			wxFileName tempFN(wxT("tmp"), exportNameASCII, wxT("obj"));
			std::string tmpfn(std::string( tempFN.GetFullPath(wxPATH_UNIX).ToAscii() ));
			retVal &= (pcl::io::saveOBJFile(tmpfn, mesh, 7) == 0);
			retVal &= wxCopyFile(tempFN.GetFullPath(), filename, true);

			wxFileName materialInFN(tempFN), materialOutFN(filename);
			materialInFN.SetExt(wxT("mtl"));
			materialOutFN.SetName( exportNameASCII );
			materialOutFN.SetExt(wxT("mtl"));
			retVal &= wxCopyFile(materialInFN.GetFullPath(), materialOutFN.GetFullPath(), true);*/
		}
	}
	else
	{
		// Unsupported case
		retVal = false;
	}

	// Remove temporary directory
#if wxCHECK_VERSION(2, 9, 0)
	tmpDir.Rmdir(wxPATH_RMDIR_RECURSIVE);
#else
	boost::system::error_code ec;
	boost::filesystem::remove_all(boost::filesystem::path(tmpDir.GetPath().c_str()), ec);
#endif

	return retVal;
}
