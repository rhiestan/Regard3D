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
#include "config.h"

#if !wxCHECK_VERSION(2, 9, 0)
// wxWidgets before 2.9.0 does not support recursive delete
// Use boost::filesystem instead
#include <boost/filesystem.hpp>
#endif

// assimp
#include <assimp/postprocess.h>
#include <assimp/version.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/Exporter.hpp>

// Tinyply
#include "tinyply.h"

// boost
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "boost/filesystem/fstream.hpp"

#include <boost/chrono.hpp>


void R3DModelOperations::combineDenseModels(R3DProject::Densification *pDensification, int numModels)
{
	wxString combinedModelName(wxT("pmvs_combined.ply"));
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsDns(paths, pDensification);

	wxFileName modelFN(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc));

	std::vector<float> combinedVertices, combinedNormals;
	std::vector<uint8_t> combinedColors;

	for(int i = 0; i < numModels; i++)
	{
		modelFN.SetFullName(wxString::Format(wxT("option-%04d.ply"), i));
		if(modelFN.FileExists())
		{

#if defined(R3D_WIN32)
			boost::filesystem::ifstream istream(boost::filesystem::path(modelFN.GetFullPath().wc_str()), std::ios::binary);
#else
			boost::filesystem::ifstream istream(boost::filesystem::path(modelFN.GetFullPath().mb_str()), std::ios::binary);
#endif

			tinyply::PlyFile file(istream);

			tinyply::PlyProperty::Type coordinateType = tinyply::PlyProperty::Type::FLOAT32;
			tinyply::PlyProperty::Type colorType = tinyply::PlyProperty::Type::UINT8;
			tinyply::PlyProperty::Type normalType = tinyply::PlyProperty::Type::FLOAT32;
			bool hasDiffuseColors = false;
			for (auto e : file.get_elements())
			{
				std::string name = e.name;
				size_t sz = e.size;
				for(auto p : e.properties)
				{
					std::string propN = p.name;
					std::string propS = tinyply::PropertyTable[p.propertyType].str;

					if(name == std::string("vertex")
						&& propN == std::string("x"))
						coordinateType = p.propertyType;
					if(name == std::string("vertex")
						&& propN == std::string("nx"))
						normalType = p.propertyType;
					if(name == std::string("vertex")
						&& propN == std::string("red"))
						colorType = p.propertyType;
					if(name == std::string("vertex")
						&& propN == std::string("diffuse_red"))
					{
						colorType = p.propertyType;
						hasDiffuseColors = true;
					}

				}
			}

			std::vector<float> vertsF, normalsF;
			std::vector<double> vertsD, normalsD;
			std::vector<uint8_t> colorsUI8;
			size_t vertexCount = 0, colorCount = 0, normalsCount = 0;

			if(coordinateType == tinyply::PlyProperty::Type::FLOAT32)
				vertexCount = file.request_properties_from_element("vertex", { "x", "y", "z" }, vertsF);
			else
				vertexCount = file.request_properties_from_element("vertex", { "x", "y", "z" }, vertsD);
			if(normalType == tinyply::PlyProperty::Type::FLOAT32)
				normalsCount = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }, normalsF);
			else
				normalsCount = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }, normalsD);
			if(hasDiffuseColors)
				colorCount = file.request_properties_from_element("vertex", { "diffuse_red", "diffuse_green", "diffuse_blue" }, colorsUI8);
			else
				colorCount = file.request_properties_from_element("vertex", { "red", "green", "blue" }, colorsUI8);

			file.read(istream);

			size_t vertsSize = vertsF.size() + vertsD.size();
			size_t colsSize = colorsUI8.size();
			if(vertsSize != colsSize)
				return;
			if(vertsSize < vertexCount*3)
				return;

			// Combine point clouds
			if(coordinateType == tinyply::PlyProperty::Type::FLOAT32)
				combinedVertices.insert(combinedVertices.end(), vertsF.begin(), vertsF.end());
			else
				combinedVertices.insert(combinedVertices.end(), vertsD.begin(), vertsD.end());
			if(normalType == tinyply::PlyProperty::Type::FLOAT32)
				combinedNormals.insert(combinedNormals.end(), normalsF.begin(), normalsF.end());
			else
				combinedNormals.insert(combinedNormals.end(), normalsD.begin(), normalsD.end());
			combinedColors.insert(combinedColors.end(), colorsUI8.begin(), colorsUI8.end());
		}
	}

	// Save combined point cloud
	wxFileName combinedModelFN(modelFN);
	combinedModelFN.SetFullName(combinedModelName);
#if defined(R3D_WIN32)
	boost::filesystem::ofstream ostream(boost::filesystem::path(combinedModelFN.GetFullPath().wc_str()), std::ios::binary);
#else
	boost::filesystem::ofstream ostream(boost::filesystem::path(combinedModelFN.GetFullPath().mb_str()), std::ios::binary);
#endif

	tinyply::PlyFile combinedFile;
	combinedFile.add_properties_to_element("vertex", { "x", "y", "z" }, combinedVertices);
	combinedFile.add_properties_to_element("vertex", { "nx", "ny", "nz" }, combinedNormals);
	combinedFile.add_properties_to_element("vertex", { "diffuse_red", "diffuse_green", "diffuse_blue" }, combinedColors);

	combinedFile.write(ostream, true);
	ostream.close();

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

	Assimp::Importer importer;
	const aiScene *pScene = importer.ReadFile( std::string(surfaceFN.GetFullPath().mb_str()),
		aiProcess_JoinIdenticalVertices);

	// Load dense point cloud
	std::ifstream istream(paths.relativeDenseModelName_, std::ios::binary);
	tinyply::PlyFile file(istream);

	tinyply::PlyProperty::Type coordinateType = tinyply::PlyProperty::Type::FLOAT32;
	tinyply::PlyProperty::Type colorType = tinyply::PlyProperty::Type::UINT8;
	bool hasDiffuseColors = false;
	for (auto e : file.get_elements())
	{
		std::string name = e.name;
		size_t sz = e.size;
		for(auto p : e.properties)
		{
			std::string propN = p.name;
			std::string propS = tinyply::PropertyTable[p.propertyType].str;

			if(name == std::string("vertex")
				&& propN == std::string("x"))
				coordinateType = p.propertyType;
			if(name == std::string("vertex")
				&& propN == std::string("red"))
				colorType = p.propertyType;
			if(name == std::string("vertex")
				&& propN == std::string("diffuse_red"))
			{
				colorType = p.propertyType;
				hasDiffuseColors = true;
			}

		}
	}

	std::vector<float> vertsF;
	std::vector<double> vertsD;
	std::vector<uint8_t> colorsUI8;
	size_t vertexCount = 0, colorCount = 0;

	if(coordinateType == tinyply::PlyProperty::Type::FLOAT32)
		vertexCount = file.request_properties_from_element("vertex", { "x", "y", "z" }, vertsF);
	else
		vertexCount = file.request_properties_from_element("vertex", { "x", "y", "z" }, vertsD);
	if(hasDiffuseColors)
		colorCount = file.request_properties_from_element("vertex", { "diffuse_red", "diffuse_green", "diffuse_blue" }, colorsUI8);
	else
		colorCount = file.request_properties_from_element("vertex", { "red", "green", "blue" }, colorsUI8);

	file.read(istream);

	size_t vertsSize = vertsF.size() + vertsD.size();
	size_t colsSize = colorsUI8.size();
	if(vertsSize != colsSize)
		return;
	if(vertsSize < vertexCount*3)
		return;

	// Create spatial index
	typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> point_type;
	typedef std::pair<point_type, size_t> value;
	typedef boost::geometry::index::rtree< value, boost::geometry::index::rstar<8> > rtree;
	typedef rtree::const_query_iterator rtree_iter;

	std::vector<value> valueVector;
	valueVector.reserve(vertsSize);
	for(size_t i = 0; i < vertexCount; i++)
	{
		if(!vertsF.empty())
			valueVector.push_back( std::make_pair( point_type(vertsF[i*3], vertsF[i*3+1], vertsF[i*3+2]), i) );
		if(!vertsD.empty())
			valueVector.push_back( std::make_pair( point_type(vertsD[i*3], vertsD[i*3+1], vertsD[i*3+2]), i) );
	}

	rtree rb(valueVector.begin(), valueVector.end());
	const int n_neighbours = pSurface->colVertNumNeighbours_;



	for(int i = 0; i < static_cast<int>(pScene->mNumMeshes); i++)
	{
		aiMesh *pMesh = pScene->mMeshes[i];
		if(pMesh != NULL)
		{
			if(pMesh->GetNumColorChannels() == 0)
				pMesh->mColors[0] = new aiColor4D[pMesh->mNumVertices];

			for(unsigned int v = 0; v < pMesh->mNumVertices; v++)
			{
				aiVector3D &vertex = pMesh->mVertices[v];

				std::vector<float> k_distances, k_weights;
				std::vector<value> result_n;
				k_distances.clear();
				k_weights.clear();

				rb.query(boost::geometry::index::nearest(point_type(vertex.x, vertex.y, vertex.z), n_neighbours), std::back_inserter(result_n));
		
				// Copy color
				if(!result_n.empty())
				{
					// Calculate distances
					k_distances.resize(result_n.size());
					for(size_t j = 0; j < result_n.size(); j++)
					{
						k_distances[j] = boost::geometry::distance(point_type(vertex.x, vertex.y, vertex.z), result_n[j].first);
					}

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
					for(size_t j = 0; j < result_n.size(); j++)
					{
						unsigned int nearestPointIndex = result_n[j].second;
						redf += k_weights[j] * static_cast<float>(colorsUI8[3*nearestPointIndex]);
						greenf += k_weights[j] * static_cast<float>(colorsUI8[3*nearestPointIndex+1]);
						bluef += k_weights[j] * static_cast<float>(colorsUI8[3*nearestPointIndex+2]);
					}

					aiColor4D &pointColor = pMesh->mColors[0][v];
					pointColor.r = redf/255.0f;
					pointColor.g = greenf/255.0f;
					pointColor.b = bluef/255.0f;
					pointColor.a = 0;
				}
			}
		}
	}

	// Save colorized model
	Assimp::Exporter exp;
	wxString colorizedModelName(wxT("model_surface_col.ply"));
	pSurface->finalSurfaceFilename_ = colorizedModelName;
	wxFileName surfaceOutFN(wxString(paths.relativeSurfacePath_.c_str(), wxConvLibc), colorizedModelName);
	const aiReturn res = exp.Export(pScene, "ply", surfaceOutFN.GetFullPath().c_str());
}

bool R3DModelOperations::exportToPointCloud(R3DProject::Densification *pDensification, const wxString &filename)
{
	R3DProjectPaths paths;
	R3DProject::getInstance()->getProjectPathsDns(paths, pDensification);
	bool retVal = true;

	// Determine export type
	wxFileName exportFN(filename);
	// Simple copy
	return wxCopyFile(wxString(paths.relativeDenseModelName_.c_str(), wxConvLibc), filename, true);
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
			// Currently disabled due to bug 1315 of AssImp
			// See also https://github.com/assimp/assimp/issues/1315
			return false;

			// Convert to Alias Wavefront Object
			if(pSurface->colorizationType_ == R3DProject::CTColoredVertices)
			{
				const aiScene* pScene = imp.ReadFile(surfaceModelFNStr.c_str(), 0);
				if(pScene == NULL)
					retVal = false;
				else
				{
					std::string tmpfn("tmp/temp_surface.obj");
					const aiReturn res = exp.Export(pScene, "obj", tmpfn.c_str());
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
				const aiScene* pScene = imp.ReadFile(surfaceModelFNStr.c_str(), aiProcess_JoinIdenticalVertices);
				if(pScene == NULL)
					retVal = false;
				else
				{
					std::string fn(filename.mb_str());
					const aiReturn res = exp.Export(pScene, "ply", fn.c_str());
				}
			}
			else
			{
				// PointCloudLibrary/AssImp do not support textures in PLY files
				retVal = false;
			}
		}
		else
		{
			// Load and save OBJ
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
