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
#include "R3DExternalPrograms.h"
#include "Regard3DSettings.h"

#include <wx/stdpaths.h>
#include <wx/dir.h>
#include <wx/ffile.h>

#include <iostream>

#if defined(R3D_WIN32)
#include <windows.h>
#elif defined(R3D_LINUX) || defined(R3D_MACOSX)
#include <dlfcn.h>
#endif

R3DExternalPrograms R3DExternalPrograms::instance_;

namespace
{
	// Cheap proxy for "an NVIDIA display driver with CUDA support is
	// installed": the CUDA driver API library ships with the display driver
	// itself (not with any particular CUDA toolkit release), so its mere
	// presence is a good signal without Regard3D having to link against CUDA
	// or parse driver/device properties. It can't guarantee the installed
	// driver is new enough for the colmap_cuda build actually shipped, so
	// this stays an auto-detection default, never the only way to choose.
	bool probeCudaDriver()
	{
#if defined(R3D_WIN32)
		HMODULE hMod = ::LoadLibraryW(L"nvcuda.dll");
		if(hMod != NULL)
		{
			::FreeLibrary(hMod);
			return true;
		}
		return false;
#elif defined(R3D_LINUX) || defined(R3D_MACOSX)
		void *pHandle = dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
		if(pHandle == NULL)
			pHandle = dlopen("libcuda.so", RTLD_LAZY | RTLD_LOCAL);
		if(pHandle != NULL)
		{
			dlclose(pHandle);
			return true;
		}
		return false;
#else
		return false;
#endif
	}
}


bool R3DExternalPrograms::initialize()
{
	if(!initialized_)
	{
		wxFileName exeFN(wxStandardPaths::Get().GetExecutablePath());
		exeFN.SetFullName(wxEmptyString);

		wxString configExePath = Regard3DSettings::getInstance().getExternalEXEPath();
		if(!configExePath.IsEmpty())
			exeFN.SetPath(configExePath);

		wxString executableExtension;
#if defined(R3D_WIN32)
		executableExtension = wxT("exe");
#endif
		bool isOK = true;
		allPaths_.Clear();
		wxFileName pmvsFN(exeFN);
		pmvsFN.AppendDir(wxT("pmvs"));
		if(pmvsFN.DirExists())
		{
			allPaths_.Add(pmvsFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("pmvs2"), executableExtension, pmvsPath_);
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("cmvs"), executableExtension, cmvsPath_);
			isOK &= checkExecutable(pmvsFN.GetPath(wxPATH_GET_VOLUME), wxT("genOption"), executableExtension, genOptionPath_);
		}
		else
			isOK = false;

		wxFileName poissonFN(exeFN);
		poissonFN.AppendDir(wxT("poisson"));
		if(poissonFN.DirExists())
		{
			allPaths_.Add(poissonFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(poissonFN.GetPath(wxPATH_GET_VOLUME), wxT("PoissonRecon"), executableExtension, poissonReconPath_);
			isOK &= checkExecutable(poissonFN.GetPath(wxPATH_GET_VOLUME), wxT("SurfaceTrimmer"), executableExtension, surfaceTrimmerPath_);
		}
		else
			isOK = false;

		wxFileName mveFN(exeFN);
		mveFN.AppendDir(wxT("mve"));
		if(mveFN.DirExists())
		{
			allPaths_.Add(mveFN.GetPath(wxPATH_GET_VOLUME));
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("makescene"), executableExtension, makescenePath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("texrecon"), executableExtension, texreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("dmrecon"), executableExtension, dmreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("scene2pset"), executableExtension, scene2psetPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("fssrecon"), executableExtension, fssreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("meshclean"), executableExtension, meshcleanPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("smvsrecon"), executableExtension, smvsreconPath_);
			isOK &= checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("smvsrecon_SSE41"), executableExtension, smvsreconSSE41Path);
		}
		else
			isOK = false;

		wxFileName cmpmvsFN(exeFN);
		cmpmvsFN.AppendDir(wxT("cmpmvs"));
		if(cmpmvsFN.DirExists())
		{
			allPaths_.Add(cmpmvsFN.GetPath(wxPATH_GET_VOLUME));
			checkExecutable(mveFN.GetPath(wxPATH_GET_VOLUME), wxT("CMPMVS"), executableExtension, cmpmvsPath_);
		}

		// OpenMVG command line tools. Optional while the built-in engine is
		// still available, so a missing directory must not fail the check below.
		wxFileName openMVGFN(exeFN);
		openMVGFN.AppendDir(wxT("openmvg"));
		if(openMVGFN.DirExists())
		{
			const wxString openMVGPath(openMVGFN.GetPath(wxPATH_GET_VOLUME));
			allPaths_.Add(openMVGPath);
			checkExecutable(openMVGPath, wxT("openMVG_main_ComputeFeatures"), executableExtension, computeFeaturesPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_ComputeFeatures_OpenCV"), executableExtension, computeFeaturesOpenCVPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_PairGenerator"), executableExtension, pairGeneratorPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_ComputeMatches"), executableExtension, computeMatchesPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_GeometricFilter"), executableExtension, geometricFilterPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_SfM"), executableExtension, sfmPath_);

			// The exports
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2PMVS"), executableExtension, openMVG2PMVSPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2MVE2"), executableExtension, openMVG2MVE2Path_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2Colmap"), executableExtension, openMVG2ColmapPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2Agisoft"), executableExtension, openMVG2AgisoftPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2WebGL"), executableExtension, openMVG2WebGLPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_ConvertSfM_DataFormat"), executableExtension, convertSfMDataFormatPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2MESHLAB"), executableExtension, openMVG2MeshLabPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2NVM"), executableExtension, openMVG2NVMPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2CMPMVS"), executableExtension, openMVG2CMPMVSPath_);
			checkExecutable(openMVGPath, wxT("openMVG_main_openMVG2openMVS"), executableExtension, openMVG2openMVSPath_);
		}

		// COLMAP, used for dense reconstruction as an alternative to CMVS/PMVS,
		// MVE and SMVS. Shipped as two separate builds since patch_match_stereo
		// needs a CUDA GPU: colmap_cuda/ (GPU-accelerated) and colmap_nocuda/
		// (CPU-only, slower but runs everywhere). Optional while CMVS/PMVS/MVE/
		// SMVS remain available, so a missing directory must not fail the check
		// below; which one is actually used is decided per-project (see
		// R3DProject::Densification::colmapUseCuda_), defaulting to whichever
		// this detects as available.
		wxFileName colmapCudaFN(exeFN);
		colmapCudaFN.AppendDir(wxT("colmap_cuda"));
		if(colmapCudaFN.DirExists())
		{
			const wxString colmapCudaPath(colmapCudaFN.GetPath(wxPATH_GET_VOLUME));
			allPaths_.Add(colmapCudaPath);
			checkExecutable(colmapCudaPath, wxT("colmap"), executableExtension, colmapCudaPath_);
		}

		wxFileName colmapNoCudaFN(exeFN);
		colmapNoCudaFN.AppendDir(wxT("colmap_nocuda"));
		if(colmapNoCudaFN.DirExists())
		{
			const wxString colmapNoCudaPath(colmapNoCudaFN.GetPath(wxPATH_GET_VOLUME));
			allPaths_.Add(colmapNoCudaPath);
			checkExecutable(colmapNoCudaPath, wxT("colmap"), executableExtension, colmapNoCudaPath_);
		}

		cudaDriverDetected_ = probeCudaDriver();

		// OpenMVS, for densification and for meshing/texturing a dense point
		// cloud it computed. Not shipped with Regard3D, so a missing directory
		// must not fail the check below; whatever is missing is simply not
		// offered (see isOpenMVSDensificationPossible()).
		wxFileName openMVSFN(exeFN);
		openMVSFN.AppendDir(wxT("openmvs"));
		if(openMVSFN.DirExists())
		{
			const wxString openMVSPath(openMVSFN.GetPath(wxPATH_GET_VOLUME));
			allPaths_.Add(openMVSPath);
			checkExecutable(openMVSPath, wxT("InterfaceCOLMAP"), executableExtension, interfaceCOLMAPPath_);
			checkExecutable(openMVSPath, wxT("DensifyPointCloud"), executableExtension, densifyPointCloudPath_);
			checkExecutable(openMVSPath, wxT("ReconstructMesh"), executableExtension, reconstructMeshPath_);
			checkExecutable(openMVSPath, wxT("RefineMesh"), executableExtension, refineMeshPath_);
			checkExecutable(openMVSPath, wxT("TextureMesh"), executableExtension, textureMeshPath_);
		}

		// Graphviz, used by OpenMVG's global SfM engine: it renders the graphs of
		// its HTML report by calling std::system("neato ..."), which searches PATH.
		// Purely optional, so a missing gv directory must not fail the check below.
		wxFileName graphvizFN(exeFN);
		graphvizFN.AppendDir(wxT("gv"));
		if(graphvizFN.DirExists())
		{
			wxString neatoPath;
			if(checkExecutable(graphvizFN.GetPath(wxPATH_GET_VOLUME), wxT("neato"), executableExtension, neatoPath))
			{
				graphvizPath_ = graphvizFN.GetPath(wxPATH_GET_VOLUME);
				allPaths_.Add(graphvizPath_);	// So the external tools find it too
			}
		}

		if(!isOK)
		{
			wxMessageBox(wxT("Third-party executables not found.\nPlease put them where the executable is located."),
				wxT("Third party executables not found"), wxICON_ERROR | wxOK, NULL);
		}

		initialized_ = true;
	}

	return true;
}

R3DExternalPrograms::R3DExternalPrograms()
	: initialized_(false), cudaDriverDetected_(false)
{
}

R3DExternalPrograms::~R3DExternalPrograms()
{
}

bool R3DExternalPrograms::isOpenMVSDensificationPossible(bool colmapTriangulation, wxString &reason)
{
	wxArrayString missing;
	if(densifyPointCloudPath_.IsEmpty())
		missing.Add(wxT("DensifyPointCloud (OpenMVS)"));
	if(colmapTriangulation)
	{
		if(interfaceCOLMAPPath_.IsEmpty())
			missing.Add(wxT("InterfaceCOLMAP (OpenMVS)"));
		if(getBestColmapPath().IsEmpty())
			missing.Add(wxT("colmap (for image_undistorter)"));
	}
	else if(openMVG2openMVSPath_.IsEmpty())
		missing.Add(wxT("openMVG_main_openMVG2openMVS"));

	if(missing.IsEmpty())
		return true;

	reason = wxT("Not found: ");
	for(size_t i = 0; i < missing.GetCount(); i++)
		reason += (i > 0 ? wxT(", ") : wxT("")) + missing[i];
	return false;
}

bool R3DExternalPrograms::isOpenMVSTool(const wxString &stepName)
{
	return stepName == wxT("InterfaceCOLMAP") || stepName == wxT("DensifyPointCloud")
		|| stepName == wxT("ReconstructMesh") || stepName == wxT("RefineMesh")
		|| stepName == wxT("TextureMesh");
}

void R3DExternalPrograms::collectOpenMVSLog(const wxString &workingDir, const wxString &toolName,
	const wxString &targetDir)
{
	wxDir dir(workingDir);
	if(!dir.IsOpened())
		return;

	wxArrayString logFilenames;
	wxString filename;
	bool cont = dir.GetFirst(&filename, toolName + wxT("-*.log"), wxDIR_FILES);
	while(cont)
	{
		logFilenames.Add(filename);
		cont = dir.GetNext(&filename);
	}

	for(size_t i = 0; i < logFilenames.GetCount(); i++)
	{
		const wxFileName logFN(workingDir, logFilenames[i]);
		{
			wxFFile logFile(logFN.GetFullPath(), wxT("rb"));
			wxString content;
			if(logFile.IsOpened() && logFile.ReadAll(&content, wxConvUTF8))
				std::cout << content.ToStdString() << std::flush;
		}

		wxFileName targetFN(targetDir, logFilenames[i]);
		targetFN.MakeAbsolute(workingDir);
		if(!wxRenameFile(logFN.GetFullPath(), targetFN.GetFullPath(), true))
			wxRemoveFile(logFN.GetFullPath());
	}
}

void R3DExternalPrograms::removeOpenMVSDepthMaps(const wxString &workingDir)
{
	wxDir dir(workingDir);
	if(!dir.IsOpened())
		return;

	wxArrayString depthMapFilenames;
	wxString filename;
	bool cont = dir.GetFirst(&filename, wxT("depth*.dmap"), wxDIR_FILES);
	while(cont)
	{
		depthMapFilenames.Add(filename);
		cont = dir.GetNext(&filename);
	}

	for(size_t i = 0; i < depthMapFilenames.GetCount(); i++)
		wxRemoveFile(wxFileName(workingDir, depthMapFilenames[i]).GetFullPath());
}

bool R3DExternalPrograms::checkExecutable(const wxString &path, const wxString &name, const wxString &extension,
	wxString &outFullPath)
{
	wxFileName fn(path, name, extension);
	if(fn.IsFileExecutable())
	{
		outFullPath = fn.GetFullPath();
		return true;
	}

	return false;
}
	
